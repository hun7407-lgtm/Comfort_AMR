#!/usr/bin/env python3
"""Send each fleet AMR its job list, one Nav2 goal at a time.

    ./scripts/run_fleet_goals.sh --config configs/fleet_nav2.yaml --scenario b3d

Runs in the SYSTEM Python with ROS 2 Jazzy, after scripts/run_fleet_nav2.sh
has a Nav2 stack up under /a1, /a2, ... This is the fleet's dispatcher:
what the scripted driver did inside the simulator -- walk a vehicle
down its job list -- done the way a real fleet does it, as goals to
the vehicle's own navigation. Nothing here steers; Nav2 does, with the
same costmap and the same Comfort Space mask as AMR1.

Timing is the scripted driver's, to the second: a vehicle sets off at
navigation.trial_start_sim_time + fleet.amrs.<name>.start_at on the
simulation clock, so the traffic AMR1 meets does not
depend on how long bringup took.

Each job is a NavigateToPose to the place's pose, facing the aisle
(the convention AMR1 parks with; see nav2_goal._pose).
SUCCEEDED moves to the next job. ABORTED is retried once after a
pause; a second ABORTED skips the job, and says so, rather than
leaving the vehicle parked in the aisle for the rest of the trial.

Progress goes to the log and to /fleet/status (std_msgs/String, one
line per vehicle per change).

It is also the fleet's measurement. The same yardstick nav2_goal.py
holds against AMR1 is held against every vehicle here, from the same
sources -- the people's /tf frames, the reference Comfort Space
(evaluation.comfort_reference), each vehicle's collision monitor --
and written on the way out as one row per vehicle in fleet_trials.csv
beside trials.csv (comfort_core/metrics.py, FLEET_FIELDS). Every
vehicle is a subject of the study on the same footing as AMR1, under
the same policy; the rows live in their own file only because the
columns differ. What only these vehicles have is the job list and the
other robots, so the row also carries legs done and the closest
another AMR came.
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Path as PathMsg
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from comfort_core import load_config  # noqa: E402
from comfort_core.clearance import clearance, footprint_half_extents  # noqa: E402
from comfort_core.comfort_space import (HeadingResolver, make_policy,  # noqa: E402
                                        shapes as build_shapes)
from comfort_core.human_state import truth_provider  # noqa: E402
from comfort_core.intrusion import bearing_sector, footprint_outline, intrusion  # noqa: E402
from comfort_core.metrics import record_fleet_trial  # noqa: E402
from human_tracker import HumanTracker  # noqa: E402

RETRY_PAUSE = 3.0


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Yardstick:
    """What one vehicle kept from people, other robots and its monitor.

    The per-tick arithmetic of nav2_goal.py's trial loop, for one more
    vehicle: clearance and contact against the people's actual
    positions, intrusion against the reference Comfort Space, seconds
    per collision-monitor action, and the closest any other robot came.
    """

    def __init__(self, node: "Dispatcher", name: str):
        self.node = node
        self.name = name
        self.min_clearance: Tuple[float, str] = (math.inf, "")
        self.min_clearance_t: Optional[float] = None
        self.worst_t: Optional[float] = None
        self.contact_s = 0.0
        self.contact_robot_s = 0.0
        self.contact_person_s = 0.0
        self._blame: Optional[str] = None
        self.worst: Tuple[float, float, str] = (0.0, math.inf, "")  # depth, k, who
        self.worst_bearing = ""
        self.comfort_s = 0.0
        self.travel_m = 0.0
        self.observed_s = 0.0
        self.min_gap_amr1 = math.inf
        self.min_gap_fleet: Tuple[float, str] = (math.inf, "")
        self.safety: Dict[int, float] = {}
        self._safety_since: Optional[float] = None
        self._safety_action: Optional[int] = None
        self._last_pose: Optional[Tuple[float, float, float]] = None
        self._last_t: Optional[float] = None
        self.glitches = 0
        self._near_logged = False
        node.create_subscription(CollisionMonitorState,
                                 f"/{name}/collision_monitor_state",
                                 self._on_monitor, 10)

    def _on_monitor(self, msg: CollisionMonitorState) -> None:
        now = self.node.sim_time
        if now is None:
            return
        acting = msg.action_type in (CollisionMonitorState.STOP,
                                     CollisionMonitorState.SLOWDOWN,
                                     CollisionMonitorState.APPROACH)
        changed = self._safety_action is not None and msg.action_type != self._safety_action
        if self._safety_since is not None and (not acting or changed):
            self.safety[self._safety_action] = (
                self.safety.get(self._safety_action, 0.0) + now - self._safety_since)
            self._safety_since = None
            self._safety_action = None
        if acting and self._safety_since is None:
            self._safety_since = now
            self._safety_action = msg.action_type

    def held_s(self, now: float) -> Tuple[float, float]:
        """(seconds the monitor has been STOPping without a break,
        seconds it has been acting in any way without a break)."""
        if self._safety_since is None:
            return (0.0, 0.0)
        acting = now - self._safety_since
        stop = acting if self._safety_action == CollisionMonitorState.STOP else 0.0
        return (stop, acting)

    def safety_s(self, action: int) -> float:
        total = self.safety.get(action, 0.0)
        if self._safety_since is not None and self._safety_action == action \
                and self.node.sim_time is not None:
            total += self.node.sim_time - self._safety_since
        return total

    def tick(self, now: float, pose: Tuple[float, float, float],
             people: Dict[str, Tuple[float, float]], shapes,
             others: Dict[str, Tuple[float, float, float]]) -> None:
        cfg = self.node.cfg
        x, y, yaw = pose
        dt = 0.0
        if self._last_pose is not None and self._last_t is not None:
            dt = max(now - self._last_t, 0.0)
            step = math.hypot(x - self._last_pose[0], y - self._last_pose[1])
            # A tick in which the vehicle "moved" faster than it can is
            # a transform glitch, not a measurement. AMR1's loop has
            # pose_is_sane() for the same reason. Counted, logged,
            # skipped -- a 3 mm clearance that lasts one tick and is
            # physically unreachable at 0.8 m/s must not become the
            # row's minimum.
            if dt > 1e-3 and step / dt > 3.0 * float(cfg["navigation"]["max_linear_speed"]):
                self.glitches += 1
                self.node.say(self.name, f"t={now:6.1f}  pose jumped {step:.2f} m in "
                                         f"{dt:.2f} s; tick ignored")
                self._last_pose, self._last_t = pose, now
                return
            self.travel_m += step
            self.observed_s += dt
        robot_v = ((x - self._last_pose[0]) / dt, (y - self._last_pose[1]) / dt) \
            if dt > 1e-3 and self._last_pose is not None else (0.0, 0.0)
        self._last_pose, self._last_t = pose, now

        # A position line every 2 s once the vehicle is working, the same
        # cadence as AMR1's progress lines, so a trial can be drawn
        # afterwards from the log alone (scripts/pilot_extra_figures.py).
        # Added 2026-11-05 for the 261105_cross_seeds1-8 campaign; logging only.
        if self.travel_m > 0.05 and \
                now - getattr(self, "_trace_t", -math.inf) >= 2.0:
            self._trace_t = now
            gap1 = min((math.hypot(ox - x, oy - y) for o, (ox, oy, _) in others.items()
                        if o == "amr1"), default=math.inf)
            self.node.say(self.name, f"t={now:6.1f}  pos=({x:6.2f},{y:6.2f})  "
                                     f"v={math.hypot(*robot_v):.2f}m/s  "
                                     f"gap_amr1={gap1:.2f}m")

        touching = None
        worst_gap = math.inf
        for key, (hx, hy) in people.items():
            gap = clearance(cfg, pose, hx, hy)
            if gap < self.min_clearance[0]:
                self.min_clearance = (gap, key)
                self.min_clearance_t = now
            if gap <= 0.0 and gap < worst_gap:
                worst_gap, touching = gap, (key, hx, hy)
        # The closest approach, with where both were, so a number in the
        # csv can be checked against the costmap video.
        if self.min_clearance[0] < 0.20 and not self._near_logged:
            self._near_logged = True
            key = self.min_clearance[1]
            hx, hy = people[key]
            self.node.say(self.name, f"t={now:6.1f}  within {self.min_clearance[0]:.3f} m of {key}: "
                                     f"robot ({x:+.2f}, {y:+.2f}) person ({hx:+.2f}, {hy:+.2f})")
        if touching is not None:
            self.contact_s += dt
            # Blame once per touch, by who closed the last centimetre,
            # the same rule nav2_goal.py applies to AMR1.
            if self._blame is None:
                key, hx, hy = touching
                d = math.hypot(hx - x, hy - y) or 1e-9
                ux, uy = (hx - x) / d, (hy - y) / d
                robot_closing = robot_v[0] * ux + robot_v[1] * uy
                hv = self.node.tracker.velocities().get(key, (0.0, 0.0))
                person_closing = -(hv[0] * ux + hv[1] * uy)
                self._blame = ("robot" if robot_closing > max(person_closing, 0.05)
                               else "person")
                self.node.say(self.name, f"t={now:6.1f}  contact with {key} at "
                                         f"{worst_gap:+.3f} m: {self._blame} closed in")
            if self._blame == "robot":
                self.contact_robot_s += dt
            else:
                self.contact_person_s += dt
        else:
            self._blame = None

        outline = footprint_outline(pose, *self.node.half_extents)
        for shape in shapes:
            k, depth = intrusion(shape, outline)
            if depth > self.worst[0] or (depth == 0.0 and k < self.worst[1]):
                self.worst = (depth, k, shape.key)
                self.worst_t = now
                self.worst_bearing = bearing_sector(shape, x, y)
            if k < 1.0:
                self.comfort_s += dt

        for other, (ox, oy, _oyaw) in others.items():
            gap = math.hypot(ox - x, oy - y)
            if other == "amr1":
                self.min_gap_amr1 = min(self.min_gap_amr1, gap)
            elif gap < self.min_gap_fleet[0]:
                self.min_gap_fleet = (gap, other)

    def row(self, v: "Vehicle", trial_t0: float) -> Dict[str, object]:
        def rel(t):
            return "" if t is None else round(t - trial_t0, 2)
        fin = math.isfinite
        return {
            "robot": self.name,
            "result": v.result,
            "deadlock_cause": v.deadlock_cause,
            "legs_total": len(v.jobs),
            "legs_done": v.legs_ok,
            "legs_skipped": v.legs_skipped,
            "set_off_at_s": rel(v.set_off_t),
            "first_leg_s": rel(v.first_leg_t),
            "travel_distance_m": round(self.travel_m, 3),
            "observed_s": round(self.observed_s, 1),
            "min_clearance_m": round(self.min_clearance[0], 3) if fin(self.min_clearance[0]) else "",
            "min_clearance_with": self.min_clearance[1],
            "contact_s": round(self.contact_s, 2),
            "contact_robot_s": round(self.contact_robot_s, 2),
            "contact_person_s": round(self.contact_person_s, 2),
            "comfort_reference": self.node.ref_name,
            "worst_intrusion_m": round(self.worst[0], 3),
            "worst_intrusion_k": round(self.worst[1], 3) if fin(self.worst[1]) else "",
            "worst_intrusion_into": self.worst[2],
            "comfort_seconds": round(self.comfort_s, 2),
            "safety_stop_s": round(self.safety_s(CollisionMonitorState.STOP), 2),
            "safety_slowdown_s": round(self.safety_s(CollisionMonitorState.SLOWDOWN), 2),
            "safety_approach_s": round(self.safety_s(CollisionMonitorState.APPROACH), 2),
            "min_gap_amr1_m": round(self.min_gap_amr1, 3) if fin(self.min_gap_amr1) else "",
            "min_gap_fleet_m": round(self.min_gap_fleet[0], 3) if fin(self.min_gap_fleet[0]) else "",
            "min_gap_fleet_with": self.min_gap_fleet[1],
            "min_clearance_t_s": rel(self.min_clearance_t),
            "worst_intrusion_t_s": rel(self.worst_t),
            "worst_intrusion_bearing": self.worst_bearing,
        }


def _facing_aisle(cfg, y: float, x: Optional[float] = None) -> float:
    """Yaw at a place: towards the nearer lane, radians.

    On a crossing (a ring via point) the heading is the ring's: south
    on the east crossing, north on the west one.
    """
    lanes = cfg["fleet"]["lanes"]
    north, south = float(lanes["north_y"]), float(lanes["south_y"])
    if x is not None and (abs(x - float(lanes["east_x"])) < 0.3
                          or abs(x - float(lanes["west_x"])) < 0.3) \
            and south < y < north:
        return -math.pi / 2.0 if x > 0 else math.pi / 2.0
    if abs(y - north) < 0.3 or abs(y - south) < 0.3:
        return 0.0
    return -math.pi / 2.0 if y > (north + south) / 2.0 else math.pi / 2.0


def _via_points(cfg) -> Dict[str, Tuple[float, float]]:
    """`via:east` / `via:west`: the crossings' midpoints."""
    lanes = cfg["fleet"]["lanes"]
    mid = (float(lanes["north_y"]) + float(lanes["south_y"])) / 2.0
    return {"via:east": (float(lanes["east_x"]), mid),
            "via:west": (float(lanes["west_x"]), mid)}


class Vehicle:
    """One fleet AMR's dispatcher state."""

    def __init__(self, node: Node, cfg, name: str, spec, places):
        self.node = node
        self.name = name
        self.cfg = cfg
        self.places = places
        # A leg is the real destinations; a "via:*" entry rides along as
        # an intermediate pose of the leg after it.
        self.vias = _via_points(cfg)
        raw = [str(j) for j in spec.get("jobs", []) if str(j) in places or str(j) in self.vias]
        self.jobs: List[str] = []
        self.leg_vias: List[List[str]] = []
        pending: List[str] = []
        for j in raw:
            if j in self.vias:
                pending.append(j)
            else:
                self.jobs.append(j)
                self.leg_vias.append(pending)
                pending = []
        self.start_at = (float(cfg["navigation"].get("trial_start_sim_time", 0.0))
                         if cfg["fleet"].get("hold_until_trial", False) else 0.0) \
            + float(spec.get("start_at", 0.0))
        self.client = ActionClient(node, NavigateToPose, f"/{name}/navigate_to_pose")
        self.through = ActionClient(node, NavigateThroughPoses,
                                    f"/{name}/navigate_through_poses")
        self.index = 0
        self.handle = None
        self.result_future = None
        self.retried = False
        self.pause_until: Optional[float] = None
        self.state = "waiting for the trial"
        self.done = False
        self.legs_ok = 0
        self.legs_skipped = 0
        self.set_off_t: Optional[float] = None
        self.first_leg_t: Optional[float] = None
        # The vehicle's own verdict, judged the way AMR1's is.
        self.result = "SUCCESS"
        self.deadlock_cause = ""
        dl = cfg["navigation"].get("deadlock", {})
        self.dl_distance = float(dl.get("d_threshold", 0.15))
        self.dl_duration = float(dl.get("duration", 12.0))
        self.goal_xy: Optional[Tuple[float, float]] = None
        self.best_distance = math.inf
        self.last_progress_t: Optional[float] = None
        # Progress is measured along Nav2's own plan, not as the crow
        # flies. A job across the bay leaves by the far crossing, and
        # for the first twenty metres of that the straight-line
        # distance to the goal GROWS while the vehicle drives at full
        # speed: judged on that, a3 was declared deadlocked at 0.8 m/s.
        # The remaining length of the published plan shrinks the whole
        # way. (nav2_goal.py's rule for AMR1 is the straight-line one;
        # its missions have happened to leave by the near crossing.)
        self.plan: Optional[List[Tuple[float, float]]] = None
        node.create_subscription(PathMsg, f"/{name}/plan", self._on_plan, 10)

    def _on_plan(self, msg: PathMsg) -> None:
        self.plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses] or None

    def remaining(self, pose: Tuple[float, float, float]) -> Optional[float]:
        """Metres of the current plan still ahead of the vehicle."""
        if not self.plan or len(self.plan) < 2:
            return None
        x, y = pose[0], pose[1]
        nearest = min(range(len(self.plan)),
                      key=lambda i: (self.plan[i][0] - x) ** 2 + (self.plan[i][1] - y) ** 2)
        total = math.hypot(self.plan[nearest][0] - x, self.plan[nearest][1] - y)
        for i in range(nearest, len(self.plan) - 1):
            total += math.hypot(self.plan[i + 1][0] - self.plan[i][0],
                                self.plan[i + 1][1] - self.plan[i][1])
        return total

    def pose_at(self, name: str) -> PoseStamped:
        x, y = self.vias.get(name) or self.places[name]
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        half = _facing_aisle(self.cfg, float(y), float(x)) / 2.0
        pose.pose.orientation.z = math.sin(half)
        pose.pose.orientation.w = math.cos(half)
        return pose

    def goal(self, job: str) -> NavigateToPose.Goal:
        msg = NavigateToPose.Goal()
        msg.pose = self.pose_at(job)
        return msg

    def through_goal(self, vias: List[str], job: str) -> NavigateThroughPoses.Goal:
        msg = NavigateThroughPoses.Goal()
        msg.poses = [self.pose_at(v) for v in vias] + [self.pose_at(job)]
        return msg

    def tick(self, now: float) -> Optional[str]:
        """Advance; returns a status line when something changed."""
        if self.done:
            return None
        if self.index >= len(self.jobs):
            self.done = True
            self.state = (f"finished: {self.legs_ok} leg(s) done, "
                          f"{self.legs_skipped} skipped")
            return self.state
        if now < self.start_at:
            return None
        if self.pause_until is not None:
            if now < self.pause_until:
                return None
            self.pause_until = None
        job = self.jobs[self.index]
        vias = self.leg_vias[self.index] if self.index < len(self.leg_vias) else []
        if self.handle is None:
            client = self.through if vias else self.client
            if not client.server_is_ready():
                if self.state != "waiting for navigate_to_pose":
                    self.state = "waiting for navigate_to_pose"
                    return self.state
                return None
            future = client.send_goal_async(
                self.through_goal(vias, job) if vias else self.goal(job))
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            self.handle = future.result()
            if self.handle is None or not self.handle.accepted:
                self.handle = None
                self.pause_until = now + RETRY_PAUSE
                self.state = f"goal {job} rejected, retrying"
                return self.state
            self.result_future = self.handle.get_result_async()
            if self.set_off_t is None:
                self.set_off_t = now
            self.goal_xy = self.places[job]
            self.best_distance = math.inf
            self.last_progress_t = now
            self.plan = None
            self.state = (f"-> {job}  ({self.index + 1}/{len(self.jobs)})"
                          + (f" via {', '.join(v.split(':')[1] for v in vias)}" if vias else ""))
            return self.state
        if self.result_future is not None and self.result_future.done():
            status = self.result_future.result().status
            self.handle = None
            self.result_future = None
            self.goal_xy = None
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.legs_ok += 1
                if self.first_leg_t is None:
                    self.first_leg_t = now
                self.index += 1
                self.retried = False
                self.state = f"reached {job}"
                return self.state
            if not self.retried:
                self.retried = True
                self.pause_until = now + RETRY_PAUSE
                self.state = f"{job} aborted, retrying once"
                return self.state
            self.legs_skipped += 1
            self.index += 1
            self.retried = False
            if self.result == "SUCCESS":
                self.result = "ABORTED"
            self.state = f"{job} aborted twice, skipped"
            return self.state
        return None

    def progress(self, now: float, pose: Tuple[float, float, float],
                 stop_held_s: float, acting_s: float) -> Optional[str]:
        """AMR1's deadlock rule, for this vehicle.

        A goal is active and the distance to it has not shrunk by
        `d_threshold` for `duration` seconds. Judged on progress toward
        the goal, not on whether the wheels turn, like nav2_goal.py.
        On a deadlock the goal is cancelled, the verdict is recorded
        and no further legs are sent; the measurement goes on.
        """
        if self.goal_xy is None or self.done or self.last_progress_t is None:
            return None
        d = self.remaining(pose)
        if d is None:
            d = math.hypot(self.goal_xy[0] - pose[0], self.goal_xy[1] - pose[1])
        if d < self.best_distance - self.dl_distance:
            self.best_distance = d
            self.last_progress_t = now
            return None
        if now - self.last_progress_t < self.dl_duration:
            return None
        if stop_held_s >= 1.0:
            why = f", safety layer holding it for the last {stop_held_s:.0f} s"
        elif acting_s >= 1.0:
            why = (f", safety layer slowing it (not stopping it) for the "
                   f"last {acting_s:.0f} s")
        else:
            why = ""
        self.result = "DEADLOCK"
        self.deadlock_cause = f"no progress for {self.dl_duration:.0f} s" + why
        if self.handle is not None:
            try:
                self.handle.cancel_goal_async()
            except Exception:
                pass
        self.handle = None
        self.result_future = None
        self.goal_xy = None
        self.done = True
        self.state = f"DEADLOCK {d:.2f} m of path left: {self.deadlock_cause}"
        return self.state


class Dispatcher(Node):
    def __init__(self, cfg, robots: List[str], common_path: str):
        super().__init__("fleet_goals")
        self.cfg = cfg
        self.sim_time: Optional[float] = None
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.status_pub = self.create_publisher(String, "/fleet/status", 10)
        fleet = cfg["fleet"]
        places = {str(k): (float(v[0]), float(v[1]))
                  for k, v in fleet.get("places", {}).items()}
        self.vehicles: Dict[str, Vehicle] = {
            name: Vehicle(self, cfg, name, fleet["amrs"][name], places)
            for name in robots}

        # The measurement side, as in nav2_goal.py: people from their
        # /tf frames, the reference Comfort Space every scenario is
        # scored against, true states refreshed each tick.
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.tracker = HumanTracker(self, cfg)
        self.ref_name = str(cfg.get("evaluation", {}).get("comfort_reference", "b3"))
        self.ref_cfg = load_config(scenario=self.ref_name, common_path=common_path)
        self.ref_policy = make_policy(self.ref_cfg)
        self.ref_headings = HeadingResolver(self.ref_cfg, self.ref_policy)
        self.truth = truth_provider(cfg)
        self.states = self.truth.states()
        self.half_extents = footprint_half_extents(cfg)
        self.amr1_frame = cfg["ros"]["frames"]["base"]
        self.yardsticks: Dict[str, Yardstick] = {n: Yardstick(self, n) for n in robots}
        self.trial_t0 = float(cfg["navigation"].get("trial_start_sim_time", 0.0))
        self._last_measure: Optional[float] = None

    def _pose(self, frame: str) -> Optional[Tuple[float, float, float]]:
        """A robot's pose in the map frame, or None until tf has it.

        map and world share an origin here: the static map -> odom
        transforms are written in world coordinates, which is also
        what the people's /tf frames are expressed in.
        """
        try:
            tf = self.tf.lookup_transform("map", frame, rclpy.time.Time(),
                                          timeout=Duration(seconds=0.0))
        except Exception:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, _yaw(q))

    def measure(self, now: float) -> None:
        """One tick of the yardstick for every vehicle, at ~10 Hz."""
        if self._last_measure is not None and now - self._last_measure < 0.1:
            return
        self._last_measure = now
        poses = {n: self._pose(f"{n}/base_footprint") for n in self.vehicles}
        amr1 = self._pose(self.amr1_frame)
        people = self.tracker.positions()
        speeds = self.tracker.speeds()
        if hasattr(self.truth, "update"):
            refreshed = self.truth.update(people, speeds, now)
            if refreshed:
                self.states = refreshed
        axes = self.ref_policy.axes(self.states, speeds)
        headings = self.ref_headings.resolve(self.states, self.tracker.headings(), speeds)
        shapes = build_shapes(self.ref_cfg, axes, people, headings, 0.0)
        for name, pose in poses.items():
            if pose is None:
                continue
            others = {o: p for o, p in poses.items() if o != name and p is not None}
            if amr1 is not None:
                others["amr1"] = amr1
            self.yardsticks[name].tick(now, pose, people, shapes, others)
            stop_held, acting = self.yardsticks[name].held_s(now)
            line = self.vehicles[name].progress(now, pose, stop_held, acting)
            if line:
                self.say(name, f"t={now:6.1f}  {line}")

    def _on_clock(self, msg: Clock) -> None:
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def say(self, name: str, line: str) -> None:
        text = f"[fleet] {name} {line}"
        print(text, flush=True)
        self.status_pub.publish(String(data=text))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="configs/common.yaml")
    ap.add_argument("--scenario", default=None)
    ap.add_argument("--robots", nargs="*", default=None,
                    help="which fleet AMRs; default: every enabled one")
    ap.add_argument("--results", default=None,
                    help="folder for fleet_trials.csv; default results/trials/<version>")
    args = ap.parse_args()
    cfg = load_config(scenario=args.scenario or None, common_path=args.config)
    fleet = cfg.get("fleet", {})
    if not fleet.get("enabled", False) or fleet.get("driver", "scripted") != "nav2":
        print("[fleet] fleet.driver is not nav2; nothing to dispatch", flush=True)
        return 0
    robots = args.robots or [n for n, s in sorted(fleet.get("amrs", {}).items())
                             if s.get("enabled", True)]

    # run_scenario.sh stops this with SIGTERM once the trial is over
    # (a background child of a non-interactive shell ignores SIGINT).
    # Either signal has to reach the rows below, not kill them.
    def _on_term(*_args):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _on_term)
    signal.signal(signal.SIGINT, _on_term)

    rclpy.init()
    node = Dispatcher(cfg, robots, args.config)
    for name, v in node.vehicles.items():
        node.say(name, f"{len(v.jobs)} job(s): {' -> '.join(v.jobs)}; "
                       f"sets off at {v.start_at:.1f} s sim time")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.sim_time is None:
                continue
            for name, v in node.vehicles.items():
                line = v.tick(node.sim_time)
                if line:
                    node.say(name, f"t={node.sim_time:6.1f}  {line}")
            node.measure(node.sim_time)
            # Keep measuring after the last job: the vehicle is parked
            # but people still walk past it until the trial ends.
    except KeyboardInterrupt:
        pass
    finally:
        # The rows first, before anything that could fail or block:
        # they are the point of this process.
        try:
            rows = [node.yardsticks[n].row(v, node.trial_t0)
                    for n, v in node.vehicles.items()]
            path = record_fleet_trial(cfg, rows, args.config, str(args.scenario or ""),
                                      out_dir=args.results)
            print(f"[fleet] recorded {path}", flush=True)
        except Exception:
            import traceback
            traceback.print_exc()
            rows = []
        for v in node.vehicles.values():
            if v.handle is not None:
                try:
                    v.handle.cancel_goal_async()
                except Exception:
                    pass
        for r in rows:
            node.say(r["robot"],
                     f"{r['result']}{' (' + r['deadlock_cause'] + ')' if r['deadlock_cause'] else ''}; "
                     f"legs {r['legs_done']}/{r['legs_total']} done, {r['legs_skipped']} skipped; "
                     f"{r['travel_distance_m']} m; "
                     f"min clearance {r['min_clearance_m']} m ({r['min_clearance_with']}); "
                     f"contact {r['contact_s']} s (robot {r['contact_robot_s']}); "
                     f"worst intrusion {r['worst_intrusion_m']} m into {r['worst_intrusion_into']} "
                     f"(k {r['worst_intrusion_k']}), {r['comfort_seconds']} s inside; "
                     f"safety stop {r['safety_stop_s']} / slow {r['safety_slowdown_s']} s; "
                     f"nearest AMR1 {r['min_gap_amr1_m']} m, nearest fleet "
                     f"{r['min_gap_fleet_m']} m ({r['min_gap_fleet_with']})")
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
