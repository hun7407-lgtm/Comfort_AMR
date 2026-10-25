#!/usr/bin/env python3
"""Run one trial through Nav2 and classify the outcome.

Runs in the SYSTEM Python with ROS 2 Jazzy, alongside Isaac Sim and Nav2
as separate processes.

Sends the configured goal to Nav2's NavigateToPose action, watches
odometry while it drives, and ends with exactly one of the four results
docs/EXPERIMENT.md defines.

    ./scripts/run_nav2_goal.sh
    ./scripts/run_nav2_goal.sh --goal 4 1.2
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from comfort_core import load_config, nav2_map, odom_to_world
from comfort_core.clearance import clearance, footprint_half_extents
from comfort_core.comfort_space import make_policy, shapes as build_shapes
from comfort_core.metrics import record_trial
from comfort_core.human_state import (GroundTruthStateProvider,
                                      SensorStateProvider)
from comfort_core.intrusion import footprint_outline, intrusion
from comfort_core.layout import human_positions, scene_bounds


def yaw_from_quaternion(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


@dataclass
class TrialResult:
    result: str = "TIMEOUT"
    goal_reached: bool = False
    planning_success: bool = False
    deadlock_detected: bool = False
    timeout: bool = False
    travel_time: float = 0.0
    travel_distance: float = 0.0
    final_distance_to_goal: float = float("nan")
    nav2_status: str = ""
    yield_seconds: float = 0.0
    yielded: bool = False
    waited_for_humans: float = 0.0
    safety_stop_seconds: float = 0.0
    safety_polygon: str = ""
    # Times the gate had to get the robot unstuck. Faults worked
    # around, not policy: a run that needs them often is saying
    # something about the layout, not about Comfort Space.
    backoffs: int = 0
    off_lane_recoveries: int = 0
    # State estimation: (true, estimated) -> ticks, and the rule used.
    state_confusion: dict = field(default_factory=dict)
    state_rule: str = ""
    deadlock_cause: str = ""
    invalid_reason: str = ""
    start_pose: tuple = (0.0, 0.0, 0.0)
    final_pose: tuple = (0.0, 0.0, 0.0)
    # Closest physical approach to each person, footprint to cylinder.
    # key -> (clearance in m, trial time in s). <= 0 means contact.
    min_clearance: dict = field(default_factory=dict)
    contact_seconds: float = 0.0
    # The same seconds, split by who closed the last centimetre: the
    # robot driving into a person, or a person walking into the robot.
    contact_robot_seconds: float = 0.0
    contact_person_seconds: float = 0.0
    evade_seconds: float = 0.0
    # Deepest intrusion into each person's reference Comfort Space.
    # key -> (depth in m, k_min, trial time). k_min < 1 means inside.
    comfort_worst: dict = field(default_factory=dict)
    comfort_seconds: dict = field(default_factory=dict)
    comfort_reference: str = ""


class Nav2Trial(Node):
    def __init__(self, cfg, common_path: str = "configs/common.yaml"):
        super().__init__("comfort_amr_nav2_trial")
        self.cfg = cfg
        self._common_path = common_path
        nav = cfg["navigation"]

        self.goal = (float(nav["goal"][0]), float(nav["goal"][1]))
        self.tolerance = float(nav["goal_tolerance"])
        self.max_trial_time = float(nav["max_trial_time"])

        self.planning_failure_duration = float(
            nav.get("planning_failure_duration", 5.0))
        self.planner_warmup = float(nav.get("planner_warmup", 6.0))
        self.trial_start_sim_time = float(nav.get("trial_start_sim_time", 0.0))
        self.bounds = scene_bounds(cfg)
        self.start = (float(nav["start"][0]), float(nav["start"][1]))

        dl = nav.get("deadlock", {})
        self.dl_v = float(dl.get("v_threshold", 0.02))
        self.dl_d = float(dl.get("d_threshold", 0.05))
        self.dl_duration = float(dl.get("duration", 8.0))

        self.create_subscription(Odometry, cfg["ros"]["topics"]["odom"],
                                 self._on_odom, 10)
        self.latest: Odometry | None = None

        # The yield policy publishes whether it is currently holding the
        # robot back. Time spent yielding is the quantity that separates
        # "stuck against something" from "declining to intrude".
        self.create_subscription(Bool, "/comfort_yield", self._on_yield, 10)
        # While a person is still walking, waiting is the correct
        # behaviour and not a deadlock. The no-progress timer is held.
        self.create_subscription(Bool, "/comfort_humans_moving",
                                 self._on_humans_moving, 10)
        self.humans_moving = False
        self.waited_for_humans = 0.0
        # Time spent stepping out of a walker's lane.
        self.create_subscription(Bool, "/comfort_evading", self._on_evading, 10)
        self.evading = False
        # Proof that the gate is alive. It publishes its state every
        # cycle, so silence means the process is not running -- and a
        # trial without the gate is not a trial: nothing reaches the
        # wheels, the robot sits at its start, and the result reads as
        # a deadlock with no cause. Nine trials were recorded that way
        # before this check existed, all because of one bad import.
        self.create_subscription(String, "/comfort_gate_state",
                                 self._on_gate_state, 10)
        self.gate_seen = False
        self._gate_state = ""
        self.backoffs = 0
        self.recoveries = 0

        # The safety layer stops the robot for physical contact, the
        # yield gate stops it for social reasons. Recording them apart is
        # what lets a deadlock be attributed to the Comfort Space policy
        # rather than to a safety stop.
        self.create_subscription(CollisionMonitorState,
                                 "/collision_monitor_state",
                                 self._on_collision_state, 10)
        self.safety_action = 0
        self.safety_stop_seconds = 0.0
        self._safety_since: float | None = None
        # Which zone last acted, and for how long it has been acting
        # without a break. A protective stop that never releases looks
        # exactly like a planner that cannot find a way through, unless
        # the record says which one it was.
        self.safety_polygon = ""
        self.yielding = False
        self.yield_seconds = 0.0
        self._yield_since: float | None = None
        # Where each person actually is, for the clearance record. Read
        # from the same /tf frames the Comfort Space follows, whose stamps
        # are simulated time like the odometry's.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.human_keys = [k for k, _x, _y in human_positions(cfg)]
        self._configured_humans = {k: (x, y) for k, x, y in human_positions(cfg)}

        # Comfort Space intrusion is measured against ONE reference
        # policy in every scenario, not against each scenario's own. B1
        # has no Comfort Space, and B2's larger one would make B2 look
        # worse for the same distance; judging all three by the same
        # yardstick is what makes the numbers comparable.
        from human_tracker import HumanTracker
        ref = str(cfg.get("evaluation", {}).get("comfort_reference", "b3"))
        self.ref_name = ref
        self.ref_cfg = load_config(scenario=ref, common_path=self._common_path)
        self.ref_policy = make_policy(self.ref_cfg)
        # The TRUE states, always, whatever the control loop is using.
        #
        # This is the measurement side: the reference Comfort Space that
        # every scenario is scored against is sized from these. Reading
        # them through whatever provider the config names breaks that
        # the moment the loop runs on perception -- the sensor provider
        # has nothing to say until it has been fed observations, so
        # `states()` came back empty, no reference shapes were built,
        # and twelve LiDAR trials recorded no intrusion and no state
        # accuracy at all while otherwise looking fine.
        self.declared_states = GroundTruthStateProvider(cfg).states()
        # The map-based estimator runs in EVERY trial, even when the
        # policy is driven by ground truth. It costs nothing and it is
        # the only way to know what the robot could have worked out for
        # itself, which is the question the Comfort Space rests on.
        self.estimator = SensorStateProvider(cfg)
        self.tracker = HumanTracker(self, cfg)
        self.half_extents = footprint_half_extents(cfg)

        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        # Asked once, before driving, to separate "no route exists" from
        # "a route exists and the robot is stuck on it".
        self.planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose")

    def _on_odom(self, msg: Odometry) -> None:
        self.latest = msg

    def _on_collision_state(self, msg: CollisionMonitorState) -> None:
        stopping = msg.action_type in (CollisionMonitorState.STOP,
                                       CollisionMonitorState.SLOWDOWN,
                                       CollisionMonitorState.APPROACH)
        if self.latest is not None:
            now = self.sim_time()
            if stopping and self._safety_since is None:
                self._safety_since = now
                self.safety_polygon = msg.polygon_name or "zone"
                self.get_logger().info(
                    f"safety layer acting: {self.safety_polygon} "
                    f"(type {msg.action_type})")
            elif not stopping and self._safety_since is not None:
                self.safety_stop_seconds += now - self._safety_since
                self._safety_since = None
        self.safety_action = msg.action_type

    def continuous_safety_seconds(self) -> float:
        """How long the safety layer has been acting without releasing."""
        if self._safety_since is None:
            return 0.0
        return self.sim_time() - self._safety_since

    def _on_gate_state(self, msg: String) -> None:
        self.gate_seen = True
        # Count the recoveries, so a trial that only finished because
        # it got itself unstuck twice does not read like a clean run.
        # Both of these are faults being worked around, not policy, and
        # a run that needs them often is telling us something about the
        # layout rather than about Comfort Space.
        label = msg.data
        if label != self._gate_state:
            if label == "BACKING OFF":
                self.backoffs += 1
            elif label == "RECOVERING":
                self.recoveries += 1
            self._gate_state = label

    def _on_evading(self, msg: Bool) -> None:
        self.evading = bool(msg.data)

    def _on_humans_moving(self, msg: Bool) -> None:
        self.humans_moving = bool(msg.data)

    def _on_yield(self, msg: Bool) -> None:
        if self.latest is None:
            return
        now = self.sim_time()
        if msg.data and self._yield_since is None:
            self._yield_since = now
        elif not msg.data and self._yield_since is not None:
            self.yield_seconds += now - self._yield_since
            self._yield_since = None
        self.yielding = bool(msg.data)

    def reset_yield_clock(self) -> None:
        """Restart the continuous-yield timer without losing the total."""
        if self._yield_since is not None:
            self.yield_seconds += self.sim_time() - self._yield_since
            self._yield_since = self.sim_time()

    def continuous_yield_seconds(self) -> float:
        """How long the robot has been yielding without a break.

        The robot is still allowed to rotate while yielding, so it never
        looks completely motionless and the velocity-based deadlock test
        does not fire. Refusing to advance for this long is the deadlock
        the study is about, so it is detected on its own terms.
        """
        if self._yield_since is None or self.latest is None:
            return 0.0
        return self.sim_time() - self._yield_since

    def total_yield_seconds(self) -> float:
        extra = 0.0
        if self._yield_since is not None and self.latest is not None:
            extra = self.sim_time() - self._yield_since
        return self.yield_seconds + extra

    def pose_is_sane(self) -> str:
        """Empty string if the reported pose is usable, else why not.

        A diverged articulation reports nan or a position kilometres
        away. Recording that as a travel distance would put a fabricated
        number in the results file, so a trial that sees it is discarded
        rather than classified. Seen once: 2960 m of travel in 33
        simulated seconds after PhysX lost the articulation.
        """
        x, y, yaw = self.pose()
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return "odometry returned a non-finite pose"
        slack = 5.0
        if not (self.bounds.x_min - slack <= x <= self.bounds.x_max + slack
                and self.bounds.y_min - slack <= y <= self.bounds.y_max + slack):
            return f"robot reported at ({x:.1f}, {y:.1f}), far outside the corridor"
        return ""

    def pose(self):
        """World-frame pose. See comfort_core/frames.py."""
        p = self.latest.pose.pose
        return odom_to_world(self.cfg, (p.position.x, p.position.y,
                                        yaw_from_quaternion(p.orientation)))

    def human_positions_now(self) -> dict:
        """(x, y) per person from /tf, or the configured spot until it arrives."""
        out = {}
        for key in self.human_keys:
            try:
                tf = self.tf_buffer.lookup_transform(
                    "world", f"human_{key}", rclpy.time.Time(),
                    timeout=Duration(seconds=0.0))
                t = tf.transform.translation
                out[key] = (t.x, t.y)
            except Exception:
                out[key] = self._configured_humans[key]
        return out

    def speed(self) -> float:
        t = self.latest.twist.twist
        return math.hypot(t.linear.x, t.linear.y)

    def world_velocity(self) -> tuple:
        """Velocity in the world frame. Odometry reports it body-frame."""
        t = self.latest.twist.twist
        _x, _y, yaw = self.pose()
        c, s = math.cos(yaw), math.sin(yaw)
        return (t.linear.x * c - t.linear.y * s,
                t.linear.x * s + t.linear.y * c)

    def closing_speeds(self, key: str, hx: float, hy: float) -> tuple:
        """How fast each party is approaching the other, m/s.

        Returns (robot toward person, person toward robot). Either can
        be negative, which means that party is moving away.
        """
        x, y, _yaw = self.pose()
        dx, dy = hx - x, hy - y
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return (0.0, 0.0)
        ux, uy = dx / d, dy / d
        rvx, rvy = self.world_velocity()
        hvx, hvy = self.tracker.velocities().get(key, (0.0, 0.0))
        return (rvx * ux + rvy * uy, -(hvx * ux + hvy * uy))

    def sim_time(self) -> float:
        stamp = self.latest.header.stamp
        return stamp.sec + stamp.nanosec * 1e-9

    def wait_for_odom(self, timeout: float = 30.0) -> bool:
        deadline = time.time() + timeout
        while self.latest is None and time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest is not None

    def plan_exists(self, timeout: float = 20.0):
        """Ask the planner directly whether a route to the goal exists.

        Returns (found, detail).

        Watching the /plan topic instead does not work. When the planner
        fails, Nav2's behaviour tree clears the global costmap and tries
        again; for a moment the obstacles are gone and a path appears, so
        /plan keeps ticking over even in a sealed corridor. Observed
        directly: a corridor with a 0.05 m opening produced 24 plans in
        40 seconds while the robot never moved.

        Asking compute_path_to_pose once, before any recovery has run,
        gives the answer the experiment actually needs.
        """
        if not self.planner.wait_for_server(timeout_sec=timeout):
            return False, "compute_path_to_pose action server unavailable"

        goal = ComputePathToPose.Goal()
        goal.goal = self._pose(*self.goal)
        # Plan from the trial's start pose, not from wherever the robot
        # happens to be. The question is whether the corridor admits a
        # route, and that answer must not change because the robot has
        # already wedged itself somewhere.
        goal.start = self._pose(*self.start)
        goal.use_start = True

        future = self.planner.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        handle = future.result()
        if handle is None or not handle.accepted:
            return False, "planner rejected the request"

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        outcome = result_future.result()
        if outcome is None:
            return False, "planner did not answer in time"

        if outcome.status != GoalStatus.STATUS_SUCCEEDED:
            detail = getattr(outcome.result, "error_msg", "") or "planner aborted"
            return False, detail

        poses = len(outcome.result.path.poses)
        if poses == 0:
            return False, "planner returned an empty path"
        return True, f"{poses} waypoints"

    def _pose(self, x: float, y: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        # Which way to be facing on arrival. Usually nothing -- the
        # study does not measure heading at the goal -- but a vehicle
        # parking in a cart slot has to face the aisle.
        #
        # A 1.05 m deep slot has a loaded cart behind it. Nose-in, the
        # robot's own protective field reaches 0.17 m into that cart
        # and it can never pull out again: measured, it drove in to let
        # somebody past and stood there for the rest of the trial.
        # Facing the aisle, the field points at open floor.
        yaw = self.cfg.get("navigation", {}).get("goal_yaw_deg")
        if yaw is None:
            pose.pose.orientation.w = 1.0
        else:
            half = math.radians(float(yaw)) / 2.0
            pose.pose.orientation.z = math.sin(half)
            pose.pose.orientation.w = math.cos(half)
        return pose

    def wait_for_rendezvous(self, verbose: bool = True) -> None:
        """Hold until the simulation clock reaches the agreed start time.

        Scripted people wait for the same instant, so the robot and the
        humans set off together however long Nav2 took to come up.
        """
        if self.trial_start_sim_time <= 0.0:
            return
        now = self.sim_time()
        if now >= self.trial_start_sim_time:
            if verbose:
                print(f"  [warn] the simulation clock is already at "
                      f"{now:.1f} s, past the agreed start of "
                      f"{self.trial_start_sim_time:.1f} s. Anyone on a script "
                      f"has set off without the robot; raise "
                      f"navigation.trial_start_sim_time.", flush=True)
            return
        if verbose:
            print(f"  waiting for the agreed start at "
                  f"{self.trial_start_sim_time:.0f} simulated s "
                  f"(now {now:.1f} s)", flush=True)
        while rclpy.ok() and self.sim_time() < self.trial_start_sim_time:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_costmap(self, verbose: bool = True) -> None:
        """Spin for planner_warmup simulated seconds before planning.

        The obstacle layer starts empty. Asking the planner immediately
        after Nav2 comes up reports a clear route through a corridor that
        is physically sealed, because nothing has been marked yet.
        """
        t0 = self.sim_time()
        while rclpy.ok() and self.sim_time() - t0 < self.planner_warmup:
            rclpy.spin_once(self, timeout_sec=0.05)
        if verbose:
            print(f"  costmap warm-up: {self.planner_warmup:.0f} simulated s",
                  flush=True)

    def send_goal(self, timeout: float = 30.0):
        """Send the goal in the map frame. Returns the goal handle."""
        if not self.client.wait_for_server(timeout_sec=timeout):
            return None

        msg = NavigateToPose.Goal()
        msg.pose = self._pose(*self.goal)

        future = self.client.send_goal_async(msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        handle = future.result()
        return handle if handle is not None and handle.accepted else None


STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
}


def run_trial(node: Nav2Trial, verbose: bool = True) -> TrialResult:
    res = TrialResult()

    node.wait_for_costmap(verbose)
    node.wait_for_rendezvous(verbose)
    if not node.gate_seen:
        res.result = "INVALID"
        res.invalid_reason = ("the comfort gate published nothing; it is not "
                              "running, so no command can reach the robot")
        return res
    found, detail = node.plan_exists()
    res.planning_success = found
    res.nav2_status = f"planner: {detail}"
    if verbose:
        print(f"  planner check: {'path found' if found else 'NO PATH'} ({detail})",
              flush=True)
    if not found:
        x, y, yaw = node.pose()
        res.result = "PLANNING_FAILURE"
        res.start_pose = res.final_pose = (x, y, yaw)
        res.final_distance_to_goal = math.hypot(node.goal[0] - x, node.goal[1] - y)
        return res

    handle = node.send_goal()
    if handle is None:
        # Nav2 never accepted the goal, so no path was ever attempted.
        res.result = "PLANNING_FAILURE"
        res.nav2_status = "navigate_to_pose rejected the goal"
        x, y, yaw = node.pose()
        res.start_pose = res.final_pose = (x, y, yaw)
        res.final_distance_to_goal = math.hypot(node.goal[0] - x, node.goal[1] - y)
        return res

    result_future = handle.get_result_async()

    x, y, yaw = node.pose()
    res.start_pose = (x, y, yaw)
    t_start = node.sim_time()
    last_x, last_y = x, y
    last_report = t_start

    # Deadlock is judged on progress toward the goal, not on how much
    # the robot wriggles. See the deadlock notes in configs/common.yaml.
    best_distance = math.hypot(node.goal[0] - x, node.goal[1] - y)
    last_progress_t = t_start
    contact_logged = False
    contact_blame = None
    last_tick = t_start

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)

        problem = node.pose_is_sane()
        if problem:
            res.result = "INVALID"
            res.invalid_reason = problem
            handle.cancel_goal_async()
            return res

        x, y, yaw = node.pose()
        now = node.sim_time()

        res.travel_distance += math.hypot(x - last_x, y - last_y)
        last_x, last_y = x, y
        tick_delta, last_tick = now - last_tick, now

        distance = math.hypot(node.goal[0] - x, node.goal[1] - y)

        # Record how close the robot actually came to each person. The
        # Comfort Space is what it should keep; this is what it kept, and
        # the only direct evidence of whether somebody walked into it.
        touching = None
        worst_gap = float("inf")
        for key, (hx, hy) in node.human_positions_now().items():
            gap = clearance(node.cfg, (x, y, yaw), hx, hy)
            best = res.min_clearance.get(key)
            if best is None or gap < best[0]:
                res.min_clearance[key] = (gap, now - t_start)
            if gap <= 0.0 and gap < worst_gap:
                worst_gap, touching = gap, (key, hx, hy)
        if touching is not None:
            res.contact_seconds += tick_delta
            # Who closed the last centimetre matters for B1. ISO 3691-4
            # verifies the protective field against a STATIONARY test
            # piece: it asks the vehicle to stop before it hits a
            # person, not to prevent a person at 1.3 m/s from walking
            # into a vehicle that has already stopped. Counting the
            # second case against the baseline would mean no baseline
            # can ever be compliant, so the two are recorded apart.
            robot_closing, person_closing = node.closing_speeds(*touching)
            # Blame is decided ONCE, by who closed the last centimetre,
            # and then holds for the whole touch.
            #
            # Deciding it again every tick scored the tail of a contact
            # against the robot: a person walks into it at 1.0 m/s,
            # stops dead against it, and from then on the only thing
            # moving is the robot creeping at 0.1 m/s, so every
            # remaining tick read as the robot driving in. Two trials
            # in the third stress wave were flagged that way; every
            # line in their logs said "person walked in".
            if contact_blame is None:
                contact_blame = ("robot" if robot_closing
                                 > max(person_closing, 0.05) else "person")
            if contact_blame == "robot":
                res.contact_robot_seconds += tick_delta
                blame = "ROBOT DROVE IN"
            else:
                res.contact_person_seconds += tick_delta
                blame = "person walked in"
            # Once per contact episode, with the numbers behind the
            # verdict, so the attribution can be checked rather than
            # taken on trust.
            if not contact_logged:
                contact_logged = True
                node.get_logger().info(
                    f"contact with {touching[0]} at {worst_gap:+.3f} m: "
                    f"{blame} (robot closing {robot_closing:+.2f} m/s, "
                    f"person closing {person_closing:+.2f} m/s)")
        else:
            contact_logged = False
            contact_blame = None

        # What the robot could have worked out for itself. The truth it
        # is scored against is the person's state RIGHT NOW, not the
        # label in the config: a walker who has reached the end of
        # their route and stopped is standing still, and calling that
        # an estimation error would be scoring the estimator against
        # something that is not true any more.
        speeds_now = node.tracker.speeds()
        here = node.human_positions_now()
        estimated = node.estimator.update(here, speeds_now, now)
        for key in node.declared_states:
            # Truth is what the person is doing NOW, worked out from
            # where they actually are and how fast they are actually
            # going -- not from the label in the config. A worker who
            # has left her machine and is standing by the racking is
            # `stationary`, whatever the file said she was at t = 0.
            #
            # While both of these come from /tf they are the same
            # measurement, so the matrix reads 100% and is testing the
            # RULE, not perception. It starts meaning something the day
            # the estimator is fed from scripts/people_tracker.py.
            # px, py, NOT x, y: those are the robot's, and this loop
            # runs in the middle of the tick that measures the robot.
            # Shadowing them put a person's position into the Comfort
            # Space measurement and into the final pose.
            px, py = here.get(key, (float("inf"), float("inf")))
            if speeds_now.get(key, 0.0) >= node.estimator.enter:
                truth = "moving"
            elif node.estimator.distance_to_spot(px, py) <= node.estimator.radius:
                truth = "station_engaged"
            else:
                truth = "stationary"
            guess = estimated.get(key)
            if guess is None:
                continue
            cell = (truth, guess)
            res.state_confusion[cell] = res.state_confusion.get(cell, 0) + 1

        # Social distance: the reference Comfort Space, sized from each
        # person's state and current speed exactly as the policy does.
        res.comfort_reference = node.ref_name
        axes = node.ref_policy.axes(node.declared_states, node.tracker.speeds())
        ref_shapes = build_shapes(node.ref_cfg, axes, node.human_positions_now(),
                                  node.tracker.headings(), 0.0)
        outline = footprint_outline((x, y, yaw), *node.half_extents)
        for shape in ref_shapes:
            k, depth = intrusion(shape, outline)
            worst = res.comfort_worst.get(shape.key)
            if worst is None or depth > worst[0] or (depth == 0.0 and k < worst[1]):
                res.comfort_worst[shape.key] = (depth, k, now - t_start)
            if k < 1.0:
                res.comfort_seconds[shape.key] = (
                    res.comfort_seconds.get(shape.key, 0.0) + tick_delta)
        if node.evading:
            res.evade_seconds += tick_delta

        def finish(label: str) -> TrialResult:
            res.result = label
            res.travel_time = now - t_start
            res.final_distance_to_goal = distance
            res.final_pose = (x, y, yaw)
            res.yield_seconds = node.total_yield_seconds()
            res.yielded = res.yield_seconds > 0.0
            # Including the interval still open. The counter only closes
            # on release, so a trial that ends while the robot is still
            # halted used to report 0 s of safety action -- the one case
            # where the number matters most.
            res.safety_stop_seconds = (node.safety_stop_seconds
                                       + node.continuous_safety_seconds())
            res.safety_polygon = node.safety_polygon
            # Every trial is scored against the same reference Comfort
            # Space, so a trial with people in it and no reference
            # measurement at all did not measure what this study
            # measures. It is a fault, not a result.
            #
            # This exists because of a silent one: with the control
            # loop fed by the LiDAR, the states the yardstick is sized
            # from came back empty, so no reference shape was ever
            # built. Twelve trials recorded no intrusion and no state
            # accuracy, and the only hint in the summary was a yield
            # time of zero.
            if node.declared_states and not res.comfort_worst:
                res.result = "INVALID"
                res.invalid_reason = (
                    "no reference Comfort Space was measured although "
                    f"{len(node.declared_states)} people were in the scene; "
                    "the yardstick did not run, so this trial says nothing "
                    "about social distance")
            res.backoffs = node.backoffs
            res.off_lane_recoveries = node.recoveries
            res.state_rule = node.estimator.source
            return res

        if result_future.done():
            status = result_future.result().status
            res.nav2_status = STATUS_NAMES.get(status, str(status))
            if status == GoalStatus.STATUS_SUCCEEDED and distance <= node.tolerance:
                res.goal_reached = True
                return finish("SUCCESS")
            return finish("DEADLOCK")

        if now - t_start >= node.max_trial_time:
            res.timeout = True
            handle.cancel_goal_async()
            return finish("TIMEOUT")

        # Waiting for somebody to walk past is not a deadlock: it ends
        # on its own. Hold both timers while anyone is still moving, and
        # record how long was spent doing it.
        if node.humans_moving:
            res.waited_for_humans += tick_delta
            last_progress_t = now
            node.reset_yield_clock()

        # Deadlock by yielding: a route exists and the robot is willing
        # to follow it, but every way forward would intrude on someone,
        # so it waits. This is the failure mode B2 is meant to produce.
        if node.continuous_yield_seconds() >= node.dl_duration:
            res.deadlock_detected = True
            res.deadlock_cause = "yielding to a Comfort Space"
            handle.cancel_goal_async()
            return finish("DEADLOCK")

        # Deadlock by lack of progress: a route exists and the robot is
        # busy, but it is not getting any closer to the goal.
        if distance < best_distance - node.dl_d:
            best_distance = distance
            last_progress_t = now
        elif now - last_progress_t >= node.dl_duration:
            res.deadlock_detected = True
            held = node.continuous_safety_seconds()
            # Attribution matters more than the number here, and the
            # two causes are not alternatives: a robot that has
            # stopped to respect somebody's Comfort Space is then
            # stood still in an aisle, and under this study's premise
            # nobody walks around it, so sooner or later one of them
            # comes inside the protective field and the safety layer
            # takes over a robot that was already waiting. Reporting
            # only the layer that happened to be active at the twelve
            # second mark split one event across two causes -- the
            # same B2 stall, at the same place in the aisle, was
            # recorded three times as comfort and five times as
            # safety.
            yielded = node.total_yield_seconds() > 0
            if held >= 1.0 and yielded:
                why = (f", a person came inside the protective field "
                       f"({node.safety_polygon}) while it was already "
                       f"yielding; held for the last {held:.0f} s")
            elif held >= 1.0:
                why = (f", safety layer holding it for the last "
                       f"{held:.0f} s ({node.safety_polygon})")
            elif yielded:
                why = " while yielding"
            else:
                why = ""
            res.deadlock_cause = f"no progress for {node.dl_duration:.0f} s" + why
            handle.cancel_goal_async()
            return finish("DEADLOCK")

        if verbose and now - last_report >= 2.0:
            print(f"  t={now - t_start:6.1f}s  pos=({x:6.2f},{y:6.2f})  "
                  f"dist={distance:5.2f}m  v={node.speed():4.2f}m/s  "
                  f"stuck={now - last_progress_t:4.1f}s"
                  f"{'  EVADING' if node.evading else '  YIELDING' if node.yielding else ''}", flush=True)
            last_report = now

    return res


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/common.yaml")
    parser.add_argument("--scenario", default=None, help="b1, b2, b3 or a path")
    parser.add_argument("--goal", nargs=2, type=float, metavar=("X", "Y"))
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--results", default=None,
                        help="where to write the row; defaults to "
                             "results/trials/<recording.version>")
    parser.add_argument("--no-record", action="store_true",
                        help="run the trial but write nothing")
    args = parser.parse_args()

    cfg = load_config(scenario=args.scenario, common_path=args.config)
    if args.goal:
        cfg["navigation"]["goal"] = list(args.goal)

    rclpy.init()
    node = Nav2Trial(cfg, args.config)

    print("\n=== Trial (Nav2) ===", flush=True)
    sc = cfg.get("scenario", {})
    print(f"  scenario    : {sc.get('name', 'common')}  "
          f"{sc.get('label', '')}", flush=True)
    print(f"  policy      : {cfg['comfort_space']['policy']}", flush=True)
    print(f"  goal        : {cfg['navigation']['goal']}", flush=True)
    print(f"  tolerance   : {cfg['navigation']['goal_tolerance']} m", flush=True)
    print(f"  time limit  : {cfg['navigation']['max_trial_time']} simulated s", flush=True)

    if not node.wait_for_odom():
        print("\n  [FAIL]   no odometry. Is scripts/run_sim.sh running?", flush=True)
        return 1

    x0, y0, _ = node.pose()
    print(f"  start       : ({x0:.2f}, {y0:.2f})  [world frame]", flush=True)
    print("\n=== Driving ===", flush=True)

    res = run_trial(node, verbose=not args.quiet)

    print("\n=== Result ===", flush=True)
    print(f"  result             : {res.result}", flush=True)
    if res.invalid_reason:
        print(f"  DISCARD THIS TRIAL : {res.invalid_reason}", flush=True)
        print("  The simulation diverged. Do not record this run.", flush=True)
    print(f"  nav2 status        : {res.nav2_status or 'n/a'}", flush=True)
    print(f"  goal reached       : {res.goal_reached}", flush=True)
    print(f"  planning success   : {res.planning_success}", flush=True)
    print(f"  deadlock detected  : {res.deadlock_detected}"
          f"{'  (' + res.deadlock_cause + ')' if res.deadlock_cause else ''}",
          flush=True)
    print(f"  travel time        : {res.travel_time:.1f} simulated s", flush=True)
    print(f"  travel distance    : {res.travel_distance:.2f} m", flush=True)
    print(f"  distance to goal   : {res.final_distance_to_goal:.3f} m", flush=True)
    print(f"  yielded            : {res.yielded} "
          f"({res.yield_seconds:.1f} simulated s held back)", flush=True)
    print(f"  safety layer       : "
          f"{res.safety_stop_seconds:.1f} simulated s acting"
          f"{'  [' + res.safety_polygon + ']' if res.safety_polygon else ''}"
          f"{'  <-- check this, B2 must deadlock on comfort, not safety'
             if res.safety_stop_seconds > 0 else ''}", flush=True)
    if res.waited_for_humans:
        print(f"  waited for people  : {res.waited_for_humans:.1f} simulated s "
              f"(not counted as a deadlock)", flush=True)
    if res.backoffs or res.off_lane_recoveries:
        print(f"  got itself unstuck : {res.backoffs} backoff(s), "
              f"{res.off_lane_recoveries} off-lane recovery(ies)",
              flush=True)
    for key, (gap, at) in sorted(res.min_clearance.items()):
        print(f"  closest to {key:<7s} : {gap:+.3f} m at t={at:.1f} s"
              f"{'  CONTACT' if gap <= 0.0 else ''}", flush=True)
    print(f"  contact time       : {res.contact_seconds:.1f} simulated s"
          f"  (robot drove in {res.contact_robot_seconds:.1f} s, "
          f"person walked in {res.contact_person_seconds:.1f} s)"
          f"{'  <-- B1 is not ISO compliant while this is above 0'
             if res.contact_robot_seconds > 0 else ''}", flush=True)
    print(f"  stepped aside      : {res.evade_seconds:.1f} simulated s", flush=True)
    if res.state_confusion:
        total = sum(res.state_confusion.values())
        right = sum(n for (t, g), n in res.state_confusion.items() if t == g)
        print(f"  state estimate     : {right / total * 100:.1f}% of "
              f"{total} readings agree with the truth  [{res.state_rule}]",
              flush=True)
        for (truth, guess), n in sorted(res.state_confusion.items(),
                                        key=lambda it: -it[1]):
            mark = "" if truth == guess else "   <-- wrong"
            print(f"      true {truth:<16s} read as {guess:<16s} "
                  f"{n / total * 100:5.1f}%{mark}", flush=True)
    print(f"  comfort reference  : {res.comfort_reference} "
          f"(same yardstick in every scenario)", flush=True)
    for key, (depth, k, at) in sorted(res.comfort_worst.items()):
        inside = res.comfort_seconds.get(key, 0.0)
        print(f"  intrusion into {key:<3s} : {depth:.2f} m deep at t={at:.1f} s, "
              f"{inside:.1f} s inside (k_min {k:.2f})", flush=True)

    if not args.no_record:
        # Written whatever the outcome, including a discarded run: a
        # trial missing from the file is indistinguishable from one
        # that was never run.
        try:
            csv_path, json_path = record_trial(
                cfg, asdict(res), args.config,
                map_path=nav2_map(cfg),
                out_dir=args.results)
            print(f"  recorded           : {csv_path}", flush=True)
            print(f"                       {json_path.name}", flush=True)
        except Exception as exc:  # noqa: BLE001 - never lose a trial to this
            print(f"  [WARN] could not record the trial: {exc}", flush=True)

    return 0 if res.result == "SUCCESS" else 1


if __name__ == "__main__":
    code = 1
    try:
        code = main()
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
    sys.exit(code)
