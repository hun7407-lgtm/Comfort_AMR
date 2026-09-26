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
from dataclasses import dataclass, field
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
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from comfort_core import load_config, odom_to_world
from comfort_core.clearance import clearance, footprint_half_extents
from comfort_core.comfort_space import make_policy, shapes as build_shapes
from comfort_core.human_state import make_state_provider
from comfort_core.intrusion import footprint_outline, intrusion
from comfort_core.layout import corridor_bounds, human_positions


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
    deadlock_cause: str = ""
    invalid_reason: str = ""
    start_pose: tuple = (0.0, 0.0, 0.0)
    final_pose: tuple = (0.0, 0.0, 0.0)
    # Closest physical approach to each person, footprint to cylinder.
    # key -> (clearance in m, trial time in s). <= 0 means contact.
    min_clearance: dict = field(default_factory=dict)
    contact_seconds: float = 0.0
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
        self.bounds = corridor_bounds(cfg)
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
        self.declared_states = make_state_provider(cfg).states()
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
                self.get_logger().info(
                    f"safety layer acting: {msg.polygon_name or 'zone'} "
                    f"(type {msg.action_type})")
            elif not stopping and self._safety_since is not None:
                self.safety_stop_seconds += now - self._safety_since
                self._safety_since = None
        self.safety_action = msg.action_type

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
        pose.pose.orientation.w = 1.0
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
        touching = False
        for key, (hx, hy) in node.human_positions_now().items():
            gap = clearance(node.cfg, (x, y, yaw), hx, hy)
            best = res.min_clearance.get(key)
            if best is None or gap < best[0]:
                res.min_clearance[key] = (gap, now - t_start)
            touching = touching or gap <= 0.0
        if touching:
            res.contact_seconds += tick_delta

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
            res.safety_stop_seconds = node.safety_stop_seconds
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
            res.deadlock_cause = (
                f"no progress for {node.dl_duration:.0f} s"
                + (" while yielding" if node.total_yield_seconds() > 0 else ""))
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
          f"{'  <-- check this, B2 must deadlock on comfort, not safety'
             if res.safety_stop_seconds > 0 else ''}", flush=True)
    if res.waited_for_humans:
        print(f"  waited for people  : {res.waited_for_humans:.1f} simulated s "
              f"(not counted as a deadlock)", flush=True)
    for key, (gap, at) in sorted(res.min_clearance.items()):
        print(f"  closest to {key:<7s} : {gap:+.3f} m at t={at:.1f} s"
              f"{'  CONTACT' if gap <= 0.0 else ''}", flush=True)
    print(f"  contact time       : {res.contact_seconds:.1f} simulated s", flush=True)
    print(f"  stepped aside      : {res.evade_seconds:.1f} simulated s", flush=True)
    print(f"  comfort reference  : {res.comfort_reference} "
          f"(same yardstick in every scenario)", flush=True)
    for key, (depth, k, at) in sorted(res.comfort_worst.items()):
        inside = res.comfort_seconds.get(key, 0.0)
        print(f"  intrusion into {key:<3s} : {depth:.2f} m deep at t={at:.1f} s, "
              f"{inside:.1f} s inside (k_min {k:.2f})", flush=True)

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
