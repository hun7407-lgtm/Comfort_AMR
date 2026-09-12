#!/usr/bin/env python3
"""STEP 5: drive the AMR from start to goal with a simple controller.

Runs in the SYSTEM Python with the system ROS 2 Jazzy install, in a
separate process from Isaac Sim. It never imports omni or pxr.

There is no planner here and no obstacle handling. The robot turns to
face the goal, drives at it, and stops inside the tolerance. That is
enough to establish the pieces every later step needs: reading pose from
/odom, commanding /cmd_vel, deciding when a trial is over, and measuring
how long it took and how far it went.

PHASE D replaces this controller with Nav2. The result reporting here is
deliberately the same shape docs/EXPERIMENT.md defines, so STEP 10 can
build on it rather than around it.

Launch it through scripts/run_goto_goal.sh while scripts/run_sim.sh is
already running. Exit code 0 means the goal was reached.
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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from comfort_core import load_config, odom_to_world, wrap_to_pi
from comfort_core.layout import (
    corridor_bounds,
    free_intervals_at,
    robot_clearance_radius,
)


def yaw_from_quaternion(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


@dataclass
class TrialResult:
    """One trial's outcome, in the shape docs/EXPERIMENT.md defines.

    STEP 10 adds DEADLOCK and PLANNING_FAILURE. This step can only
    produce SUCCESS or TIMEOUT: with no planner there is nothing to fail
    at planning, and deadlock detection needs the corridor and the
    Comfort Space to be meaningful.
    """
    result: str = "TIMEOUT"
    goal_reached: bool = False
    blocked_seconds: float = 0.0
    travel_time: float = 0.0
    travel_distance: float = 0.0
    final_distance_to_goal: float = float("nan")
    start_pose: tuple = (0.0, 0.0, 0.0)
    final_pose: tuple = (0.0, 0.0, 0.0)


class GoToGoal(Node):
    def __init__(self, cfg):
        super().__init__("comfort_amr_goto_goal")
        nav = cfg["navigation"]
        topics = cfg["ros"]["topics"]

        self.goal = (float(nav["goal"][0]), float(nav["goal"][1]))
        self.tolerance = float(nav["goal_tolerance"])
        self.max_v = float(nav["max_linear_speed"])
        self.max_w = float(nav["max_angular_speed"])
        self.k_v = float(nav["k_linear"])
        self.k_w = float(nav["k_angular"])
        self.align_threshold = math.radians(float(nav["heading_align_threshold_deg"]))
        self.max_trial_time = float(nav["max_trial_time"])

        self.cmd_pub = self.create_publisher(Twist, topics["cmd_vel"], 10)
        self.create_subscription(Odometry, topics["odom"], self._on_odom, 10)

        self.cfg = cfg
        self.latest: Odometry | None = None

        avoid = nav.get("avoidance", {})
        self.avoid_enabled = bool(avoid.get("enabled", False))
        self.lookahead = float(avoid.get("lookahead", 1.5))
        self.planning_failure_duration = float(
            nav.get("planning_failure_duration", 5.0))
        self.clearance = robot_clearance_radius(cfg)
        self.bounds = corridor_bounds(cfg)

        # Per-human obstacle radius. None means "use the humans' physical
        # radius". STEP 8 supplies Comfort Space radii here instead, which
        # is the only thing that changes between B2 and B3.
        self.radii: dict | None = None

    def _on_odom(self, msg: Odometry) -> None:
        self.latest = msg

    def pose(self):
        """Current pose in WORLD coordinates.

        /odom is relative to the spawn pose, and the goal, the corridor
        and the humans are all specified in world coordinates. Convert
        once here so nothing downstream mixes the two frames.
        See comfort_core/frames.py.
        """
        p = self.latest.pose.pose
        odom_pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        return odom_to_world(self.cfg, odom_pose)

    def sim_time(self) -> float:
        """Simulation clock from the odometry stamp.

        Never use wall-clock time here: Isaac Sim does not run at a
        real-time factor of 1. See docs/SETUP.md.
        """
        stamp = self.latest.header.stamp
        return stamp.sec + stamp.nanosec * 1e-9

    def publish(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def stop(self, sim_seconds: float = 1.0) -> None:
        t0 = self.sim_time()
        wall_deadline = time.time() + 30.0
        while rclpy.ok() and time.time() < wall_deadline:
            self.publish(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.sim_time() - t0 >= sim_seconds:
                break

    def wait_for_odom(self, timeout: float = 30.0) -> bool:
        deadline = time.time() + timeout
        while self.latest is None and time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest is not None

    def choose_waypoint(self, x: float, y: float):
        """Where to steer right now. None means no gap is wide enough.

        Looks one `lookahead` further along the corridor, asks
        comfort_core.layout for the free spans of y at that cross-section,
        and picks the span closest to the goal line that the AMR fits
        through. Aiming at a gap rather than at the goal is what makes
        the robot weave past staggered obstacles.
        """
        goal_x, goal_y = self.goal
        if not self.avoid_enabled:
            return goal_x, goal_y

        direction = 1.0 if goal_x >= x else -1.0
        x_ahead = x + direction * self.lookahead

        # Never look past the goal, or the robot starts steering around
        # obstacles it has already cleared.
        if (goal_x - x_ahead) * direction <= 0.0:
            return goal_x, goal_y

        usable = [
            (lo, hi)
            for lo, hi in free_intervals_at(self.cfg, x_ahead, self.radii)
            if hi - lo >= 2.0 * self.clearance
        ]
        if not usable:
            return None

        def target_in(span):
            lo, hi = span
            return min(max(goal_y, lo + self.clearance), hi - self.clearance)

        # Prefer the gap that needs the least sideways movement from here.
        # Chasing the gap nearest the goal line instead makes the robot
        # cross the corridor in front of an obstacle it is about to pass.
        best = min(usable, key=lambda span: abs(target_in(span) - y))
        return x_ahead, target_in(best)

    def run(self, verbose: bool = True) -> TrialResult:
        res = TrialResult()

        x, y, yaw = self.pose()
        res.start_pose = (x, y, yaw)
        t_start = self.sim_time()
        last_x, last_y = x, y
        last_report = t_start
        blocked_since = None

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            x, y, yaw = self.pose()
            now = self.sim_time()

            res.travel_distance += math.hypot(x - last_x, y - last_y)
            last_x, last_y = x, y

            dx = self.goal[0] - x
            dy = self.goal[1] - y
            distance = math.hypot(dx, dy)

            if distance <= self.tolerance:
                res.result = "SUCCESS"
                res.goal_reached = True
                res.travel_time = now - t_start
                res.final_distance_to_goal = distance
                res.final_pose = (x, y, yaw)
                break

            if now - t_start >= self.max_trial_time:
                res.result = "TIMEOUT"
                res.travel_time = now - t_start
                res.final_distance_to_goal = distance
                res.final_pose = (x, y, yaw)
                break

            waypoint = self.choose_waypoint(x, y)
            if waypoint is None:
                # Nowhere wide enough to drive. Hold still and let the
                # blocked timer decide whether this is a real dead end.
                self.publish(0.0, 0.0)
                blocked_since = blocked_since if blocked_since is not None else now
                res.blocked_seconds = now - blocked_since
                if res.blocked_seconds >= self.planning_failure_duration:
                    res.result = "PLANNING_FAILURE"
                    res.travel_time = now - t_start
                    res.final_distance_to_goal = distance
                    res.final_pose = (x, y, yaw)
                    break
                if verbose and now - last_report >= 2.0:
                    print(f"  t={now - t_start:6.1f}s  pos=({x:6.2f},{y:6.2f})  "
                          f"no gap wide enough for {res.blocked_seconds:.1f}s",
                          flush=True)
                    last_report = now
                continue
            blocked_since = None

            wx, wy = waypoint
            heading_error = wrap_to_pi(math.atan2(wy - y, wx - x) - yaw)

            angular = max(-self.max_w, min(self.max_w, self.k_w * heading_error))
            if abs(heading_error) > self.align_threshold:
                # Turn in place. Driving through a large heading error
                # makes a differential base swing out in a wide arc,
                # which matters once the corridor is narrow.
                linear = 0.0
            else:
                linear = max(0.0, min(self.max_v, self.k_v * distance))

            self.publish(linear, angular)

            if verbose and now - last_report >= 2.0:
                print(f"  t={now - t_start:6.1f}s  pos=({x:6.2f},{y:6.2f})  "
                      f"dist={distance:5.2f}m  aim=({wx:5.2f},{wy:+5.2f})  "
                      f"heading_err={math.degrees(heading_error):+6.1f}deg",
                      flush=True)
                last_report = now

        self.stop(1.0)
        return res


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--goal", nargs=2, type=float, metavar=("X", "Y"),
                        help="override the goal from the config")
    parser.add_argument("--quiet", action="store_true",
                        help="suppress the periodic progress lines")
    parser.add_argument("--config", default="configs/common.yaml",
                        help="path to the common config")
    args = parser.parse_args()

    cfg = load_config(common_path=args.config)
    if args.goal:
        cfg["navigation"]["goal"] = list(args.goal)

    nav = cfg["navigation"]
    rclpy.init()
    node = GoToGoal(cfg)

    print("\n=== Trial ===", flush=True)
    print(f"  start (config) : {nav['start']}", flush=True)
    print(f"  goal           : {nav['goal']}", flush=True)
    print(f"  tolerance      : {nav['goal_tolerance']} m", flush=True)
    avoid = nav.get("avoidance", {})
    print(f"  avoidance      : "
          f"{'on, lookahead ' + str(avoid.get('lookahead')) + ' m' if avoid.get('enabled') else 'off'}",
          flush=True)
    print(f"  time limit     : {nav['max_trial_time']} simulated s", flush=True)

    if not node.wait_for_odom():
        print(f"\n  [FAIL]   no odometry on {cfg['ros']['topics']['odom']}", flush=True)
        print("           Is scripts/run_sim.sh running?", flush=True)
        return 1

    x0, y0, _ = node.pose()
    print(f"  start (actual) : ({x0:.2f}, {y0:.2f})  [world frame]", flush=True)

    straight_line = math.hypot(nav["goal"][0] - x0, nav["goal"][1] - y0)

    print("\n=== Driving ===", flush=True)
    res = node.run(verbose=not args.quiet)

    print("\n=== Result ===", flush=True)
    print(f"  result             : {res.result}", flush=True)
    print(f"  goal reached       : {res.goal_reached}", flush=True)
    print(f"  travel time        : {res.travel_time:.1f} simulated s", flush=True)
    print(f"  travel distance    : {res.travel_distance:.2f} m", flush=True)
    print(f"  start to goal      : {straight_line:.2f} m (straight line)", flush=True)
    # Compare the path actually driven against the straight line to where
    # the robot actually ended up. Comparing against the full start-to-goal
    # distance would score a trial that stopped short as better than
    # perfect, since the goal tolerance leaves a gap uncovered.
    #
    # Both endpoints come from the result, captured at the instant the
    # trial ended. Re-reading the pose here would include the coast
    # during the final stop, which the travelled distance does not.
    ideal = math.hypot(res.final_pose[0] - res.start_pose[0],
                       res.final_pose[1] - res.start_pose[1])
    if ideal > 1e-6:
        print(f"  detour ratio       : {res.travel_distance / ideal:.3f} "
              f"(1.0 = perfectly straight)", flush=True)
    print(f"  distance to goal   : {res.final_distance_to_goal:.3f} m", flush=True)

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
