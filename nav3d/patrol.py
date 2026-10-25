#!/usr/bin/env python3
"""Drive the bay once around so slam_toolbox can see all of it.

Runs in the SYSTEM Python with the system ROS 2 Jazzy install, in a
separate process from Isaac Sim. It never imports omni or pxr.

Launched by scripts/run_mapping.sh, between bringing slam_toolbox up and
saving the map.

No planner, no obstacle avoidance, no Comfort Space. The route is the
aisle centrelines, taken from the same layout model the bay was built
from, and the bay is empty: the mapping config switches the people off.
Driving open-loop is safe precisely because of that, and it keeps the map
from depending on a planner that does not have a map yet.

The route, for any bay the layout model can describe:

    with cross aisles        a closed lap -- along one aisle, down a
                             crossing, back along the next, up the other
                             crossing. Two aisles and two crossings make
                             a rectangle; more aisles snake between them.
    without cross aisles     down the aisle and back again.

Laps are driven more than once by default. The second lap closes the loop
on scans the first lap only saw from one side, which is what pulls the
far wall into agreement with the near one.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import List, Tuple

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from comfort_core import load_config, odom_to_world, wrap_to_pi
from comfort_core.factory_layout import build_layout, drivable_rects

Point = Tuple[float, float]


def yaw_from_quaternion(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def lap_points(cfg) -> List[Point]:
    """One lap of the bay, in world coordinates.

    Derived from the layout rather than written down, so it still follows
    the aisles after the aisles have been moved, widened or renumbered.
    """
    layout = build_layout(cfg)
    aisles = sorted(layout.aisles, key=lambda b: -b.centre)
    if not aisles:
        raise ValueError("the layout has no aisle to drive down")

    if layout.crosses:
        # Turn where there is somewhere to turn into. Running to the very
        # end of the aisle and then turning across a work zone is how an
        # open-loop lap ends up inside a workbench.
        xs = sorted(c.centre[0] for c in layout.crosses)
        x_lo, x_hi = xs[0], xs[-1]
    else:
        margin = float(cfg["factory"].get("route", {}).get("end_margin", 1.5))
        x_lo, x_hi = layout.x_min + margin, layout.x_max - margin

    points: List[Point] = []
    at_hi = False
    for aisle in aisles:
        y = aisle.centre
        a, b = (x_hi, x_lo) if at_hi else (x_lo, x_hi)
        points.append((a, y))
        points.append((b, y))
        at_hi = not at_hi
    # Back to where the lap started. With two aisles that is up the far
    # crossing; with one it is back down the aisle we came along.
    points.append(points[0])
    return points


def check_drivable(cfg, points: List[Point]) -> List[str]:
    """Every waypoint has to be somewhere the robot is allowed to be."""
    rects = drivable_rects(build_layout(cfg))
    problems = []
    for i, (x, y) in enumerate(points):
        if not any(r.x_min <= x <= r.x_max and r.y_min <= y <= r.y_max
                   for r in rects):
            problems.append(f"waypoint {i} ({x:+.2f}, {y:+.2f}) is not in "
                            f"an aisle, a crossing or a refuge")
    return problems


class Patrol(Node):
    """Turn towards the next waypoint, drive to it, repeat."""

    def __init__(self, cfg, points: List[Point], speed: float,
                 turn_speed: float) -> None:
        super().__init__("mapping_patrol")
        self.cfg = cfg
        self.points = points
        self.speed = speed
        self.turn_speed = turn_speed
        topics = cfg["ros"]["topics"]
        self.cmd = self.create_publisher(Twist, topics["cmd_vel"], 10)
        self.create_subscription(Odometry, topics["odom"], self._on_odom, 10)
        self.latest: Odometry | None = None
        # Close enough: there is nothing at a waypoint, it is only a
        # direction to stop going in. Stopping dead at each corner would
        # add four standing starts per lap and nothing to the map.
        self.arrive = 0.25
        self.align = math.radians(12.0)

    def _on_odom(self, msg: Odometry) -> None:
        self.latest = msg

    def pose(self):
        """Pose in WORLD coordinates; /odom is relative to the spawn."""
        p = self.latest.pose.pose
        return odom_to_world(
            self.cfg,
            (p.position.x, p.position.y, yaw_from_quaternion(p.orientation)))

    def sim_time(self) -> float:
        stamp = self.latest.header.stamp
        return stamp.sec + stamp.nanosec * 1e-9

    def publish(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd.publish(msg)

    def wait_for_odom(self, timeout: float = 60.0) -> bool:
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest is not None:
                return True
        return False

    def hold(self, sim_seconds: float) -> None:
        """Sit still for a while, on the simulation's clock."""
        t0 = self.sim_time()
        wall_deadline = time.time() + 10.0 * sim_seconds + 30.0
        while rclpy.ok() and time.time() < wall_deadline:
            self.publish(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.sim_time() - t0 >= sim_seconds:
                return

    def back_off(self) -> None:
        """Reverse a little, after driving into something.

        Isaac Sim occasionally has the robot catch on a wheel at the
        start of a leg -- seen once, half a metre of travel in ninety
        seconds with 0.40 m/s commanded. Backing up and setting off
        again clears it, and it is a far better answer than losing a
        twenty-minute mapping run to a wheel.
        """
        t0 = self.sim_time()
        wall_deadline = time.time() + 60.0
        while rclpy.ok() and time.time() < wall_deadline:
            self.publish(-0.15, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.sim_time() - t0 >= 3.0:
                break
        self.publish(0.0, 0.0)

    def go_to(self, target: Point, timeout: float) -> str:
        """Drive to `target`. Returns "arrived", "stuck" or "timeout"."""
        t0 = self.sim_time()
        said = t0
        moved_at = t0
        was = None
        wall_deadline = time.time() + 20.0 * timeout + 60.0
        while rclpy.ok() and time.time() < wall_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            x, y, yaw = self.pose()
            dx, dy = target[0] - x, target[1] - y
            dist = math.hypot(dx, dy)
            if dist <= self.arrive:
                return "arrived"
            if was is None or math.hypot(x - was[0], y - was[1]) > 0.08:
                was, moved_at = (x, y), self.sim_time()
            elif self.sim_time() - moved_at > 6.0:
                self.publish(0.0, 0.0)
                print(f"      [WARN] no progress for 6 s at "
                      f"({x:+.2f}, {y:+.2f})", flush=True)
                return "stuck"
            err = wrap_to_pi(math.atan2(dy, dx) - yaw)
            if abs(err) > self.align:
                # Turn on the spot first. Mapping while sliding sideways
                # through a long arc is what smears a wall.
                v, w = 0.0, math.copysign(self.turn_speed, err)
            else:
                v, w = self.speed, 1.2 * err
            self.publish(v, w)
            now = self.sim_time()
            # Progress, so a leg that is going nowhere says so while it
            # happens rather than ninety seconds later.
            if now - said >= 5.0:
                said = now
                print(f"      {now - t0:5.1f}s  at ({x:+6.2f}, {y:+6.2f}) "
                      f"hdg {math.degrees(yaw):+6.1f}  {dist:5.2f} m to go  "
                      f"cmd v {v:.2f} w {w:+.2f}", flush=True)
            if self.sim_time() - t0 > timeout:
                self.publish(0.0, 0.0)
                print(f"      [WARN] gave up on ({target[0]:+.2f}, "
                      f"{target[1]:+.2f}) after {timeout:.0f} s, "
                      f"{dist:.2f} m short", flush=True)
                return "timeout"
        self.publish(0.0, 0.0)
        return "timeout"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="nav3d/mapping_factory_3d.yaml")
    parser.add_argument("--laps", type=int, default=2)
    parser.add_argument("--speed", type=float, default=0.40,
                        help="m/s along a leg; slower than a trial, because "
                             "the scan matcher sees more of the bay per "
                             "metre travelled")
    parser.add_argument("--turn-speed", type=float, default=0.50,
                        help="rad/s when turning on the spot")
    parser.add_argument("--leg-timeout", type=float, default=90.0,
                        help="simulated seconds before a leg is abandoned")
    args = parser.parse_args()

    cfg = load_config(common_path=args.config)
    if "factory" not in cfg:
        print("[ERROR] this patrol only knows how to drive a factory bay",
              file=sys.stderr)
        return 2

    points = lap_points(cfg)
    problems = check_drivable(cfg, points)
    if problems:
        for p in problems:
            print(f"[ERROR] {p}", file=sys.stderr)
        return 1

    print(f"  lap of {len(points) - 1} leg(s), {args.laps} time(s):", flush=True)
    for x, y in points:
        print(f"      ({x:+6.2f}, {y:+6.2f})", flush=True)

    rclpy.init()
    node = Patrol(cfg, points, args.speed, args.turn_speed)
    try:
        if not node.wait_for_odom():
            print("[ERROR] no /odom; is the simulation running?",
                  file=sys.stderr)
            return 1
        start = node.pose()
        print(f"  robot at ({start[0]:+.2f}, {start[1]:+.2f}), "
              f"heading {math.degrees(start[2]):+.0f} deg", flush=True)

        # Settle first: the scan matcher's first keyframe should be of a
        # stationary robot, and Isaac Sim's first second of physics is
        # the robot dropping onto its wheels.
        node.hold(2.0)

        # The lap closes on its own first point, so later laps pick up
        # from where the previous one ended rather than re-driving a leg
        # of zero length.
        legs = [(1, i, t) for i, t in enumerate(points)]
        for lap in range(2, args.laps + 1):
            legs += [(lap, i, t) for i, t in enumerate(points) if i > 0]
        # The robot spawns in an aisle but not necessarily on a corner. If
        # it is already as good as standing on the first one, going there
        # would mean turning round, reversing half a metre and turning
        # back -- two spins the scan matcher gains nothing from.
        if legs and math.dist(start[:2], legs[0][2]) <= 1.0:
            skipped = legs.pop(0)
            print(f"  already at ({skipped[2][0]:+.2f}, "
                  f"{skipped[2][1]:+.2f}), starting from the next corner",
                  flush=True)

        reached = 0
        for lap, i, target in legs:
            label = f"lap {lap} leg {i}"
            print(f"  {label}: -> ({target[0]:+.2f}, {target[1]:+.2f})",
                  flush=True)
            for attempt in range(3):
                outcome = node.go_to(target, args.leg_timeout)
                if outcome != "stuck" or attempt == 2:
                    break
                print(f"      backing off and trying again "
                      f"({attempt + 1} of 2)", flush=True)
                node.back_off()
            if outcome == "arrived":
                reached += 1
            else:
                print(f"[ERROR] {label} did not finish ({outcome}); the map "
                      f"would have a hole in it", file=sys.stderr)
                return 1
        node.publish(0.0, 0.0)
        # Let the last scans arrive and the map update tick over before
        # anything tries to save it.
        node.hold(3.0)
        here = node.pose()
        print(f"  {reached} leg(s) driven, finished at "
              f"({here[0]:+.2f}, {here[1]:+.2f})", flush=True)
        return 0
    finally:
        try:
            node.publish(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
