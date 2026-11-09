#!/usr/bin/env python3
"""Are the other AMRs real, independent ROS 2 robots now?

    ./scripts/run_sim.sh --headless --config configs/fleet_nav2.yaml \
        --scenario b2 &
    python3 scripts/check_fleet_nav2.py

Runs in the SYSTEM Python with ROS 2 Jazzy, against a simulation that
is already up. No Nav2: this checks the simulator's half of stage 2 in
docs/NAV2.md section 5c, which is that each vehicle is a physical robot
on its own namespace.

What it proves, in order:

    1. each robot advertises its own topics
    2. each robot's tf chain is prefixed and complete
    3. driving one moves that one and leaves the others where they are
    4. AMR1 still has the bare topics it always had

Point 3 is the one that matters. Namespacing that looks right in the
attributes can still collapse onto one topic at runtime, and the way
that shows up is every vehicle moving together.

Exit code 0 means every check passed.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

FAILS: list[str] = []
PASSES = 0


def check(label: str, ok: bool, detail: str = "") -> None:
    global PASSES
    if ok:
        PASSES += 1
        print(f"  ok    {label}{'  ' + detail if detail else ''}")
    else:
        FAILS.append(f"{label}  {detail}")
        print(f"  FAIL  {label}{'  ' + detail if detail else ''}")


class Watcher(Node):
    def __init__(self, robots: list[str]):
        super().__init__("check_fleet_nav2")
        self.robots = robots
        self.odom: dict[str, Odometry] = {}
        self.clouds: dict[str, int] = {r: 0 for r in robots}
        self.cmd = {}
        for r in robots:
            self.create_subscription(
                Odometry, f"/{r}/odom",
                lambda m, r=r: self.odom.__setitem__(r, m), 10)
            self.create_subscription(
                PointCloud2, f"/{r}/lidar/cloud",
                lambda m, r=r: self.clouds.__setitem__(r, self.clouds[r] + 1), 1)
            self.cmd[r] = self.create_publisher(Twist, f"/{r}/cmd_vel", 10)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

    def spin(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def drive(self, robot: str, vx: float, seconds: float) -> None:
        msg = Twist()
        msg.linear.x = vx
        end = time.time() + seconds
        while time.time() < end:
            self.cmd[robot].publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd[robot].publish(Twist())
        self.spin(0.5)

    def xy(self, robot: str):
        m = self.odom.get(robot)
        if m is None:
            return None
        return (m.pose.pose.position.x, m.pose.pose.position.y)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", nargs="+", default=["a1", "a2"])
    ap.add_argument("--wait", type=float, default=60.0,
                    help="seconds to wait for the simulation's topics")
    ap.add_argument("--drive-seconds", type=float, default=4.0)
    ap.add_argument("--speed", type=float, default=0.3)
    args = ap.parse_args()

    rclpy.init()
    node = Watcher(args.robots)

    print(f"[1] waiting up to {args.wait:.0f} s for odometry from "
          f"{', '.join(args.robots)}")
    deadline = time.time() + args.wait
    while time.time() < deadline and len(node.odom) < len(args.robots):
        rclpy.spin_once(node, timeout_sec=0.2)
    node.spin(2.0)

    names = dict(node.get_topic_names_and_types())
    for r in args.robots:
        check(f"/{r}/odom advertised", f"/{r}/odom" in names)
        check(f"/{r}/cmd_vel advertised", f"/{r}/cmd_vel" in names)
        check(f"/{r}/lidar/cloud advertised", f"/{r}/lidar/cloud" in names)
        check(f"/{r}/odom receiving", node.xy(r) is not None,
              f"pose {node.xy(r)}")
        check(f"/{r}/lidar/cloud receiving", node.clouds[r] > 0,
              f"{node.clouds[r]} message(s)")

    print("\n[2] tf chain is prefixed and complete")
    for r in args.robots:
        for parent, child in ((f"{r}/odom", f"{r}/base_footprint"),
                              (f"{r}/base_footprint", f"{r}/base_link"),
                              (f"{r}/base_link", f"{r}/lidar_link")):
            ok = node.tf.can_transform(parent, child, rclpy.time.Time())
            check(f"{parent} -> {child}", ok)

    print("\n[3] driving one robot moves only that one")
    before = {r: node.xy(r) for r in args.robots}
    target = args.robots[0]
    node.drive(target, args.speed, args.drive_seconds)
    node.spin(1.0)
    after = {r: node.xy(r) for r in args.robots}

    for r in args.robots:
        if before[r] is None or after[r] is None:
            check(f"{r} pose readable", False)
            continue
        moved = math.dist(before[r], after[r])
        if r == target:
            check(f"{r} moved when driven", moved > 0.10, f"{moved:.3f} m")
        else:
            check(f"{r} stayed put", moved < 0.05, f"{moved:.3f} m")

    print("\n[4] AMR1 keeps its own topics")
    check("/odom still advertised", "/odom" in names)
    check("/cmd_vel still advertised", "/cmd_vel" in names)

    print(f"\n{PASSES} passed, {len(FAILS)} failed")
    for line in FAILS:
        print(f"  FAILED  {line}")

    node.destroy_node()
    rclpy.shutdown()
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
