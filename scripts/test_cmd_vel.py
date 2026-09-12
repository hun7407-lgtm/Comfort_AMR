#!/usr/bin/env python3
"""STEP 4: verify that ROS 2 can drive the AMR in Isaac Sim.

Runs in the SYSTEM Python with the system ROS 2 Jazzy install, in a
separate process from Isaac Sim. It never imports omni or pxr.
See docs/SETUP.md section 2.

The sequence is stop, forward, stop, turn, stop. After each phase the
odometry is compared against what the command should have produced.

Launch it through scripts/run_cmd_vel_test.sh while scripts/run_sim.sh
is already running. Exit code 0 means every check passed.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from comfort_core import load_config


def yaw_from_quaternion(q) -> float:
    """Extract the Z rotation from a quaternion, in radians."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class CmdVelTester(Node):
    def __init__(self, cfg):
        super().__init__("comfort_amr_cmd_vel_test")
        topics = cfg["ros"]["topics"]

        # Isaac Sim's ROS2PublishOdometry uses the default sensor-ish
        # profile; best effort matches it and avoids a silent no-match.
        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.cmd_pub = self.create_publisher(Twist, topics["cmd_vel"], 10)
        self.odom_sub = self.create_subscription(
            Odometry, topics["odom"], self._on_odom, odom_qos
        )
        self.latest: Odometry | None = None
        self.odom_count = 0

    def _on_odom(self, msg: Odometry) -> None:
        self.latest = msg
        self.odom_count += 1

    def pose(self):
        """Current (x, y, yaw) from the last odometry message."""
        p = self.latest.pose.pose
        return p.position.x, p.position.y, yaw_from_quaternion(p.orientation)

    def sim_time(self) -> float:
        """Simulation clock, read from the odometry header stamp.

        Everything here is timed in simulated seconds, never wall-clock
        seconds. Isaac Sim does not run at a real-time factor of 1: it is
        faster than real time when headless and slower with a viewport,
        so a wall-clock timer would compare the measured distance against
        a duration the robot never experienced.
        """
        stamp = self.latest.header.stamp
        return stamp.sec + stamp.nanosec * 1e-9

    def drive(self, linear: float, angular: float, sim_seconds: float,
              wall_timeout: float = 180.0) -> float:
        """Hold a velocity command for `sim_seconds` of simulated time.

        The command is republished continuously. Isaac Sim's twist
        subscriber holds the last value it received, but a single message
        can be missed while the subscription is still matching.

        Returns the simulated seconds actually elapsed.
        """
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        t0 = self.sim_time()
        wall_deadline = time.time() + wall_timeout
        elapsed = 0.0
        while rclpy.ok() and time.time() < wall_deadline:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            elapsed = self.sim_time() - t0
            if elapsed >= sim_seconds:
                break
        return elapsed

    def stop(self, sim_seconds: float = 1.0) -> float:
        return self.drive(0.0, 0.0, sim_seconds)


FAILURES: list[str] = []


def check(passed: bool, label: str, detail: str = "") -> None:
    print(f"  {'[ OK ]' if passed else '[FAIL]'}   {label}", flush=True)
    if detail:
        print(f"           {detail}", flush=True)
    if not passed:
        FAILURES.append(label)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--linear", type=float, default=0.2,
                        help="forward speed in m/s (default 0.2)")
    parser.add_argument("--angular", type=float, default=0.5,
                        help="turn rate in rad/s (default 0.5)")
    parser.add_argument("--duration", type=float, default=4.0,
                        help="simulated seconds to hold each motion (default 4)")
    parser.add_argument("--timeout", type=float, default=30.0,
                        help="seconds to wait for the first odometry message")
    args = parser.parse_args()

    cfg = load_config()
    topics = cfg["ros"]["topics"]

    rclpy.init()
    node = CmdVelTester(cfg)

    print("\n=== Connecting ===", flush=True)
    print(f"  cmd_vel : {topics['cmd_vel']}", flush=True)
    print(f"  odom    : {topics['odom']}", flush=True)

    deadline = time.time() + args.timeout
    while node.latest is None and time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.latest is None:
        check(False, f"odometry received on {topics['odom']}",
              f"nothing arrived in {args.timeout} s. Is scripts/run_sim.sh "
              f"running, and do both sides share ROS_DOMAIN_ID?")
        return 1
    check(True, f"odometry received on {topics['odom']}",
          f"{node.odom_count} messages so far")

    # Let the robot settle so the baseline is not taken mid-bounce.
    node.stop(1.5)

    # ---------------- forward ----------------
    print("\n=== Forward ===", flush=True)
    x0, y0, _ = node.pose()
    elapsed = node.drive(args.linear, 0.0, args.duration)
    node.stop(1.5)
    x1, y1, _ = node.pose()

    travelled = math.hypot(x1 - x0, y1 - y0)
    expected = args.linear * elapsed
    print(f"  commanded {args.linear} m/s for {elapsed:.2f} simulated s "
          f"= {expected:.3f} m", flush=True)
    print(f"  odometry reports {travelled:.3f} m", flush=True)

    # Band allows for the acceleration ramp at each end and for the
    # extra distance covered while the stop command takes effect.
    check(travelled > 0.6 * expected, "AMR moved forward",
          f"{travelled:.3f} m, needs more than {0.6 * expected:.3f} m")
    check(travelled < 1.4 * expected, "forward distance is plausible",
          f"{travelled:.3f} m, must stay under {1.4 * expected:.3f} m")

    # ---------------- stop ----------------
    print("\n=== Stop holds ===", flush=True)
    x2, y2, _ = node.pose()
    node.stop(2.0)
    x3, y3, _ = node.pose()
    creep = math.hypot(x3 - x2, y3 - y2)
    check(creep < 0.05, "AMR stays still on zero command",
          f"drifted {creep:.4f} m over 2 s")

    # ---------------- turn ----------------
    print("\n=== Turn in place ===", flush=True)
    xa, ya, yaw0 = node.pose()
    elapsed = node.drive(0.0, args.angular, args.duration)
    node.stop(1.5)
    xb, yb, yaw1 = node.pose()

    turned = abs(math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0)))
    expected_turn = abs(args.angular) * elapsed
    print(f"  commanded {args.angular} rad/s for {elapsed:.2f} simulated s "
          f"= {expected_turn:.3f} rad", flush=True)
    print(f"  odometry reports {turned:.3f} rad", flush=True)

    # Yaw is measured modulo 2*pi, so a commanded turn at or beyond pi
    # cannot be compared against the expected magnitude.
    if expected_turn < math.pi:
        check(turned > 0.5 * expected_turn, "AMR rotated",
              f"{turned:.3f} rad, needs more than {0.5 * expected_turn:.3f} rad")
    else:
        check(turned > 0.3, "AMR rotated",
              f"{turned:.3f} rad (commanded turn exceeds pi, so it wraps)")

    # base_link sits ahead of the axle midpoint, so a spin traces a small
    # arc rather than staying exactly put.
    drift = math.hypot(xb - xa, yb - ya)
    check(drift < 0.5, "turn stayed roughly in place",
          f"centre moved {drift:.3f} m")

    node.stop(0.5)

    print("\n=== Summary ===", flush=True)
    if FAILURES:
        print(f"  {len(FAILURES)} check(s) failed:", flush=True)
        for f in FAILURES:
            print(f"    - {f}", flush=True)
        return 1
    print("  All checks passed. STEP 4 complete.", flush=True)
    return 0


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
