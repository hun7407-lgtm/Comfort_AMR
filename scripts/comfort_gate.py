#!/usr/bin/env python3
"""Yield policy: stop rather than enter someone's Comfort Space.

Runs in the SYSTEM Python with ROS 2 Jazzy. No omni, no pxr.

Sits between Nav2 and the robot:

    controller_server ──/cmd_vel_nav──> comfort_gate ──/cmd_vel──> Isaac Sim

Nav2 sees the Comfort Space as expensive but crossable, which is the
honest way to model a person's personal space: it is not a wall, and the
robot is physically able to drive through it. Whether it *should* is a
separate decision, and this node is where that decision lives.

The rule is simple. Look ahead a fraction of a second along whatever
Nav2 just commanded. If that would put the robot inside a Comfort Space,
publish zero instead and keep waiting.

This is what produces the deadlock the study is about. Under a static
policy there is no route that does not intrude, so a robot that yields
waits forever. Under the state-adaptive policy the person absorbed in
their workstation needs less space, a gap opens, and the robot gets
through without ever intruding. The deadlock is caused by the policy,
not by a wall, which is the point.

Publishes /comfort_yield (std_msgs/Bool) so a trial can record how long
it spent yielding and to whom.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String

from comfort_core import load_config, odom_to_world
from comfort_core.comfort_space import make_policy, robot_grow, shapes as build_shapes
from comfort_core.human_state import make_state_provider
from comfort_core.shapes import ComfortShape


def yaw_from_quaternion(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class ComfortGate(Node):
    def __init__(self, cfg, in_topic: str, out_topic: str):
        super().__init__("comfort_gate")
        self.cfg = cfg

        y = cfg["comfort_space"]["yield"]
        self.enabled = bool(y.get("enabled", True))
        self.lookahead = float(y.get("lookahead_seconds", 0.6))
        self.margin = float(y.get("margin", 0.05))

        self.provider = make_state_provider(cfg)
        self.policy = make_policy(cfg)

        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.yield_pub = self.create_publisher(Bool, "/comfort_yield", 10)
        self.reason_pub = self.create_publisher(String, "/comfort_yield_reason", 10)
        self.create_subscription(Twist, in_topic, self._on_cmd, 10)
        self.create_subscription(Odometry, cfg["ros"]["topics"]["odom"],
                                 self._on_odom, 10)

        # The Comfort Spaces, kept up to date by comfort_publisher.
        # Seeded from the config so the gate is correct from the first
        # message onward, and it never reads /tf itself: doing that from
        # this node's callback delayed the velocity command enough to
        # break navigation.
        states = self.provider.states()
        self.shapes = build_shapes(cfg, self.policy.axes(states),
                                   grow=robot_grow(cfg))
        self.create_subscription(Float32MultiArray, "/comfort_shapes",
                                 self._on_shapes, 10)
        self.pose = None
        self.yielding = False

        self.get_logger().info(
            f"policy={self.policy.name} enabled={self.enabled} "
            f"shapes={[(s.key, round(s.along, 2), round(s.across, 2)) for s in self.shapes]} "
            f"({in_topic} -> {out_topic})")

    def _on_shapes(self, msg: Float32MultiArray) -> None:
        data = list(msg.data)
        if len(data) % 5:
            return
        self.shapes = [
            ComfortShape(str(i // 5), data[i], data[i + 1],
                         data[i + 2], data[i + 3], data[i + 4])
            for i in range(0, len(data), 5)
        ]

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.pose = odom_to_world(
            self.cfg, (p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation)))

    def _intrusion(self, cmd: Twist):
        """Which human the commanded move would intrude on, if any.

        Returns (key, overlap_in_metres) or None. The check is on the
        predicted pose rather than the current one, so the robot stops at
        the boundary instead of a step past it.
        """
        if self.pose is None:
            return None

        x, y, yaw = self.pose
        self._last_seen = (x, y)
        dt = self.lookahead
        # Straight-line prediction is enough at these speeds and over
        # this horizon; arcs differ from it by millimetres.
        nx = x + cmd.linear.x * math.cos(yaw) * dt
        ny = y + cmd.linear.x * math.sin(yaw) * dt

        worst = None
        for shape in self.shapes:
            if not shape.contains(nx, ny, self.margin):
                continue
            # How far inside, measured along the line from the centre.
            # Only used to report which intrusion is worst, so the
            # straight-line distance is a good enough stand-in for the
            # true depth into an ellipse.
            depth = shape.bounding_radius + self.margin - math.hypot(
                nx - shape.x, ny - shape.y)
            if worst is None or depth > worst[1]:
                worst = (shape, max(depth, 0.0), (x, y), (nx, ny))
        return worst

    def _on_cmd(self, cmd: Twist) -> None:
        intrusion = None
        if self.enabled:
            intrusion = self._intrusion(cmd)

        if intrusion is None:
            if self.yielding:
                self.get_logger().info("clear of every Comfort Space, resuming")
            self.yielding = False
            self.pub.publish(cmd)
        else:
            shape, overlap, (x, y), (nx, ny) = intrusion
            if not self.yielding:
                self.get_logger().info(
                    f"yielding to human {shape.key} at "
                    f"({shape.x:+.2f}, {shape.y:+.2f}) "
                    f"[{shape.along:.2f} x {shape.across:.2f} m]: "
                    f"robot at ({x:+.2f}, {y:+.2f}) heading for "
                    f"({nx:+.2f}, {ny:+.2f}) would enter it by {overlap:.2f} m")
                reason = String()
                reason.data = shape.key
                self.reason_pub.publish(reason)
            self.yielding = True
            # Rotation is allowed while yielding. Turning in place does
            # not carry the robot into anyone, and forbidding it would
            # stop Nav2 from ever lining up with an alternative route.
            held = Twist()
            held.angular.z = cmd.angular.z
            self.pub.publish(held)

        flag = Bool()
        flag.data = self.yielding
        self.yield_pub.publish(flag)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/common.yaml")
    parser.add_argument("--scenario", default=None, help="b2, b3 or a path")
    parser.add_argument("--in-topic", default="/cmd_vel_nav")
    parser.add_argument("--out-topic", default="/cmd_vel")
    args, _ = parser.parse_known_args()

    cfg = load_config(scenario=args.scenario, common_path=args.config)

    rclpy.init()
    node = ComfortGate(cfg, args.in_topic, args.out_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
