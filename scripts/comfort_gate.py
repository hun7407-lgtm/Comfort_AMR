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

A person walking at the robot is the exception. Stopping in their lane
means waiting to be walked into, so for them the gate steps the robot
aside instead, on a timer of its own rather than on Nav2's commands, and
publishes /comfort_evading while it does. See comfort_core/evasion.py.
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
from nav2_msgs.msg import CollisionMonitorState
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String

from comfort_core import load_config, odom_to_world
from comfort_core.comfort_space import make_policy, robot_grow, shapes as build_shapes
from comfort_core.evasion import EvasionController, Person
from comfort_core.clearance import footprint_half_extents
from comfort_core.intrusion import footprint_outline
from comfort_core.human_state import make_state_provider
from comfort_core.layout import plannable_area, inside_drivable
from comfort_core.shapes import ComfortShape


def yaw_from_quaternion(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _nearest_drivable_point(rects, xy):
    """The closest point inside any drivable rectangle, inset a little.

    Inset, because the nearest point ON the boundary is still a pose
    whose footprint hangs over it. 0.30 m is a little more than the
    robot's half width.
    """
    x, y = xy
    best = None
    for x0, x1, y0, y1 in rects:
        cx = min(max(x, x0 + 0.30), x1 - 0.30)
        cy = min(max(y, y0 + 0.30), y1 - 0.30)
        if x0 + 0.30 > x1 - 0.30 or y0 + 0.30 > y1 - 0.30:
            continue
        gap = math.hypot(cx - x, cy - y)
        if best is None or gap < best[0]:
            best = (gap, (cx, cy))
    return None if best is None else best[1]


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

        # Backing off when the safety layer has the wheels.
        #
        # The collision monitor sits downstream of this gate and zeroes
        # the command it does not like. Nav2 never learns: the
        # controller reports success, no recovery runs, and a robot
        # with somebody standing 0.6 m in front of it waits for the
        # rest of the trial. See recovery.backoff in the config for the
        # measurement that produced this.
        rec = cfg.get("recovery", {}).get("backoff", {})
        self.backoff_enabled = bool(rec.get("enabled", True))
        self.backoff_after = float(rec.get("after_seconds", 5.0))
        self.backoff_speed = abs(float(rec.get("speed", 0.12)))
        self.backoff_distance = float(rec.get("distance", 0.40))
        self.backoff_attempts_left = int(rec.get("max_attempts", 2))
        self.backing_off = False
        self._backoff_from: tuple[float, float] | None = None
        self._backoff_ticks = 0
        self._held_since: float | None = None
        self._wanted_forward = 0.0
        self._measured_speed = 0.0
        if self.backoff_enabled:
            self.create_subscription(CollisionMonitorState,
                                     "/collision_monitor_state",
                                     self._on_safety, 10)
            self.create_timer(0.1, self._backoff_tick)

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

        # Stepping aside for walkers.
        self.evade_enabled = bool(y.get("evade", {}).get("enabled", False))
        self.evader = EvasionController(cfg, log=lambda m: self.get_logger().info(m))
        self.grow = robot_grow(cfg)
        self.people: list[Person] = []
        self.sim_now = 0.0
        self.evade_pub = self.create_publisher(Bool, "/comfort_evading", 10)
        # One word for what the gate is doing, for the video overlay and
        # for anyone reading a log: DRIVING, YIELDING, STEPPING ASIDE,
        # WAITING, RESUMING.
        self.state_pub = self.create_publisher(String, "/comfort_gate_state", 10)
        self.hold_still = bool(y.get("hold_still", True))

        # Getting back onto the painted floor. Independent of the
        # Comfort Space policy -- it is a fault recovery, not a courtesy
        # -- so it runs in every scenario, B1 included.
        # The recovery's boundary is the MAP's, not the yield logic's:
        # see comfort_core.layout.plannable_area.
        self.drivable = plannable_area(cfg)
        self.half = footprint_half_extents(cfg)
        self.recovering = False
        if self.drivable:
            self.create_timer(0.1, self._recover_tick)

        # A heartbeat on /comfort_gate_state.
        #
        # The gate used to speak only when spoken to: it published its
        # state when a velocity command arrived or the sidestep acted,
        # and said nothing at all before the robot was given a goal.
        # Anything watching for it -- the trial's own "is the gate
        # running" check, the video overlay -- then could not tell a
        # quiet gate from a dead one, and nine trials were thrown away
        # on that mistake. A node that is asked whether it is alive has
        # to answer.
        self.create_timer(0.2, self._heartbeat)

        if self.enabled and self.evade_enabled:
            # Independent of Nav2's command stream. If Nav2 went quiet
            # during a recovery the robot would otherwise freeze in the
            # middle of the manoeuvre, which is the one place it must not.
            self.create_timer(0.05, self._evade_tick)

        self.get_logger().info(
            f"policy={self.policy.name} enabled={self.enabled} "
            f"shapes={[(s.key, round(s.along, 2), round(s.across, 2)) for s in self.shapes]} "
            f"({in_topic} -> {out_topic})")

    def _on_shapes(self, msg: Float32MultiArray) -> None:
        # x, y, along, across, heading, speed per person.
        data = list(msg.data)
        n = 6
        if len(data) % n:
            return
        self.shapes = [
            ComfortShape(str(i // n), data[i], data[i + 1],
                         data[i + 2], data[i + 3], data[i + 4])
            for i in range(0, len(data), n)
        ]
        self.people = [
            Person(str(i // n), data[i], data[i + 1], data[i + 4], data[i + 5])
            for i in range(0, len(data), n)
        ]

    def _on_odom(self, msg: Odometry) -> None:
        self.sim_now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        v = msg.twist.twist.linear
        self._measured_speed = math.hypot(v.x, v.y)
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

    def _heartbeat(self) -> None:
        """Say what the gate is doing, whether or not anything changed."""
        label = String()
        if self.backing_off:
            label.data = "BACKING OFF"
        elif self.recovering:
            label.data = "RECOVERING"
        elif self.evader.active:
            label.data = self.evader.state or "STEPPING ASIDE"
        else:
            label.data = "YIELDING" if self.yielding else "DRIVING"
        self.state_pub.publish(label)

    def _on_safety(self, msg: CollisionMonitorState) -> None:
        """How long the safety layer has been stopping the robot."""
        stopping = msg.action_type == CollisionMonitorState.STOP
        if self.backing_off:
            # The monitor is judging the reverse command now, and it
            # likes it -- that is why the robot is moving. Reading that
            # as "the way ahead is clear" cancelled the manoeuvre one
            # tick after it started, every time: "backed off 0.00 m;
            # the way ahead is clear again".
            return
        if stopping and self._wanted_forward > 0.05:
            if self._held_since is None:
                self._held_since = self.sim_now
        else:
            self._held_since = None

    def _backoff_tick(self) -> None:
        """Give up on going forward and take half a step back.

        Only once the safety layer has held the robot still for several
        seconds while Nav2 was asking to go forward. Somebody crossing
        the robot's path is waited for; somebody who has stopped in
        front of it is backed away from, so that the planner -- which
        can see them in the costmap -- has the room to route round.

        It owns the wheels while it runs, and it refuses to move into
        anybody's Comfort Space behind the robot, which the monitor's
        own reverse field would not catch: that field is about not
        hitting people, and this is about not crowding them.
        """
        if not self.backoff_enabled or self.pose is None:
            return

        if self.backing_off:
            x, y, _yaw = self.pose
            gone = math.hypot(x - self._backoff_from[0], y - self._backoff_from[1])
            # Give up on the manoeuvre if the wheels are not turning --
            # something behind is in the monitor's reverse field, and
            # creeping into it for ever is not a recovery.
            self._backoff_ticks += 1
            stalled = (self._backoff_ticks
                       > 3.0 * self.backoff_distance / self.backoff_speed / 0.1)
            if gone >= self.backoff_distance or stalled:
                self.backing_off = False
                self._held_since = None
                self.get_logger().info(
                    f"backed off {gone:.2f} m"
                    + ("; it would not move, so stopping there"
                       if stalled else
                       "; letting Nav2 look for another route"))
                self.pub.publish(Twist())
                return
            reverse = Twist()
            reverse.linear.x = -self.backoff_speed
            if self.enabled and self._intrusion(reverse) is not None:
                self.backing_off = False
                self.get_logger().info(
                    "stopped backing off: somebody's Comfort Space is behind us")
                self.pub.publish(Twist())
                return
            self.pub.publish(reverse)
            return

        if (self._held_since is None or self.evader.active or self.recovering
                or self.yielding or self.backoff_attempts_left <= 0):
            return
        if self.sim_now - self._held_since < self.backoff_after:
            return
        if self._measured_speed > 0.05:
            return
        self.backing_off = True
        self._backoff_ticks = 0
        self.backoff_attempts_left -= 1
        self._backoff_from = (self.pose[0], self.pose[1])
        self.get_logger().warn(
            f"the safety layer has held us for "
            f"{self.sim_now - self._held_since:.1f} s at "
            f"({self.pose[0]:+.2f}, {self.pose[1]:+.2f}) while Nav2 asked to "
            f"drive on; backing off {self.backoff_distance:.2f} m "
            f"({self.backoff_attempts_left} attempt(s) left after this)")

    def _recover_tick(self) -> None:
        """Drive back onto the painted floor if the robot has left it.

        Nav2 will not plan from a pose whose footprint touches a solid
        cell, and in a lanes-only map everything outside the paint is
        solid. So a robot that ends up half a metre off the lane -- a
        yield that overshot, a corner cut, a shove from a person -- is
        not just off course, it is permanently stuck: the planner
        aborts, the trial reports a deadlock with no cause, and nothing
        in the stack tries to put it back.

        Measured over three stress waves: trials parked at (7.88,
        -3.08), (-9.50, -1.52), (-9.34, -2.70), all a couple of
        hand-widths outside an aisle, all reporting nav2 ABORTED.

        This is a last resort and it owns the wheels while it runs. It
        drives at creep speed toward the nearest point of the nearest
        drivable rectangle, and stops as soon as the whole footprint is
        back inside.
        """
        if self.pose is None or self.drivable is None or self.evader.active:
            return
        x, y, yaw = self.pose
        outline = footprint_outline(self.pose, *self.half)
        if inside_drivable(self.drivable, outline):
            if self.recovering:
                self.get_logger().info(
                    f"back on the painted floor at ({x:+.2f}, {y:+.2f})")
            self.recovering = False
            return

        target = _nearest_drivable_point(self.drivable, (x, y))
        if target is None:
            return
        if not self.recovering:
            self.recovering = True
            self.get_logger().warn(
                f"off the painted floor at ({x:+.2f}, {y:+.2f}); "
                f"backing onto it at ({target[0]:+.2f}, {target[1]:+.2f})")
        dx, dy = target[0] - x, target[1] - y
        want = math.atan2(dy, dx)
        error = math.atan2(math.sin(want - yaw), math.cos(want - yaw))
        cmd = Twist()
        if abs(error) > 0.25:
            # Turn first. Reversing blind out of a machine zone is how a
            # recovery becomes a collision.
            cmd.angular.z = max(-0.6, min(0.6, 1.5 * error))
        else:
            cmd.linear.x = 0.15
            cmd.angular.z = max(-0.4, min(0.4, 1.0 * error))
        self.pub.publish(cmd)
        self._publish_flags(evading=False, state="RECOVERING")

    def _evade_tick(self) -> None:
        """Let the walker controller decide; publish its command if it has one.

        The decisions live in comfort_core.evasion.EvasionController so
        that the offline stress test runs exactly this logic.
        """
        if self.pose is None:
            return
        # The controller wants each person's own Comfort Space, not the
        # copy enlarged by the robot's half width for the planner.
        grow = self.grow
        raw = {sh.key: ComfortShape(sh.key, sh.x, sh.y, max(sh.along - grow, 0.0),
                                    max(sh.across - grow, 0.0), sh.heading)
               for sh in self.shapes}
        was_active = self.evader.active
        out = self.evader.step(self.sim_now, self.pose, self.people, raw)
        if out is None:
            if was_active:
                self._publish_flags(evading=False, state="DRIVING")
            return
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = out
        self.pub.publish(cmd)
        self.yielding = True
        self._publish_flags(evading=True, state=self.evader.state)

    def _publish_flags(self, evading: bool, state: str | None = None) -> None:
        flag = Bool()
        flag.data = evading
        self.evade_pub.publish(flag)
        flag = Bool()
        flag.data = self.yielding
        self.yield_pub.publish(flag)
        label = String()
        label.data = state or ("YIELDING" if self.yielding else "DRIVING")
        self.state_pub.publish(label)

    def _on_cmd(self, cmd: Twist) -> None:
        self._wanted_forward = float(cmd.linear.x)
        if self.evader.active or self.recovering or self.backing_off:
            # The sidestep owns the wheels until the walker has passed,
            # and getting back on the lane owns them until it is done.
            return
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
            # Stand still by default. Rotation used to be passed through so
            # Nav2 could line up with an alternative route, but a robot
            # turning on the spot does not look like a robot yielding.
            held = Twist()
            if not self.hold_still:
                held.angular.z = cmd.angular.z
            self.pub.publish(held)

        self._publish_flags(evading=False,
                            state="YIELDING" if self.yielding else "DRIVING")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/common.yaml")
    parser.add_argument("--scenario", default=None, help="b1, b2, b3 or a path")
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
