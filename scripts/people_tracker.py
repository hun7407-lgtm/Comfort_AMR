#!/usr/bin/env python3
"""Estimate where people are and how fast they walk, from /scan.

Runs in the SYSTEM Python with ROS 2 Jazzy. No omni, no pxr.

The B2/B3 comparison reads people straight out of the scene, on purpose:
detection error would be a confound (see comfort_core/human_state.py).
This node estimates the same thing from the LiDAR instead, so the two can
be put side by side, and so a Comfort Space could one day be sized from
what the robot sees.

    /scan --> world points --> clusters --> tracks --> /comfort_detections

Walls and workstations are known structure, not people, so clusters
sitting on them are dropped using the same geometry the map is built
from. What is left is person sized and moves.

    ./scripts/run_people_tracker.sh --evaluate

`--evaluate` also reads the true positions from /tf and reports the error,
which is the only way to know whether the estimate is worth anything.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformListener

from comfort_core import load_config
from comfort_core.evasion import station_boxes
from comfort_core.layout import corridor_bounds, human_positions
from comfort_core.tracking import PeopleTracker, TrackingParams

WORLD_FRAME = "world"
MAP_FRAME = "map"


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class StructureFilter:
    """Rejects clusters that sit on the walls or on a workstation.

    The robot is given a static map of this room, so structure in it is
    not a detection. Everything here comes from the same config the map
    is generated from, so the filter cannot drift away from the map.
    """

    def __init__(self, cfg, margin: float = 0.25):
        self.bounds = corridor_bounds(cfg)
        self.boxes = station_boxes(cfg)
        self.margin = margin

    def is_structure(self, x: float, y: float) -> bool:
        if (x < self.bounds.x_min + self.margin or x > self.bounds.x_max - self.margin
                or y < self.bounds.y_min + self.margin or y > self.bounds.y_max - self.margin):
            return True
        return any(x0 - self.margin <= x <= x1 + self.margin
                   and y0 - self.margin <= y <= y1 + self.margin
                   for x0, x1, y0, y1 in self.boxes)


class PeopleTrackerNode(Node):
    def __init__(self, cfg, evaluate: bool):
        super().__init__("people_tracker")
        self.cfg = cfg
        self.evaluate = evaluate
        self.tracker = PeopleTracker(TrackingParams.from_config(cfg))
        self.structure = StructureFilter(
            cfg, float(cfg.get("perception", {}).get("tracking", {}).get("structure_margin", 0.25)))

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(Float32MultiArray, "/comfort_detections", 10)
        self.create_subscription(LaserScan, cfg["ros"]["topics"]["scan"],
                                 self._on_scan, qos_profile_sensor_data)

        self.human_keys = [k for k, _x, _y in human_positions(cfg)]
        self.scans = 0
        self.errors: list[float] = []
        self.speed_errors: list[float] = []
        self.moving_speed_errors: list[float] = []
        # True positions over time, to differentiate the same way the
        # estimate is differentiated. Comparing a windowed estimate
        # against an instantaneous truth would charge the estimate for
        # the window.
        self._truth_history: dict[str, list[tuple[float, float, float]]] = {}
        self._window = float(cfg.get("perception", {}).get("tracking", {})
                             .get("velocity_window", 1.0))
        self._moving_threshold = float(
            cfg["comfort_space"].get("moving_speed_threshold", 0.15))
        self.matched = 0
        self.missed = 0
        self.spurious = 0
        self.get_logger().info(
            f"tracking people from {cfg['ros']['topics']['scan']}"
            + (" (evaluating against /tf)" if evaluate else ""))

    # ------------------------------------------------------------------
    def _sensor_pose(self, frame: str, stamp):
        for when in (rclpy.time.Time.from_msg(stamp), rclpy.time.Time()):
            try:
                t = self.tf.lookup_transform(MAP_FRAME, frame, when,
                                             timeout=Duration(seconds=0.0))
            except Exception:
                continue
            tr = t.transform
            return tr.translation.x, tr.translation.y, yaw_from_quaternion(tr.rotation)
        return None

    def _truth(self):
        """True (x, y) of each person, from the frames Isaac Sim publishes."""
        out = {}
        for key in self.human_keys:
            try:
                t = self.tf.lookup_transform(WORLD_FRAME, f"human_{key}",
                                             rclpy.time.Time(), timeout=Duration(seconds=0.0))
            except Exception:
                continue
            out[key] = (t.transform.translation.x, t.transform.translation.y)
        return out

    def _on_scan(self, msg: LaserScan) -> None:
        pose = self._sensor_pose(msg.header.frame_id, msg.header.stamp)
        if pose is None:
            return
        sx, sy, syaw = pose
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        points = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= msg.range_min or r >= msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment + syaw
            points.append((sx + r * math.cos(a), sy + r * math.sin(a)))

        tracks = [t for t in self.tracker.update(points, now)
                  if not self.structure.is_structure(t.x, t.y)]

        flat = []
        for t in tracks:
            vx, vy = t.confirmed_speed
            flat.extend([t.x, t.y, vx, vy, float(t.hits)])
        msg_out = Float32MultiArray()
        msg_out.data = flat
        self.pub.publish(msg_out)

        self.scans += 1
        if self.evaluate:
            self._score(tracks, now)

    def _truth_speed(self, key: str, x: float, y: float, now: float) -> float:
        history = self._truth_history.setdefault(key, [])
        history.append((now, x, y))
        while len(history) > 2 and now - history[1][0] >= self._window:
            history.pop(0)
        t0, x0, y0 = history[0]
        if now - t0 < 1e-3:
            return 0.0
        return math.hypot(x - x0, y - y0) / (now - t0)

    def _score(self, tracks, now: float) -> None:
        truth = self._truth()
        taken = set()
        for key, (tx, ty) in truth.items():
            true_speed = self._truth_speed(key, tx, ty, now)
            best, best_d = None, 1.0
            for t in tracks:
                d = math.hypot(t.x - tx, t.y - ty)
                if d < best_d:
                    best, best_d = t, d
            if best is None:
                self.missed += 1
                continue
            taken.add(id(best))
            self.matched += 1
            self.errors.append(best_d)
            error = abs(best.speed - true_speed)
            self.speed_errors.append(error)
            if true_speed >= self._moving_threshold:
                self.moving_speed_errors.append(error)
        self.spurious += sum(1 for t in tracks if id(t) not in taken)

        if self.scans % 20 == 0:
            lines = []
            for t in tracks:
                vx, vy = t.confirmed_speed
                near = min(((math.hypot(t.x - x, t.y - y), k) for k, (x, y) in truth.items()),
                           default=(math.inf, "?"))
                lines.append(f"({t.x:+.2f},{t.y:+.2f}) {math.hypot(vx, vy):.2f} m/s "
                             f"[{near[1]} {near[0]:.2f} m off]")
            self.get_logger().info(f"{len(tracks)} people: " + "  ".join(lines))

    def report(self) -> None:
        if not self.evaluate:
            return
        n = len(self.errors)
        print("\n=== People tracking vs ground truth ===", flush=True)
        print(f"  scans              : {self.scans}", flush=True)
        print(f"  matched detections : {self.matched}", flush=True)
        print(f"  people missed      : {self.missed}", flush=True)
        print(f"  spurious tracks    : {self.spurious}", flush=True)
        if n:
            ordered = sorted(self.errors)
            print(f"  position error     : median {ordered[n // 2]:.2f} m, "
                  f"90th pct {ordered[int(n * 0.9)]:.2f} m, worst {ordered[-1]:.2f} m", flush=True)
        for label, values in (("speed error, all  ", self.speed_errors),
                              ("speed error, walking", self.moving_speed_errors)):
            if not values:
                continue
            ordered = sorted(values)
            m = len(ordered)
            print(f"  {label} : median {ordered[m // 2]:.2f} m/s, "
                  f"90th pct {ordered[int(m * 0.9)]:.2f} m/s, "
                  f"worst {ordered[-1]:.2f} m/s", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/common.yaml")
    parser.add_argument("--scenario", default=None, help="b1, b2, b3 or a path")
    parser.add_argument("--evaluate", action="store_true",
                        help="compare the estimate against the true positions on /tf")
    args, _ = parser.parse_known_args()

    cfg = load_config(scenario=args.scenario, common_path=args.config)
    rclpy.init()
    node = PeopleTrackerNode(cfg, args.evaluate)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.report()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
