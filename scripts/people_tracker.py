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

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformListener

from comfort_core import load_config
from comfort_core.layout import obstacle_boxes
from comfort_core.layout import human_positions, scene_bounds
from comfort_core.tracking import PeopleTracker, TrackingParams

WORLD_FRAME = "world"
MAP_FRAME = "map"


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _clusters(X, Y, gap: float, min_points: int):
    """Indices of each group of points within `gap` of one another.

    Single linkage on the flat positions, the same as the real robot's
    detector uses. Small enough sets that the simple version is fine:
    one sweep of this LiDAR is about 550 points and the height band and
    the structure filter take most of them out first.
    """
    n = len(X)
    labels = -np.ones(n, dtype=int)
    group = 0
    for i in range(n):
        if labels[i] >= 0:
            continue
        stack = [i]
        labels[i] = group
        while stack:
            j = stack.pop()
            near = np.where((np.hypot(X - X[j], Y - Y[j]) < gap)
                            & (labels < 0))[0]
            labels[near] = group
            stack.extend(near.tolist())
        group += 1
    for g in range(group):
        idx = np.where(labels == g)[0]
        if len(idx) >= min_points:
            yield idx


class StructureFilter:
    """Rejects clusters that sit on the walls or on a workstation.

    The robot is given a static map of this room, so structure in it is
    not a detection. Everything here comes from the same config the map
    is generated from, so the filter cannot drift away from the map.
    """

    def __init__(self, cfg, margin: float = 0.25):
        self.bounds = scene_bounds(cfg)
        self.boxes = obstacle_boxes(cfg)
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

        # Which sensor stream the detector works from.
        #
        #   cloud  the 3D point cloud, with the height of every return.
        #          A person is told from the furniture by their vertical
        #          profile, which is how the real robot does it.
        #   scan   the flattened LaserScan. Keeps the earlier behaviour,
        #          and cannot tell a person from a pillar: flattening
        #          throws away the one cue that distinguishes them.
        det = cfg.get("perception", {}).get("detector", {})
        self.source = str(det.get("source", "cloud"))
        self.sensor_z = float(cfg["ros"]["scan"].get("lidar_world_z", 0.31))
        self.z_min = float(det.get("z_min", 0.20))
        self.z_max = float(det.get("z_max", 2.00))
        self.min_top = float(det.get("min_top", 0.90))
        self.min_extent = float(det.get("min_extent", 0.50))
        self.min_spread = float(det.get("min_spread", 0.20))
        self.max_spread = float(det.get("max_spread", 0.90))
        self.min_points = int(det.get("min_points", 8))
        self.gap = float(det.get("gap", 0.35))
        self.window = float(det.get("window_seconds", 0.3))
        self.body_offset = (2.0 / math.pi) * float(
            cfg.get("humans", {}).get("radius", 0.25))
        from collections import deque
        self._recent = deque()

        if self.source == "cloud":
            self.create_subscription(PointCloud2,
                                     cfg["ros"]["topics"]["lidar_cloud"],
                                     self._on_cloud, qos_profile_sensor_data)
        else:
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
        # [found, present] per two metres of range, out to 14 m+.
        self.by_range = [[0, 0] for _ in range(8)]
        self._last_sensor_xy: tuple[float, float] | None = None
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

    def _on_cloud(self, msg: PointCloud2) -> None:
        """People out of the 3D cloud, by their vertical profile.

        This is the real robot's method, ported: see
        results/real/amr_deploy/bin/detect_person.py, which was tuned on
        a recording of the actual corridor. A person is a column that

            reaches past `min_top` above the floor,
            has at least `min_extent` of vertical extent,
            and is between `min_spread` and `max_spread` wide.

        The width bounds are what separate a person from the furniture:
        the lower one rejects slivers of wall, the upper one rejects
        benches and loaded carts, which are wide and stop low. None of
        that survives being flattened into a LaserScan, which is why
        this reads the cloud and /scan is left to the costmap.
        """
        pose = self._sensor_pose(msg.header.frame_id, msg.header.stamp)
        if pose is None:
            return
        sx, sy, syaw = pose
        self._last_sensor_xy = (sx, sy)
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        raw = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pts = np.asarray([[p[0], p[1], p[2]] for p in raw], dtype=float)
        if len(pts) == 0:
            return
        c, s_ = math.cos(syaw), math.sin(syaw)
        X = sx + pts[:, 0] * c - pts[:, 1] * s_
        Y = sy + pts[:, 0] * s_ + pts[:, 1] * c
        Z = self.sensor_z + pts[:, 2]

        # Several sweeps together, in world coordinates.
        #
        # One sweep of this sensor puts five to seven returns on a
        # person at 4 m, which is not enough to cluster: measured, with
        # a per-message minimum of eight points the detector found
        # nobody at all in 503 sweeps. The real robot's detector stacks
        # several seconds of cloud before clustering for the same
        # reason.
        #
        # The window is short on purpose. Stacking in world coordinates
        # means a walker smears along their path, so a second of it
        # would stretch a person at 1 m/s into a metre-long sausage and
        # the width test would throw them out. 0.3 s costs 0.3 m of
        # smear and buys three times the points.
        self._recent.append((now, X, Y, Z))
        while self._recent and now - self._recent[0][0] > self.window:
            self._recent.popleft()
        X = np.concatenate([r[1] for r in self._recent])
        Y = np.concatenate([r[2] for r in self._recent])
        Z = np.concatenate([r[3] for r in self._recent])

        band = (Z >= self.z_min) & (Z <= self.z_max)
        X, Y, Z = X[band], Y[band], Z[band]
        keep = np.array([not self.structure.is_structure(x, y)
                         for x, y in zip(X, Y)], dtype=bool) if len(X) else band
        X, Y, Z = X[keep], Y[keep], Z[keep]

        centres = []
        shapes = []
        for idx in _clusters(X, Y, self.gap, self.min_points):
            cz, cx, cy = Z[idx], X[idx], Y[idx]
            top = float(cz.max())
            extent = top - float(cz.min())
            spread = float(max(cx.max() - cx.min(), cy.max() - cy.min()))
            shapes.append((len(idx), top, extent, spread))
            if (top >= self.min_top and extent >= self.min_extent
                    and self.min_spread <= spread <= self.max_spread):
                centres.append(self._centre(float(cx.mean()), float(cy.mean()),
                                            sx, sy))

        # What the filters did, every couple of seconds. Counting them
        # at every stage is the only way to see which one is eating the
        # people: the first two ports of this found nobody at all, and
        # the reason was a different stage each time.
        self._stage = (len(pts), int(band.sum()), len(X), len(shapes),
                       len(centres))
        if self.evaluate and shapes and self.scans % 40 == 0:
            worst = sorted(shapes, key=lambda r: -r[1])[:3]
            self.get_logger().info(
                "cloud " + " -> ".join(str(v) for v in self._stage)
                + "  (raw, in band, off structure, clusters, person-shaped); "
                + "tallest: " + ", ".join(
                    f"{n}pt top {t:.2f} ext {e:.2f} wide {w:.2f}"
                    for n, t, e, w in worst))

        tracks = [t for t in self.tracker.update_detections(centres, now)
                  if not self.structure.is_structure(t.x, t.y)]
        flat = []
        for t in tracks:
            vx, vy = t.confirmed_speed
            flat.extend([t.x, t.y, vx, vy, float(t.hits)])
        out = Float32MultiArray()
        out.data = flat
        self.pub.publish(out)

        self.scans += 1
        if self.evaluate:
            self._score(tracks, now)

    def _centre(self, mx: float, my: float, sx: float, sy: float):
        """From the middle of the returns to the middle of the person.

        The sensor only ever sees somebody's front, so the mean of the
        returns sits in front of them, not inside them. For an upright
        cylinder of radius r seen from a few metres away the visible
        points run over half a turn and their mean lies 2r/pi in front
        of the centre, which is 0.16 m for the 0.25 m body this study
        uses. Uncorrected it was measured at 0.20 m, consistently and
        in the same direction for everybody.

        It matters because every Comfort Space distance in the results
        is measured from this position: leaving the bias in makes the
        robot treat people as 0.2 m nearer than they are, which looks
        like caution and is really a measurement error.
        """
        gap = math.hypot(mx - sx, my - sy)
        if gap < 1e-6:
            return (mx, my)
        return (mx + self.body_offset * (mx - sx) / gap,
                my + self.body_offset * (my - sy) / gap)

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
        sensor = self._last_sensor_xy
        for key, (tx, ty) in truth.items():
            # Recall against range, because the plain count cannot be
            # read. Twelve people stand around this bay and the sensor
            # can see one to five of them: the rest are behind a
            # machine, behind each other, or beyond the range where
            # this LiDAR puts any returns on a body at floor level. A
            # raw miss count therefore reads as a 17% hit rate for a
            # detector that misses nobody it can see. What the robot
            # needs to be judged on is how reliably it finds people
            # close enough to matter.
            if sensor is not None:
                gap = math.hypot(tx - sensor[0], ty - sensor[1])
                bucket = min(int(gap / 2.0), 7)
                seen = any(math.hypot(t.x - tx, t.y - ty) < 1.0 for t in tracks)
                self.by_range[bucket][1] += 1
                if seen:
                    self.by_range[bucket][0] += 1
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

        if any(present for _found, present in self.by_range):
            print("  found, by how far away they were:", flush=True)
            for i, (found, present) in enumerate(self.by_range):
                if not present:
                    continue
                band = f"{i * 2}-{i * 2 + 2} m" if i < 7 else "14 m+"
                bar = "#" * int(round(20 * found / present))
                print(f"    {band:>8s} {100.0 * found / present:5.1f}%  "
                      f"({found} of {present})  {bar}", flush=True)


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
