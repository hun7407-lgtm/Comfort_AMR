#!/usr/bin/env python3
"""Record what Nav2 sees: costmaps, LiDAR returns, plan and Comfort Spaces.

Runs in the SYSTEM Python with ROS 2 Jazzy. No omni, no pxr.

The camera video shows the scene. This shows the robot's side of it,
drawn from the same topics Nav2 plans on:

    top     global costmap, whole corridor, with the planned path
    bottom  local costmap around the robot, 6 m x 6 m

Over both, the LiDAR returns of the latest /scan, the footprint, and the
outline of each Comfort Space as the planner has to avoid it (already
enlarged by the robot's half width).

Frames are taken on the simulation clock, like isaac_sim/utils/recorder.py,
so the result plays at simulated real time. Nav2 publishes its costmaps
at 2 Hz (costmap publish_frequency), so the grids step at that rate while
the scan and the pose move every frame.

    ./scripts/run_scenario.sh b3 --record-costmap
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String
from tf2_ros import Buffer, TransformListener

from comfort_core import load_config
from comfort_core.clearance import footprint_half_extents
from comfort_core.palette import fleet_rgb255, mission_rgb255

WIDTH = 1280
BG = (30, 30, 30)

# BGR, since everything is drawn with OpenCV.
COLOR_SCAN = (60, 60, 255)
COLOR_FOOTPRINT = (80, 220, 80)
COLOR_PLAN = (255, 80, 255)
COLOR_COMFORT = (255, 170, 40)
COLOR_TEXT = (240, 240, 240)
COLOR_MISSION = mission_rgb255()  # AMR1's start and goal
# Fleet AMRs take their colour from comfort_core/palette.py, the same
# table that paints the stripe on them in the simulator.

LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
COSTMAP_QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def cost_image(grid: OccupancyGrid) -> np.ndarray:
    """OccupancyGrid values to a BGR image, row 0 at the bottom of the map.

    Published costmap values: 0 free, 1..98 scaled cost, 99 inscribed,
    100 lethal, -1 unknown. Low costs are shaded light, high costs dark
    red, so the Comfort Space (a finite 98) and a wall (lethal) stay
    distinguishable.
    """
    w, h = grid.info.width, grid.info.height
    data = np.asarray(grid.data, dtype=np.int16).reshape(h, w)
    img = np.empty((h, w, 3), dtype=np.uint8)
    img[:] = (235, 235, 235)                       # free
    cost = np.clip(data, 1, 98).astype(np.float32) / 98.0
    ramp = np.stack([(1.0 - cost) * 200 + 20,       # B
                     (1.0 - cost) * 210 + 20,       # G
                     (1.0 - 0.35 * cost) * 255],    # R
                    axis=-1).astype(np.uint8)
    mid = (data >= 1) & (data <= 98)
    img[mid] = ramp[mid]
    img[data == 99] = (40, 40, 150)                 # inscribed
    img[data == 100] = (20, 20, 20)                 # lethal
    img[data < 0] = (120, 120, 120)                 # unknown
    return img[::-1]                                # flip so +y is up


def _destinations(cfg):
    """Every point an AMR in this scenario is sent to, named.

    AMR1 has one start and one goal. A fleet AMR has
    neither: it works a job list and comes back, so the honest label is
    the leg number and the slot name. Without this the video shows
    vehicles stopping at places the viewer cannot identify.

    Returns (x, y, text, colour, filled).
    """
    raw = []
    nav = cfg.get("navigation", {})
    if nav.get("start") and nav.get("goal"):
        sx, sy = nav["start"][:2]
        gx, gy = nav["goal"][:2]
        raw.append((sx, sy, "AMR1 START", COLOR_MISSION, False))
        raw.append((gx, gy, "AMR1 GOAL", COLOR_MISSION, True))

    fleet = cfg.get("fleet", {})
    places = fleet.get("places", {})
    for name, spec in sorted(fleet.get("amrs", {}).items()):
        if not spec.get("enabled", False):
            continue
        home = places.get(str(spec.get("start", "")))
        if home is not None:
            raw.append((float(home[0]), float(home[1]),
                        f"{name.upper()} home", fleet_rgb255(name), False))
        for order, job in enumerate(spec.get("jobs", []), start=1):
            point = places.get(str(job))
            if point is None:
                continue
            raw.append((float(point[0]), float(point[1]),
                        f"{name.upper()}-{order} {job}", fleet_rgb255(name), False))

    # A job list visits the same slot twice and ends where it began, so
    # several labels land on one point. Printed separately they overlap
    # into something unreadable; joined, they say what the route does.
    merged = {}
    for x, y, text, colour, filled in raw:
        key = (round(x, 2), round(y, 2))
        if key in merged:
            prev = merged[key]
            merged[key] = (prev[0], prev[1], f"{prev[2]} / {text}",
                           prev[3], prev[4] or filled)
        else:
            merged[key] = (x, y, text, colour, filled)
    return list(merged.values())


class Panel:
    """A grid drawn at some pixel scale, with world->pixel mapping."""

    def __init__(self, grid: OccupancyGrid, origin_xy_yaw, scale: float):
        self.grid = grid
        self.scale = scale
        self.ox, self.oy, self.oyaw = origin_xy_yaw
        self.h = grid.info.height
        self.res = grid.info.resolution
        base = cost_image(grid)
        size = (int(base.shape[1] * scale), int(base.shape[0] * scale))
        self.image = cv2.resize(base, size, interpolation=cv2.INTER_NEAREST)

    def px(self, x: float, y: float):
        """World (map frame) point to integer pixel."""
        c, s = math.cos(-self.oyaw), math.sin(-self.oyaw)
        dx, dy = x - self.ox, y - self.oy
        gx = (dx * c - dy * s) / self.res
        gy = (dx * s + dy * c) / self.res
        return int(round(gx * self.scale)), int(round((self.h - gy) * self.scale))


class CostmapRecorder(Node):
    def __init__(self, cfg, out_dir: Path, label: str, fps: float, start: float):
        super().__init__("costmap_recorder")
        self.cfg = cfg
        self.out_dir = out_dir
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.label = label
        self.interval = 1.0 / fps
        self.start = start
        self.next_time = start
        self.count = 0
        self.half_length, self.half_width = footprint_half_extents(cfg)
        self.marks = _destinations(cfg)
        fleet = cfg.get("fleet", {})
        self.fleet_names = [name for name, spec in sorted(
            fleet.get("amrs", {}).items())
            if fleet.get("enabled", False) and spec.get("enabled", True)]

        self.global_grid = None
        self.local_grid = None
        self.scan = None
        self.plan = None
        self.shapes = []
        self.evading = False
        self.yielding = False
        self.state = ""

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

        self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                 lambda m: setattr(self, "global_grid", m), COSTMAP_QOS)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap",
                                 lambda m: setattr(self, "local_grid", m), COSTMAP_QOS)
        self.create_subscription(LaserScan, cfg["ros"]["topics"]["scan"],
                                 lambda m: setattr(self, "scan", m), qos_profile_sensor_data)
        self.create_subscription(PathMsg, "/plan",
                                 lambda m: setattr(self, "plan", m), 10)
        self.create_subscription(Float32MultiArray, "/comfort_shapes",
                                 self._on_shapes, 10)
        self.create_subscription(Bool, "/comfort_evading",
                                 lambda m: setattr(self, "evading", bool(m.data)), 10)
        self.create_subscription(Bool, "/comfort_yield",
                                 lambda m: setattr(self, "yielding", bool(m.data)), 10)
        self.create_subscription(String, "/comfort_gate_state",
                                 lambda m: setattr(self, "state", m.data), 10)
        self.create_subscription(Odometry, cfg["ros"]["topics"]["odom"],
                                 self._on_odom, 10)
        self.get_logger().info(f"recording costmap frames to {out_dir} from "
                               f"{start:.0f} simulated s at {fps:g} fps")

    def _on_shapes(self, msg: Float32MultiArray) -> None:
        d = list(msg.data)
        n = 6
        if len(d) % n == 0:
            self.shapes = [d[i:i + n] for i in range(0, len(d), n)]

    def _lookup(self, target: str, source: str, stamp=None):
        """Pose of `source` in `target`, at `stamp` if given, else the latest.

        The scan has to be placed where the robot was when it was taken.
        Drawn at the latest pose instead, a scan taken mid-turn came out
        as long diagonal streaks across the corridor.
        """
        when = rclpy.time.Time.from_msg(stamp) if stamp is not None else rclpy.time.Time()
        try:
            t = self.tf.lookup_transform(target, source, when,
                                         timeout=Duration(seconds=0.0))
        except Exception:
            if stamp is None:
                return None
            return self._lookup(target, source)
        tr = t.transform
        return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)

    def _on_odom(self, msg: Odometry) -> None:
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if now + 1e-9 < self.next_time or self.global_grid is None:
            return
        frame = self.render(now)
        if frame is not None:
            cv2.imwrite(str(self.out_dir / f"frame_{self.count:05d}.jpg"), frame,
                        [cv2.IMWRITE_JPEG_QUALITY, 90])
            self.count += 1
        while self.next_time <= now:
            self.next_time += self.interval

    # ------------------------------------------------------------------
    def _marks(self, panel: Panel) -> None:
        """Only on the global panel: the local one is 6 m wide and the
        text would be larger than the view it labels."""
        img = panel.image
        font, size = cv2.FONT_HERSHEY_SIMPLEX, 0.42
        for x, y, text, colour, filled in self.marks:
            centre = panel.px(x, y)
            cv2.circle(img, centre, 7, colour, -1 if filled else 2, cv2.LINE_AA)
            # The docks sit at the very edge of the map, so text placed
            # to the right of the marker runs off the panel. Flip it to
            # the left side there rather than lose it.
            (tw, _), _ = cv2.getTextSize(text, font, size, 1)
            tx = centre[0] + 10
            if tx + tw > img.shape[1] - 4:
                tx = max(4, centre[0] - 10 - tw)
            ty = max(14, centre[1] - 8)
            for colr, thk in ((0, 0, 0), 3), (colour, 1):
                cv2.putText(img, text, (tx, ty), font, size, colr, thk,
                            cv2.LINE_AA)

    def _footprint(self, panel: Panel, pose, colour, thick: int) -> None:
        rx, ry, ryaw = pose
        c, s = math.cos(ryaw), math.sin(ryaw)
        corners = [(self.half_length, self.half_width), (self.half_length, -self.half_width),
                   (-self.half_length, -self.half_width), (-self.half_length, self.half_width)]
        pts = np.array([panel.px(rx + u * c - v * s, ry + u * s + v * c)
                        for u, v in corners], dtype=np.int32)
        cv2.polylines(panel.image, [pts], True, colour, thick + 1, cv2.LINE_AA)
        nose = panel.px(rx + self.half_length * c, ry + self.half_length * s)
        cv2.line(panel.image, panel.px(rx, ry), nose, colour, thick + 1, cv2.LINE_AA)

    def _overlay(self, panel: Panel, robot, scan_pose) -> None:
        img = panel.image
        thick = max(1, int(panel.scale * 0.5))

        # The other AMRs, each in its own colour -- the one on its body
        # and on its destination labels -- read from the same global
        # tf tree (a1/base_footprint). Drawn under AMR1's footprint.
        for name in self.fleet_names:
            pose = self._lookup("map", f"{name}/base_footprint")
            if pose is not None:
                self._footprint(panel, pose, fleet_rgb255(name), thick)

        # Comfort Spaces, as the planner sees them.
        for x, y, along, across, heading, _speed in self.shapes:
            center = panel.px(x, y)
            axes = (max(int(along / panel.res * panel.scale), 1),
                    max(int(across / panel.res * panel.scale), 1))
            angle = -math.degrees(heading - panel.oyaw)
            cv2.ellipse(img, center, axes, angle, 0, 360, COLOR_COMFORT, thick + 1,
                        cv2.LINE_AA)

        if self.plan is not None and len(self.plan.poses) > 1:
            pts = np.array([panel.px(p.pose.position.x, p.pose.position.y)
                            for p in self.plan.poses], dtype=np.int32)
            cv2.polylines(img, [pts], False, COLOR_PLAN, thick, cv2.LINE_AA)

        if self.scan is not None and scan_pose is not None:
            sx, sy, syaw = scan_pose
            angles = self.scan.angle_min + np.arange(len(self.scan.ranges)) * self.scan.angle_increment
            ranges = np.asarray(self.scan.ranges, dtype=np.float32)
            ok = np.isfinite(ranges) & (ranges > self.scan.range_min) & (ranges < self.scan.range_max)
            wx = sx + ranges[ok] * np.cos(angles[ok] + syaw)
            wy = sy + ranges[ok] * np.sin(angles[ok] + syaw)
            radius = max(1, int(panel.scale * 0.6))
            for x, y in zip(wx, wy):
                cv2.circle(img, panel.px(float(x), float(y)), radius, COLOR_SCAN, -1)

        if robot is not None:
            self._footprint(panel, robot, COLOR_FOOTPRINT, thick)

    def render(self, now: float):
        robot = self._lookup("map", self.cfg["ros"]["frames"]["base"])
        scan_pose = (self._lookup("map", self.scan.header.frame_id, self.scan.header.stamp)
                     if self.scan is not None else None)

        g = self.global_grid
        g_origin = (g.info.origin.position.x, g.info.origin.position.y,
                    yaw_of(g.info.origin.orientation))
        top = Panel(g, g_origin, WIDTH / g.info.width)
        self._overlay(top, robot, scan_pose)
        self._marks(top)

        bottom_img = None
        if self.local_grid is not None:
            l = self.local_grid
            odom = self._lookup("map", l.header.frame_id)
            if odom is not None:
                # Local grid origin is in the odom frame; place it in map.
                ox, oy, oyaw = odom
                lx, ly = l.info.origin.position.x, l.info.origin.position.y
                c, s = math.cos(oyaw), math.sin(oyaw)
                origin = (ox + lx * c - ly * s, oy + lx * s + ly * c,
                          oyaw + yaw_of(l.info.origin.orientation))
                side = 420
                bottom = Panel(l, origin, side / l.info.width)
                self._overlay(bottom, robot, scan_pose)
                bottom_img = bottom.image

        header_h = 50
        legend_h = 440
        canvas = np.full((header_h + top.image.shape[0] + 20 + legend_h, WIDTH, 3),
                         BG, dtype=np.uint8)
        canvas[header_h:header_h + top.image.shape[0], :top.image.shape[1]] = top.image
        y0 = header_h + top.image.shape[0] + 20
        if bottom_img is not None:
            bh, bw = bottom_img.shape[:2]
            canvas[y0:y0 + bh, 20:20 + bw] = bottom_img

        status = self.state or ("STEPPING ASIDE" if self.evading
                                else ("YIELDING" if self.yielding else ""))
        if status == "DRIVING":
            status = ""
        cv2.putText(canvas, f"{self.label}   t = {now - self.start:6.1f} s   {status}",
                    (16, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, COLOR_TEXT, 2, cv2.LINE_AA)
        cv2.putText(canvas, "global costmap", (WIDTH - 230, 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_TEXT, 1, cv2.LINE_AA)

        lx = 480
        items = [
            ("local costmap (6 m x 6 m, around the robot)", None),
            ("LiDAR returns (/scan)", COLOR_SCAN),
            ("AMR footprint", COLOR_FOOTPRINT),
            ("global plan (/plan)", COLOR_PLAN),
            ("Comfort Space, planner size", COLOR_COMFORT),
            ("AMR1 start (ring) and goal (disc)", COLOR_MISSION),
            *[(f"{name.upper()} footprint, home and job legs", fleet_rgb255(name))
              for name in self.fleet_names],
            ("cost: light = low, red = Comfort Space", (90, 110, 230)),
            ("dark = wall / lethal", (20, 20, 20)),
        ]
        for i, (text, color) in enumerate(items):
            yy = y0 + 30 + i * 40
            if color is not None:
                cv2.rectangle(canvas, (lx, yy - 16), (lx + 22, yy + 4), color, -1)
                cv2.rectangle(canvas, (lx, yy - 16), (lx + 22, yy + 4), (200, 200, 200), 1)
            cv2.putText(canvas, text, (lx + 34, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        COLOR_TEXT, 1, cv2.LINE_AA)
        return canvas


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/common.yaml")
    parser.add_argument("--scenario", default=None)
    parser.add_argument("--out", required=True)
    parser.add_argument("--fps", type=float, default=5.0)
    args, _ = parser.parse_known_args()

    cfg = load_config(scenario=args.scenario, common_path=args.config)
    label = cfg.get("scenario", {}).get("name", "common").upper()
    start = float(cfg["navigation"].get("trial_start_sim_time", 0.0))

    rclpy.init()
    node = CostmapRecorder(cfg, Path(args.out), label, args.fps, start)

    stop = {"flag": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
    try:
        while rclpy.ok() and not stop["flag"]:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        print(f"[costmap_recorder] wrote {node.count} frames", flush=True)
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
