#!/usr/bin/env python3
"""Publish the Comfort Space to Nav2 as a costmap filter mask.

Runs in the SYSTEM Python with ROS 2 Jazzy. No omni, no pxr.

Why a mask and not fake LiDAR points
------------------------------------
Injecting points into the obstacle layer was the obvious alternative and
it does not work. Measured in this project: a LiDAR only ever sees the
near face of a person, the far side stays unobserved, and because the
static map declares that ground free, the costmap leaves a hole exactly
where the person is standing. A Comfort Space delivered that way loses
its far half. Points also land as LETHAL and then get inflated again, so
the effective radius stops matching the configured one.

A filter mask is drawn analytically from the human's position, so the
whole circle is present, occluded or not, and the value written is the
cost that results. See docs/NAV2.md section 8.

The mask is a high but finite cost, never lethal. The Comfort Space is
expensive to cross, not impossible. What stops the AMR from crossing it
is scripts/comfort_gate.py, which is a separate policy decision.

    ./scripts/run_comfort.sh --scenario b2
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import rclpy
import yaml
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, Float32MultiArray
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from comfort_core import load_config, resolve_path
from comfort_core.comfort_space import make_policy, robot_grow, shapes as build_shapes
from comfort_core.human_state import make_state_provider
from human_tracker import HumanTracker

# KeepoutFilter's type code.
FILTER_TYPE_KEEPOUT = 0

# Both topics must be latched: Nav2's filter subscribes once, at
# activation, and will never see a volatile message published before it
# was ready.
LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)


def read_map_geometry(map_yaml: Path):
    """Take the mask's extent and resolution from the generated map.

    Sharing the map's grid means a mask cell and a costmap cell line up
    exactly, so the painted radius is the radius Nav2 applies.
    """
    meta = yaml.safe_load(map_yaml.read_text())
    image = (map_yaml.parent / meta["image"]).resolve()

    # Minimal PGM header parse: magic, width, height, maxval, skipping
    # comments. Avoids pulling in an image library.
    data = image.read_bytes()
    fields, i = [], 0
    while len(fields) < 4:
        if data[i:i + 1] == b"#":
            while data[i:i + 1] != b"\n":
                i += 1
            i += 1
            continue
        j = i
        while data[j:j + 1] not in (b" ", b"\n", b"\t"):
            j += 1
        token = data[i:j]
        if token:
            fields.append(token)
        i = j + 1

    return {
        "width": int(fields[1]),
        "height": int(fields[2]),
        "resolution": float(meta["resolution"]),
        "origin": [float(v) for v in meta["origin"]],
    }


class ComfortPublisher(Node):
    def __init__(self, cfg, geometry, mask_topic: str, info_topic: str):
        super().__init__("comfort_space_publisher")
        self.cfg = cfg
        self.geom = geometry

        cs = cfg["comfort_space"]
        self.mask_value = int(cs["costmap"]["mask_value"])
        self.buffer_width = float(cs["costmap"].get("buffer_width", 0.0))
        self.buffer_value = int(cs["costmap"].get("buffer_value", 0))
        if not 0 <= self.mask_value < 100:
            raise ValueError(
                f"comfort_space.costmap.mask_value is {self.mask_value}; must be "
                "0..99. 100 makes the Comfort Space a lethal obstacle, which "
                "this study deliberately avoids.")

        self.provider = make_state_provider(cfg)
        self.policy = make_policy(cfg)
        # Where the humans actually are, which is not the same as where
        # the config put them once one of them starts walking.
        self.tracker = HumanTracker(self, cfg)
        self.speed_threshold = float(cs.get("moving_speed_threshold", 0.15))
        self._reported_states: dict = {}

        self.mask_pub = self.create_publisher(OccupancyGrid, mask_topic, LATCHED)
        # The circles the yield gate has to respect, as a flat
        # [x, y, radius, x, y, radius, ...] array.
        #
        # The gate used to read /tf itself. Subscribing to a 60 Hz
        # transform stream inside the node that sits in the velocity path
        # slowed its callback enough to make Nav2 stutter, and a B3 that
        # reached the goal started deadlocking. One node does the
        # tracking; the gate gets a small message at a modest rate.
        self.circle_pub = self.create_publisher(
            Float32MultiArray, "/comfort_shapes", 10)
        # True while anybody is actually walking.
        #
        # A robot waiting for a person to cross in front of it is not
        # deadlocked; the situation resolves by itself in a few seconds.
        # Counting that wait as a deadlock would label correct yielding
        # as a failure, which is precisely the outcome this study is
        # trying to measure. The trial holds its no-progress timer while
        # this is true.
        self.moving_pub = self.create_publisher(Bool, "/comfort_humans_moving", 10)
        self.info_pub = self.create_publisher(CostmapFilterInfo, info_topic, LATCHED)

        info = CostmapFilterInfo()
        info.header.frame_id = "map"
        info.header.stamp = self.get_clock().now().to_msg()
        info.type = FILTER_TYPE_KEEPOUT
        info.filter_mask_topic = mask_topic
        # cost = base + multiplier * mask_value, then scaled to 0..254.
        info.base = 0.0
        info.multiplier = 1.0
        self.info_pub.publish(info)

        rate = float(cs["costmap"].get("publish_rate_hz", 2.0))
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.publish_mask)
        self.published = 0

    def build_mask(self) -> OccupancyGrid:
        g = self.geom
        w, h, res = g["width"], g["height"], g["resolution"]
        ox, oy, _ = g["origin"]

        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = res
        grid.info.width = w
        grid.info.height = h
        grid.info.origin.position.x = ox
        grid.info.origin.position.y = oy
        grid.info.origin.orientation.w = 1.0

        data = bytearray(w * h)  # 0 everywhere, meaning no extra cost

        shapes = self.current_shapes()

        def paint_shape(shape, value: int) -> None:
            """Fill one Comfort Space, never lowering a higher cell.

            Rows are filled by solving the shape for its y band at each
            column, so an ellipse costs no more than a circle and the
            painted edge lands exactly where the geometry says it does.
            """
            if value <= 0:
                return
            reach = shape.bounding_radius
            c0 = max(int((shape.x - reach - ox) / res), 0)
            c1 = min(int((shape.x + reach - ox) / res) + 1, w)
            for col in range(c0, c1):
                wx = ox + (col + 0.5) * res
                span = shape.y_span_at(wx)
                if span is None:
                    continue
                lo = max(int((span[0] - oy) / res), 0)
                hi = min(int((span[1] - oy) / res) + 1, h)
                for row in range(lo, hi):
                    index = row * w + col
                    if data[index] < value:
                        data[index] = value

        # Buffer first, then the Comfort Space over the top of it, so the
        # boundary keeps the full value and only the outside is softened.
        for shape in shapes:
            paint_shape(shape.grown(self.buffer_width), self.buffer_value)
        for shape in shapes:
            paint_shape(shape, self.mask_value)

        grid.data = list(data)
        return grid

    def current_shapes(self):
        """The Comfort Spaces as they are right now.

        Positions and headings come from /tf, and a person the config
        calls `moving` is treated as stationary once they actually stop.
        That is what makes the walker's Comfort Space shrink when they
        arrive, which under the state-adaptive policy is what opens the
        corridor.
        """
        declared = self.provider.states()
        speeds = self.tracker.speeds()
        observed = self.tracker.observed_states(declared, self.speed_threshold)
        if observed != self._reported_states:
            changed = {k: v for k, v in observed.items()
                       if self._reported_states.get(k) != v}
            if self._reported_states:
                self.get_logger().info(f"human state changed: {changed}")
            self._reported_states = dict(observed)

        # The declared states are handed to the policy, not the observed
        # ones. A person's state does not stop being `moving` because
        # they paused; what changes is their speed, and the policy is
        # what decides how much that shrinks their Comfort Space.
        axes = self.policy.axes(declared, speeds)
        return build_shapes(self.cfg, axes, self.tracker.positions(),
                            self.tracker.headings(), robot_grow(self.cfg))

    def publish_shapes(self, shapes) -> None:
        msg = Float32MultiArray()
        flat = []
        for shape in shapes:
            flat.extend([shape.x, shape.y, shape.along, shape.across,
                         shape.heading])
        msg.data = flat
        self.circle_pub.publish(msg)

    def publish_mask(self) -> None:
        shapes = self.current_shapes()
        grid = self.build_mask()
        self.mask_pub.publish(grid)
        self.publish_shapes(shapes)

        speeds = self.tracker.speeds()
        flag = Bool()
        flag.data = any(v >= self.speed_threshold for v in speeds.values())
        self.moving_pub.publish(flag)

        if self.published == 0:
            inner = sum(1 for v in grid.data if v == self.mask_value)
            painted = sum(1 for v in grid.data if v)
            self.get_logger().info(
                f"policy={self.policy.name} "
                f"shape={self.cfg['comfort_space'].get('shape', 'circle')} "
                f"states={self._reported_states}")
            self.get_logger().info(
                f"mask {grid.info.width}x{grid.info.height} at "
                f"{grid.info.resolution} m/px, {inner} cells at "
                f"{self.mask_value} plus {painted - inner} buffer cells at "
                f"{self.buffer_value} (buffer {self.buffer_width:.2f} m)")

        # Every few seconds, so a stalled tracker or a Comfort Space that
        # is not following its person shows up in the log instead of
        # having to be inferred from the robot's behaviour.
        if self.published % 8 == 0:
            positions = self.tracker.positions()
            self.get_logger().info(
                "tracked " + "  ".join(
                    f"{k}=({p[0]:+.2f},{p[1]:+.2f}) {speeds.get(k, 0.0):.2f}m/s"
                    for k, p in sorted(positions.items())))

        self.published += 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/common.yaml")
    parser.add_argument("--scenario", default=None, help="b2, b3 or a path")
    parser.add_argument("--map", default="configs/nav2/corridor_map.yaml")
    parser.add_argument("--mask-topic", default="/comfort_mask")
    parser.add_argument("--info-topic", default="/costmap_filter_info")
    args, _ = parser.parse_known_args()

    cfg = load_config(scenario=args.scenario, common_path=args.config)
    if not cfg["comfort_space"]["costmap"].get("enabled", True):
        print("comfort_space.costmap.enabled is false; nothing to publish")
        return 0

    geometry = read_map_geometry(resolve_path(args.map))

    rclpy.init()
    node = ComfortPublisher(cfg, geometry, args.mask_topic, args.info_topic)
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
