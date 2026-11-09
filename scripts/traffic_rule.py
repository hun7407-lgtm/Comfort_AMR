#!/usr/bin/env python3
"""Keep right: a cost on the left half of each lane, per robot, by its route.

    ./scripts/run_traffic_rule.sh --config configs/fleet_nav2.yaml --scenario b3d

Two Nav2 robots meeting head-on in a 1.5 m lane each see the other as
an obstacle and each pick a side; pick the same side and they face
each other again. A traffic rule breaks the symmetry. This one is the
ordinary factory rule, keep to the right, and it is applied the only
way a costmap can carry it: as a SECOND KeepoutFilter, separate from
the Comfort Space mask, so a recording can always say whether a
vehicle moved over for a person or for the rule.

Why a mask per robot. "Left" depends on which way the robot is going
along the lane, so one shared mask cannot encode it: east-bound, the
left half of the north aisle is its north side; west-bound, its south
side. Each robot therefore gets its own mask.

Why from the robot's MOTION and not from its plan. The first version
painted every lane the robot's plan ran through. That closes a loop:
the mask raises the cost of cells along the chosen route, the planner
switches to the other crossing, the mask follows the new plan, and
the planner switches back -- measured on a3, whose two routes are
nearly the same length: its plan flipped between the crossings until
the progress rule called it deadlocked at full speed. So the rule is
local, the way the real one is: only the lane the robot is IN is
painted, and the direction is the way it is actually moving along
that lane. Nothing ahead of it changes cost, so route choice is
untouched; what changes is which half of the current lane is cheap.

The value is comfort_space.costmap.buffer_value (55) unless
traffic_rule.value says otherwise: the left half costs what the outer
buffer of somebody's Comfort Space costs -- weaker than the space
itself (98), so a person's space still wins, and well above inflation
noise. A preference, not a wall: a robot whose only way past is on the
left pays and goes.

Drawn on the generated map's own grid, like the Comfort Space mask,
so a mask cell and a costmap cell line up exactly.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from comfort_core import load_config, nav2_map, resolve_path  # noqa: E402
from comfort_core.factory_layout import build_layout  # noqa: E402
from comfort_core.mapgrid import read_map_geometry  # noqa: E402

Point = Tuple[float, float]


class Lane:
    """One driveable rectangle and the axis traffic runs along it."""

    def __init__(self, name: str, x0: float, x1: float, y0: float, y1: float,
                 axis: str):
        self.name = name
        self.x0, self.x1, self.y0, self.y1 = x0, x1, y0, y1
        self.axis = axis            # "x" for an aisle, "y" for a crossing
        self.centre = (y0 + y1) / 2.0 if axis == "x" else (x0 + x1) / 2.0

    def contains(self, p: Point) -> bool:
        return self.x0 <= p[0] <= self.x1 and self.y0 <= p[1] <= self.y1

    def direction(self, plan: List[Point], min_run: float = 0.5) -> int:
        """+1 / -1 along the axis as the plan runs through, 0 if it does not."""
        total = 0.0
        for a, b in zip(plan, plan[1:]):
            mid = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
            if self.contains(mid):
                total += (b[0] - a[0]) if self.axis == "x" else (b[1] - a[1])
        if abs(total) < min_run:
            return 0
        return 1 if total > 0 else -1

    def left_half(self, direction: int) -> Tuple[float, float, float, float]:
        """The rectangle on the robot's left, for travel in `direction`."""
        if self.axis == "x":
            # Heading +x, left is +y.
            if direction > 0:
                return (self.x0, self.x1, self.centre, self.y1)
            return (self.x0, self.x1, self.y0, self.centre)
        # Heading +y, left is -x.
        if direction > 0:
            return (self.x0, self.centre, self.y0, self.y1)
        return (self.centre, self.x1, self.y0, self.y1)


def lanes_of(cfg) -> List[Lane]:
    """Aisles and crossings, widened by the map's lane margin.

    The occupancy map carves `factory.map.lane_margin` of slack either
    side of the paint; the rule has to cover that slack too, or the
    slack becomes a free lane on the wrong side.
    """
    layout = build_layout(cfg)
    margin = float(cfg["factory"].get("map", {}).get("lane_margin", 0.0))
    x0, x1 = layout.x_min, layout.x_min + layout.length
    lanes = [Lane(b.name, x0, x1, b.y_min - margin, b.y_max + margin, "x")
             for b in layout.aisles]
    lanes += [Lane(c.name, c.x_min - margin, c.x_max + margin, c.y_min, c.y_max, "y")
              for c in layout.crosses]
    return lanes


def directions_of(lanes: List[Lane], plan: List[Point]) -> Dict[str, int]:
    """{lane: +1/-1} for every lane a polyline runs through (dry run only)."""
    return {l.name: d for l in lanes if (d := l.direction(plan)) != 0}


def paint(geom: Dict, lanes: List[Lane], wanted: Dict[str, int], value: int) -> Tuple[bytearray, Dict[str, int]]:
    """The mask bytes for {lane: direction}, and what was painted."""
    w, h, res = geom["width"], geom["height"], geom["resolution"]
    ox, oy = geom["origin"][0], geom["origin"][1]
    data = bytearray(w * h)
    painted: Dict[str, int] = {}
    for lane in lanes:
        d = wanted.get(lane.name, 0)
        if d == 0:
            continue
        painted[lane.name] = d
        lx0, lx1, ly0, ly1 = lane.left_half(d)
        c0 = max(int((lx0 - ox) / res), 0)
        c1 = min(int(math.ceil((lx1 - ox) / res)), w)
        r0 = max(int((ly0 - oy) / res), 0)
        r1 = min(int(math.ceil((ly1 - oy) / res)), h)
        # Split at a cell boundary, not inside the centre cell: with
        # the centreline rounded outwards both halves owned the row it
        # fell in, and the two directions overlapped by one row.
        if lane.axis == "x":
            rc = int(round((lane.centre - oy) / res))
            r0, r1 = (rc, r1) if d > 0 else (r0, rc)
        else:
            cc = int(round((lane.centre - ox) / res))
            c0, c1 = (c0, cc) if d > 0 else (cc, c1)
        for r in range(r0, r1):
            row = r * w
            for c in range(c0, c1):
                data[row + c] = value
    return data, painted


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="configs/common.yaml")
    ap.add_argument("--scenario", default=None)
    ap.add_argument("--dry-run", action="store_true",
                    help="paint a synthetic east-bound plan and print the "
                         "cell counts; no ROS")
    # parse_known_args, because the launch file appends --ros-args; a
    # strict parse rejected them and the node died at startup, so the
    # first "rule on" run had the filter configured and no mask at all.
    args, _ros_args = ap.parse_known_args()
    cfg = load_config(scenario=args.scenario or None, common_path=args.config)
    rule = cfg.get("traffic_rule", {}) or {}
    value = int(rule.get("value", cfg["comfort_space"]["costmap"].get("buffer_value", 55)))
    lanes = lanes_of(cfg)

    geom = read_map_geometry(resolve_path(nav2_map(cfg)))

    if args.dry_run:
        plan = [(x, 4.215) for x in [i * 0.5 for i in range(-20, 21)]]  # east along the north aisle
        data, painted = paint(geom, lanes, directions_of(lanes, plan), value)
        print(f"lanes: {[(l.name, l.axis, round(l.centre, 3)) for l in lanes]}")
        print(f"east-bound north aisle -> painted {painted}, {sum(1 for v in data if v)} cells at {value}")
        plan = list(reversed(plan))
        data2, painted2 = paint(geom, lanes, directions_of(lanes, plan), value)
        print(f"west-bound             -> painted {painted2}, {sum(1 for v in data2 if v)} cells")
        # The two halves must not overlap and together must cover the lane.
        both = sum(1 for a, b in zip(data, data2) if a and b)
        print(f"overlap between the two directions: {both} cells (expect 0)")
        return 0 if both == 0 and painted and painted2 else 1

    import rclpy
    from nav2_msgs.msg import CostmapFilterInfo
    from nav_msgs.msg import OccupancyGrid
    from rclpy.duration import Duration
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener
    from comfort_publisher import LATCHED, FILTER_TYPE_KEEPOUT  # noqa: E402

    base_frame = cfg["ros"]["frames"]["base"]
    # How far the robot has to have moved along a lane's axis before
    # its direction there counts. Below this it keeps the last known
    # direction (a robot stopped in a lane is still facing the way it
    # was going), and a robot that has only just entered paints nothing.
    min_run = float(rule.get("min_run_m", 0.3))

    robots = [""]  # AMR1, no namespace
    fleet = cfg.get("fleet", {})
    if fleet.get("enabled", False) and fleet.get("driver", "scripted") == "nav2":
        robots += [n for n, s in sorted(fleet.get("amrs", {}).items()) if s.get("enabled", True)]
    mask_name = str(rule.get("mask_topic", "traffic_mask"))
    info_name = str(rule.get("info_topic", "traffic_filter_info"))

    class TrafficRule(Node):
        def __init__(self):
            super().__init__("traffic_rule")
            self.pubs = {}
            self.last: Dict[str, Dict[str, int]] = {}
            for r in robots:
                ns = f"/{r}" if r else ""
                mask_topic, info_topic = f"{ns}/{mask_name}", f"{ns}/{info_name}"
                mask_pub = self.create_publisher(OccupancyGrid, mask_topic, LATCHED)
                info_pub = self.create_publisher(CostmapFilterInfo, info_topic, LATCHED)
                info = CostmapFilterInfo()
                info.header.frame_id = "map"
                info.header.stamp = self.get_clock().now().to_msg()
                info.type = FILTER_TYPE_KEEPOUT
                info.filter_mask_topic = mask_topic
                info.base = 0.0
                info.multiplier = 1.0
                info_pub.publish(info)
                self.pubs[r] = (mask_pub, info_pub)
                self.publish(r, {})      # an empty mask until it is in a lane
                self.get_logger().info(f"keep-right mask for {r or 'AMR1'} on {mask_topic}")
            self.tf = Buffer()
            self.tf_listener = TransformListener(self.tf, self)
            # Per robot: the lane it is in, where it entered it, and the
            # direction decided there.
            self.track: Dict[str, Dict] = {r: {"lane": None, "entry": None, "dir": 0} for r in robots}
            self.create_timer(0.2, self.poll)

        def pose(self, r: str):
            frame = f"{r}/{base_frame}" if r else base_frame
            try:
                tf = self.tf.lookup_transform("map", frame, rclpy.time.Time(),
                                              timeout=Duration(seconds=0.0))
            except Exception:
                return None
            return (tf.transform.translation.x, tf.transform.translation.y)

        def poll(self) -> None:
            for r in robots:
                p = self.pose(r)
                if p is None:
                    continue
                st = self.track[r]
                lane = next((l for l in lanes if l.contains(p)), None)
                if lane is None or lane.name != st["lane"]:
                    st.update(lane=lane.name if lane else None, entry=p, dir=0)
                    if lane is None:
                        self.publish(r, {})
                    continue
                along = (p[0] - st["entry"][0]) if lane.axis == "x" else (p[1] - st["entry"][1])
                if abs(along) >= min_run:
                    d = 1 if along > 0 else -1
                    if d != st["dir"]:
                        st["dir"] = d
                        self.publish(r, {lane.name: d})

        def publish(self, r: str, wanted: Dict[str, int]) -> None:
            data, painted = paint(geom, lanes, wanted, value)
            if self.last.get(r) == painted and r in self.last:
                return
            self.last[r] = painted
            grid = OccupancyGrid()
            grid.header.frame_id = "map"
            grid.header.stamp = self.get_clock().now().to_msg()
            grid.info.resolution = geom["resolution"]
            grid.info.width = geom["width"]
            grid.info.height = geom["height"]
            grid.info.origin.position.x = geom["origin"][0]
            grid.info.origin.position.y = geom["origin"][1]
            grid.info.origin.orientation.w = 1.0
            grid.data = [int(v) for v in data]
            self.pubs[r][0].publish(grid)
            if painted:
                self.get_logger().info(
                    f"{r or 'AMR1'}: keep right on "
                    + ", ".join(f"{k} {'+' if d > 0 else '-'}" for k, d in painted.items()))

    rclpy.init(args=sys.argv)
    node = TrafficRule()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # rclpy's own SIGINT handler may already have shut the context
        # down; a second shutdown raises, and there is nothing to add.
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
