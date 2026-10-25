#!/usr/bin/env python3
"""Move a saved SLAM map into world coordinates.

Runs in any Python. No ROS, no Isaac Sim.

    ./nav3d/place_map.py --config configs/factory_3d.yaml \
                         --map nav3d/maps/factory_bay.yaml

Why this is needed
------------------
slam_toolbox starts its map frame wherever the robot was when it heard
the first scan, which is `navigation.start`. Everything else in this
project speaks world coordinates: the goals, the station positions, the
people, the Comfort Space mask, the clearance checks. A map whose origin
is the robot's parking spot would put the goal half a bay away from where
the goal is.

Rather than translate every consumer, the map is translated once, here,
by rewriting `origin` in the saved yaml. The pixels are untouched. After
this the SLAM map and the generated map describe the same world, so the
static map -> odom transform in scripts/nav2_bringup.launch.py stays
exactly as it is for the 2D runs, and a 3D trial's goal means what it
says.

It also prints the mapped extent beside the extent the layout model
expects, which is the cheapest check that the lap actually saw the
building: a map two metres short on one side is a lap that was cut off,
and it is far better to find that out here than from a planner that will
not route into a wall it thinks is solid.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import yaml

from comfort_core import load_config, resolve_path
from comfort_core.factory_layout import build_layout
from comfort_core.frames import odom_to_world

MARKER = "# placed in world coordinates by nav3d/place_map.py"


def pgm_size(path: Path) -> tuple[int, int]:
    """Width and height from a binary PGM header, comments and all."""
    with open(path, "rb") as f:
        fields: list[bytes] = []
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError(f"{path} is not a binary PGM (got {magic!r})")
        while len(fields) < 3:
            line = f.readline()
            if not line:
                raise ValueError(f"{path} has a truncated header")
            if line.lstrip().startswith(b"#"):
                continue
            fields += line.split()
    return int(fields[0]), int(fields[1])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True,
                        help="the scene config the map was made for")
    parser.add_argument("--map", default=None,
                        help="map yaml to place; defaults to the one the "
                             "config navigates on")
    args = parser.parse_args()

    cfg = load_config(common_path=args.config)
    map_path = resolve_path(args.map or cfg["nav2"]["map"])
    if not map_path.is_file():
        print(f"[ERROR] no map at {map_path}", file=sys.stderr)
        return 1

    text = map_path.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"[ERROR] {map_path.name} has already been placed. Shifting it "
              f"again would move it off the building.", file=sys.stderr)
        return 1

    data = yaml.safe_load(text)
    ox, oy, oyaw = (float(v) for v in data["origin"])

    # The SLAM map frame and the odom frame share an origin: slam_toolbox
    # anchors the map at the robot's odometry pose when mapping started,
    # and odometry starts at zero. So the odom -> world transform is
    # exactly the transform this map needs.
    wx, wy, wyaw = odom_to_world(cfg, (ox, oy, oyaw))

    res = float(data["resolution"])
    width, height = pgm_size(map_path.parent / data["image"])

    if abs(wyaw) > 1e-6:
        print(f"[ERROR] the robot spawns at {math.degrees(wyaw):+.1f} deg, so "
              f"the map is rotated relative to the world. nav2's static "
              f"layer ignores a rotated origin, so this would be wrong in a "
              f"way nothing downstream would report. Map from a spawn yaw "
              f"of 0, or add rotation support here.", file=sys.stderr)
        return 1

    data["origin"] = [round(wx, 4), round(wy, 4), 0.0]
    map_path.write_text(
        MARKER + "\n"
        f"# origin was [{ox:.4f}, {oy:.4f}, {oyaw:.4f}] in the frame SLAM\n"
        f"# built it in, which starts at navigation.start.\n"
        + yaml.safe_dump(data, sort_keys=False),
        encoding="utf-8")

    layout = build_layout(cfg)
    x_hi, y_hi = wx + width * res, wy + height * res
    print(f"placed {map_path}")
    print(f"      {width} x {height} px at {res} m/px")
    print(f"      origin ({ox:+.2f}, {oy:+.2f}) -> ({wx:+.2f}, {wy:+.2f})")
    print(f"      covers  x[{wx:+6.2f}, {x_hi:+6.2f}]  "
          f"y[{wy:+6.2f}, {y_hi:+6.2f}]")
    print(f"      bay is  x[{layout.x_min:+6.2f}, {layout.x_max:+6.2f}]  "
          f"y[{layout.y_min:+6.2f}, {layout.y_max:+6.2f}]")

    # A lap that was cut short shows up as a map that does not reach a
    # wall. Half a metre of slack covers the wall thickness and the
    # saver's border; more than that is worth looking at.
    short = []
    for name, mapped, expected, sign in (
            ("x_min", wx, layout.x_min, +1), ("x_max", x_hi, layout.x_max, -1),
            ("y_min", wy, layout.y_min, +1), ("y_max", y_hi, layout.y_max, -1)):
        if sign * (mapped - expected) > 0.5:
            short.append(f"{name} {mapped:+.2f} vs {expected:+.2f}")
    if short:
        print("      [WARN] the map does not reach the bay's edge: "
              + "; ".join(short))
        print("             the lap may not have seen the whole building")
    else:
        print("      extent agrees with the layout")
    return 0


if __name__ == "__main__":
    sys.exit(main())
