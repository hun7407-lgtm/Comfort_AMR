#!/usr/bin/env python3
"""Fill configs/nav2/nav2_params.template.yaml from configs/common.yaml.

Nav2 needs the AMR's footprint, speed limits and goal tolerance, and so
does the rest of the experiment. Keeping two copies by hand is how the
planner ends up believing the robot is a different size than the passage
calculation assumes. The template holds every Nav2 setting that is
genuinely Nav2's business; everything shared comes from here.

    ./scripts/generate_nav2_params.py
    ./scripts/generate_nav2_params.py --config configs/demo_weave.yaml
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from string import Template

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core import load_config, resolve_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/common.yaml")
    parser.add_argument("--template", default="configs/nav2/nav2_params.template.yaml")
    parser.add_argument("--out", default="configs/nav2/nav2_params.yaml")
    parser.add_argument("--map", default="configs/nav2/corridor_map.yaml")
    args = parser.parse_args()

    cfg = load_config(common_path=args.config)
    nav = cfg["navigation"]
    amr = cfg["amr"]

    half_len = float(amr["footprint_length"]) / 2.0
    half_wid = float(amr["footprint_width"]) / 2.0
    footprint = (f"[[{half_len:.3f}, {half_wid:.3f}], [{half_len:.3f}, {-half_wid:.3f}], "
                 f"[{-half_len:.3f}, {-half_wid:.3f}], [{-half_len:.3f}, {half_wid:.3f}]]")

    # Inflation has to reach past the robot's inscribed radius or the
    # planner will happily clip a corner, but a large radius makes it
    # shy away from gaps it could actually use.
    #
    # Set to exactly the clearance comfort_core.layout.robot_clearance_radius
    # uses, so "the geometry says this gap is passable" and "Nav2 will
    # drive through this gap" mean the same thing. An earlier version
    # added the margin twice; Nav2 then refused gaps the passage check
    # had declared open, and the robot stalled beside an obstacle it had
    # room to pass.
    inflation = half_wid + float(nav.get("passage_margin", 0.10))

    cm = cfg["navigation"].get("collision_monitor", {})
    values = {
        "CM_TIME_BEFORE_COLLISION": f"{float(cm.get('time_before_collision', 0.9)):.2f}",
        "CM_SIM_STEP": f"{float(cm.get('simulation_time_step', 0.1)):.2f}",
        "CM_MIN_POINTS": str(int(cm.get("min_points", 5))),
        "SCAN_MIN_HEIGHT": f"{float(cfg['ros']['scan']['min_height']):.2f}",
        "SCAN_MAX_HEIGHT": f"{float(cfg['ros']['scan']['max_height']):.2f}",
        "GLOBAL_UPDATE_HZ": "3.0",
        "MAP_YAML": str(resolve_path(args.map)),
        "BASE_FRAME": cfg["ros"]["frames"]["base"],
        "FOOTPRINT": footprint,
        "MAX_VEL_X": f"{float(nav['max_linear_speed']):.3f}",
        "MAX_VEL_THETA": f"{float(nav['max_angular_speed']):.3f}",
        "XY_TOLERANCE": f"{float(nav['goal_tolerance']):.3f}",
        # Orientation at the goal is not part of what this study measures,
        # so it is left loose. A tight yaw tolerance would turn "arrived
        # but facing the wrong way" into a failure.
        "YAW_TOLERANCE": "3.15",
        "INFLATION_RADIUS": f"{inflation:.3f}",
        # Higher means cost falls off faster with distance, which leaves a
        # clearer low-cost channel down the middle of a narrow gap. At the
        # default 3.0 the cost from two facing obstacles overlapped across
        # the whole opening and DWB had no good trajectory to pick.
        "COST_SCALING": "5.0",
        # DWB's obstacle critic. The Nav2 default of 0.02 barely
        # influences the choice, so the robot drifts to whichever side the
        # path happens to hug and then cannot recover. Weighting it enough
        # to matter keeps the robot near the middle of a gap.
        "BASE_OBSTACLE_SCALE": "0.2",
    }

    template = Template(resolve_path(args.template).read_text(encoding="utf-8"))
    try:
        rendered = template.substitute(values)
    except KeyError as exc:
        print(f"[ERROR] template uses ${exc.args[0]} but nothing supplies it", file=sys.stderr)
        return 1

    out_path = resolve_path(args.out)
    out_path.write_text(
        "# GENERATED FILE. Do not edit.\n"
        f"# Written by scripts/generate_nav2_params.py from {args.config}\n"
        "# and configs/nav2/nav2_params.template.yaml. Edit those instead.\n"
        + rendered,
        encoding="utf-8",
    )

    print(f"wrote {out_path}")
    for k, v in values.items():
        print(f"      {k:18s} = {v}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
