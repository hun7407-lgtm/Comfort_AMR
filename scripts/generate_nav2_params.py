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

from comfort_core import load_config, nav2_map, resolve_path


def default_template(cfg) -> str:
    """Which parameter template to render. 2D unless the config says 3D."""
    return str(cfg.get("nav2", {}).get(
        "template", "configs/nav2/nav2_params.template.yaml"))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/common.yaml")
    parser.add_argument("--template", default=None,
                        help="parameter template; defaults to the one this "
                             "config asks for (2D unless it says otherwise)")
    parser.add_argument("--out", default="configs/nav2/nav2_params.yaml")
    parser.add_argument("--map", default=None,
                        help="map yaml; defaults to the one for this scene")
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

    map_path = resolve_path(args.map or nav2_map(cfg))
    if not map_path.is_file():
        # Nav2 would come up and then refuse to activate map_server, with
        # the real reason several hundred lines up its log. Say it here.
        print(f"[ERROR] no map at {map_path}", file=sys.stderr)
        if cfg.get("nav2", {}).get("map"):
            print(f"        this config navigates on a map built by SLAM. "
                  f"Build it once with:\n"
                  f"          ./scripts/run_mapping.sh --config {args.config}",
                  file=sys.stderr)
        return 1

    cm = cfg["navigation"].get("collision_monitor", {})
    # ISO 3691-4 protective field. The standard's stopping distance is
    #
    #     reaction * v  +  v^2 / (2a)  +  margin
    #
    # which depends on speed, so the field does too: one sub-polygon per
    # speed band, each sized for the fastest speed in its band and
    # measured forward from the front of the footprint.
    #
    # A single fixed field does not fit here. The footprint grown by
    # 0.25 m all round is 0.49 m half-width; the lane edge is 0.75 m
    # from the centreline, so 0.03 m of tracking error put the field
    # over the line, it caught the floor outside the lane and the robot
    # stopped for good -- B1 deadlocked with the nearest person 13 m
    # away. Sideways the field now adds the scanner tolerance only.
    stop_margin = float(cm.get("stop_margin", 0.25))
    reaction = float(cm.get("reaction_time", 0.25))
    decel = float(cm.get("decel", 1.0))
    tolerance = float(cm.get("scanner_tolerance", 0.10))
    # Nothing commands more than max_linear_speed, but the controller
    # overshoots it slightly; size the top band for that, not for the
    # nominal limit.
    v_cruise = float(nav["max_linear_speed"]) * 1.1
    v_reverse = float(cm.get("reverse_speed", 0.25))
    # Where one band ends and the next begins, as a fraction of the top
    # speed. A robot at 0.3 of its top speed gets a field sized for
    # 0.3, not for the top.
    # What a standing vehicle keeps clear: the scanner's own error, and
    # nothing else. See CM_FIELD_STOPPED below.
    restart = float(cm.get("restart_margin", tolerance))
    v_slow = v_cruise * 0.25
    v_mid = v_cruise * 0.60

    def reach(v: float) -> float:
        """How far ahead of the footprint the field has to extend."""
        return half_len + reaction * v + v * v / (2.0 * decel) + stop_margin

    sy = half_wid + tolerance

    # The same sum read as a duration, for the approach layer. Derived
    # rather than configured: a hand-written number goes stale the
    # moment the speed changes, and this one going stale meant B1
    # touching people.
    approach_time = (reaction + v_cruise / (2.0 * decel)
                     + stop_margin / max(v_cruise, 1e-3))
    approach_time = float(cm.get("time_before_collision", approach_time))

    def field(v: float, backward: bool = False) -> str:
        far = reach(v)
        front, back = (half_len, -far) if backward else (far, -half_len)
        return (f"[[{front:.3f}, {sy:.3f}], [{front:.3f}, {-sy:.3f}], "
                f"[{back:.3f}, {-sy:.3f}], [{back:.3f}, {sy:.3f}]]")
    voxel = cfg.get("nav2", {}).get("voxel", {})
    max_wait = float(cfg.get("comfort_space", {}).get("yield", {})
                     .get("evade", {}).get("max_wait_seconds", 20.0))

    values = {
        "CLOUD_TOPIC": cfg["ros"]["topics"]["lidar_cloud"],
        "VOXEL_MIN_HEIGHT": f"{float(voxel.get('min_height', 0.08)):.2f}",
        "VOXEL_MAX_HEIGHT": f"{float(voxel.get('max_height', 1.80)):.2f}",
        "VOXEL_Z_RES": f"{float(voxel.get('z_resolution', 0.12)):.2f}",
        "VOXEL_Z_COUNT": str(int(voxel.get("z_voxels", 16))),
        "VOXEL_MARK_THRESHOLD": str(int(voxel.get("mark_threshold", 0))),
        "VOXEL_UNKNOWN_THRESHOLD": str(int(voxel.get("unknown_threshold", 15))),
        # Four forward bands, because one field for everything above
        # walking pace is the fixed field again by another name: sized
        # for the top speed, it would stop the robot for things a metre
        # away while it creeps. The thresholds follow the top speed so
        # raising it re-cuts the bands instead of leaving them stale.
        "CM_BAND_SLOW": f"{v_slow:.2f}",
        "CM_BAND_MID": f"{v_mid:.2f}",
        "CM_FIELD_CRUISE": field(v_cruise),
        "CM_FIELD_MID": field(v_mid),
        "CM_FIELD_SLOW": field(v_slow),
        "CM_FIELD_REVERSE": field(v_reverse, backward=True),
        # The restart field, and the only one not sized by a speed.
        #
        # A vehicle at rest has no reaction distance and no braking
        # distance to cover -- both terms are zero -- so what it needs
        # in front of it is its own footprint plus what the scanner
        # might be wrong by. ISO's stopping-distance sum gives nothing
        # more than that at v = 0.
        #
        # Sizing this band for 0.05 m/s instead gave 0.614 m, and that
        # number is a trap: anything that stops the robot and then
        # settles 0.5 m in front of it holds it there forever, because
        # the collision monitor zeroes the command and the vehicle
        # cannot reverse or turn out of its own protective field.
        # Measured in the swap scenario -- the robot stopped for two
        # people crossing, they finished crossing, and it never moved
        # again. The field grows back the instant it rolls: at
        # 0.05 m/s the slow band takes over at 0.679 m.
        "CM_FIELD_STOPPED": (f"[[{half_len + restart:.3f}, {sy:.3f}], "
                             f"[{half_len + restart:.3f}, {-sy:.3f}], "
                             f"[{-half_len:.3f}, {-sy:.3f}], "
                             f"[{-half_len:.3f}, {sy:.3f}]]"),
        "CM_TIME_BEFORE_COLLISION": f"{approach_time:.2f}",
        "CM_SIM_STEP": f"{float(cm.get('simulation_time_step', 0.1)):.2f}",
        "CM_MIN_POINTS": str(int(cm.get("min_points", 5))),
        "SCAN_MIN_HEIGHT": f"{float(cfg['ros']['scan']['min_height']):.2f}",
        "SCAN_MAX_HEIGHT": f"{float(cfg['ros']['scan']['max_height']):.2f}",
        "GLOBAL_UPDATE_HZ": "3.0",
        "MAP_YAML": str(map_path),
        "BASE_FRAME": cfg["ros"]["frames"]["base"],
        "FOOTPRINT": footprint,
        "MAX_VEL_X": f"{float(nav['max_linear_speed']):.3f}",
        "MAX_VEL_THETA": f"{float(nav['max_angular_speed']):.3f}",
        "XY_TOLERANCE": f"{float(nav['goal_tolerance']):.3f}",
        # How long Nav2 tolerates the robot not moving before it gives
        # up on the goal.
        #
        # Derived from the policy's own cap on waiting, because under
        # this study's premise nobody steps out of the robot's way: a
        # Comfort Space across the aisle has to be waited out, and the
        # yield logic itself gives up after max_wait_seconds and looks
        # for another way round. Nav2 has to outlast that, or every
        # deliberate wait becomes an aborted goal. Measured at the
        # template's old 15 s: the robot waited 38 s for people, which
        # is the policy working as designed, and Nav2 reported ABORTED
        # 7.3 m from the goal.
        #
        # Our own deadlock detector is what should make this call -- it
        # knows the difference between standing still for a person and
        # standing still for no reason -- so this is set past the point
        # where it will have spoken.
        "PROGRESS_ALLOWANCE": f"{max_wait + 25.0:.1f}",
        # Orientation at the goal is not part of what this study measures,
        # so it is left loose. A tight yaw tolerance would turn "arrived
        # but facing the wrong way" into a failure.
        # Loose unless the goal states a heading. See the note above
        # the default: orientation at the goal is not part of what this
        # study measures, EXCEPT where the goal is a parking slot the
        # robot has to be able to drive out of again.
        "YAW_TOLERANCE": ("3.15" if nav.get("goal_yaw_deg") is None
                          else f"{float(nav.get('goal_yaw_tolerance', 0.35)):.2f}"),
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

    template_path = resolve_path(args.template or default_template(cfg))
    template = Template(template_path.read_text(encoding="utf-8"))
    try:
        rendered = template.substitute(values)
    except KeyError as exc:
        print(f"[ERROR] template uses ${exc.args[0]} but nothing supplies it", file=sys.stderr)
        return 1

    out_path = resolve_path(args.out)
    out_path.write_text(
        "# GENERATED FILE. Do not edit.\n"
        f"# Written by scripts/generate_nav2_params.py from {args.config}\n"
        f"# and {template_path.name}. Edit those instead.\n"
        + rendered,
        encoding="utf-8",
    )

    print(f"wrote {out_path}")
    for k, v in values.items():
        print(f"      {k:18s} = {v}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
