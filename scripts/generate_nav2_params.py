#!/usr/bin/env python3
"""Fill configs/nav2/nav2_params.template.yaml from configs/common.yaml.

Nav2 needs the AMR's footprint, speed limits and goal tolerance, and so
does the rest of the experiment. Keeping two copies by hand is how the
planner ends up believing the robot is a different size than the passage
calculation assumes. The template holds every Nav2 setting that is
genuinely Nav2's business; everything shared comes from here.

    ./scripts/generate_nav2_params.py
    ./scripts/generate_nav2_params.py --config configs/demo_weave.yaml
    ./scripts/generate_nav2_params.py --config configs/fleet_nav2.yaml --namespace a1

With --namespace the same template is rendered for one fleet AMR: its
frames carry the prefix (a1/odom, a1/base_footprint), its topics the
namespace (/a1/scan, /a1/odom), and the whole file is nested under the
namespace key, which is how a node at /a1/controller_server finds its
parameters. Every number is the same formula as AMR1 --
they are the same vehicle. The Comfort Space filter topic is left
global on purpose: one mask, published once, read by every robot.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from string import Template

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core import load_config, nav2_map, resolve_path
from comfort_core import nav2_tuning
from comfort_core.frames import namespaced_topic, prefixed


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
    parser.add_argument("--out", default=None,
                        help="defaults to configs/nav2/nav2_params.yaml, or "
                             "nav2_params_<namespace>.yaml with --namespace")
    parser.add_argument("--namespace", default="",
                        help="render for one fleet AMR (e.g. a1) instead of "
                             "AMR1")
    parser.add_argument("--map", default=None,
                        help="map yaml; defaults to the one for this scene")
    args = parser.parse_args()

    cfg = load_config(common_path=args.config)
    nav = cfg["navigation"]
    ns = args.namespace.strip("/")
    if ns and ns not in cfg.get("fleet", {}).get("amrs", {}):
        print(f"[ERROR] {args.config} has no fleet AMR called {ns!r}",
              file=sys.stderr)
        return 1
    frames = cfg["ros"]["frames"]
    topics = cfg["ros"]["topics"]
    traffic = cfg.get("traffic_rule", {}) or {}
    traffic_on = bool(traffic.get("enabled", False))

    half_len, half_wid = nav2_tuning.half_extents(cfg)
    footprint = nav2_tuning.footprint_polygon(cfg)

    # Inflation, the DWB horizon, the critic weights and the observation
    # ranges all come from comfort_core/nav2_tuning.py, which the real
    # AMR's generator calls as well. Two robots, one formula.
    inflation = nav2_tuning.inflation_radius(cfg)
    horizon = nav2_tuning.dwb_horizon(cfg)
    scales = nav2_tuning.critic_scales(cfg)
    obstacle_range, raytrace_range = nav2_tuning.observation_ranges(cfg)

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
    band_mid = 0.60
    v_slow = v_cruise * 0.25
    v_mid = v_cruise * band_mid

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
    # The warning field. Slowing to the mid band's top speed puts the
    # vehicle inside the protective field sized for that band, so the
    # field has to reach as far as it travels while slowing, as far as
    # a walking person closes in the meantime, and the mid-band stop
    # field beyond that. configs/common.yaml has the derivation.
    slow_cfg = cm.get("slowdown", {}) or {}
    slowdown_on = bool(slow_cfg.get("enabled", False))
    approach = float(slow_cfg.get("approach_speed", 1.6))
    t_slow = reaction + (v_cruise - v_mid) / decel
    slowdown_reach = (reaction * v_cruise
                      + (v_cruise ** 2 - v_mid ** 2) / (2.0 * decel)
                      + approach * t_slow
                      + reach(v_mid))
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
        "CM_SLOWDOWN_ENABLED": "True" if slowdown_on else "False",
        "CM_SLOWDOWN_RATIO": f"{band_mid:.2f}",
        "CM_FIELD_SLOWDOWN": (f"[[{slowdown_reach:.3f}, {sy:.3f}], "
                              f"[{slowdown_reach:.3f}, {-sy:.3f}], "
                              f"[{-half_len:.3f}, {-sy:.3f}], "
                              f"[{-half_len:.3f}, {sy:.3f}]]"),
        "CM_TIME_BEFORE_COLLISION": f"{approach_time:.2f}",
        "CM_SIM_STEP": f"{float(cm.get('simulation_time_step', 0.1)):.2f}",
        "CM_MIN_POINTS": str(int(cm.get("min_points", 5))),
        "SCAN_MIN_HEIGHT": f"{float(cfg['ros']['scan']['min_height']):.2f}",
        "SCAN_MAX_HEIGHT": f"{float(cfg['ros']['scan']['max_height']):.2f}",
        "GLOBAL_UPDATE_HZ": "3.0",
        "MAP_YAML": str(map_path),
        "BASE_FRAME": prefixed(ns, frames["base"]),
        "ODOM_FRAME": prefixed(ns, frames["odom"]),
        "ODOM_TOPIC": namespaced_topic(ns, topics["odom"]),
        "SCAN_TOPIC": namespaced_topic(ns, topics["scan"]),
        "FOOTPRINT_TOPIC": namespaced_topic(
            ns, "/local_costmap/published_footprint"),
        # Keep right: a second filter per robot, reading that robot's
        # own mask. Listed only when the rule is on.
        "COSTMAP_FILTERS": ('["comfort_filter", "traffic_filter"]'
                            if traffic_on else '["comfort_filter"]'),
        # The filter's own block is rendered only when the rule is on,
        # so an off rule leaves the file byte-identical to before it
        # existed.
        "TRAFFIC_FILTER_BLOCK": (
            "      # Keep right (traffic_rule in configs/common.yaml): a second\n"
            "      # filter reading this robot's own mask.\n"
            "      traffic_filter:\n"
            "        plugin: \"nav2_costmap_2d::KeepoutFilter\"\n"
            "        enabled: True\n"
            "        filter_info_topic: \""
            + namespaced_topic(ns, "/" + str(traffic.get("info_topic", "traffic_filter_info")))
            + "\"\n"
        ) if traffic_on else "",
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
        "COST_SCALING": f"{nav2_tuning.cost_scaling_factor(cfg):.1f}",
        "OBSTACLE_MAX_RANGE": f"{obstacle_range:.1f}",
        "RAYTRACE_MAX_RANGE": f"{raytrace_range:.1f}",
        "SIM_TIME": f"{horizon['sim_time']:.2f}",
        "LINEAR_GRANULARITY": f"{horizon['linear_granularity']:.3f}",
        "ANGULAR_GRANULARITY": f"{horizon['angular_granularity']:.3f}",
        "VX_SAMPLES": str(horizon["vx_samples"]),
        "VTHETA_SAMPLES": str(horizon["vtheta_samples"]),
        # DWB's obstacle critic, the only one that reads the Comfort
        # Space. Derived, not set: it has to be solved against the
        # trajectory horizon, which is speed-dependent, so the simulated
        # and the real AMR need different numbers to behave the same way.
        # comfort_core/nav2_tuning.py:base_obstacle_scale has the
        # derivation and the dwb_critics header lines it was checked
        # against.
        "BASE_OBSTACLE_SCALE": f"{scales['BaseObstacle.scale']:.6f}",
        "BASE_OBSTACLE_SUM_SCORES": (
            "True" if nav2_tuning.base_obstacle_sum_scores(cfg) else "False"),
        "PATH_DIST_SCALE": f"{scales['PathDist.scale']:.1f}",
        "PATH_ALIGN_SCALE": f"{scales['PathAlign.scale']:.1f}",
        "GOAL_DIST_SCALE": f"{scales['GoalDist.scale']:.1f}",
        "GOAL_ALIGN_SCALE": f"{scales['GoalAlign.scale']:.1f}",
        "ROTATE_TO_GOAL_SCALE": f"{scales['RotateToGoal.scale']:.1f}",
    }

    template_path = resolve_path(args.template or default_template(cfg))
    template = Template(template_path.read_text(encoding="utf-8"))
    try:
        rendered = template.substitute(values)
    except KeyError as exc:
        print(f"[ERROR] template uses ${exc.args[0]} but nothing supplies it", file=sys.stderr)
        return 1

    if ns:
        # One level deeper, under the namespace: the same thing
        # nav2_bringup's RewrittenYaml(root_key=...) does at launch time,
        # done here so the file on disk is the file Nav2 reads.
        rendered = f"{ns}:\n" + "".join(
            ("  " + line) if line.strip() else line
            for line in rendered.splitlines(keepends=True))

    out = args.out or (f"configs/nav2/nav2_params_{ns}.yaml" if ns
                       else "configs/nav2/nav2_params.yaml")
    out_path = resolve_path(out)
    out_path.write_text(
        "# GENERATED FILE. Do not edit.\n"
        f"# Written by scripts/generate_nav2_params.py from {args.config}\n"
        f"# and {template_path.name}. Edit those instead.\n"
        + (f"# Fleet AMR {ns}: prefixed frames, namespaced topics.\n" if ns else "")
        + rendered,
        encoding="utf-8",
    )

    print(f"wrote {out_path}")
    print(nav2_tuning.summary(
        cfg, f"fleet AMR {ns}, from {args.config}" if ns
        else f"simulated AMR, from {args.config}"))
    for k, v in values.items():
        print(f"      {k:24s} = {v}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
