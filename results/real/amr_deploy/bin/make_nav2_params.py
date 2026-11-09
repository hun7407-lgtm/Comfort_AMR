#!/usr/bin/env python3
"""Humble Nav2 params for the real AMR, from Humble's own defaults.

Built on the robot rather than ported from
configs/nav2/nav2_params.template.yaml because the plugin names differ
between distros: Humble wants "nav2_navfn_planner/NavfnPlanner", Jazzy
"nav2_navfn_planner::NavfnPlanner", and Jazzy's `navigators:` block does
not exist in Humble at all. Starting from the installed defaults means
only the things this robot actually changes are changed.

The VALUES, though, are not this script's to choose. They come from
comfort_core/nav2_tuning.py in the repo at ~/comfort, which the
simulator's generator calls as well, so a tuning change cannot land on
one machine and not the other. The two robots are different sizes and
drive at different speeds, so the same formula gives them different
numbers -- that is the point, and it is why this script no longer holds a
footprint or a speed of its own.

    ./make_nav2_params.py                       # -> ~/nav2_real.yaml
    ./make_nav2_params.py ~/nav2_real.yaml
    ./make_nav2_params.py --config configs/real_corridor.yaml

What it changes, and where each value comes from:

  footprint      configs/real_amr.yaml, measured 0.864 x 0.570 m, not
                 Humble's 0.22 m default disc
  speed          configs/real_amr.yaml, 0.20 m/s -- the LiDAR is blind
                 below 0.31 m and within ~1 m, and this robot publishes
                 no ultrasonic topic
  observation    configs/real_amr.yaml, 10 / 12 m against Humble's
                 2.5 / 3.0, because the Comfort Space mask is bounded by
                 these
  inflation      derived, pinned to 0.35 m for this corridor
  DWB critics    derived, including BaseObstacle.sum_scores -- without it
                 DWB scores the final trajectory pose only and a Comfort
                 Space the trajectory crosses and leaves costs nothing
  comfort_filter a KeepoutFilter on BOTH costmaps, reading the Comfort
                 Space mask that scripts/comfort_publisher.py paints
"""
import argparse
import copy
import sys
from pathlib import Path

import yaml

DEFAULT_SRC = "/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml"
DEFAULT_OUT = "/home/clabil/nav2_real.yaml"
DEFAULT_REPO = Path.home() / "comfort"
DEFAULT_CONFIG = "configs/real_amr.yaml"

FILTER = {
    "plugin": "nav2_costmap_2d::KeepoutFilter",
    "enabled": True,
    "filter_info_topic": "/costmap_filter_info",
}


def load_tuning(repo: Path, config: str):
    """The repo is the single source of these numbers, so a missing repo is
    a hard error. Falling back to built-in constants is exactly the
    divergence this script was rewritten to remove."""
    if not (repo / "comfort_core" / "nav2_tuning.py").is_file():
        sys.exit(
            f"[ERROR] no comfort_core/nav2_tuning.py under {repo}\n"
            f"        This script takes the footprint, the speed and the DWB\n"
            f"        tuning from the repo so the real robot and the simulator\n"
            f"        stay in step. Deploy the repo there, or pass --repo.")
    sys.path.insert(0, str(repo))
    from comfort_core import load_config
    from comfort_core import nav2_tuning

    cfg_path = Path(config)
    if not cfg_path.is_absolute():
        cfg_path = repo / config
    if not cfg_path.is_file():
        sys.exit(f"[ERROR] no config at {cfg_path}")
    return load_config(common_path=str(cfg_path)), nav2_tuning, cfg_path


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("out", nargs="?", default=DEFAULT_OUT)
    ap.add_argument("--src", default=DEFAULT_SRC,
                    help="Humble's installed defaults to start from")
    ap.add_argument("--repo", default=str(DEFAULT_REPO),
                    help="the Comfort_AMR checkout holding comfort_core")
    ap.add_argument("--config", default=DEFAULT_CONFIG,
                    help="platform config; a per-capture corridor config "
                         "based on it works too")
    args = ap.parse_args()

    cfg, tuning, cfg_path = load_tuning(Path(args.repo).expanduser(),
                                        args.config)

    footprint = tuning.footprint_polygon(cfg)
    obstacle_range, raytrace_range = tuning.observation_ranges(cfg)
    inflation = tuning.inflation_radius(cfg)
    scaling = tuning.cost_scaling_factor(cfg)
    max_vel_x, max_vel_theta = tuning.speed_limits(cfg)
    horizon = tuning.dwb_horizon(cfg)
    scales = tuning.critic_scales(cfg)

    params = yaml.safe_load(open(args.src))

    for name in ("local_costmap", "global_costmap"):
        p = params[name][name]["ros__parameters"]
        p.pop("robot_radius", None)
        p["footprint"] = footprint
        # Filters run after the plugins, so the Comfort Space cost is laid
        # on top of the obstacles rather than being inflated again.
        p["filters"] = ["comfort_filter"]
        p["comfort_filter"] = copy.deepcopy(FILTER)
        # Humble ships obstacle_max_range 2.5 m and raytrace_max_range
        # 3.0 m. This LiDAR reaches 12 m, and with the defaults the costmap
        # is only refreshed within about 3 m of the robot -- CostmapFilter
        # does not widen the update bounds itself, so a Comfort Space
        # painted further away than that never reached the costmap at all.
        # Measured: a mask correctly published at x=5.28 left costmap=0
        # there, while the same mask at x=2.24 came through at 96.
        for lname in ("obstacle_layer", "voxel_layer"):
            if lname in p and isinstance(p[lname], dict):
                p[lname]["obstacle_max_range"] = obstacle_range
                p[lname]["raytrace_max_range"] = raytrace_range
                src = p[lname].get("observation_sources", "")
                for sname in str(src).split():
                    if sname in p[lname]:
                        p[lname][sname]["obstacle_max_range"] = obstacle_range
                        p[lname][sname]["raytrace_max_range"] = raytrace_range
                        p[lname][sname]["obstacle_min_range"] = 0.0
                        p[lname][sname]["raytrace_min_range"] = 0.0
        if "inflation_layer" in p:
            # Not Humble's 0.55 m default. Below the robot's 0.285 m
            # inscribed radius the cost is lethal whatever this says, and a
            # wide inflation eats a 2.2 m corridor outright, so this is
            # pinned in configs/real_amr.yaml with the reason.
            p["inflation_layer"]["inflation_radius"] = inflation
            p["inflation_layer"]["cost_scaling_factor"] = scaling

    # The global costmap takes its size from the SLAM map on /map.
    g = params["global_costmap"]["global_costmap"]["ros__parameters"]
    g["rolling_window"] = False
    g["track_unknown_space"] = True

    follow = params["controller_server"]["ros__parameters"]["FollowPath"]
    plugin = str(follow.get("plugin", ""))
    follow["max_vel_x"] = max_vel_x
    follow["max_speed_xy"] = max_vel_x
    follow["max_vel_theta"] = max_vel_theta
    follow["min_speed_xy"] = 0.0

    if "DWBLocalPlanner" not in plugin:
        # Nothing below applies to another controller, and writing DWB keys
        # into one would be silently ignored. Say so instead.
        print(f"[WARN] FollowPath is {plugin!r}, not DWB -- critic weights "
              f"and the trajectory horizon were NOT written. The Comfort "
              f"Space tuning does not transfer to this controller.",
              file=sys.stderr)
    else:
        follow["sim_time"] = horizon["sim_time"]
        follow["linear_granularity"] = horizon["linear_granularity"]
        follow["angular_granularity"] = horizon["angular_granularity"]
        follow["vx_samples"] = horizon["vx_samples"]
        follow["vtheta_samples"] = horizon["vtheta_samples"]
        # BaseObstacle is the only critic that reads the Comfort Space, and
        # with sum_scores off it reads the final trajectory pose only --
        # dwb_critics/base_obstacle.hpp. A trajectory that crosses someone's
        # space and ends clear of it then costs nothing.
        follow["BaseObstacle.sum_scores"] = tuning.base_obstacle_sum_scores(cfg)
        for key, value in scales.items():
            follow[key] = value

    yaml.safe_dump(params, open(args.out, "w"),
                   default_flow_style=False, sort_keys=False)
    print("wrote", args.out)
    print(tuning.summary(cfg, f"real AMR, from {cfg_path}"))
    for name in ("local_costmap", "global_costmap"):
        p = params[name][name]["ros__parameters"]
        print(f"  {name}: footprint={p['footprint']} filters={p['filters']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
