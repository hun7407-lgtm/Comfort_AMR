#!/usr/bin/env python3
"""Humble Nav2 params for the real AMR, from Humble's own defaults.

Written on the robot rather than ported from configs/nav2/nav2_params.template.yaml
because the plugin names differ between distros: Humble wants
"nav2_navfn_planner/NavfnPlanner", Jazzy "nav2_navfn_planner::NavfnPlanner",
and Jazzy's `navigators:` block does not exist in Humble at all. Starting from
the installed defaults means only the things this robot actually changes are
changed.

What it changes:
  footprint      the real 0.864 x 0.570 m, not the 0.22 m default disc
  speed          0.2 m/s -- the LiDAR is blind below 0.31 m and within ~1 m,
                 and this robot publishes no ultrasonic topic
  comfort_filter a KeepoutFilter on BOTH costmaps, reading the Comfort Space
                 mask that scripts/comfort_publisher.py paints
"""
import sys, yaml, copy

SRC = "/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml"
OUT = sys.argv[1] if len(sys.argv) > 1 else "/home/clabil/nav2_real.yaml"

FOOTPRINT = "[[0.432,0.285],[0.432,-0.285],[-0.432,-0.285],[-0.432,0.285]]"
MAX_VEL_X = 0.20
MAX_VEL_THETA = 0.40

cfg = yaml.safe_load(open(SRC))

FILTER = {
    "plugin": "nav2_costmap_2d::KeepoutFilter",
    "enabled": True,
    "filter_info_topic": "/costmap_filter_info",
}

for name in ("local_costmap", "global_costmap"):
    p = cfg[name][name]["ros__parameters"]
    p.pop("robot_radius", None)
    p["footprint"] = FOOTPRINT
    # Filters run after the plugins, so the Comfort Space cost is laid on
    # top of the obstacles rather than being inflated again.
    p["filters"] = ["comfort_filter"]
    p["comfort_filter"] = copy.deepcopy(FILTER)
    # Humble ships obstacle_max_range 2.5 m and raytrace_max_range 3.0 m.
    # This LiDAR reaches 12 m, and with the defaults the costmap is only
    # refreshed within about 3 m of the robot -- CostmapFilter does not widen
    # the update bounds itself, so a Comfort Space painted further away than
    # that never reached the costmap at all. Measured: a mask correctly
    # published at x=5.28 left costmap=0 there, while the same mask at x=2.24
    # came through at 96.
    for lname in ("obstacle_layer", "voxel_layer"):
        if lname in p and isinstance(p[lname], dict):
            p[lname]["obstacle_max_range"] = 10.0
            p[lname]["raytrace_max_range"] = 12.0
            src = p[lname].get("observation_sources", "")
            for sname in str(src).split():
                if sname in p[lname]:
                    p[lname][sname]["obstacle_max_range"] = 10.0
                    p[lname][sname]["raytrace_max_range"] = 12.0
                    p[lname][sname]["obstacle_min_range"] = 0.0
                    p[lname][sname]["raytrace_min_range"] = 0.0
    if "inflation_layer" in p:
        # 0.35 m, not the 0.55 m default. The robot's inscribed radius is
        # 0.285 m and inflation cannot go below that -- inside it the cost is
        # lethal whatever this says -- so 0.35 leaves a 6.5 cm decay band and
        # no more. In a 2.2 m corridor a wide inflation eats the corridor
        # itself; at 0.45 the two walls left 1.45 m for the robot centre.
        p["inflation_layer"]["inflation_radius"] = 0.35
        p["inflation_layer"]["cost_scaling_factor"] = 5.0

# The global costmap takes its size from the SLAM map on /map.
g = cfg["global_costmap"]["global_costmap"]["ros__parameters"]
g["rolling_window"] = False
g["track_unknown_space"] = True

c = cfg["controller_server"]["ros__parameters"]
c["FollowPath"]["max_vel_x"] = MAX_VEL_X
c["FollowPath"]["max_speed_xy"] = MAX_VEL_X
c["FollowPath"]["max_vel_theta"] = MAX_VEL_THETA
c["FollowPath"]["min_speed_xy"] = 0.0

yaml.safe_dump(cfg, open(OUT, "w"), default_flow_style=False, sort_keys=False)
print("wrote", OUT)
for n in ("local_costmap", "global_costmap"):
    p = cfg[n][n]["ros__parameters"]
    print(f"  {n}: footprint={p['footprint']} filters={p['filters']}")
