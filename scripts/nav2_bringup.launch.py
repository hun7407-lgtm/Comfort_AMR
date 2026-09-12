"""Bring up Nav2 for the Comfort_AMR corridor.

Run through scripts/run_nav2.sh, which regenerates the map and the
parameters first so they cannot drift from configs/common.yaml.

Deliberately smaller than nav2_bringup's own navigation_launch.py, which
also starts the smoother, route, waypoint follower, collision monitor,
docking and velocity smoother. Every extra node is one more thing that
has to be configured correctly or the whole lifecycle refuses to
activate, and none of them affect what this study measures.

No AMCL. The simulation knows exactly where the robot started, so
map -> odom is a static transform taken from navigation.start. That
keeps localisation error out of a comparison that is meant to isolate
the Comfort Space policy.

Nodes started here:

    comfort_publisher         Comfort Space -> costmap filter mask
    comfort_gate              /cmd_vel_nav  ->  /cmd_vel_safe, yields
    collision_monitor         /cmd_vel_safe ->  /cmd_vel, stops on contact
    pointcloud_to_laserscan   /lidar/cloud  ->  /scan
    static_transform_publisher  map -> odom
    map_server                the corridor, generated from config
    planner_server            NavFn global plan
    controller_server         DWB local control  ->  /cmd_vel
    behavior_server           spin / backup / wait recoveries
    bt_navigator              NavigateToPose action server
    lifecycle_manager         configures and activates the above
"""

import math
import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core import load_config  # noqa: E402

MANAGED_NODES = [
    "map_server",
    "planner_server",
    "controller_server",
    "behavior_server",
    "bt_navigator",
    "collision_monitor",
]


def generate_launch_description():
    config_path = os.environ.get("COMFORT_AMR_CONFIG", "configs/common.yaml")
    scenario = os.environ.get("COMFORT_AMR_SCENARIO", "") or None
    cfg = load_config(scenario=scenario, common_path=config_path)

    params_file = str(PROJECT_ROOT / "configs" / "nav2" / "nav2_params.yaml")

    # map -> odom. The odom frame's origin is wherever the robot spawned,
    # which is navigation.start with the configured heading.
    start_x, start_y = (float(v) for v in cfg["navigation"]["start"])
    start_yaw = math.radians(float(cfg["amr"]["spawn"]["yaw_deg"]))

    frames = cfg["ros"]["frames"]
    lidar_frame = frames["lidar"]

    use_sim_time = LaunchConfiguration("use_sim_time")

    common = {"use_sim_time": use_sim_time}

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="follow Isaac Sim's /clock rather than wall time"),

        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            output="screen",
            remappings=[("cloud_in", cfg["ros"]["topics"]["lidar_cloud"]),
                        ("scan", cfg["ros"]["topics"]["scan"])],
            parameters=[{
                **common,
                "target_frame": lidar_frame,
                "transform_tolerance": 0.05,
                # The LiDAR sits 0.31 m above the floor. Take a slab
                # around that height so the walls and the humans are
                # seen but the ground plane is not.
                "min_height": -0.20,
                "max_height": 1.20,
                "angle_min": -math.pi,
                "angle_max": math.pi,
                "angle_increment": 0.0087,
                "scan_time": 0.1,
                "range_min": 0.20,
                "range_max": 20.0,
                "use_inf": True,
            }],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom",
            output="screen",
            arguments=[
                "--x", str(start_x), "--y", str(start_y), "--z", "0.0",
                "--yaw", str(start_yaw), "--pitch", "0.0", "--roll", "0.0",
                "--frame-id", "map", "--child-frame-id", frames["odom"],
            ],
            parameters=[common],
        ),

        Node(package="nav2_map_server", executable="map_server",
             name="map_server", output="screen",
             parameters=[params_file, common]),

        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", output="screen",
             parameters=[params_file, common]),

        # Nav2's velocity goes to the gate, not straight to the robot.
        # The gate decides whether obeying it would intrude on someone.
        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen",
             parameters=[params_file, common],
             remappings=[("cmd_vel", "cmd_vel_nav")]),

        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen",
             parameters=[params_file, common],
             remappings=[("cmd_vel", "cmd_vel_nav")]),

        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen",
             parameters=[params_file, common]),

        # Last thing between navigation and the wheels. It reads the
        # laser, not the costmap, so the Comfort Space is invisible to
        # it: this is about not hitting anything, which is a different
        # question from not crowding anyone.
        Node(package="nav2_collision_monitor", executable="collision_monitor",
             name="collision_monitor", output="screen",
             parameters=[params_file, common]),

        ExecuteProcess(
            cmd=[sys.executable,
                 str(PROJECT_ROOT / "scripts" / "comfort_publisher.py"),
                 "--config", config_path, "--scenario", str(scenario)],
            output="screen",
        ),

        ExecuteProcess(
            cmd=[sys.executable,
                 str(PROJECT_ROOT / "scripts" / "comfort_gate.py"),
                 "--config", config_path, "--scenario", str(scenario),
                 "--out-topic", "/cmd_vel_safe"],
            output="screen",
        ),

        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_navigation", output="screen",
            parameters=[{
                **common,
                "autostart": True,
                "node_names": MANAGED_NODES,
                # Isaac Sim's first frames are slow; do not give up early.
                "bond_timeout": 20.0,
            }],
        ),
    ])
