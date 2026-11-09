"""Bring up Nav2 for the Comfort_AMR corridor.

Run through scripts/run_nav2.sh, which regenerates the map and the
parameters first so they cannot drift from configs/common.yaml.

Deliberately smaller than nav2_bringup's own navigation_launch.py, which
also starts the smoother, route, waypoint follower, docking and velocity
smoother. Every extra node is one more thing that has to be configured
correctly or the whole lifecycle refuses to activate, and none of them
affect what this study measures.

No AMCL. The simulation knows exactly where the robot started, so
map -> odom is a static transform taken from navigation.start. That
keeps localisation error out of a comparison that is meant to isolate
the Comfort Space policy.

Nodes started here, for AMR1:

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

With COMFORT_AMR_NAMESPACE=a1 (scripts/run_nav2.sh --namespace a1) the
same stack comes up under /a1 for one fleet AMR, reading
configs/nav2/nav2_params_a1.yaml, with map -> a1/odom at that vehicle's
spawn pose. Three things are left out on purpose:

    comfort_publisher   one mask for everyone; AMR1
                        publishes it and the fleet's filters read the
                        same global /costmap_filter_info
    people_tracker      same reason
    comfort_gate        it is the policy being measured, and the fleet
                        is traffic, not the subject

so the controller's output goes to /a1/cmd_vel_safe and from there
through the collision monitor to /a1/cmd_vel. The safety layer is the
one thing a fleet vehicle is never without.
"""

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import launch_nodes  # noqa: E402
from comfort_core import load_config  # noqa: E402
from comfort_core.frames import fleet_spawn, odom_origin_in_world  # noqa: E402


def generate_launch_description():
    config_path = os.environ.get("COMFORT_AMR_CONFIG", "configs/common.yaml")
    scenario = os.environ.get("COMFORT_AMR_SCENARIO", "") or None
    namespace = os.environ.get("COMFORT_AMR_NAMESPACE", "").strip("/")
    cfg = load_config(scenario=scenario, common_path=config_path)
    perception_source = str(cfg.get("perception", {}).get("source", "tf"))

    use_sim_time = LaunchConfiguration("use_sim_time")
    common = {"use_sim_time": use_sim_time}

    declare = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="follow Isaac Sim's /clock rather than wall time")

    if namespace:
        params_file = str(PROJECT_ROOT / "configs" / "nav2"
                          / f"nav2_params_{namespace}.yaml")
        stack = launch_nodes.nav2_stack(
            cfg, common, params_file, fleet_spawn(cfg, namespace), namespace)
        return LaunchDescription([
            declare,
            GroupAction([PushRosNamespace(namespace), *stack]),
        ])

    params_file = str(PROJECT_ROOT / "configs" / "nav2" / "nav2_params.yaml")
    stack = launch_nodes.nav2_stack(
        cfg, common, params_file, odom_origin_in_world(cfg))

    return LaunchDescription([
        declare,
        *stack,

        # Both of these run on simulated time, like every Nav2 node
        # above. Without it their timers run on the wall clock: with
        # recording on, the simulation drops to a third of real time,
        # the gate's 20 Hz evade loop becomes 6 Hz in simulated time,
        # and the robot crawls out of the way too late to matter. That
        # is a recording artefact changing a result, which is worse
        # than a slow recording.
        ExecuteProcess(
            cmd=[sys.executable,
                 str(PROJECT_ROOT / "scripts" / "comfort_publisher.py"),
                 "--config", config_path, "--scenario", str(scenario),
                 "--ros-args", "-p", "use_sim_time:=true"],
            output="screen",
        ),

        # The person detector, when the control loop is fed by the
        # sensor rather than by the simulator's own poses. It reads the
        # 3D cloud, finds people by their vertical profile and
        # publishes them on /comfort_detections, which is where
        # comfort_publisher then looks. Started here so the two can
        # never disagree about whether perception is in the loop.
        *([ExecuteProcess(
            cmd=[sys.executable,
                 str(PROJECT_ROOT / "scripts" / "people_tracker.py"),
                 "--config", config_path, "--scenario", str(scenario),
                 "--ros-args", "-p", "use_sim_time:=true"],
            output="screen",
        )] if perception_source == "lidar" else []),

        # Keep right, when the config turns it on: one process paints a
        # mask per robot (AMR1 and every fleet vehicle) from each one's
        # plan. Started once, here, like the Comfort Space mask.
        *([ExecuteProcess(
            cmd=[sys.executable,
                 str(PROJECT_ROOT / "scripts" / "traffic_rule.py"),
                 "--config", config_path, "--scenario", str(scenario),
                 "--ros-args", "-p", "use_sim_time:=true"],
            output="screen",
        )] if cfg.get("traffic_rule", {}).get("enabled", False) else []),

        # Nav2's velocity goes to the gate, not straight to the robot.
        # The gate decides whether obeying it would intrude on someone.
        ExecuteProcess(
            cmd=[sys.executable,
                 str(PROJECT_ROOT / "scripts" / "comfort_gate.py"),
                 "--config", config_path, "--scenario", str(scenario),
                 "--out-topic", "/cmd_vel_safe",
                 "--ros-args", "-p", "use_sim_time:=true"],
            output="screen",
        ),
    ])
