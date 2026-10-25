"""Bring up slam_toolbox so the robot can map the bay it will work in.

Run through scripts/run_mapping.sh, which starts Isaac Sim first, drives
the lap, saves the map and puts it in world coordinates. Nothing else
needs this launch file.

What runs here
--------------

    pointcloud_to_laserscan   /lidar/cloud -> /scan, the shared slab
    slam_toolbox (async)      /scan + odom -> /map, and map -> odom

Deliberately NOT here
---------------------

No static map -> odom publisher. During a trial that transform is a
constant taken from navigation.start, because the simulation knows where
the robot began. During mapping it is slam_toolbox's output -- the
correction between where odometry thinks the robot is and where the scans
say it is -- and two publishers on one transform is a TF tree that
flickers between them.

No Nav2, no planner, no Comfort Space. The lap is driven open-loop by
nav3d/patrol.py along the aisle centrelines, with nobody in the bay. A
planner would need a map, which is what we are here to make.

Why 2D SLAM from a 3D sensor
----------------------------

Nav2's static layer is a 2D occupancy grid; that is the interface, and a
3D map would have to be flattened to fit it. So the map is built from the
scan slab, while the costmaps -- see nav3d/nav2_params_3d.template.yaml --
read the full cloud and mark obstacles at the height they are really at.
The map answers "where is the building", the voxel layer answers "what is
in front of me now", and only the second one needs the third dimension.
"""

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

import launch_nodes  # noqa: E402
from comfort_core import load_config  # noqa: E402


def generate_launch_description():
    config_path = os.environ.get("COMFORT_AMR_CONFIG",
                                 "nav3d/mapping_factory_3d.yaml")
    cfg = load_config(common_path=config_path)

    frames = cfg["ros"]["frames"]
    scan = cfg["ros"]["scan"]
    slam_cfg = cfg.get("nav2", {}).get("slam", {})
    params_file = str(PROJECT_ROOT / "nav3d" / "slam_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    common = {"use_sim_time": use_sim_time}

    # These override the tuning file. They are not tuning: they are what
    # the rest of the project already says the robot and the sensor are.
    from_config = {
        "use_sim_time": use_sim_time,
        "use_lifecycle_manager": False,
        "odom_frame": frames["odom"],
        "base_frame": frames["base"],
        "map_frame": "map",
        "scan_topic": cfg["ros"]["topics"]["scan"],
        "max_laser_range": float(scan["range_max"]),
        # The same 0.05 m/px the generated maps use, so a 3D trial's
        # static layer has the same cell size as a 2D trial's.
        "resolution": float(slam_cfg.get("resolution", 0.05)),
    }

    slam = LifecycleNode(
        package="slam_toolbox", executable="async_slam_toolbox_node",
        name="slam_toolbox", namespace="", output="screen",
        parameters=[params_file, from_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="follow Isaac Sim's /clock rather than wall time"),

        launch_nodes.laserscan_node(cfg, common),

        slam,

        # slam_toolbox is a lifecycle node and does nothing until it is
        # configured and activated. Its own online_async_launch.py does
        # this the same way; it is repeated here rather than included
        # because that launch file would also pull in its default
        # parameters and its own idea of the frames.
        EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_CONFIGURE)),

        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=slam,
            start_state="configuring", goal_state="inactive",
            entities=[
                LogInfo(msg="slam_toolbox configured, activating"),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam),
                    transition_id=Transition.TRANSITION_ACTIVATE)),
            ])),
    ])
