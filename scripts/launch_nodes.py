"""Launch node descriptions shared by the 2D and 3D bringups.

Both nav2_bringup.launch.py and nav3d/mapping.launch.py need the LiDAR
flattened into a `sensor_msgs/LaserScan`: Nav2's static layer and
slam_toolbox are both 2D. Describing that node twice is how the mapping
run ends up sampling a different slab of the cloud than the trials do,
and a map that disagrees with what the robot sees while navigating is
worse than no map.

Imported by launch files, so it may use launch_ros. It must not import
omni or pxr: this runs in the system Python with system ROS 2.
"""

from __future__ import annotations

import math
from typing import Any, Dict

from launch_ros.actions import Node


def laserscan_node(cfg: Dict[str, Any], common: Dict[str, Any]) -> Node:
    """`/lidar/cloud` -> `/scan`, flattened around the LiDAR's height.

    The slab is taken relative to lidar_link, which sits
    `ros.scan.lidar_world_z` above the floor, so the walls, the machines
    and the people are all in it and the floor itself is not.
    """
    scan = cfg["ros"]["scan"]
    topics = cfg["ros"]["topics"]
    return Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        remappings=[("cloud_in", topics["lidar_cloud"]),
                    ("scan", topics["scan"])],
        parameters=[{
            **common,
            "target_frame": cfg["ros"]["frames"]["lidar"],
            "transform_tolerance": 0.05,
            "min_height": float(scan["min_height"]),
            "max_height": float(scan["max_height"]),
            "angle_min": -math.pi,
            "angle_max": math.pi,
            "angle_increment": float(scan["angle_increment"]),
            "scan_time": 0.1,
            "range_min": float(scan["range_min"]),
            "range_max": float(scan["range_max"]),
            "use_inf": True,
        }],
    )
