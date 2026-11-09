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
from typing import Any, Dict, List

from launch_ros.actions import Node

from comfort_core.frames import namespaced_topic, prefixed


def laserscan_node(cfg: Dict[str, Any], common: Dict[str, Any],
                   namespace: str = "") -> Node:
    """`/lidar/cloud` -> `/scan`, flattened around the LiDAR's height.

    The slab is taken relative to lidar_link, which sits
    `ros.scan.lidar_world_z` above the floor, so the walls, the machines
    and the people are all in it and the floor itself is not.

    With a namespace it is one fleet AMR's: `/a1/lidar/cloud` ->
    `/a1/scan` in `a1/lidar_link`, the same slab of the same sensor.
    """
    scan = cfg["ros"]["scan"]
    topics = cfg["ros"]["topics"]
    return Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        remappings=[("cloud_in", namespaced_topic(namespace, topics["lidar_cloud"])),
                    ("scan", namespaced_topic(namespace, topics["scan"]))],
        parameters=[{
            **common,
            "target_frame": prefixed(namespace, cfg["ros"]["frames"]["lidar"]),
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


MANAGED_NODES = [
    "map_server",
    "planner_server",
    "controller_server",
    "behavior_server",
    "bt_navigator",
    "collision_monitor",
]


def nav2_stack(cfg: Dict[str, Any], common: Dict[str, Any], params_file: str,
               start: tuple, namespace: str = "") -> List[Node]:
    """One robot's Nav2: the nodes the lifecycle manager brings up.

    The same list for AMR1 and for a fleet AMR. The one
    difference is what the controller's velocity is called on its way
    out:

        AMR1   cmd_vel_nav -> comfort_gate -> cmd_vel_safe
        fleet AMR          cmd_vel_safe

    because the gate is the policy being measured, and the fleet does
    not run it. Both end at the collision monitor, which is the safety
    layer and is never left out.

    No tf remapping. tf2_ros subscribes to the absolute `/tf`, so a
    node pushed into a namespace still reads the one global tree; the
    fleet AMRs are kept apart there by frame prefix (a1/odom), which is
    what `params_file` was rendered with. nav2_bringup remaps /tf into
    the namespace to get the opposite, one tree per robot -- that is
    not wanted here, the simulator publishes one tree.
    """
    frames = cfg["ros"]["frames"]
    start_x, start_y, start_yaw = start
    cmd_out = "cmd_vel_safe" if namespace else "cmd_vel_nav"
    return [
        laserscan_node(cfg, common, namespace),

        # map -> odom. The odom frame's origin is wherever the robot
        # spawned, which is the start pose with the configured heading.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom",
            output="screen",
            arguments=[
                "--x", str(start_x), "--y", str(start_y), "--z", "0.0",
                "--yaw", str(start_yaw), "--pitch", "0.0", "--roll", "0.0",
                "--frame-id", "map",
                "--child-frame-id", prefixed(namespace, frames["odom"]),
            ],
            parameters=[common],
        ),

        Node(package="nav2_map_server", executable="map_server",
             name="map_server", output="screen",
             parameters=[params_file, common]),

        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", output="screen",
             parameters=[params_file, common]),

        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen",
             parameters=[params_file, common],
             remappings=[("cmd_vel", cmd_out)]),

        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen",
             parameters=[params_file, common],
             remappings=[("cmd_vel", cmd_out)]),

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
    ]
