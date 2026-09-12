#!/usr/bin/env bash

# ---------------------------------------------------------------
# Adapted from: https://github.com/dongtak2/AMR-Digital-Twin
#   original file : scripts/start_scan.sh
#   commit        : c54269ee80032c29b0548eb77c3ee38d68b87e78 (2026-09-11)
#   modifications : ROS distro humble -> jazzy
# See docs/AMR_DIGITAL_TWIN_REFERENCE.md
# ---------------------------------------------------------------

set -e

source /opt/ros/jazzy/setup.bash

ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args \
  -r cloud_in:=/lidar/cloud \
  -r scan:=/scan \
  -p target_frame:=lidar_link \
  -p transform_tolerance:=0.01 \
  -p min_height:=-10.0 \
  -p max_height:=10.0 \
  -p angle_min:=-3.14159 \
  -p angle_max:=3.14159 \
  -p angle_increment:=0.0087 \
  -p scan_time:=0.1 \
  -p range_min:=0.05 \
  -p range_max:=100.0
