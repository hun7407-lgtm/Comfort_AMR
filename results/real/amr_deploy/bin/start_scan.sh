#!/bin/bash
# pointcloud_to_laserscan with the slab the simulation uses.
# lidar_link sits 0.31 m above the floor, so -0.20..+1.20 relative to it is
# 0.11..1.51 m above the floor: walls and people in, floor and ceiling out.
source /opt/ros/humble/setup.bash
source $HOME/setup/install/setup.bash
export ROS_DOMAIN_ID=30
exec ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -p target_frame:=lidar_link \
  -p transform_tolerance:=0.05 \
  -p min_height:=-0.20 \
  -p max_height:=1.20 \
  -p angle_min:=-3.14159 \
  -p angle_max:=3.14159 \
  -p angle_increment:=0.0087 \
  -p scan_time:=0.1 \
  -p range_min:=0.20 \
  -p range_max:=15.0 \
  -p use_inf:=true \
  --remap cloud_in:=/lidar/cloud --remap scan:=/scan
