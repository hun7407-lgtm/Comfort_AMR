#!/bin/bash
# Keyboard teleop, slow. Run this IN AN SSH TERMINAL on the AMR.
#   i forward   , back   j/l turn   k stop
# Speeds are deliberately low: the LiDAR is blind below 0.31 m and within
# about 1 m, and this robot has no ultrasonic sensors on any topic.
source /opt/ros/humble/setup.bash
source $HOME/setup/install/setup.bash
export ROS_DOMAIN_ID=30
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:=0.15 -p turn:=0.4
