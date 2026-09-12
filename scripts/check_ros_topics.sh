#!/usr/bin/env bash

# ---------------------------------------------------------------
# Adapted from: https://github.com/dongtak2/AMR-Digital-Twin
#   original file : scripts/check_ros.sh
#   commit        : c54269ee80032c29b0548eb77c3ee38d68b87e78 (2026-09-11)
#   modifications : renamed from check_ros.sh; ROS distro humble -> jazzy
# See docs/AMR_DIGITAL_TWIN_REFERENCE.md
# ---------------------------------------------------------------

source /opt/ros/jazzy/setup.bash

echo "=== Required topics ==="
for topic in /clock /cmd_vel /joint_states /odom /lidar/cloud /scan; do
  if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
    echo "[OK] $topic"
  else
    echo "[--] $topic"
  fi
done

echo
echo "=== TF quick checks ==="
echo "Run manually if needed:"
echo "ros2 run tf2_ros tf2_echo odom base_footprint"
echo "ros2 run tf2_ros tf2_echo base_footprint base_link"
echo "ros2 run tf2_ros tf2_echo base_link lidar_link"
echo "ros2 run tf2_ros tf2_echo map base_footprint"
