#!/bin/bash
# Stop every ROS node on the AMR. Kept in a file so that the pattern strings
# do not appear in an ssh command line -- pkill -f would otherwise match the
# ssh session's own shell and kill the connection.
tmux kill-server 2>/dev/null
sleep 1
for pat in robot_bringup unitree_lidar_ros2_node safeway_motion robot_state_publisher \
           safeway_robot_urdf slam_toolbox pointcloud_to_laserscan foxglove_bridge; do
  pkill -f "$pat"
done
sleep 3
echo "stopped"
