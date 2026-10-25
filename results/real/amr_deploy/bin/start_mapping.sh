#!/bin/bash
# slam_toolbox async mapping on the AMR. Needs bringup + start_scan.sh already up.
source /opt/ros/humble/setup.bash
source $HOME/setup/install/setup.bash
export ROS_DOMAIN_ID=30
exec ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:=$HOME/slam_corridor.yaml
