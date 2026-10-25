#!/bin/bash
# Nav2 costmaps on the real AMR. slam_toolbox supplies /map and map->odom,
# so no map_server and no AMCL: the costmap's static layer subscribes to the
# live SLAM map.
source /opt/ros/humble/setup.bash
source $HOME/setup/install/setup.bash
export ROS_DOMAIN_ID=30
exec ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  params_file:=$HOME/nav2_real.yaml
