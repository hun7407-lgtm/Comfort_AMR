#!/bin/bash
# Save the current slam_toolbox map. Usage: ./save_map.sh <name>
source /opt/ros/humble/setup.bash
source $HOME/setup/install/setup.bash
export ROS_DOMAIN_ID=30
NAME=${1:-corridor}
mkdir -p $HOME/maps
ros2 run nav2_map_server map_saver_cli -f $HOME/maps/$NAME --ros-args -p save_map_timeout:=10.0
echo "saved to $HOME/maps/$NAME.{pgm,yaml}"
