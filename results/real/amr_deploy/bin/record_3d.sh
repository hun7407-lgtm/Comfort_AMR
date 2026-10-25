#!/bin/bash
# Record everything needed to rebuild a 3D point-cloud map afterwards.
#
# slam_toolbox only ever produces a 2D occupancy grid. To get a 3D map you
# accumulate the raw cloud along the SLAM trajectory, which needs the cloud,
# the full TF chain (map->odom from SLAM, odom->base from the wheels,
# base->lidar from the URDF) and the clock they share. Record them together or
# the clouds cannot be placed afterwards.
#
#   ./record_3d.sh <name>
source /opt/ros/humble/setup.bash
source $HOME/setup/install/setup.bash
export ROS_DOMAIN_ID=30
NAME=${1:-drive}
mkdir -p $HOME/bags
cd $HOME/bags
exec ros2 bag record -o "${NAME}_$(date +%H%M%S)" \
  /lidar/cloud /lidar/imu /scan /tf /tf_static /odom /joint_states /map
