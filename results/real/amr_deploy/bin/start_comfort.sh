#!/bin/bash
# Publish the Comfort Space keepout mask.  ./start_comfort.sh <x> <y> <state>
source /opt/ros/humble/setup.bash
source $HOME/setup/install/setup.bash
export ROS_DOMAIN_ID=30
python3 $HOME/make_comfort_cfg.py "$1" "$2" "$3" >/dev/null
cd $HOME/comfort
exec python3 scripts/comfort_publisher.py \
  --config configs/real_corridor.yaml \
  --mask-topic /comfort_mask \
  --info-topic /costmap_filter_info
