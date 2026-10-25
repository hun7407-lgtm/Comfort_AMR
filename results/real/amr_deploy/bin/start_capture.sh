#!/bin/bash
# One corridor visit, everything captured. Runs entirely onboard in tmux so a
# WiFi drop cannot interrupt it.
#
#   1. restart SLAM so the map origin is the robot's CURRENT pose
#   2. let it see the empty corridor and save that as the clean static map
#   3. start recording cloud + tf + scan + odom + map
#
# After this the operator walks the three states; mark() writes timestamps.
source /opt/ros/humble/setup.bash
source $HOME/setup/install/setup.bash
export ROS_DOMAIN_ID=30
NAME=${1:-states}
STAMP=$(date +%H%M%S)

tmux kill-session -t mapping 2>/dev/null
tmux kill-session -t record  2>/dev/null
sleep 2
tmux new-session -d -s mapping "$HOME/start_mapping.sh 2>&1 | tee /tmp/mapping.log"
sleep 22

# Clean map of the empty corridor, before anyone walks into it.
ros2 run nav2_map_server map_saver_cli -f "$HOME/maps/${NAME}_clean" \
    --ros-args -p save_map_timeout:=10.0 2>&1 | tail -2

mkdir -p "$HOME/bags"
BAG="$HOME/bags/${NAME}_${STAMP}"
tmux new-session -d -s record "cd $HOME/bags && source /opt/ros/humble/setup.bash && source $HOME/setup/install/setup.bash && export ROS_DOMAIN_ID=30 && ros2 bag record -o '$BAG' /lidar/cloud /lidar/imu /scan /tf /tf_static /odom /joint_states /map 2>&1 | tee /tmp/record.log"
echo "$BAG" > /tmp/current_bag
sleep 5
echo "=== recording to $BAG ==="
tmux ls
