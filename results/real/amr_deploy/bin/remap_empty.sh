#!/bin/bash
# Re-map the empty corridor by driving it, then save. Runs in its own tmux
# session so a WiFi drop cannot interrupt the drive -- the first attempt at
# this lost the link mid-corridor and the robot had to be carried back.
exec > /tmp/remap.log 2>&1
source /opt/ros/humble/setup.bash
source $HOME/setup/install/setup.bash
export ROS_DOMAIN_ID=30

echo "=== $(date +%H:%M:%S) fresh SLAM ==="
tmux kill-session -t comfort 2>/dev/null
tmux kill-session -t nav2 2>/dev/null
tmux kill-session -t mapping 2>/dev/null
sleep 3
tmux new-session -d -s mapping "$HOME/start_mapping.sh 2>&1 | tee /tmp/mapping.log"
sleep 20

drive () {
  echo "--- drive $1 m ---"
  python3 $HOME/drive.py "$1" 0.15
  sleep 3
}

# Out and back twice. Two passes over the same wall dilute any transient
# return and give slam_toolbox a second look at each stretch.
drive  3.5
drive -3.5
drive -3.5
drive  3.5
drive  3.5
drive -3.5

echo "=== $(date +%H:%M:%S) saving ==="
ros2 run nav2_map_server map_saver_cli -f "$HOME/maps/empty_corridor" \
    --ros-args -p save_map_timeout:=10.0
echo "=== done ==="
