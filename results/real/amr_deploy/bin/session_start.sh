#!/bin/bash
# Start a capture session: stack up, preflight, then ONE bag for the whole run.
#
#   ./session_start.sh [name]
#
# Recording starts BEFORE the first pose and is not stopped until the session
# ends. On 2026-10-18 the station_engaged and stationary captures were taken
# before recording had been restarted, and their point clouds were lost; only
# the walk was in a bag.
set -e
NAME=${1:-states}
source /opt/ros/humble/setup.bash
source $HOME/setup/install/setup.bash
export ROS_DOMAIN_ID=30

$HOME/start_all.sh
sleep 3
echo
if ! $HOME/preflight.sh | tee /tmp/preflight.log | tail -3; then :; fi
if grep -q "NOT ready" /tmp/preflight.log; then
  echo
  echo "ABORT: preflight failed. Fix the FAIL lines before recording."
  exit 1
fi

tmux kill-session -t mapping 2>/dev/null || true
tmux new-session -d -s mapping "$HOME/start_mapping.sh 2>&1 | tee /tmp/mapping.log"
sleep 20

BAG="$HOME/bags/${NAME}_$(date +%y%m%d_%H%M%S)"
mkdir -p "$HOME/bags"
tmux kill-session -t record 2>/dev/null || true
tmux new-session -d -s record "source /opt/ros/humble/setup.bash; source \$HOME/setup/install/setup.bash; export ROS_DOMAIN_ID=30; exec ros2 bag record -o '$BAG' /lidar/cloud /lidar/imu /scan /tf /tf_static /odom /joint_states /map 2>&1 | tee /tmp/record.log"
echo "$BAG" > /tmp/current_bag
sleep 5
echo
echo "=== recording to $BAG ==="
echo
echo "now, in order, marking each one:"
echo "   ~/mark.sh 'engage start'    ... stand at the desk        ~20 s"
echo "   ~/mark.sh 'idle start'      ... stand against the wall   ~20 s"
echo "   ~/mark.sh 'walk start'      ... walk toward the robot"
echo "   ~/session_stop.sh           ... closes the bag properly"
