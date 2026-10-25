#!/bin/bash
# One Comfort Space capture.   ./capture_state.sh <state> [seconds]
#
#   1. find the person (background subtraction + height profile)
#   2. write a config putting them there with the given state
#   3. restart the mask publisher
#   4. save the global costmap, the mask and the scan
#
# The STATE is declared, not measured. `moving` here means "this person is
# one the policy should treat as walking", and the capture is taken while
# they hold still so the detector gets a clean fix. Measuring the speed needs
# the person tracked over time, which is the next step, not this one.
source /opt/ros/humble/setup.bash
source $HOME/setup/install/setup.bash
export ROS_DOMAIN_ID=30
STATE=${1:?usage: capture_state.sh <station_engaged|stationary|moving> [seconds]}
SECS=${2:-4}

LINE=$(python3 $HOME/detect_person.py "$SECS" | tee /dev/stderr | grep '^PERSON')
if [ -z "$LINE" ]; then echo "ABORT: no person found"; exit 1; fi
X=$(echo "$LINE" | awk '{print $2}')
Y=$(echo "$LINE" | awk '{print $3}')
echo "--- person at ($X, $Y), state $STATE ---"

python3 $HOME/make_comfort_cfg.py "$X" "$Y" "$STATE"
tmux kill-session -t comfort 2>/dev/null
sleep 2
tmux new-session -d -s comfort "cd $HOME/comfort && source /opt/ros/humble/setup.bash && source $HOME/setup/install/setup.bash && export ROS_DOMAIN_ID=30 && python3 scripts/comfort_publisher.py --config configs/real_corridor.yaml --mask-topic /comfort_mask --info-topic /costmap_filter_info 2>&1 | tee /tmp/comfort.log"
sleep 12

python3 $HOME/grab_costmap.py "/tmp/cap_${STATE}.npz"
echo "$X $Y $STATE" > "/tmp/cap_${STATE}.txt"
echo "=== captured ${STATE} ==="
