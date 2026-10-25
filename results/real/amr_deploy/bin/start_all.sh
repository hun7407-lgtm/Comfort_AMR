#!/bin/bash
# Bring the whole AMR stack up in tmux sessions: bringup, scan conversion,
# foxglove bridge. Mapping is started separately (start_mapping.sh) so that
# a map is never begun by accident.
$HOME/stop_all.sh >/dev/null 2>&1
tmux new-session -d -s bringup "bash -lc 'source /opt/ros/humble/setup.bash; source \$HOME/setup/install/setup.bash; export ROS_DOMAIN_ID=30; exec ros2 launch robot_bringup robot_bringup.launch.py pkg:=\"robot lidar\" 2>&1 | tee /tmp/bringup.log'"
sleep 10
tmux new-session -d -s scan "$HOME/start_scan.sh 2>&1 | tee /tmp/scan.log"
tmux new-session -d -s foxglove "bash -lc 'source /opt/ros/humble/setup.bash; source \$HOME/setup/install/setup.bash; export ROS_DOMAIN_ID=30; exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 address:=0.0.0.0 2>&1 | tee /tmp/foxglove.log'"
sleep 5
tmux ls
