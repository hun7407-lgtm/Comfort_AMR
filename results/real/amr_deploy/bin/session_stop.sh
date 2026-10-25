#!/bin/bash
# Close the bag CLEANLY and report what is in it.
#
# `tmux kill-session` kills the pane without letting rosbag2 finalise, and the
# bag is then left with no metadata.yaml -- `ros2 bag info|play` refuse to open
# it and the .db3 has to be read with sqlite by hand. That happened on
# 2026-10-18. SIGINT to the recorder lets it write the metadata.
BAG=$(cat /tmp/current_bag 2>/dev/null)
pid=$(pgrep -f "ros2 bag record" | head -1)
if [ -n "$pid" ]; then
  kill -INT "$pid"
  for i in $(seq 1 20); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
fi
tmux kill-session -t record 2>/dev/null
sleep 1
echo "bag: $BAG"
ls -la "$BAG" 2>/dev/null
if [ -f "$BAG/metadata.yaml" ]; then
  echo "metadata.yaml written -- ros2 bag info will work"
else
  echo "WARNING: no metadata.yaml; read the .db3 with bag_to_3d.py instead"
fi
