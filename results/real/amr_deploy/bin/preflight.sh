#!/bin/bash
# Is this robot actually fit to map? Run FIRST, before anything else.
#
# 2026-10-18: three hours went into a robot whose drive board had two broken
# wires. Every symptom (battery 0.0 V, encoders NaN, USB error -71) was visible
# in the first minute. This script looks at all of them at once.
source /opt/ros/humble/setup.bash
source $HOME/setup/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}

pass=0; fail=0
chk() { if [ "$2" = ok ]; then echo "  OK    $1"; pass=$((pass+1)); else echo "  FAIL  $1 -- $3"; fail=$((fail+1)); fi; }

echo "=== 1. control board answers (the one that mattered) ==="
n=$(grep -c "parsed successfully" /tmp/bringup.log 2>/dev/null); n=${n:-0}
w=$(grep -c "written .* bytes" /tmp/bringup.log 2>/dev/null); w=${w:-0}
echo "    written=$w  parsed=$n"
[ "$n" -gt 0 ] && chk "board responds" ok || chk "board responds" no "no 'parsed successfully' in bringup.log"

echo "=== 2. USB stability ==="
d=$(echo 123 | sudo -S dmesg 2>/dev/null | grep -c "usb 1-2.2: USB disconnect")
e=$(echo 123 | sudo -S dmesg 2>/dev/null | grep -c "error -71")
echo "    disconnects=$d  error-71=$e"
[ "$d" -lt 2 ] && [ "$e" -eq 0 ] && chk "control board USB stable" ok || chk "control board USB stable" no "re-enumerating"

echo "=== 0. bringup actually running ==="
nn=$(timeout 8 ros2 node list 2>/dev/null | wc -l)
echo "    nodes=$nn"
[ "$nn" -ge 5 ] && chk "bringup nodes present" ok || chk "bringup nodes present" no "only $nn nodes -- run ~/start_all.sh"

echo "=== 3. battery ==="
v=$(timeout 6 ros2 topic echo /battery_voltage --once 2>/dev/null | grep -oP '(?<=data: ).*')
echo "    voltage=${v:-<silent>}"
awk -v v="${v:-0}" 'BEGIN{exit !(v>10)}' && chk "battery reports voltage" ok || chk "battery reports voltage" no "reads $v (expect ~24)"

echo "=== 4. encoders finite ==="
# An empty reply is NOT a pass. A topic that never publishes produces no
# "nan" to grep for, and an earlier version of this script reported OK for a
# robot whose bringup had not even started.
enc=$(timeout 6 ros2 topic echo /motor_encoder_sub --once 2>/dev/null | head -3)
if [ -z "$enc" ]; then chk "encoders finite" no "topic silent -- is bringup up?"
elif echo "$enc" | grep -q nan; then chk "encoders finite" no "NaN"
else chk "encoders finite" ok; fi

echo "=== 5. odometry finite ==="
od=$(timeout 6 ros2 topic echo /odom --once 2>/dev/null | sed -n '8,12p')
if [ -z "$od" ]; then chk "odom finite" no "topic silent -- is bringup up?"
elif echo "$od" | grep -q nan; then chk "odom finite" no "NaN"
else chk "odom finite" ok; fi

echo "=== 6. lidar ==="
hz=$(timeout 8 stdbuf -oL ros2 topic hz /lidar/cloud 2>/dev/null | grep -oP '(?<=average rate: )[0-9.]+' | head -1)
echo "    /lidar/cloud ${hz:-0} Hz"
awk -v h="${hz:-0}" 'BEGIN{exit !(h>5)}' && chk "lidar publishing" ok || chk "lidar publishing" no "restart bringup (driver opens port before sensor streams)"

echo
echo "=== $pass passed, $fail failed ==="
[ "$fail" -eq 0 ] && echo "READY to map." || echo "NOT ready -- fix the FAIL lines before driving."
