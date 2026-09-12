#!/usr/bin/env bash
# ---------------------------------------------------------------
# Stop everything this project starts: Isaac Sim, Nav2 and the helpers.
#
#     ./scripts/stop_all.sh
#
# Ctrl+C in each terminal is the normal way to stop. Use this when a
# launch was killed and left its child nodes running, which shows up as
# Nav2 still answering on topics after the terminal is gone, or as the
# next simulation refusing to start because the GPU is still busy.
# ---------------------------------------------------------------

set -uo pipefail

# Bracketed characters keep each pattern from matching this script's own
# command line, which would make it kill itself before finishing.
PATTERNS=(
    "isaac_sim/scripts/run_am[r]_sim.py"
    "isaac_sim/scripts/spaw[n]_amr.py"
    "nav2_bringu[p].launch.py"
    "nav2_[m]ap_server/map_server"
    "nav2_[p]lanner/planner_server"
    "nav2_[c]ontroller/controller_server"
    "nav2_[b]ehaviors/behavior_server"
    "nav2_[b]t_navigator/bt_navigator"
    "nav2_[l]ifecycle_manager/lifecycle_manager"
    "pointcloud_to_[l]aserscan_node"
    "static_transform_[p]ublisher"
    "scripts/goto_[g]oal.py"
    "scripts/nav2_[g]oal.py"
    "scripts/amr_arrow_[t]eleop.py"
    # These two were missing at first and it cost a day of confusion: a
    # gate left over from a B2 run kept publishing the static radii, so
    # the next B3 run was held back by boundaries that scenario never
    # configured, and B3 looked like it deadlocked.
    "scripts/comfort_[g]ate.py"
    "scripts/comfort_[p]ublisher.py"
)

# Never kill this script or anything it is running inside. pgrep -f
# matches the whole command line, so a shell that merely mentions one of
# the paths above matches too, and the script would kill its own caller.
PROTECTED=" "
walk_pid=$$
while [ -n "$walk_pid" ] && [ "$walk_pid" != "1" ] && [ "$walk_pid" != "0" ]; do
    PROTECTED="$PROTECTED$walk_pid "
    walk_pid=$(ps -o ppid= -p "$walk_pid" 2>/dev/null | tr -d ' ')
done

is_protected() {
    case "$PROTECTED" in *" $1 "*) return 0 ;; esac
    return 1
}

kill_matching() {
    local signal="$1"
    local count=0
    for pattern in "${PATTERNS[@]}"; do
        for pid in $(pgrep -f "$pattern" || true); do
            is_protected "$pid" && continue
            kill "$signal" "$pid" 2>/dev/null && count=$((count + 1))
        done
    done
    echo "$count"
}

killed=$(kill_matching -TERM)

if [ "$killed" -eq 0 ]; then
    echo "nothing was running"
    exit 0
fi

echo "asked $killed process(es) to stop"
sleep 3

# Isaac Sim occasionally ignores SIGTERM while a frame is in flight.
remaining=$(kill_matching -KILL)
[ "$remaining" -gt 0 ] && echo "forced $remaining process(es)"

sleep 1
echo
echo "GPU:"
nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv 2>/dev/null || true
