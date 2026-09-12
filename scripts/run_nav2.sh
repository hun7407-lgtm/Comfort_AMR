#!/usr/bin/env bash
# ---------------------------------------------------------------
# PHASE D - bring up Nav2 against the running Isaac Sim.
#
# Start the simulation FIRST, in another terminal:
#
#     ./scripts/run_sim.sh
#
# then run this:
#
#     ./scripts/run_nav2.sh
#     ./scripts/run_nav2.sh --config configs/demo_weave.yaml
#
# The map and the Nav2 parameters are regenerated from the config every
# time, so they can never disagree with the corridor the simulation is
# actually running. Stop with Ctrl+C.
#
# Send a goal from a third terminal with ./scripts/send_goal.sh
# ---------------------------------------------------------------

set -euo pipefail

PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

CONFIG="configs/common.yaml"
SCENARIO=""
EXTRA=()
while [ $# -gt 0 ]; do
    case "$1" in
        --config)   CONFIG="$2"; shift 2 ;;
        --scenario) SCENARIO="$2"; shift 2 ;;
        *) EXTRA+=("$1"); shift ;;
    esac
done

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "[ERROR] ROS 2 Jazzy not found at /opt/ros/jazzy. See docs/SETUP.md." >&2
    exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

export PYTHONPATH="${PROJECT_ROOT}:${PYTHONPATH:-}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export PYTHONUNBUFFERED=1
export COMFORT_AMR_CONFIG="$CONFIG"
export COMFORT_AMR_SCENARIO="$SCENARIO"

echo "[info] config        : $CONFIG"
echo "[info] scenario      : ${SCENARIO:-<none, common.yaml only>}"
echo "[info] ROS_DOMAIN_ID : $ROS_DOMAIN_ID"
echo

echo "[step] regenerating the map"
python3 "$PROJECT_ROOT/scripts/generate_map.py" --config "$CONFIG"
echo
echo "[step] regenerating the Nav2 parameters"
python3 "$PROJECT_ROOT/scripts/generate_nav2_params.py" --config "$CONFIG"
echo

echo "[step] launching Nav2"
exec ros2 launch "$PROJECT_ROOT/scripts/nav2_bringup.launch.py" "${EXTRA[@]}"
