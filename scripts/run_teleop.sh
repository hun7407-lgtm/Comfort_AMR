#!/usr/bin/env bash
# ---------------------------------------------------------------
# Drive the AMR manually with the arrow keys.
#
# Start the simulation FIRST, in another terminal:
#
#     ./scripts/run_sim.sh
#
# then run this:
#
#     ./scripts/run_teleop.sh
#
#   up / down     forward / reverse
#   left / right  turn
#   space         stop
#   q             quit
#
# Quit the teleop before starting Nav2. Two nodes publishing /cmd_vel at
# the same time fight each other.
# ---------------------------------------------------------------

set -euo pipefail

PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "[ERROR] ROS 2 Jazzy not found at /opt/ros/jazzy. See docs/SETUP.md." >&2
    exit 1
fi

if [ ! -t 0 ]; then
    echo "[ERROR] no terminal on stdin. Run this directly in a terminal," >&2
    echo "        not through a pipe or a background job." >&2
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

echo "[info] ROS 2         : $ROS_DISTRO"
echo "[info] ROS_DOMAIN_ID : $ROS_DOMAIN_ID"

exec python3 "$PROJECT_ROOT/scripts/amr_arrow_teleop.py" "$@"
