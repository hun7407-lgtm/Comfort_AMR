#!/usr/bin/env bash
# ---------------------------------------------------------------
# PHASE D - run one trial through Nav2.
#
# Start the simulation FIRST, in another terminal:
#
#     ./scripts/run_sim.sh
#
# then run this:
#
#     ./scripts/run_nav2_goal.sh
#
# Unlike the Isaac Sim scripts, this one DOES source the system ROS 2:
# it is a plain ROS 2 node running in the system Python 3.12.
# See docs/SETUP.md section 4.
#
# Requires scripts/run_nav2.sh to be running too. Exit 0 means SUCCESS.
# ---------------------------------------------------------------

set -euo pipefail

PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

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

echo "[info] ROS 2         : $ROS_DISTRO"
echo "[info] ROS_DOMAIN_ID : $ROS_DOMAIN_ID"
echo "[info] RMW           : $RMW_IMPLEMENTATION"

exec python3 "$PROJECT_ROOT/scripts/nav2_goal.py" "$@"
