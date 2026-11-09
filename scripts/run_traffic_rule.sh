#!/usr/bin/env bash
# ---------------------------------------------------------------
# Keep-right masks, one per robot, from each robot's own plan.
#
# Needs the simulation (fleet.driver: nav2), AMR1's
# Nav2 and ./scripts/run_fleet_nav2.sh all running:
#
#     ./scripts/run_traffic_rule.sh --config configs/fleet_nav2.yaml --scenario b3d
#
# A plain ROS 2 node in the system Python 3.12, so it sources the
# system ROS 2 like run_nav2_goal.sh does. See docs/NAV2.md section 5c.
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

exec python3 "$PROJECT_ROOT/scripts/traffic_rule.py" "$@"
