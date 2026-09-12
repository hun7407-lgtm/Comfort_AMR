#!/usr/bin/env bash
# ---------------------------------------------------------------
# STEP 3 - spawn the AMR in Isaac Sim and verify it stands up.
#
#   ./scripts/run_amr_test.sh              viewport, exits when done
#   ./scripts/run_amr_test.sh --hold       viewport, stays open
#   ./scripts/run_amr_test.sh --headless   no window, for a quick check
#
# Exit code 0 means every check passed.
# ---------------------------------------------------------------

set -euo pipefail

PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

# Locate Isaac Sim without hardcoding a path. ISAACSIM_PATH wins.
if [ -n "${ISAACSIM_PATH:-}" ] && [ -x "$ISAACSIM_PATH/python.sh" ]; then
    ISAACSIM_ROOT="$ISAACSIM_PATH"
elif [ -x "$HOME/isaacsim/python.sh" ]; then
    ISAACSIM_ROOT="$HOME/isaacsim"
else
    FOUND=$(find "$HOME" -maxdepth 4 -name isaac-sim.sh -type f 2>/dev/null | head -1)
    if [ -z "$FOUND" ]; then
        echo "[ERROR] Isaac Sim not found. Set ISAACSIM_PATH to the install directory." >&2
        exit 1
    fi
    ISAACSIM_ROOT="$(dirname "$FOUND")"
fi

echo "[info] Isaac Sim   : $ISAACSIM_ROOT"
echo "[info] project root: $PROJECT_ROOT"

# Let the script import comfort_core and isaac_sim.utils.
export PYTHONPATH="${PROJECT_ROOT}:${PYTHONPATH:-}"

# Do NOT source /opt/ros/jazzy here.
#
# Isaac Sim's interpreter is Python 3.11; system Jazzy is built for 3.12.
# Sourcing the system setup puts 3.12 site-packages ahead of Isaac Sim's
# own cp311 rclpy, and the ROS 2 bridge then fails to import it. Isaac Sim
# ships a complete Jazzy library set for exactly this reason. Its
# setup_ros_env.sh selects that set whenever ROS_DISTRO is unset, which is
# the case here.
#
# The ROS 2 nodes we write run in a separate process with the system
# install. Both sides speak Jazzy over FastDDS, so they interoperate.
# See docs/SETUP.md section 2.
set +u
# shellcheck disable=SC1091
source "$ISAACSIM_ROOT/setup_ros_env.sh"
set -u
echo "[info] ROS 2      : bundled $ROS_DISTRO libraries (RMW $RMW_IMPLEMENTATION)"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# Isaac Sim tears the process down hard on shutdown; without this the
# script's own output can be lost in the stdout buffer.
export PYTHONUNBUFFERED=1

exec "$ISAACSIM_ROOT/python.sh" \
     "$PROJECT_ROOT/isaac_sim/scripts/spawn_amr.py" "$@"
