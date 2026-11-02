#!/usr/bin/env bash
# ---------------------------------------------------------------
# Photograph the scene from several viewpoints.
#
#     ./scripts/scene_shots.sh
#     ./scripts/scene_shots.sh --config configs/factory_u.yaml --out results/notion/261027
#
# Wraps isaac_sim/scripts/scene_shots.py with the same environment
# run_sim.sh uses. The environment is the whole reason this wrapper
# exists: the scene build publishes the humans' TF frames through the
# ROS 2 bridge, so the bridge has to load even for a screenshot, and it
# only loads against Isaac Sim's OWN bundled Jazzy libraries. Running
# the python script bare fails with "Could not create node using
# unrecognized type 'isaacsim.ros2.bridge.ROS2Context'".
# ---------------------------------------------------------------
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

if [ -n "${ISAACSIM_PATH:-}" ] && [ -x "$ISAACSIM_PATH/python.sh" ]; then
    ISAACSIM_ROOT="$ISAACSIM_PATH"
elif [ -x "$HOME/isaacsim/python.sh" ]; then
    ISAACSIM_ROOT="$HOME/isaacsim"
else
    echo "[ERROR] Isaac Sim not found. Set ISAACSIM_PATH to the install directory." >&2
    exit 1
fi
echo "[info] Isaac Sim   : $ISAACSIM_ROOT"

export PYTHONPATH="${PROJECT_ROOT}:${PYTHONPATH:-}"

# Isaac Sim's bundled ROS 2 libraries, not the system install: its
# interpreter is cp311 and system Jazzy is built for 3.12. See the long
# note in scripts/run_sim.sh.
set +u
# shellcheck disable=SC1091
source "$ISAACSIM_ROOT/setup_ros_env.sh"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export PYTHONUNBUFFERED=1

exec "$ISAACSIM_ROOT/python.sh" \
    "$PROJECT_ROOT/isaac_sim/scripts/scene_shots.py" "$@"
