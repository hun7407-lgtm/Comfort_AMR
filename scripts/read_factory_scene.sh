#!/usr/bin/env bash
# ---------------------------------------------------------------
# Read a hand-edited factory scene back into the config.
#
#   ./scripts/read_factory_scene.sh [--apply]
#
# Moves you made in the viewport become explicit poses in the config.
# ---------------------------------------------------------------
set -euo pipefail

PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

if [ -n "${ISAACSIM_PATH:-}" ] && [ -x "$ISAACSIM_PATH/python.sh" ]; then
    ISAACSIM_ROOT="$ISAACSIM_PATH"
elif [ -x "$HOME/isaacsim/python.sh" ]; then
    ISAACSIM_ROOT="$HOME/isaacsim"
else
    FOUND=$(find "$HOME" -maxdepth 4 -name isaac-sim.sh -type f 2>/dev/null | head -1)
    [ -n "$FOUND" ] || { echo "[ERROR] Isaac Sim not found. Set ISAACSIM_PATH." >&2; exit 1; }
    ISAACSIM_ROOT="$(dirname "$FOUND")"
fi

export PYTHONPATH="${PROJECT_ROOT}:${PYTHONPATH:-}"
export PYTHONUNBUFFERED=1
cd "$PROJECT_ROOT"
exec "$ISAACSIM_ROOT/python.sh" "$PROJECT_ROOT/isaac_sim/scripts/read_factory_scene.py" "$@"
