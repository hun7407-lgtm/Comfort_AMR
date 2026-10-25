#!/usr/bin/env bash
# ---------------------------------------------------------------
# Open the saved factory bay in a window, camera already aimed at it.
#
#   ./scripts/view_factory_scene.sh
#
# Edits made in the window save back to the same file (File > Save).
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
exec "$ISAACSIM_ROOT/python.sh" "$PROJECT_ROOT/isaac_sim/scripts/view_scene.py" "$@"
