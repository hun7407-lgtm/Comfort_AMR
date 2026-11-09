#!/usr/bin/env bash
# ---------------------------------------------------------------
# One Nav2 stack per fleet AMR, each in its own namespace.
#
# Start the simulation with fleet.driver: nav2 and the robot under
# test's own Nav2 first:
#
#     ./scripts/run_sim.sh  --config configs/fleet_nav2.yaml
#     ./scripts/run_nav2.sh --config configs/fleet_nav2.yaml
#
# then:
#
#     ./scripts/run_fleet_nav2.sh --config configs/fleet_nav2.yaml
#
# Every enabled fleet.amrs entry gets ./scripts/run_nav2.sh --namespace
# <name>. The stacks are children of this shell; Ctrl+C takes them all
# down. Logs go to $FLEET_LOG_DIR (default: a temp dir printed below).
# ---------------------------------------------------------------

set -euo pipefail

PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

CONFIG="configs/common.yaml"
SCENARIO=""
ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --config)   CONFIG="$2"; ARGS+=(--config "$2"); shift 2 ;;
        --scenario) SCENARIO="$2"; ARGS+=(--scenario "$2"); shift 2 ;;
        *) ARGS+=("$1"); shift ;;
    esac
done

export PYTHONPATH="${PROJECT_ROOT}:${PYTHONPATH:-}"
mapfile -t ROBOTS < <(python3 - "$CONFIG" "$SCENARIO" <<'PY'
import sys
from comfort_core import load_config
cfg = load_config(scenario=sys.argv[2] or None, common_path=sys.argv[1])
fleet = cfg.get("fleet", {})
if not fleet.get("enabled", False) or fleet.get("driver", "scripted") != "nav2":
    sys.exit(0)
for name, spec in fleet.get("amrs", {}).items():
    if spec.get("enabled", True):
        print(name)
PY
)

if [ "${#ROBOTS[@]}" -eq 0 ]; then
    echo "[ERROR] $CONFIG has no enabled fleet AMR with fleet.driver: nav2" >&2
    exit 1
fi

LOG_DIR="${FLEET_LOG_DIR:-$(mktemp -d)}"
mkdir -p "$LOG_DIR"
echo "[info] fleet   : ${ROBOTS[*]}"
echo "[info] logs    : $LOG_DIR"

PIDS=()
cleanup() {
    for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
    wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT

for name in "${ROBOTS[@]}"; do
    "$PROJECT_ROOT/scripts/run_nav2.sh" "${ARGS[@]}" --namespace "$name" \
        > "$LOG_DIR/nav2_$name.log" 2>&1 &
    PIDS+=($!)
    echo "[info] $name    : pid $! -> $LOG_DIR/nav2_$name.log"
done

wait
