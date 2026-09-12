#!/usr/bin/env bash
# ---------------------------------------------------------------
# Run one complete trial of a scenario, start to finish.
#
#     ./scripts/run_scenario.sh b2
#     ./scripts/run_scenario.sh b3 --headless
#     ./scripts/run_scenario.sh b3 --config configs/demo_walker.yaml
#
# Normally invoked through ./scripts/run_b2.sh or ./scripts/run_b3.sh.
#
# Starts Isaac Sim, brings up Nav2 with the Comfort Space filter and the
# yield gate, runs the trial, then shuts everything down.
#
# A fresh simulation per trial is deliberate. Reusing one leaves the
# robot wherever the last trial abandoned it, pressed against an
# obstacle with whatever contact state that produced, and B2 and B3 then
# start from different conditions. See docs/EXPERIMENT.md section 3.
#
# Exit code 0 means the trial ended in SUCCESS.
# ---------------------------------------------------------------

set -uo pipefail

PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
cd "$PROJECT_ROOT"

SCENARIO="${1:-}"
if [ -z "$SCENARIO" ]; then
    echo "usage: $0 <b2|b3> [--headless]" >&2
    exit 2
fi
shift

HEADLESS=()
CONFIG=()
while [ $# -gt 0 ]; do
    case "$1" in
        --headless) HEADLESS+=("--headless"); shift ;;
        --config)   CONFIG+=("--config" "$2"); shift 2 ;;
        *) shift ;;
    esac
done

RUN_DIR="$(mktemp -d)"
SIM_LOG="$RUN_DIR/sim.log"
NAV_LOG="$RUN_DIR/nav2.log"

cleanup() {
    echo
    echo "[step] shutting down"
    ./scripts/stop_all.sh >/dev/null 2>&1
    echo "[info] logs kept in $RUN_DIR"
}
trap cleanup EXIT

echo "[step] clearing anything already running"
./scripts/stop_all.sh >/dev/null 2>&1
sleep 2

echo "[step] starting Isaac Sim   (log: $SIM_LOG)"
./scripts/run_sim.sh "${HEADLESS[@]+"${HEADLESS[@]}"}" \
    "${CONFIG[@]+"${CONFIG[@]}"}" --scenario "$SCENARIO" > "$SIM_LOG" 2>&1 &

# Isaac Sim's first start compiles shaders and can take minutes.
for _ in $(seq 1 120); do
    grep -q "Simulation running" "$SIM_LOG" 2>/dev/null && break
    sleep 3
done
if ! grep -q "Simulation running" "$SIM_LOG" 2>/dev/null; then
    echo "[ERROR] Isaac Sim did not come up. Last lines:" >&2
    tail -20 "$SIM_LOG" >&2
    exit 1
fi
grep -E "comfort space:|human [a-z] at|with Comfort Space:" "$SIM_LOG" | sed 's/^/       /'

echo "[step] starting Nav2        (log: $NAV_LOG)"
./scripts/run_nav2.sh "${CONFIG[@]+"${CONFIG[@]}"}" --scenario "$SCENARIO" \
    > "$NAV_LOG" 2>&1 &

for _ in $(seq 1 60); do
    grep -q "Managed nodes are active" "$NAV_LOG" 2>/dev/null && break
    sleep 3
done
if ! grep -q "Managed nodes are active" "$NAV_LOG" 2>/dev/null; then
    echo "[ERROR] Nav2 did not activate. Last lines:" >&2
    tail -20 "$NAV_LOG" >&2
    exit 1
fi
sleep 5

echo "[step] running the trial"
./scripts/run_nav2_goal.sh "${CONFIG[@]+"${CONFIG[@]}"}" --scenario "$SCENARIO"
exit $?
