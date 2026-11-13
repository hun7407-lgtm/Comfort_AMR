#!/usr/bin/env bash
# ---------------------------------------------------------------
# Run a list of trials unattended, with a watchdog and one retry.
#
#     ./scripts/run_campaign.sh --manifest results/campaign_X/manifest.txt \
#         --out results/campaign_X
#
# The manifest has one trial per line:
#
#     <scenario> <config> [extra run_scenario.sh args...]
#     b3d configs/scenarios/paper_h8_a4_scr_s0002.yaml --fleet-tail 150
#     configs/stage1/b3o_r1.00.yaml configs/campaigns/cross/campaign_h8_a3_cross_s1.yaml --fleet-tail 150
#
# The scenario is a short name or a path to a scenario YAML (only the
# `scenario` and `comfort_space` sections), exactly as run_scenario.sh
# accepts it.
#
# Lines starting with # are skipped. Every trial runs headless with
# --results <out>, so all rows land in <out>/trials.csv and
# <out>/fleet_trials.csv. A trial that fails before it starts (Nav2 or
# the fleet did not activate -- seen once under load) is retried once.
# Each trial has a hard time limit; a hung one is killed and recorded.
# Progress goes to <out>/campaign.log; the run can be stopped and the
# manifest resumed by passing --skip N.
# ---------------------------------------------------------------
set -uo pipefail
PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
cd "$PROJECT_ROOT"

MANIFEST=""; OUT=""; SKIP=0; LIMIT_S=1500
while [ $# -gt 0 ]; do
    case "$1" in
        --manifest) MANIFEST="$2"; shift 2 ;;
        --out)      OUT="$2"; shift 2 ;;
        --skip)     SKIP="$2"; shift 2 ;;
        --limit)    LIMIT_S="$2"; shift 2 ;;
        *) echo "[ERROR] unknown option $1" >&2; exit 2 ;;
    esac
done
[ -f "$MANIFEST" ] || { echo "[ERROR] no manifest at $MANIFEST" >&2; exit 2; }
[ -n "$OUT" ] || { echo "[ERROR] --out is required" >&2; exit 2; }
mkdir -p "$OUT/logs"
LOG="$OUT/campaign.log"
mapfile -t LINES < <(grep -v '^\s*#' "$MANIFEST" | grep -v '^\s*$')
TOTAL=${#LINES[@]}
echo "campaign  : $TOTAL trial(s) from $MANIFEST, starting at $((SKIP + 1)), limit ${LIMIT_S}s each" | tee -a "$LOG"
echo "started   : $(date -Iseconds)   commit $(git rev-parse --short HEAD 2>/dev/null)" | tee -a "$LOG"

stop_everything() {
    ./scripts/stop_all.sh >/dev/null 2>&1
    pkill -f '[r]un_amr_sim' 2>/dev/null
    pkill -f '[c]ollision_monitor' 2>/dev/null
    sleep 4
}

n=0
for line in "${LINES[@]}"; do
    n=$((n + 1))
    [ "$n" -le "$SKIP" ] && continue
    read -r scenario config extra <<< "$line"
    # A scenario may be a short name (b2) or a path to a scenario YAML
    # (configs/stage1/b3o_r1.00.yaml); the log is named after its stem
    # either way.
    name="$(printf '%03d' "$n")_$(basename "$config" .yaml)_$(basename "$scenario" .yaml)"
    for attempt in 1 2; do
        stop_everything
        t0=$(date +%s)
        # shellcheck disable=SC2086
        timeout -k 30 "$LIMIT_S" ./scripts/run_scenario.sh "$scenario" --config "$config" \
            --headless --results "$OUT" $extra > "$OUT/logs/$name.log" 2>&1
        code=$?
        dt=$(( $(date +%s) - t0 ))
        result="$(grep -m1 '  result             : ' "$OUT/logs/$name.log" | sed 's/.*: //')"
        if grep -q '\[ERROR\]' "$OUT/logs/$name.log" && [ -z "$result" ] && [ "$attempt" -eq 1 ]; then
            echo "$(date +%H:%M:%S)  $name  attempt 1 did not start ($(grep -m1 '\[ERROR\]' "$OUT/logs/$name.log" | cut -c1-70)); retrying" | tee -a "$LOG"
            mv "$OUT/logs/$name.log" "$OUT/logs/$name.attempt1.log"
            continue
        fi
        fleet="$(grep -c '^\s*\[fleet\] a[0-9] \(SUCCESS\|DEADLOCK\|ABORTED\)' "$OUT/logs/$name.log")"
        echo "$(date +%H:%M:%S)  $name  exit $code  ${dt}s  AMR1 ${result:-<none>}  fleet rows $fleet  [$n/$TOTAL]" | tee -a "$LOG"
        break
    done
done
stop_everything
echo "finished  : $(date -Iseconds)" | tee -a "$LOG"
