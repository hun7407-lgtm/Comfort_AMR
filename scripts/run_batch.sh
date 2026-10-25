#!/usr/bin/env bash
# ---------------------------------------------------------------
# Run the same trial many times, across scenarios and layouts.
#
#     ./scripts/run_batch.sh                       b1,b2,b3 x the three bays, 5 each
#     ./scripts/run_batch.sh --repeats 3
#     ./scripts/run_batch.sh --scenarios b2,b3 --configs configs/factory_u.yaml
#     ./scripts/run_batch.sh --dry-run             print the plan, run nothing
#     ./scripts/run_batch.sh --out results/batches/my_sweep
#
# Every trial is a fresh simulation, started and torn down by
# run_scenario.sh. That is deliberate and it is why this is slow:
# reusing one leaves the robot wherever the last trial abandoned it and
# two scenarios then start from different conditions.
#
# All the rows land in ONE file:
#
#     <out>/trials.csv        one row per trial
#     <out>/<stamp>_*.json    per-trial detail
#     <out>/logs/*.log        the console output of each trial
#     <out>/batch.txt         what was asked for, and what happened
#
# A trial that deadlocks or times out exits non-zero. That is a result,
# not an error, so the batch carries on and the summary counts it. The
# batch only stops early if asked to with --stop-on-error.
#
# When it is done it prints the summary. Same thing by hand:
#
#     python3 scripts/summarise_trials.py <out>/trials.csv
# ---------------------------------------------------------------

set -uo pipefail

PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
cd "$PROJECT_ROOT"

SCENARIOS="b1,b2,b3"
CONFIGS="configs/factory.yaml,configs/factory_u.yaml,configs/factory_long.yaml"
REPEATS=5
OUT=""
DRY_RUN=0
STOP_ON_ERROR=0

while [ $# -gt 0 ]; do
    case "$1" in
        --scenarios) SCENARIOS="$2"; shift 2 ;;
        --configs)   CONFIGS="$2"; shift 2 ;;
        --repeats)   REPEATS="$2"; shift 2 ;;
        --out)       OUT="$2"; shift 2 ;;
        --dry-run)   DRY_RUN=1; shift ;;
        --stop-on-error) STOP_ON_ERROR=1; shift ;;
        -h|--help)   sed -n '2,30p' "$0"; exit 0 ;;
        *) echo "[ERROR] unknown option $1" >&2; exit 2 ;;
    esac
done

STAMP="$(date +%y%m%d_%H%M%S)"
[ -n "$OUT" ] || OUT="results/batches/$STAMP"
# A dry run leaves nothing behind, not even an empty results folder.
MANIFEST=/dev/null
if [ "$DRY_RUN" -eq 0 ]; then
    mkdir -p "$OUT/logs"
    MANIFEST="$OUT/batch.txt"
fi

IFS=',' read -r -a SCENARIO_LIST <<< "$SCENARIOS"
IFS=',' read -r -a CONFIG_LIST <<< "$CONFIGS"
TOTAL=$(( ${#SCENARIO_LIST[@]} * ${#CONFIG_LIST[@]} * REPEATS ))

{
    echo "batch     : $STAMP"
    echo "scenarios : $SCENARIOS"
    echo "configs   : $CONFIGS"
    echo "repeats   : $REPEATS"
    echo "trials    : $TOTAL"
    echo "commit    : $(git rev-parse --short HEAD 2>/dev/null || echo n/a)"
    echo "started   : $(date -Iseconds)"
} | tee "$MANIFEST"

# A missing config is worth finding out about now rather than forty
# minutes in.
for config in "${CONFIG_LIST[@]}"; do
    [ -f "$config" ] || { echo "[ERROR] no config at $config" >&2; exit 2; }
done

# And so is a broken chain.
#
# These three take about a second between them and each one exists
# because something silently did nothing for a whole batch: a node
# that would not import, a yield that raised the first time it was
# tried, and a Comfort Space policy that produced no spaces at all so
# that B2 and B3 ran as B1. An hour of simulation is too expensive to
# spend finding that out at the end.
if [ "$DRY_RUN" -eq 0 ]; then
    for config in "${CONFIG_LIST[@]}"; do
        for check in check_nodes check_evasion check_perception; do
            # check_nodes takes no --config; the others do. Try with,
            # fall back to without, and only complain if both fail.
            if python3 "scripts/$check.py" --config "$config" >/dev/null 2>&1; then
                continue
            fi
            if python3 "scripts/$check.py" >/dev/null 2>&1; then
                continue
            fi
            echo "[ERROR] scripts/$check.py fails on $config:" >&2
            python3 "scripts/$check.py" --config "$config" >&2 \
                || python3 "scripts/$check.py" >&2 || true
            exit 2
        done
    done
    echo "preflight : check_nodes, check_evasion, check_perception all pass"
fi

if [ "$DRY_RUN" -eq 1 ]; then
    echo
    for config in "${CONFIG_LIST[@]}"; do
        for scenario in "${SCENARIO_LIST[@]}"; do
            echo "  $scenario x $REPEATS  on $config"
        done
    done
    echo
    echo "nothing run (--dry-run). Results would go to $OUT"
    exit 0
fi

DONE=0
FAILED=0
START=$(date +%s)

for config in "${CONFIG_LIST[@]}"; do
    for scenario in "${SCENARIO_LIST[@]}"; do
        for run in $(seq 1 "$REPEATS"); do
            DONE=$(( DONE + 1 ))
            name="$(basename "$config" .yaml)_${scenario}_$(printf '%02d' "$run")"
            left=$(( TOTAL - DONE ))
            elapsed=$(( $(date +%s) - START ))
            if [ "$DONE" -gt 1 ]; then
                eta=$(( elapsed / (DONE - 1) * left / 60 ))
                extra="  (about ${eta} min left)"
            else
                extra=""
            fi
            echo
            echo "[$DONE/$TOTAL] $scenario on $(basename "$config") run $run$extra"
            ./scripts/run_scenario.sh "$scenario" --headless \
                --config "$config" --results "$OUT" \
                > "$OUT/logs/$name.log" 2>&1
            code=$?
            verdict=$(grep -m1 "result             :" "$OUT/logs/$name.log" \
                      | sed 's/.*: //' | tr -d ' ')
            echo "        ${verdict:-NO RESULT}"
            if [ -z "$verdict" ]; then
                # No result line at all means the trial never ran: Isaac
                # or Nav2 did not come up. That IS an error, unlike a
                # deadlock.
                FAILED=$(( FAILED + 1 ))
                echo "        [WARN] see $OUT/logs/$name.log"
                if [ "$STOP_ON_ERROR" -eq 1 ]; then
                    echo "[ERROR] stopping: trial did not run (exit $code)" >&2
                    break 3
                fi
            fi
        done
    done
done

{
    echo "finished  : $(date -Iseconds)"
    echo "ran       : $DONE of $TOTAL"
    echo "no result : $FAILED"
    echo "minutes   : $(( ($(date +%s) - START) / 60 ))"
} | tee -a "$MANIFEST"

echo
if [ -f "$OUT/trials.csv" ]; then
    python3 "$PROJECT_ROOT/scripts/summarise_trials.py" "$OUT/trials.csv" \
        | tee -a "$MANIFEST"
else
    echo "[WARN] no trials.csv in $OUT -- nothing was recorded" >&2
    exit 1
fi
