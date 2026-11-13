#!/usr/bin/env bash
# ---------------------------------------------------------------
# Stage 1: sweep the Comfort Space SIZE and find where it starts to
# deadlock the robot.
#
#     ./scripts/stage1_sweep.sh --dry-run                 plan only, run nothing
#     ./scripts/stage1_sweep.sh --smoke                   seed 5 x {0.8, 1.2, 1.6} x {static, b3o}
#     ./scripts/stage1_sweep.sh                            33 seeds x 5 sizes x 2 policies = 330 trials
#     ./scripts/stage1_sweep.sh --sizes "1.25 1.3 1.35"   refinement around a threshold
#     ./scripts/stage1_sweep.sh --out results/batches/261112_stage1_size_sweep --skip 120
#                                                          resume an interrupted sweep
#
# What it does, in order:
#   1. writes the condition overlays   scripts/make_size_conditions.py
#   2. validates them offline          scripts/check_stage1.py
#   3. writes the trial list           scripts/stage1_manifest.py
#   4. preflight (import / policy)     check_nodes, check_evasion, check_perception
#   5. runs the trials                 scripts/run_campaign.sh  (fresh sim per trial)
#   6. validates the CSV and reports   scripts/check_stage1.py --dir, scripts/stage1_report.py
#
# Every trial of a seed runs on the SAME campaign config; only the
# condition overlay (scenario + comfort_space) differs. Results:
#
#     <out>/manifest.txt        what was asked for
#     <out>/trials.csv          one row per trial, AMR1
#     <out>/fleet_trials.csv    one row per trial per fleet vehicle (a1, a2)
#     <out>/*.json              per-trial detail incl. trajectory and plan
#     <out>/logs/               console output per trial
#     <out>/campaign.log        progress
#     <out>/REPORT.md, tables.md, figures/P1..P8   from stage1_report.py
#
# A deadlock is a result, not an error; the sweep carries on. Nothing is
# ever inferred: a trial that did not run has no row.
# ---------------------------------------------------------------
set -uo pipefail
PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
cd "$PROJECT_ROOT"

SIZES="0.8 1.0 1.2 1.4 1.6"
KINDS="static b3o"
SEEDS="1-33"
OUT=""
DRY_RUN=0
SMOKE=0
SKIP=0
LIMIT_S=1500
FLEET_TAIL=150
NO_REPORT=0
while [ $# -gt 0 ]; do
    case "$1" in
        --sizes)   SIZES="$2"; shift 2 ;;
        --kinds)   KINDS="$2"; shift 2 ;;
        --seeds)   SEEDS="$2"; shift 2 ;;
        --out)     OUT="$2"; shift 2 ;;
        --skip)    SKIP="$2"; shift 2 ;;
        --limit)   LIMIT_S="$2"; shift 2 ;;
        --fleet-tail) FLEET_TAIL="$2"; shift 2 ;;
        --dry-run) DRY_RUN=1; shift ;;
        --smoke)   SMOKE=1; shift ;;
        --no-report) NO_REPORT=1; shift ;;
        -h|--help) sed -n '2,36p' "$0"; exit 0 ;;
        *) echo "[ERROR] unknown option $1" >&2; exit 2 ;;
    esac
done
if [ "$SMOKE" -eq 1 ]; then
    SIZES="0.8 1.2 1.6"; SEEDS="5"
    [ -n "$OUT" ] || OUT="results/batches/$(date +%y%m%d)_stage1_smoke_s5"
fi
[ -n "$OUT" ] || OUT="results/batches/$(date +%y%m%d)_stage1_size_sweep"

echo "== Stage 1 size sweep"
echo "   sizes  : $SIZES m"
echo "   kinds  : $KINDS"
echo "   seeds  : $SEEDS"
echo "   out    : $OUT"
echo "   commit : $(git rev-parse --short HEAD 2>/dev/null || echo n/a)$(git status --porcelain 2>/dev/null | grep -q . && echo '+')"

# 1. conditions
# shellcheck disable=SC2086
python3 scripts/make_size_conditions.py --sizes $SIZES --kinds $KINDS --out configs/stage1 >/dev/null \
    || { echo "[ERROR] could not write the condition files" >&2; exit 2; }
echo "   conditions written to configs/stage1/"

# 2. offline validation
if ! python3 scripts/check_stage1.py --conditions configs/stage1 --seeds "$( [ "$SEEDS" = "${SEEDS%-*}" ] && echo "$SEEDS-$SEEDS" || echo "$SEEDS" )" > /tmp/stage1_check.log 2>&1; then
    echo "[ERROR] offline checks failed:" >&2
    grep -E "FAIL|checks" /tmp/stage1_check.log >&2
    exit 2
fi
echo "   $(grep -m1 'checks passed' /tmp/stage1_check.log)"

# 3. manifest
MANIFEST_DIR="$OUT"
[ "$DRY_RUN" -eq 1 ] && MANIFEST_DIR="$(mktemp -d)"
mkdir -p "$MANIFEST_DIR"
# shellcheck disable=SC2086
python3 scripts/stage1_manifest.py --seeds $SEEDS --sizes $SIZES --kinds $KINDS \
    --fleet-tail "$FLEET_TAIL" --out "$MANIFEST_DIR/manifest.txt" || exit 2
TOTAL=$(grep -vc '^\s*#' "$MANIFEST_DIR/manifest.txt")

if [ "$DRY_RUN" -eq 1 ]; then
    echo
    echo "   plan ($TOTAL trials, first and last 3 lines):"
    grep -v '^\s*#' "$MANIFEST_DIR/manifest.txt" | head -3 | sed 's/^/     /'
    echo "     ..."
    grep -v '^\s*#' "$MANIFEST_DIR/manifest.txt" | tail -3 | sed 's/^/     /'
    echo
    echo "   applied extents per condition:"
    # shellcheck disable=SC2086
    python3 scripts/make_size_conditions.py --sizes $SIZES --kinds $KINDS --out configs/stage1 --table \
        | sed -n '/^condition/,$p' | sed 's/^/     /'
    echo
    echo "   nothing run (--dry-run). At ~4.5 min per trial this is about $(( TOTAL * 9 / 2 / 60 )) h."
    rm -rf "$MANIFEST_DIR"
    exit 0
fi

# 4. preflight, as run_batch.sh does: each exists because something once
# silently did nothing for a whole batch.
FIRST_CFG="$(grep -v '^\s*#' "$MANIFEST_DIR/manifest.txt" | head -1 | awk '{print $2}')"
for check in check_nodes check_evasion check_perception; do
    if ! python3 "scripts/$check.py" --config "$FIRST_CFG" >/dev/null 2>&1 \
       && ! python3 "scripts/$check.py" >/dev/null 2>&1; then
        echo "[ERROR] scripts/$check.py fails on $FIRST_CFG" >&2
        python3 "scripts/$check.py" --config "$FIRST_CFG" >&2 || true
        exit 2
    fi
done
echo "   preflight: check_nodes, check_evasion, check_perception pass"

# 5. run
echo "   running $TOTAL trials (skip $SKIP), log: $OUT/campaign.log"
./scripts/run_campaign.sh --manifest "$OUT/manifest.txt" --out "$OUT" --skip "$SKIP" --limit "$LIMIT_S"

# 6. validate and report
echo
if [ -f "$OUT/trials.csv" ]; then
    python3 scripts/check_stage1.py --conditions configs/stage1 --seeds 1-1 --dir "$OUT" 2>&1 | tail -4
    if [ "$NO_REPORT" -eq 0 ]; then
        python3 scripts/stage1_report.py --dir "$OUT" || echo "[WARN] report failed; see above" >&2
    fi
else
    echo "[WARN] no trials.csv in $OUT -- nothing was recorded" >&2
    exit 1
fi
