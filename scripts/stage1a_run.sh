#!/usr/bin/env bash
# ---------------------------------------------------------------
# Stage 1 controlled experiment, every phase in order, unattended.
#
#     ./scripts/stage1a_run.sh --out results/batches/261114_stage1a
#     ./scripts/stage1a_run.sh --out <same> --from refine     resume at a phase
#     ./scripts/stage1a_run.sh --variant nobuf --out <dir> --from scan
#         the buffer-off CANDIDATE (scripts/stage1a_plan.py VARIANTS); its scan
#         manifest must exist (stage1a_plan.py --phase scan --variant nobuf)
#
# Phases (scripts/stage1a_plan.py has the design):
#   scan      step0 (B1, every width) and the 1A-1 Static scan -> <out>/step0, <out>/scan
#   refine    3 runs at the observed boundary of each width     -> <out>/refine
#   fit       one parameter, the wall clearance -> configs/stage1a/model_exec.json
#   predict   R_exec_model for 1A-2 and 1-B, written BEFORE they run
#             -> configs/stage1a/predictions_exec.json
#   validate  1A-2 contexts and 1-B hold-outs                    -> <out>/1a2, <out>/1b
#   lowe      low-e replication (buffer candidates only; manifest written at scan)
#                                                                -> <out>/lowe
#   report    scripts/stage1a_report.py over all of them         -> <out>/REPORT.md
#
# The model is fitted on 1A-1 only; 1A-2 and 1-B are scored against the
# predictions file, which the plan script refuses to overwrite.
# ---------------------------------------------------------------
set -uo pipefail
PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
cd "$PROJECT_ROOT"

OUT=""; FROM="scan"; LIMIT_S=600; VARIANT="default"
while [ $# -gt 0 ]; do
    case "$1" in
        --out)   OUT="$2"; shift 2 ;;
        --from)  FROM="$2"; shift 2 ;;
        --limit) LIMIT_S="$2"; shift 2 ;;
        --variant) VARIANT="$2"; shift 2 ;;
        -h|--help) sed -n '2,24p' "$0"; exit 0 ;;
        *) echo "[ERROR] unknown option $1" >&2; exit 2 ;;
    esac
done
[ -n "$OUT" ] || { echo "[ERROR] --out is required" >&2; exit 2; }
mkdir -p "$OUT"
LOG="$OUT/stage1a.log"
SFX=""; [ "$VARIANT" != "default" ] && SFX="_$VARIANT"
MODEL="configs/stage1a/model_exec$SFX.json"
say() { echo "$(date +%H:%M:%S)  $*" | tee -a "$LOG"; }

PHASES=(scan refine fit predict validate lowe report)
start=0
for i in "${!PHASES[@]}"; do [ "${PHASES[$i]}" = "$FROM" ] && start=$i; done

run_manifest() {   # manifest, out dir
    local m="$1" d="$2"
    mkdir -p "$d"
    cp "$m" "$d/manifest.txt"
    local n; n=$(grep -vc '^\s*#' "$d/manifest.txt")
    say "running $n trials from $m -> $d"
    ./scripts/run_campaign.sh --manifest "$d/manifest.txt" --out "$d" --limit "$LIMIT_S" >> "$LOG" 2>&1
}

for phase in "${PHASES[@]:$start}"; do
    say "== phase $phase"
    case "$phase" in
        scan)
            [ -z "$SFX" ] && run_manifest configs/stage1a/manifests/step0.txt "$OUT/step0"
            run_manifest "configs/stage1a/manifests/1a1_scan$SFX.txt" "$OUT/scan" ;;
        refine)
            python3 scripts/stage1a_calibrate.py --variant "$VARIANT" --dir "$OUT/scan" --refine "$OUT/refine_manifest.txt" \
                | tee -a "$LOG"
            if [ "$(grep -vc '^\s*#' "$OUT/refine_manifest.txt")" -gt 0 ]; then
                run_manifest "$OUT/refine_manifest.txt" "$OUT/refine"
            else
                say "nothing to refine"
            fi ;;
        fit)
            dirs=(--dir "$OUT/scan"); [ -d "$OUT/refine" ] && dirs+=(--dir "$OUT/refine")
            python3 scripts/stage1a_calibrate.py --variant "$VARIANT" "${dirs[@]}" --fit | tee -a "$LOG"
            [ -f "$MODEL" ] || { say "no model fitted; stopping"; exit 1; } ;;
        predict)
            python3 scripts/stage1a_plan.py --phase predict --variant "$VARIANT" --model "$MODEL" \
                | tee -a "$LOG"
            [ -f "configs/stage1a/predictions_exec$SFX.json" ] || { say "no predictions; stopping"; exit 1; } ;;
        validate)
            run_manifest "configs/stage1a/manifests/1a2$SFX.txt" "$OUT/1a2"
            run_manifest "configs/stage1a/manifests/1b$SFX.txt" "$OUT/1b" ;;
        lowe)
            if [ -n "$SFX" ] && [ -f "configs/stage1a/manifests/lowe$SFX.txt" ]; then
                run_manifest "configs/stage1a/manifests/lowe$SFX.txt" "$OUT/lowe"
            else
                say "no low-e manifest for this variant"
            fi ;;
        report)
            dirs=()
            for d in step0 scan refine 1a2 1b lowe; do [ -d "$OUT/$d" ] && dirs+=(--dir "$OUT/$d"); done
            python3 scripts/stage1a_report.py "${dirs[@]}" --out "$OUT" \
                --predictions "configs/stage1a/predictions$SFX.json" \
                $( [ -f "configs/stage1a/predictions_exec$SFX.json" ] && echo --predictions "configs/stage1a/predictions_exec$SFX.json" ) \
                | tee -a "$LOG" ;;
    esac
done
say "== done"
