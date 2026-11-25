#!/usr/bin/env bash
# ---------------------------------------------------------------
# After scripts/final_pipeline.py: reports and videos, when no trial runs.
#
#     setsid nohup ./scripts/final_post.sh > results/batches/261125_final_post.log 2>&1 &
#
# Waits for "all phases finished" in results/batches/261125_final_pipeline.log
# (checked every 5 min), then: Stage 1 report, Stage 2 proxy (as recorded),
# Stage 3 report, Stage 1 and Stage 3 videos. Video rendering is CPU-heavy,
# so it never overlaps a trial.
# ---------------------------------------------------------------
set -uo pipefail
PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
cd "$PROJECT_ROOT"
LOG=results/batches/261125_final_pipeline.log
until grep -q "all phases finished" "$LOG" 2>/dev/null; do
    pgrep -f "scripts/final_pipeline.py" >/dev/null || { echo "$(date) pipeline not running and not finished; stop"; exit 1; }
    sleep 300
done
echo "$(date) pipeline finished; reports and videos"
python3 scripts/final_stage1_report.py
python3 scripts/final_stage3_report.py
nice -n 10 python3 scripts/final_videos.py stage1
nice -n 10 python3 scripts/final_videos.py stage3
echo "$(date) post done"
