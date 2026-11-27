#!/usr/bin/env bash
# FINAL stage23_900 MAIN runs, after the pilots are frozen and the manifests committed.
#   ./scripts/final_stage23_900_main.sh            # resumable: finished trials (DONE) are skipped
# Order: Stage 2 MAIN (750) -> Stage 3 MAIN (150) -> Stage 3 duplicate control (6, outside the 900).
# ETA_REPORT.md / progress_<ts>.json are rewritten by final_stage23_900_eta.py --watch every 30 MAIN trials.
set -u
cd "$(dirname "$0")/.."
P=results/batches/261127_stage23_900
mkdir -p ${P}_status
python3 scripts/final_stage23_900_eta.py --watch > ${P}_status/eta_watch.log 2>&1 &
ETA_PID=$!
for b in s2main s3main s3dup; do
  echo "$(TZ=Asia/Seoul date '+%F %T') start $b"
  python3 scripts/final_run.py --dir ${P}_$b
  echo "$(TZ=Asia/Seoul date '+%F %T') end $b (exit $?)"
done
python3 scripts/final_stage23_900_eta.py --note "all runs finished" > /dev/null
kill $ETA_PID 2>/dev/null
echo "$(TZ=Asia/Seoul date '+%F %T') MAIN_DONE"
