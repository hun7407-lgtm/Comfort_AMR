#!/usr/bin/env bash
# ---------------------------------------------------------------
# Paper / presentation captures, unattended (visual documentation only).
#
#     python3 scripts/paper_capture_plan.py --final-variant <v>   # writes the jobs
#     ./scripts/paper_captures.sh                                 # stills, then runs
#     ./scripts/paper_captures.sh --only stills|runs [--match '^S1_']
#     python3 scripts/paper_capture_post.py                       # frames, annotations, index
#
# Stills: isaac_sim/scripts/paper_shots.py, one Isaac Sim per job.
# Runs:   ./scripts/run_scenario.sh <overlay> --config <capture config> --record
#         --record-costmap, one trial per line of runs.txt. Videos land in
#         results/videos/paper_261114/ (git-ignored); the capture trial rows
#         in <root>/capture_trials (NOT a results batch).
# Nothing here writes into results/batches or changes a config it did not generate.
# ---------------------------------------------------------------
set -uo pipefail
PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
cd "$PROJECT_ROOT"

ROOT="results/captures/paper_261114"; ONLY=""; MATCH=""; LIMIT_S=600
while [ $# -gt 0 ]; do
    case "$1" in
        --root) ROOT="$2"; shift 2 ;;
        --only) ONLY="$2"; shift 2 ;;
        --match) MATCH="$2"; shift 2 ;;     # runs: only tags matching this regex
        -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
        *) echo "[ERROR] unknown option $1" >&2; exit 2 ;;
    esac
done
mkdir -p "$ROOT/logs" "$ROOT/capture_trials"
LOG="$ROOT/captures.log"
say() { echo "$(date +%H:%M:%S)  $*" | tee -a "$LOG"; }
stop_everything() {
    ./scripts/stop_all.sh >/dev/null 2>&1
    pkill -f '[r]un_amr_sim' 2>/dev/null
    pkill -f '[p]aper_shots' 2>/dev/null
    sleep 4
}

if [ "$ONLY" != "runs" ]; then
    for job in "$ROOT"/jobs/still_*.json; do
        name="$(basename "$job" .json)"
        stop_everything
        say "still $name"
        # Bundled Jazzy libraries, as scripts/run_sim.sh (never the system /opt/ros).
        ( unset ROS_DISTRO; set +u; source "$HOME/isaacsim/setup_ros_env.sh"; set -u
          export PYTHONPATH="$PROJECT_ROOT:${PYTHONPATH:-}" PYTHONUNBUFFERED=1
          timeout -k 30 900 "$HOME/isaacsim/python.sh" isaac_sim/scripts/paper_shots.py --job "$job" ) \
            > "$ROOT/logs/$name.log" 2>&1
        say "  exit $? $(grep -m1 '\[paper_shots\]' "$ROOT/logs/$name.log")"
    done
fi
if [ "$ONLY" != "stills" ]; then
    while read -r scenario config tag; do
        case "$scenario" in ''|\#*) continue ;; esac
        [ -n "$MATCH" ] && ! [[ "$tag" =~ $MATCH ]] && continue
        stop_everything
        say "run $tag"
        timeout -k 30 "$LIMIT_S" ./scripts/run_scenario.sh "$scenario" --config "$config" --headless \
            --record --record-costmap --results "$ROOT/capture_trials" > "$ROOT/logs/$tag.log" 2>&1 < /dev/null
        say "  exit $? result $(grep -m1 '  result             : ' "$ROOT/logs/$tag.log" | sed 's/.*: //')"
    done < "$ROOT/runs.txt"
fi
stop_everything
say "done"
