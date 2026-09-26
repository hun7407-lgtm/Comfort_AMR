#!/usr/bin/env bash
# ---------------------------------------------------------------
# Run one complete trial of a scenario, start to finish.
#
#     ./scripts/run_scenario.sh b2
#     ./scripts/run_scenario.sh b3 --headless
#     ./scripts/run_scenario.sh b3 --config configs/demo_walker.yaml
#     ./scripts/run_scenario.sh b3 --record      also save an mp4 of the trial
#     ./scripts/run_scenario.sh b3 --record-costmap
#                                    also save what Nav2 sees, as mp4 and gif
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
    echo "usage: $0 <b1|b2|b3> [--headless]" >&2
    exit 2
fi
shift

HEADLESS=()
CONFIG=()
RECORD=0
RECORD_COSTMAP=0
while [ $# -gt 0 ]; do
    case "$1" in
        --headless) HEADLESS+=("--headless"); shift ;;
        --config)   CONFIG+=("--config" "$2"); shift 2 ;;
        --record)   RECORD=1; shift ;;
        --record-costmap) RECORD_COSTMAP=1; shift ;;
        *) shift ;;
    esac
done

# Recordings go to results/videos/<version>/<scenario>/<condition>/<stamp>_<view>.*
#   version  : recording.version in configs/common.yaml, one folder per
#              behaviour worth comparing
#   scenario : b1_no_comfort_space, b2_static, b3_state_adaptive
#   condition: standing_only (common.yaml) or with_walker (demo_walker.yaml);
#              any other --config is used by its file name
#   view     : camera.mp4, costmap.mp4, costmap.gif, costmap_small.gif
RECORD_ARGS=()
case "$SCENARIO" in
    b1) SCENARIO_DIR="b1_no_comfort_space" ;;
    b2) SCENARIO_DIR="b2_static" ;;
    b3) SCENARIO_DIR="b3_state_adaptive" ;;
    *)  SCENARIO_DIR="$(basename "$SCENARIO" .yaml)" ;;
esac
CONDITION="standing_only"
if [ ${#CONFIG[@]} -gt 0 ]; then
    CONDITION="$(basename "${CONFIG[1]}" .yaml)"
    [ "$CONDITION" = "demo_walker" ] && CONDITION="with_walker"
    [ "$CONDITION" = "common" ] && CONDITION="standing_only"
fi
STAMP="$(date +%Y%m%d_%H%M)"
VERSION="$(python3 -c "import yaml;print(yaml.safe_load(open('$PROJECT_ROOT/configs/common.yaml'))['recording'].get('version','unversioned'))")"
VIDEO_DIR="$PROJECT_ROOT/results/videos/$VERSION/$SCENARIO_DIR/$CONDITION"
VIDEO_NAME="${STAMP}_camera"
COSTMAP_NAME="${STAMP}_costmap"
COSTMAP_FPS=5
COSTMAP_DIR="$VIDEO_DIR/.${COSTMAP_NAME}_frames"
COSTMAP_PID=""
if [ "$RECORD" -eq 1 ]; then
    FRAME_DIR="$VIDEO_DIR/.${VIDEO_NAME}_frames"
    mkdir -p "$FRAME_DIR"
    RECORD_ARGS=(--record "$FRAME_DIR")
fi

RUN_DIR="$(mktemp -d)"
SIM_LOG="$RUN_DIR/sim.log"
NAV_LOG="$RUN_DIR/nav2.log"

cleanup() {
    echo
    if [ "$RECORD" -eq 1 ] || [ "$RECORD_COSTMAP" -eq 1 ]; then
        # A few more seconds of the robot at rest, so the video does not
        # cut off on the frame the trial was decided.
        sleep 3
    fi
    if [ -n "$COSTMAP_PID" ]; then
        kill -INT "$COSTMAP_PID" 2>/dev/null
        wait "$COSTMAP_PID" 2>/dev/null
    fi
    echo "[step] shutting down"
    ./scripts/stop_all.sh >/dev/null 2>&1
    if [ "$RECORD_COSTMAP" -eq 1 ]; then
        echo "[step] encoding costmap view"
        ./scripts/encode_video.sh "$COSTMAP_DIR" "$VIDEO_DIR/${COSTMAP_NAME}.mp4" "$COSTMAP_FPS" \
            && python3 scripts/frames_to_gif.py "$COSTMAP_DIR" "$VIDEO_DIR/${COSTMAP_NAME}.gif" \
                   --fps "$COSTMAP_FPS" \
            && python3 scripts/frames_to_gif.py "$COSTMAP_DIR" "$VIDEO_DIR/${COSTMAP_NAME}_small.gif" \
                   --fps "$COSTMAP_FPS" --width 640 \
            && rm -rf "$COSTMAP_DIR"
    fi
    if [ "$RECORD" -eq 1 ]; then
        echo "[step] encoding video"
        ./scripts/encode_video.sh "$FRAME_DIR" "$VIDEO_DIR/$VIDEO_NAME.mp4" \
            && rm -rf "$FRAME_DIR"
    fi
    echo "[info] logs kept in $RUN_DIR"
}
trap cleanup EXIT

echo "[step] clearing anything already running"
./scripts/stop_all.sh >/dev/null 2>&1
sleep 2

echo "[step] starting Isaac Sim   (log: $SIM_LOG)"
./scripts/run_sim.sh "${HEADLESS[@]+"${HEADLESS[@]}"}" \
    "${CONFIG[@]+"${CONFIG[@]}"}" --scenario "$SCENARIO" \
    "${RECORD_ARGS[@]+"${RECORD_ARGS[@]}"}" > "$SIM_LOG" 2>&1 &

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

if [ "$RECORD_COSTMAP" -eq 1 ]; then
    mkdir -p "$COSTMAP_DIR"
    echo "[step] recording costmap view"
    # Same environment as the other ROS 2 nodes.
    ( set +u; source /opt/ros/jazzy/setup.bash; set -u
      exec python3 scripts/costmap_recorder.py "${CONFIG[@]+"${CONFIG[@]}"}" \
          --scenario "$SCENARIO" --out "$COSTMAP_DIR" --fps "$COSTMAP_FPS" ) \
        > "$RUN_DIR/costmap.log" 2>&1 &
    COSTMAP_PID=$!
fi

echo "[step] running the trial"
./scripts/run_nav2_goal.sh "${CONFIG[@]+"${CONFIG[@]}"}" --scenario "$SCENARIO"
exit $?
