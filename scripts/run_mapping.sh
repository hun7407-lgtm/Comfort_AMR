#!/usr/bin/env bash
# ---------------------------------------------------------------
# Build the map the 3D trials navigate on, once.
#
#     ./scripts/run_mapping.sh --config configs/factory_3d.yaml
#     ./scripts/run_mapping.sh --config configs/factory_long_3d.yaml --headless
#     ./scripts/run_mapping.sh --config configs/factory_3d.yaml --laps 3
#
# Starts Isaac Sim with the bay empty, brings up slam_toolbox, drives a
# lap of the aisles, saves the map and moves it into world coordinates.
# Then every trial on that config loads the same file.
#
# One map for all trials is deliberate. Mapping per trial would give B1,
# B2 and B3 three slightly different buildings to plan in, and the
# comparison is supposed to be about the Comfort Space policy and nothing
# else. See docs/EXPERIMENT.md section 2.
#
# Re-run it when the bay changes. Nothing re-runs it automatically: an
# experiment should not be able to silently change its own map.
# ---------------------------------------------------------------

set -uo pipefail

PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
cd "$PROJECT_ROOT"

CONFIG="configs/factory_3d.yaml"
HEADLESS=()
LAPS=2
PATROL_ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --config)   CONFIG="$2"; shift 2 ;;
        --headless) HEADLESS+=("--headless"); shift ;;
        --laps)     LAPS="$2"; shift 2 ;;
        --speed)    PATROL_ARGS+=("--speed" "$2"); shift 2 ;;
        *) echo "[ERROR] unknown option $1" >&2; exit 2 ;;
    esac
done

STEM="$(basename "$CONFIG" .yaml)"
MAPPING_CONFIG="nav3d/mapping_${STEM}.yaml"
if [ ! -f "$MAPPING_CONFIG" ]; then
    echo "[ERROR] $CONFIG has no mapping config at $MAPPING_CONFIG." >&2
    echo "        The map is made of the building with nobody in it, so each" >&2
    echo "        scene needs one. Copy nav3d/mapping_factory_3d.yaml and" >&2
    echo "        point its base: at $CONFIG." >&2
    exit 2
fi

# Where the map goes, and what its name has to be: whatever the scene
# config says it navigates on.
MAP_YAML="$(python3 -c "
import sys; sys.path.insert(0, '.')
from comfort_core import load_config, resolve_path
cfg = load_config(common_path='$CONFIG')
named = cfg.get('nav2', {}).get('map')
if not named:
    raise SystemExit('no nav2.map in the config')
print(resolve_path(named))")"
if [ -z "$MAP_YAML" ]; then
    echo "[ERROR] $CONFIG does not name a map under nav2.map, so it is not a" >&2
    echo "        SLAM config. The 2D configs generate their map instead." >&2
    exit 2
fi
MAP_DIR="$(dirname "$MAP_YAML")"
MAP_STEM="$(basename "$MAP_YAML" .yaml)"
mkdir -p "$MAP_DIR"

RUN_DIR="$(mktemp -d)"
SIM_LOG="$RUN_DIR/sim.log"
SLAM_LOG="$RUN_DIR/slam.log"

echo "[info] scene config  : $CONFIG"
echo "[info] mapping scene : $MAPPING_CONFIG   (people off)"
echo "[info] map goes to   : $MAP_YAML"
echo "[info] laps          : $LAPS"
echo

cleanup() {
    echo "[step] shutting down"
    ./scripts/stop_all.sh >/dev/null 2>&1
    echo "[info] logs kept in $RUN_DIR"
}
trap cleanup EXIT

echo "[step] clearing anything already running"
./scripts/stop_all.sh >/dev/null 2>&1
sleep 2

echo "[step] starting Isaac Sim   (log: $SIM_LOG)"
./scripts/run_sim.sh "${HEADLESS[@]+"${HEADLESS[@]}"}" --config "$MAPPING_CONFIG" \
    > "$SIM_LOG" 2>&1 &

for _ in $(seq 1 120); do
    grep -q "Simulation running" "$SIM_LOG" 2>/dev/null && break
    sleep 3
done
if ! grep -q "Simulation running" "$SIM_LOG" 2>/dev/null; then
    echo "[ERROR] Isaac Sim did not come up. Last lines:" >&2
    tail -20 "$SIM_LOG" >&2
    exit 1
fi
grep -E "humans placed|bay built|r 0\.[0-9]+" "$SIM_LOG" | sed 's/^/       /'

echo "[step] starting slam_toolbox (log: $SLAM_LOG)"
(
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    set -u
    export PYTHONPATH="${PROJECT_ROOT}:${PYTHONPATH:-}"
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    export PYTHONUNBUFFERED=1
    export COMFORT_AMR_CONFIG="$MAPPING_CONFIG"
    exec ros2 launch "$PROJECT_ROOT/nav3d/mapping.launch.py"
) > "$SLAM_LOG" 2>&1 &

for _ in $(seq 1 40); do
    grep -q "Registering sensor" "$SLAM_LOG" 2>/dev/null && break
    sleep 2
done
if ! grep -qE "Registering sensor|activating" "$SLAM_LOG" 2>/dev/null; then
    echo "[ERROR] slam_toolbox did not start mapping. Last lines:" >&2
    tail -25 "$SLAM_LOG" >&2
    exit 1
fi
echo "       mapping"

echo "[step] driving the lap"
(
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    set -u
    export PYTHONPATH="${PROJECT_ROOT}:${PYTHONPATH:-}"
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    export PYTHONUNBUFFERED=1
    exec python3 "$PROJECT_ROOT/nav3d/patrol.py" --config "$MAPPING_CONFIG" \
        --laps "$LAPS" "${PATROL_ARGS[@]+"${PATROL_ARGS[@]}"}"
)
PATROL_RC=$?
if [ "$PATROL_RC" -ne 0 ]; then
    echo "[ERROR] the lap did not finish, so the map would have holes in it." >&2
    echo "        Not saving. Simulation log: $SIM_LOG" >&2
    exit 1
fi

echo "[step] saving the map"
# Saved from the map directory so the yaml refers to its pgm by name and
# the pair can be moved or committed together.
(
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    set -u
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    cd "$MAP_DIR"
    exec ros2 run nav2_map_server map_saver_cli -f "$MAP_STEM" \
        --ros-args -p use_sim_time:=true
) > "$RUN_DIR/save.log" 2>&1
SAVE_RC=$?
if [ "$SAVE_RC" -ne 0 ] || [ ! -f "$MAP_YAML" ]; then
    echo "[ERROR] map_saver_cli failed. Last lines:" >&2
    tail -20 "$RUN_DIR/save.log" >&2
    exit 1
fi

echo "[step] placing the map in world coordinates"
python3 nav3d/place_map.py --config "$CONFIG" || exit 1

echo
echo "[done] $MAP_YAML"
echo "       run a trial on it with:"
echo "         ./scripts/run_b3.sh --headless --config $CONFIG"
