#!/usr/bin/env bash
# ---------------------------------------------------------------
# Look at a scenario in Isaac Sim before deciding to run anything.
#
#   ./scripts/preview_scenario.sh --seed 1 --humans 8 --amrs 1
#   ./scripts/preview_scenario.sh --config configs/scenarios/paper_h8_a1_scr_s0001.yaml
#   ./scripts/preview_scenario.sh --seed 5 --humans 8 --amrs 3 --seconds 90
#
# What it does, in order:
#
#   1. generates the scenario if it was given factors rather than a
#      config, and REFUSES to go on if it does not validate
#   2. prints everything about it -- people, states, exchanges, walk
#      routes, mission, how tight the aisle is under each policy
#   3. draws the plan as a PNG, with everything labelled
#   4. opens Isaac Sim with the scene, the people walking and the
#      preview decals on the floor
#
# It does NOT start Nav2 and does NOT drive the robot. The AMR stands
# in its start slot so its pose can be seen; the people walk, which is
# the part worth watching. Close the window, or wait for --seconds.
#
# Nothing is measured and no trial row is written.
# ---------------------------------------------------------------
set -uo pipefail

PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
cd "$PROJECT_ROOT"

CONFIG=""
SEED=1
HUMANS=8
AMRS=1
PROVIDER="scripted"
POLICY="b3"
SECONDS_LIMIT=0
HEADLESS=()

while [ $# -gt 0 ]; do
    case "$1" in
        --config)   CONFIG="$2"; shift 2 ;;
        --seed)     SEED="$2"; shift 2 ;;
        --humans)   HUMANS="$2"; shift 2 ;;
        --amrs)     AMRS="$2"; shift 2 ;;
        --provider) PROVIDER="$2"; shift 2 ;;
        --policy)   POLICY="$2"; shift 2 ;;
        --seconds)  SECONDS_LIMIT="$2"; shift 2 ;;
        --headless) HEADLESS+=("--headless"); shift ;;
        -h|--help)  sed -n '2,24p' "$0"; exit 0 ;;
        *) echo "[ERROR] unknown option $1" >&2; exit 2 ;;
    esac
done

if [ -z "$CONFIG" ]; then
    echo "[step] generating the scenario"
    python3 scripts/make_scenario.py --seed "$SEED" --humans "$HUMANS" \
        --amrs "$AMRS" --provider "$PROVIDER" --policy "$POLICY" || exit 1
    CONFIG="configs/scenarios/paper_h${HUMANS}_a${AMRS}_${PROVIDER:0:3}_s$(printf '%04d' "$SEED").yaml"
else
    echo "[step] reading $CONFIG"
    python3 - "$CONFIG" <<'PY' || exit 1
import sys
sys.path.insert(0, '.')
from comfort_core import load_config
cfg = load_config(common_path=sys.argv[1])
meta = cfg.get("scenario_meta") or {}
if not meta:
    print(f"[warn] {sys.argv[1]} has no scenario_meta; it was not written by "
          f"scripts/make_scenario.py", file=sys.stderr)
PY
fi

[ -f "$CONFIG" ] || { echo "[ERROR] no such config: $CONFIG" >&2; exit 1; }

echo "[step] drawing the plan"
python3 scripts/preview_figure.py --config "$CONFIG" --policy "$POLICY"

echo "[step] opening Isaac Sim  (close the window when you have seen enough)"
echo "       preview decals ON; this is NOT a measurement run"

# markers.enabled and preview.enabled are off in every config, on
# purpose -- see configs/common.yaml. A preview turns them on for the
# length of the window by overlaying a tiny config on top of the
# scenario, so the scenario file itself still says what a measured run
# reads.
OVERLAY="$(mktemp --suffix=.yaml)"
cat > "$OVERLAY" <<YAML
# Written by scripts/preview_scenario.sh. Throwaway: it exists only to
# switch the floor decals on for a look, and no trial ever reads it.
base: $CONFIG
markers:
  enabled: true
preview:
  enabled: true
YAML
trap 'rm -f "$OVERLAY"' EXIT

ARGS=(--config "$OVERLAY" --scenario "$POLICY")
[ "$SECONDS_LIMIT" != "0" ] && ARGS+=(--max-seconds "$SECONDS_LIMIT")
./scripts/run_sim.sh "${HEADLESS[@]+"${HEADLESS[@]}"}" "${ARGS[@]}"
