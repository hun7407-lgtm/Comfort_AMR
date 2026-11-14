#!/usr/bin/env bash
# ---------------------------------------------------------------
# Stage 3: validate the human-study-calibrated B3-O in simulation.
#
#     ./scripts/stage3_run.sh --dry-run
#     ./scripts/stage3_run.sh                       b3o_initial vs b3o_optimized, 33 seeds
#     ./scripts/stage3_run.sh --with-static         + static_baseline (3 conditions)
#     ./scripts/stage3_run.sh --r-opt 1.0           write b3o_optimized from R_opt first
#     ./scripts/stage3_run.sh --params my_b3o.yaml  ... or from a calibrated block
#     ./scripts/stage3_run.sh --seeds 1-8 --out results/batches/261215_stage3_check
#
# Main comparison (revised design, docs/STAGES.md):
#     ./scripts/stage3_run.sh --main
#         B2 Static Calibrated vs B3-O Dynamic Selection, seeds 1-100, paired;
#         the policy order alternates with the seed (odd: B2 first). Model,
#         buffer and R_nav_safe bound come from configs/frozen/navigation_requirements.yaml
#         (Stage 1); refused while that file is missing or
#         configs/frozen/comfort_requirements.yaml (Stage 2) is null. Both
#         policies must carry the same frozen buffer.
#
# Conditions are the overlays in configs/stage3/ (scripts/stage3_make_configs.py).
# Every condition of a seed runs on the same cross campaign config, as in
# Stage 1. b3o_optimized is refused while its target_size_m is still null:
# R_opt is a Stage 2 measurement, not a default.
#
# Afterwards: scripts/stage3_report.py --dir <out>  (tables, paired
# deltas, Initial-vs-Optimized trajectory overlays).
# ---------------------------------------------------------------
set -uo pipefail
PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
cd "$PROJECT_ROOT"

SEEDS=""; OUT=""; DRY_RUN=0; WITH_STATIC=0; SKIP=0; LIMIT_S=1500; FLEET_TAIL=150; MAIN=0
MAKE_ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --seeds)   SEEDS="$2"; shift 2 ;;
        --out)     OUT="$2"; shift 2 ;;
        --skip)    SKIP="$2"; shift 2 ;;
        --limit)   LIMIT_S="$2"; shift 2 ;;
        --fleet-tail) FLEET_TAIL="$2"; shift 2 ;;
        --with-static) WITH_STATIC=1; shift ;;
        --main)    MAIN=1; shift ;;
        --selection-model) MAKE_ARGS+=(--selection-model "$2"); shift 2 ;;
        --r-opt)   MAKE_ARGS+=(--r-opt "$2"); shift 2 ;;
        --params)  MAKE_ARGS+=(--params "$2"); shift 2 ;;
        --static-size) MAKE_ARGS+=(--static-size "$2"); shift 2 ;;
        --dry-run) DRY_RUN=1; shift ;;
        -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "[ERROR] unknown option $1" >&2; exit 2 ;;
    esac
done
if [ "$MAIN" -eq 1 ]; then
    [ -f configs/frozen/navigation_requirements.yaml ] || {
        echo "[ERROR] --main needs configs/frozen/navigation_requirements.yaml (scripts/stage1a_finalize.py --freeze)" >&2; exit 2; }
    MAKE_ARGS+=(--frozen)
    [ -n "$SEEDS" ] || SEEDS="1-100"
    [ -n "$OUT" ] || OUT="results/batches/$(date +%y%m%d)_stage3b_main"
    CONDS="b2_static_calibrated b3o_selection"
else
    [ -n "$SEEDS" ] || SEEDS="1-33"
    [ -n "$OUT" ] || OUT="results/batches/$(date +%y%m%d)_stage3_validation"
    CONDS="b3o_initial b3o_optimized"
    [ "$WITH_STATIC" -eq 1 ] && CONDS="static_baseline $CONDS"
fi

echo "== Stage 3 validation"
echo "   conditions : $CONDS"
echo "   seeds      : $SEEDS"
echo "   out        : $OUT"

python3 scripts/stage3_make_configs.py "${MAKE_ARGS[@]+"${MAKE_ARGS[@]}"}" || exit 2

# Refuse a template optimized condition, and check the overlays load.
python3 - "$CONDS" "$SEEDS" "$MAIN" <<'EOF' || exit 2
import sys
sys.path.insert(0, '.')
from comfort_core import load_config
from comfort_core.config import load_with_base, SCENARIO_ALLOWED_SECTIONS
from comfort_core.comfort_space import make_policy, geometry_record
conds, seeds = sys.argv[1].split(), sys.argv[2]
a, b = (seeds.split('-') + [seeds])[:2]
if sys.argv[3] == "1":
    from comfort_core.selection import frozen_inputs, frozen_navigation
    try:
        nav = frozen_navigation()
        frozen_inputs()
    except ValueError as exc:
        sys.exit(f"[ERROR] {exc}")
    bufs = {c: (load_with_base(f"configs/stage3/{c}.yaml")["comfort_space"].get("costmap") or {}).get("buffer_width")
            for c in conds}
    if set(bufs.values()) != {nav["comfort_buffer_m"]}:
        sys.exit(f"[ERROR] buffers {bufs} differ from the frozen {nav['comfort_buffer_m']} m")
for c in conds:
    path = f"configs/stage3/{c}.yaml"
    ov = load_with_base(path)
    assert set(ov) <= SCENARIO_ALLOWED_SECTIONS, f"{path}: sections {set(ov)}"
    if ov["comfort_space"].get("policy") == "static" and ov["comfort_space"].get("static_radius") is None:
        sys.exit(f"[ERROR] {path} is a TEMPLATE (static_radius null): B2 is calibrated from the "
                 f"frozen Stage 2 file, which is not filled yet.")
    sc = ov["comfort_space"].get("aisle_scaling", {}) or {}
    if sc.get("enabled") and sc.get("target_size_m") is None:
        sys.exit(f"[ERROR] {path} is a TEMPLATE (target_size_m null). Give --r-opt R or --params FILE "
                 f"with the Stage 2 result; nothing is run with an invented R_opt.")
    ref = None
    for s in range(int(a), int(b) + 1):
        cfg = load_config(scenario=path, common_path=f"configs/campaigns/cross/campaign_h8_a3_cross_s{s}.yaml")
        try:
            pol = make_policy(cfg)
        except ValueError as exc:
            sys.exit(f"[ERROR] {path}: {exc}")
        g = geometry_record(cfg, pol)
        ref = ref or g
        assert g == ref, f"{path}: geometry differs between seeds"
    print(f"   {c:16s} size {g['comfort_size_m']} m  engaged F/S/R {g['cs_engaged_front_m']}/{g['cs_engaged_side_m']}/{g['cs_engaged_rear_m']}"
          f"  stationary {g['cs_stationary_front_m']}/{g['cs_stationary_side_m']}/{g['cs_stationary_rear_m']}"
          f"  moving@1.0 {g['cs_moving_front_m']}/{g['cs_moving_side_m']}/{g['cs_moving_rear_m']}")
EOF

MANIFEST_DIR="$OUT"; [ "$DRY_RUN" -eq 1 ] && MANIFEST_DIR="$(mktemp -d)"
mkdir -p "$MANIFEST_DIR"
python3 - "$CONDS" "$SEEDS" "$FLEET_TAIL" "$MANIFEST_DIR/manifest.txt" "$MAIN" <<'EOF' || exit 2
import sys
sys.path.insert(0, 'scripts')
from stage1_manifest import parse_seeds
conds, seeds, tail, out = sys.argv[1].split(), parse_seeds([sys.argv[2]]), sys.argv[3], sys.argv[4]
alternate = sys.argv[5] == "1"
lines = [f"# Stage 3 validation: {' vs '.join(conds)} x {len(seeds)} seeds = {len(conds) * len(seeds)} trials",
         "# cross scenario; same campaign config for every condition of a seed"]
for s in seeds:
    # Paired main comparison: alternate which policy runs first, so a slow
    # drift over the batch does not always land on the same one.
    order = conds if not alternate or s % 2 else list(reversed(conds))
    for c in order:
        lines.append(f"configs/stage3/{c}.yaml configs/campaigns/cross/campaign_h8_a3_cross_s{s}.yaml --fleet-tail {tail}")
open(out, 'w').write("\n".join(lines) + "\n")
print(f"   manifest: {len(lines) - 2} trials -> {out}")
EOF
TOTAL=$(grep -vc '^\s*#' "$MANIFEST_DIR/manifest.txt")
if [ "$DRY_RUN" -eq 1 ]; then
    grep -v '^\s*#' "$MANIFEST_DIR/manifest.txt" | head -3 | sed 's/^/     /'
    echo "     ..."
    echo "   nothing run (--dry-run); about $(( TOTAL * 9 / 2 / 60 )) h at ~4.5 min per trial"
    rm -rf "$MANIFEST_DIR"; exit 0
fi
FIRST_CFG="$(grep -v '^\s*#' "$OUT/manifest.txt" | head -1 | awk '{print $2}')"
for check in check_nodes check_evasion check_perception; do
    python3 "scripts/$check.py" --config "$FIRST_CFG" >/dev/null 2>&1 \
        || python3 "scripts/$check.py" >/dev/null 2>&1 \
        || { echo "[ERROR] scripts/$check.py fails" >&2; exit 2; }
done
echo "   preflight passed; running $TOTAL trials (skip $SKIP), log: $OUT/campaign.log"
./scripts/run_campaign.sh --manifest "$OUT/manifest.txt" --out "$OUT" --skip "$SKIP" --limit "$LIMIT_S"
[ -f "$OUT/trials.csv" ] && python3 scripts/stage3_report.py --dir "$OUT"
