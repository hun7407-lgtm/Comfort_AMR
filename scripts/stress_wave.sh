#!/usr/bin/env bash
# ---------------------------------------------------------------
# Run randomised trials in parallel and collect them in one place.
#
#     ./scripts/stress_wave.sh --seeds 1-12
#     ./scripts/stress_wave.sh --seeds 1-8 --workers 2 --scenarios b3
#     ./scripts/stress_wave.sh --seeds 20-23 --out results/stress/wave03
#
# Why worker copies
# -----------------
# Two trials cannot share a checkout. Each run writes the nav2
# parameters, the occupancy map and the omni.anim.people command file
# into the tree by path, so a second run started beside the first one
# overwrites what the first is using. Each worker therefore gets its
# own copy of the project and runs its trials there, one at a time.
#
# They cannot share a ROS graph either: every node name and every topic
# would collide. Worker N gets ROS_DOMAIN_ID 30+N.
#
# What they DO share is one GPU, which is why the default is 2 workers.
# Isaac Sim wants several GB each; three at a time on a 16 GB card is
# asking for a run that dies half way with no useful log.
#
# Results from every worker land in <out>/trials.csv, one row a trial,
# alongside each trial's console log. Read them with
#
#     python3 scripts/triage_trials.py <out>/trials.csv
# ---------------------------------------------------------------

set -uo pipefail

PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
cd "$PROJECT_ROOT"

SEEDS=""
WORKERS=2
SCENARIOS="b3"
OUT=""
WORK_ROOT="${STRESS_WORK_ROOT:-/tmp/comfort_stress}"

while [ $# -gt 0 ]; do
    case "$1" in
        --seeds)     SEEDS="$2"; shift 2 ;;
        --workers)   WORKERS="$2"; shift 2 ;;
        --scenarios) SCENARIOS="$2"; shift 2 ;;
        --out)       OUT="$2"; shift 2 ;;
        -h|--help)   sed -n '2,30p' "$0"; exit 0 ;;
        *) echo "[ERROR] unknown option $1" >&2; exit 2 ;;
    esac
done

[ -n "$SEEDS" ] || { echo "[ERROR] --seeds 1-12 or --seeds 1,4,9" >&2; exit 2; }

# 1-12 or 1,4,9 or a mix.
expand_seeds() {
    local spec="$1" part lo hi
    IFS=',' read -r -a parts <<< "$spec"
    for part in "${parts[@]}"; do
        if [[ "$part" == *-* ]]; then
            lo="${part%%-*}"; hi="${part##*-}"
            seq "$lo" "$hi"
        else
            echo "$part"
        fi
    done
}

mapfile -t SEED_LIST < <(expand_seeds "$SEEDS")
IFS=',' read -r -a SCENARIO_LIST <<< "$SCENARIOS"

STAMP="$(date +%y%m%d_%H%M%S)"
[ -n "$OUT" ] || OUT="results/stress/wave_$STAMP"
mkdir -p "$OUT/logs"
OUT_ABS="$PROJECT_ROOT/$OUT"

echo "[wave] $STAMP"
echo "[wave] seeds     : ${SEED_LIST[*]}"
echo "[wave] scenarios : $SCENARIOS"
echo "[wave] workers   : $WORKERS"
echo "[wave] results   : $OUT"

# Every node has to at least import before a wave starts.
#
# One bad import in comfort_gate.py cost a whole wave: the gate died at
# startup, nothing reached the wheels, and nine trials were recorded as
# deadlocks with the robot sitting at its start position. Forty minutes
# to find out what two seconds of checking says.
echo "[wave] checking the nodes import"
python3 "$PROJECT_ROOT/scripts/check_nodes.py" || exit 1
# And the yield logic has to survive being called. An import check
# cannot see a bad name inside a function body, and that is the mistake
# that has cost the most: one `self.cp` in a module-level function
# killed the gate the first time a walker triggered a sidestep, and the
# trials came back looking like polite Comfort Space deadlocks.
python3 "$PROJECT_ROOT/scripts/check_evasion.py" >/dev/null || {
    python3 "$PROJECT_ROOT/scripts/check_evasion.py"
    exit 1
}

# The variants themselves, generated once in the main tree so every
# worker gets the same ones.
for seed in "${SEED_LIST[@]}"; do
    python3 scripts/random_scenario.py --seed "$seed" --quiet || {
        echo "[wave] seed $seed produced no usable scene; skipping" >&2
    }
done

# One job per (seed, scenario), dealt round robin to the workers.
JOBS=()
for seed in "${SEED_LIST[@]}"; do
    config="configs/stress/stress_$(printf '%04d' "$seed").yaml"
    [ -f "$config" ] || continue
    for scenario in "${SCENARIO_LIST[@]}"; do
        JOBS+=("$scenario|$config|$seed")
    done
done
echo "[wave] trials    : ${#JOBS[@]}"
[ "${#JOBS[@]}" -gt 0 ] || { echo "[ERROR] nothing to run" >&2; exit 1; }

run_worker() {
    local index="$1"; shift
    local dir="$WORK_ROOT/w$index"
    mkdir -p "$dir"
    # A copy, not a link: the point is that writes do not collide.
    # Results and the saved scene are not copied -- the scene is built
    # from the config at the start of every trial anyway.
    rsync -a --delete \
        --exclude '.git' --exclude 'results' --exclude 'assets/scene' \
        --exclude '__pycache__' \
        "$PROJECT_ROOT/" "$dir/" || return 1
    mkdir -p "$dir/results"

    export ROS_DOMAIN_ID=$(( 30 + index ))
    # Each trial starts by clearing leftovers. Scope that to this
    # worker's own copy, or it kills the other workers' simulators --
    # which is exactly what happened in the first wave.
    export STOP_ONLY_UNDER="$dir"
    local job scenario config seed name run=0
    for job in "$@"; do
        IFS='|' read -r scenario config seed <<< "$job"
        run=$(( run + 1 ))
        # The run number is in the name so a seed can be repeated.
        # Measuring whether the same scenario gives the same answer
        # needs the same seed several times, and without this they
        # overwrote each other's log.
        name="seed$(printf '%04d' "$seed")_${scenario}_w${index}r${run}"
        echo "[w$index] $name  (domain $ROS_DOMAIN_ID)"
        ( cd "$dir" && ./scripts/run_scenario.sh "$scenario" --headless \
            --config "$config" --results "$OUT_ABS" ) \
            > "$OUT_ABS/logs/$name.log" 2>&1
        local verdict
        verdict=$(grep -m1 "result             :" "$OUT_ABS/logs/$name.log" \
                  | sed 's/.*: //' | tr -d ' ')
        echo "[w$index] $name -> ${verdict:-NO RESULT}"
    done
}

# Deal the jobs out.
declare -a BUCKET
for i in $(seq 0 $(( WORKERS - 1 ))); do BUCKET[$i]=""; done
for i in "${!JOBS[@]}"; do
    w=$(( i % WORKERS ))
    BUCKET[$w]="${BUCKET[$w]} ${JOBS[$i]}"
done

START=$(date +%s)
PIDS=()
for i in $(seq 0 $(( WORKERS - 1 ))); do
    # shellcheck disable=SC2086
    run_worker "$i" ${BUCKET[$i]} &
    PIDS+=("$!")
    # Stagger the starts: two Isaac Sims asking for the GPU in the same
    # second is how a wave loses its first two trials.
    sleep 20
done
for pid in "${PIDS[@]}"; do wait "$pid"; done

echo "[wave] finished in $(( ($(date +%s) - START) / 60 )) min"
if [ -f "$OUT/trials.csv" ]; then
    python3 "$PROJECT_ROOT/scripts/triage_trials.py" "$OUT/trials.csv" \
        | tee "$OUT/triage.txt"
else
    echo "[ERROR] no trials.csv in $OUT" >&2
    exit 1
fi
