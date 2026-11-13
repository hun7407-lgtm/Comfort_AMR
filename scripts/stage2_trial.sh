#!/usr/bin/env bash
# ---------------------------------------------------------------
# Stage 2: record one passing of the real AMR past a participant.
#
#     ./scripts/stage2_trial.sh --scene configs/stage2/scenes/<corridor>.yaml \
#         --scenario configs/stage2/conditions/b3o_r1.00.yaml \
#         --session 261201_pilot --participant P01 --order 3
#     ./scripts/stage2_trial.sh ... --check        validate only, drive nothing
#
# Nav2, comfort_publisher and comfort_gate must already be running for
# the same --scene and --scenario (see docs/STAGE2_REAL.md). This sources
# the system ROS 2 and runs scripts/stage2_trial.py, then the
# questionnaire prompt.
# ---------------------------------------------------------------
set -euo pipefail
PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
if [ ! -f "$ROS_SETUP" ]; then
    echo "[ERROR] ROS 2 setup not found at $ROS_SETUP (set ROS_SETUP for the robot's distro)" >&2
    exit 1
fi
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u
export PYTHONPATH="${PROJECT_ROOT}:${PYTHONPATH:-}"
export PYTHONUNBUFFERED=1
exec python3 "$PROJECT_ROOT/scripts/stage2_trial.py" "$@"
