#!/usr/bin/env bash
# ---------------------------------------------------------------
# Estimate people from the LiDAR while a scenario runs.
#
#   ./scripts/run_people_tracker.sh --evaluate
#   ./scripts/run_people_tracker.sh --config configs/demo_walker.yaml --evaluate
#
# Start a scenario first (scripts/run_scenario.sh), then this alongside.
# Ctrl+C prints the comparison against the true positions.
# ---------------------------------------------------------------
set -uo pipefail
PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
cd "$PROJECT_ROOT"
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
export PYTHONUNBUFFERED=1
exec python3 scripts/people_tracker.py "$@"
