#!/usr/bin/env bash
# Run scenario B3-R, the circular ablation of B3. See
# scripts/run_scenario.sh, configs/scenario_b3r.yaml and
# docs/EXPERIMENT.md.
#
#     ./scripts/run_b3r.sh
#     ./scripts/run_b3r.sh --headless
exec "$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/run_scenario.sh" b3r "$@"
