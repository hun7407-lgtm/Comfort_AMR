#!/usr/bin/env bash
# Run scenario B1. See scripts/run_scenario.sh and docs/EXPERIMENT.md.
#
#     ./scripts/run_b1.sh
#     ./scripts/run_b1.sh --headless
exec "$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/run_scenario.sh" b1 "$@"
