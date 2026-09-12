#!/usr/bin/env bash
# Run scenario B3. See scripts/run_scenario.sh and docs/EXPERIMENT.md.
#
#     ./scripts/run_b3.sh
#     ./scripts/run_b3.sh --headless
exec "$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/run_scenario.sh" b3 "$@"
