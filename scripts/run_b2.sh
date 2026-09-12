#!/usr/bin/env bash
# Run scenario B2. See scripts/run_scenario.sh and docs/EXPERIMENT.md.
#
#     ./scripts/run_b2.sh
#     ./scripts/run_b2.sh --headless
exec "$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/run_scenario.sh" b2 "$@"
