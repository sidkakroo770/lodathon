#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$COVERAGE_ROOT"
exec /usr/bin/python3 -m coverage_mission.runtime "$@"
