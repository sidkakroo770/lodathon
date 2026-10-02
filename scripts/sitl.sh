#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
mkdir -p "$COVERAGE_ROOT/artifacts/sitl"
cd /home/sid/ardupilot/ArduCopter
exec ../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-mavproxy --use-dir "$COVERAGE_ROOT/artifacts/sitl" "$@"
