#!/usr/bin/env bash
# Source from any directory. This experiment owns no original mission paths.
COVERAGE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export COVERAGE_ROOT
export GZ_VERSION=harmonic
# Use the existing multicast-capable interface; no sudo/network changes needed.
# Override with COVERAGE_GZ_IP when the user's loopback multicast setup is active.
export GZ_IP="${COVERAGE_GZ_IP:-$(ip -4 route get 239.255.0.7 | sed -n 's/.* src \([^ ]*\).*/\1/p')}"
export GZ_PARTITION=sae_coverage
export GZ_DISCOVERY_MULTICAST_IP=239.255.0.7
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export PYTHONPATH="$COVERAGE_ROOT:/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/home/sid/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$COVERAGE_ROOT/artifacts/gazebo_fixture/models:/home/sid/ardupilot_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"
export SDF_PATH="$GZ_SIM_RESOURCE_PATH"
