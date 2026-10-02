#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
mkdir -p "$COVERAGE_ROOT/artifacts/mavproxy"
cd "$COVERAGE_ROOT/artifacts/mavproxy"
exec mavproxy.py --master=tcp:127.0.0.1:5760 --out=udp:127.0.0.1:14650 --out=udp:127.0.0.1:14652 --streamrate=30 "$@"
