#!/usr/bin/env bash
set -eo pipefail

if [[ ! -f install/setup.bash ]]; then
  echo "Build the workspace first; install/setup.bash was not found." >&2
  exit 2
fi

set +u
source install/setup.bash
set -u
mkdir -p results
ros2 launch vif_bringup synthetic_demo.launch.py >results/synthetic_demo.log 2>&1 &
launch_pid=$!
cleanup() {
  kill -INT "$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

python3 tools/benchmark/observe_demo.py --timeout 18
