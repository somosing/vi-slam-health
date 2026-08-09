#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy was not found at /opt/ros/jazzy." >&2
  exit 2
fi
if [[ ! -f "${project_root}/install/setup.bash" ]]; then
  echo "Build the workspace first; install/setup.bash was not found." >&2
  exit 2
fi

set +u
source /opt/ros/jazzy/setup.bash
source "${project_root}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export PYTHONPATH="${project_root}/tools/benchmark${PYTHONPATH:+:${PYTHONPATH}}"

exec python3 "${project_root}/tools/benchmark/run_orbslam3_benchmark.py" "$@"
