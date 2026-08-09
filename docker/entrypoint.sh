#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
export PATH="/opt/vif-venv/bin:$PATH"
exec "$@"
