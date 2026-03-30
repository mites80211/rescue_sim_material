#!/bin/sh
set -e

. "$(dirname "$0")/common_env.sh"
WORKSPACE_ROOT="$(workspace_root "$0")"
source_ros_setup
cd "$WORKSPACE_ROOT"
colcon build --merge-install "$@"
