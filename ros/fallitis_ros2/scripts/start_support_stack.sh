#!/bin/sh
set -e

export FALLITIS_WEB_PORT="${FALLITIS_WEB_PORT:-8080}"

. "$(dirname "$0")/common_env.sh"
WORKSPACE_ROOT="$(workspace_root "$0")"
source_ros_setup
source_workspace_setup "$WORKSPACE_ROOT"
pkill -f 'ros2 launch fallitis_bringup stack.launch.py' 2>/dev/null || true
pkill -f '/fallitis_web/web_server' 2>/dev/null || true
pkill -f '/fallitis_perception/perception_node' 2>/dev/null || true
pkill -f '/fallitis_mapping/mapping_node' 2>/dev/null || true
pkill -f '/fallitis_actuation/actuation_node' 2>/dev/null || true
exec ros2 launch fallitis_bringup stack.launch.py "$@"
