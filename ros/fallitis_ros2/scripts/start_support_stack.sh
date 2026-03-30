#!/bin/sh
set -e

export FALLITIS_WEB_PORT="${FALLITIS_WEB_PORT:-8080}"

. "$(dirname "$0")/common_env.sh"
source_ros_setup
source_workspace_setup "$HOME/fallitis_ros2"
WEB_PARAMS="${WEB_PARAMS:-$HOME/fallitis_ros2/install/fallitis_bringup/share/fallitis_bringup/config/stack.yaml}"
if [ ! -f "$WEB_PARAMS" ]; then
  WEB_PARAMS="$HOME/fallitis_ros2/install/share/fallitis_bringup/config/stack.yaml"
fi
if [ ! -f "$WEB_PARAMS" ]; then
  WEB_PARAMS="$HOME/fallitis_ros2/src/fallitis_bringup/config/stack.yaml"
fi
pkill -f '/fallitis_web/web_server' 2>/dev/null || true
exec ros2 run fallitis_web web_server --ros-args --params-file "$WEB_PARAMS" "$@"
