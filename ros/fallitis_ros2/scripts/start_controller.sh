#!/bin/sh
set -e

. "$(dirname "$0")/common_env.sh"
source_ros_setup
source_workspace_setup "$HOME/fallitis_ros2"
export WEBOTS_HOME="${WEBOTS_HOME:-/usr/local/webots}"
export PYTHONPATH="$WEBOTS_HOME/lib/controller/python${PYTHONPATH:+:$PYTHONPATH}"
export WEBOTS_CONTROLLER_HOST="${WEBOTS_CONTROLLER_HOST:-10.0.2.2}"
export WEBOTS_CONTROLLER_PORT="${WEBOTS_CONTROLLER_PORT:-1234}"
export WEBOTS_ROBOT_NAME="${WEBOTS_ROBOT_NAME:-Erebus_Bot}"
export WEBOTS_CONTROLLER_URL="${WEBOTS_CONTROLLER_URL:-tcp://$WEBOTS_CONTROLLER_HOST:$WEBOTS_CONTROLLER_PORT/$WEBOTS_ROBOT_NAME}"
CONTROLLER_PARAMS="${CONTROLLER_PARAMS:-$HOME/fallitis_ros2/install/fallitis_bringup/share/fallitis_bringup/config/stack.yaml}"
if [ ! -f "$CONTROLLER_PARAMS" ]; then
  CONTROLLER_PARAMS="$HOME/fallitis_ros2/install/share/fallitis_bringup/config/stack.yaml"
fi
if [ ! -f "$CONTROLLER_PARAMS" ]; then
  CONTROLLER_PARAMS="$HOME/fallitis_ros2/src/fallitis_bringup/config/stack.yaml"
fi
CONTROLLER_BIN="$HOME/fallitis_ros2/install/lib/fallitis_robot/webots_robot_node"
if [ ! -x "$CONTROLLER_BIN" ]; then
  CONTROLLER_BIN="$HOME/fallitis_ros2/install/fallitis_robot/lib/fallitis_robot/webots_robot_node"
fi
if [ ! -x "$CONTROLLER_BIN" ]; then
  echo "controller executable not found: $CONTROLLER_BIN" >&2
  echo "run: cd ~/fallitis_ros2 && ./scripts/build_workspace.sh" >&2
  exit 1
fi
pkill -f '/fallitis_robot/webots_robot_node' 2>/dev/null || true
exec "$CONTROLLER_BIN" --ros-args --params-file "$CONTROLLER_PARAMS" "$@"
