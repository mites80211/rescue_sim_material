#!/bin/sh
set -e

. "$(dirname "$0")/common_env.sh"
source_ros_setup
cd "$HOME/fallitis_ros2"
colcon build --merge-install "$@"
