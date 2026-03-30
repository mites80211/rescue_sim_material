#!/bin/sh

set -e

find_ros_setup() {
  if [ -n "${ROS_SETUP_FILE:-}" ] && [ -f "${ROS_SETUP_FILE}" ]; then
    printf '%s\n' "$ROS_SETUP_FILE"
    return 0
  fi

  for candidate in \
    "/opt/ros/jazzy/setup.sh" \
    "$HOME/ros2_jazzy/install/setup.sh"
  do
    if [ -f "$candidate" ]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done

  return 1
}

prepend_unique_path() {
  list_name="$1"
  value="$2"
  eval current_values=\"\${$list_name:-}\"
  old_ifs=$IFS
  IFS=:
  for item in $current_values; do
    if [ "$item" = "$value" ]; then
      IFS=$old_ifs
      unset old_ifs item current_values value list_name
      return 0
    fi
  done
  IFS=$old_ifs
  if [ -n "$current_values" ]; then
    eval "export $list_name=\"$value:\$${list_name}\""
  else
    eval "export $list_name=\"$value\""
  fi
  unset old_ifs item current_values value list_name
}

source_ros_setup() {
  ros_setup="$(find_ros_setup)" || {
    echo "ROS 2 Jazzy setup.sh not found." >&2
    echo "Set ROS_SETUP_FILE or install ROS 2 Jazzy under /opt/ros/jazzy." >&2
    exit 1
  }
  # shellcheck source=/dev/null
  . "$ros_setup"
}

source_workspace_setup() {
  workspace_root="${1:-$HOME/fallitis_ros2}"
  workspace_install="$workspace_root/install"
  workspace_setup="$workspace_install/setup.sh"
  if [ ! -f "$workspace_setup" ]; then
    echo "Workspace setup not found: $workspace_setup" >&2
    echo "Run: cd $workspace_root && ./scripts/build_workspace.sh" >&2
    exit 1
  fi

  # shellcheck source=/dev/null
  . "$workspace_setup"
  prepend_unique_path AMENT_PREFIX_PATH "$workspace_install"
  prepend_unique_path COLCON_PREFIX_PATH "$workspace_install"

  for prefix in "$workspace_install"/*; do
    if [ ! -d "$prefix" ]; then
      continue
    fi
    if [ ! -d "$prefix/share/ament_index/resource_index/packages" ]; then
      continue
    fi
    prepend_unique_path AMENT_PREFIX_PATH "$prefix"
    prepend_unique_path COLCON_PREFIX_PATH "$prefix"
    if [ -d "$prefix/bin" ]; then
      prepend_unique_path PATH "$prefix/bin"
    fi
    for site_packages in "$prefix"/lib/python*/site-packages; do
      if [ -d "$site_packages" ]; then
        prepend_unique_path PYTHONPATH "$site_packages"
      fi
    done
  done

  unset workspace_root workspace_install workspace_setup prefix site_packages
}
