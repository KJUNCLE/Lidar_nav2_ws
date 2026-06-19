#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT"

set +u
source install/setup.bash
set -u

missing_packages=()
for package in nav2_bringup nav2_map_server nav2_lifecycle_manager pointcloud_to_laserscan robot_state_publisher; do
  if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
    missing_packages+=("$package")
  fi
done

if ((${#missing_packages[@]} > 0)); then
  echo "Missing required ROS package(s): ${missing_packages[*]}" >&2
  echo "Install runtime dependencies with:" >&2
  echo "  sudo apt update" >&2
  echo "  sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-pointcloud-to-laserscan ros-humble-robot-state-publisher" >&2
  echo "Or from the workspace root:" >&2
  echo "  rosdep install --from-paths src --ignore-src --rosdistro humble -r -y" >&2
  exit 1
fi

exec ros2 launch me_nav2_bringup cpu_real_nav.launch.py "$@"
