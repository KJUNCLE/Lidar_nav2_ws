#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT"

ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"

CPU_REAL_PACKAGES=(
  ros2_socketcan_msgs
  ros2_socketcan
  autodrive_d02_can_msgs
  autodrive_d02_can
  fast_lio
  small_gicp
  lio_interface
  sensor_scan_generation
  gld_robot_description
  me_nav2_bringup
  pcd2pgm
  small_gicp_relocalization
  global_relocalization_kiss_matcher
)

colcon build \
  --symlink-install \
  --packages-select livox_sdk2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --parallel-workers 2 \
  "$@"

set +u
source install/setup.bash
set -u

colcon build \
  --symlink-install \
  --packages-select livox_ros_driver2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS="${ROS_DISTRO_NAME}" \
  --parallel-workers 2 \
  "$@"

set +u
source install/setup.bash
set -u

colcon build \
  --symlink-install \
  --packages-select "${CPU_REAL_PACKAGES[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --parallel-workers 2 \
  "$@"
