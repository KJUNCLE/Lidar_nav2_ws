#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT"

HUMANOID_LOCALIZATION_PACKAGES=(
  fast_lio_humanoid
  open3d_loc_humanoid
)

HUMANOID_CMAKE_ARGS=(-DCMAKE_BUILD_TYPE=Release)
if [[ -n "${Open3D_DIR:-}" ]]; then
  HUMANOID_CMAKE_ARGS+=("-DOpen3D_DIR=${Open3D_DIR}")
fi

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
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --parallel-workers 2 \
  "$@"

set +u
source install/setup.bash
set -u

colcon build \
  --symlink-install \
  --packages-select "${HUMANOID_LOCALIZATION_PACKAGES[@]}" \
  --cmake-args "${HUMANOID_CMAKE_ARGS[@]}" \
  --parallel-workers 2 \
  "$@"
