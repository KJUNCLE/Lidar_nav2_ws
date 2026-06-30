#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT"

MAP_NAME="nav_test_4_27"
HAS_PRIOR_PCD_FILE=false
for arg in "$@"; do
  case "$arg" in
    map_name:=*)
      MAP_NAME="${arg#map_name:=}"
      ;;
    prior_pcd_file:=*)
      HAS_PRIOR_PCD_FILE=true
      ;;
  esac
done

echo "$MAP_NAME" > "$WORKSPACE_ROOT/src/me_nav2_bringup/.last_map_name"

LAUNCH_ARGS=("$@")
if [ "$HAS_PRIOR_PCD_FILE" = false ]; then
  mkdir -p "$WORKSPACE_ROOT/src/me_nav2_bringup/pcd"
  LAUNCH_ARGS+=("prior_pcd_file:=$WORKSPACE_ROOT/src/me_nav2_bringup/pcd/$MAP_NAME.pcd")
fi

set +u
source install/setup.bash
set -u
exec ros2 launch me_nav2_bringup cpu_real_mapping.launch.py "${LAUNCH_ARGS[@]}"
