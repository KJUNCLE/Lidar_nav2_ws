#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

LAST_MAP_NAME_FILE="$WORKSPACE_ROOT/src/me_nav2_bringup/.last_map_name"
MAP_NAME="${1:-}"
if [ -z "$MAP_NAME" ] && [ -f "$LAST_MAP_NAME_FILE" ]; then
  MAP_NAME="$(tr -d '[:space:]' < "$LAST_MAP_NAME_FILE")"
fi
MAP_NAME="${MAP_NAME:-nav_test_4_27}"
MAP_NAME="${MAP_NAME#map_name:=}"
PCD_FILE="$WORKSPACE_ROOT/src/me_nav2_bringup/pcd/$MAP_NAME.pcd"

mkdir -p "$(dirname -- "$PCD_FILE")"
echo "Saving PCD to $PCD_FILE"
ros2 service call /map_save std_srvs/srv/Trigger
