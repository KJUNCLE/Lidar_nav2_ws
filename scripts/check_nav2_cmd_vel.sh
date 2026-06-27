#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT"

set +u
source install/setup.bash
set -u

echo "== Nav2 lifecycle states =="
for node in \
  /controller_server \
  /planner_server \
  /bt_navigator \
  /velocity_smoother \
  /local_costmap/local_costmap \
  /global_costmap/global_costmap
do
  echo "-- ${node}"
  ros2 lifecycle get "${node}" || true
done

echo
echo "== Core topic rates =="
for topic in /plan /local_costmap/costmap /scan /odom /cmd_vel_nav /cmd_vel
do
  echo "-- ${topic}"
  timeout 3 ros2 topic hz "${topic}" || true
done

echo
echo "== TF checks =="
timeout 3 ros2 run tf2_ros tf2_echo map odom || true
timeout 3 ros2 run tf2_ros tf2_echo odom base_footprint || true

echo
echo "If /cmd_vel_nav has data but /cmd_vel does not, check velocity_smoother."
echo "If neither has data, check controller_server logs, local_costmap, and odom->base_footprint TF."
