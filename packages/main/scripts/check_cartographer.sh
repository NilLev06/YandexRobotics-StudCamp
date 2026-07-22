#!/usr/bin/env bash
set -euo pipefail

fail=0
check_topic() {
  local topic="$1" expected="$2"
  actual="$(ros2 topic type "$topic" 2>/dev/null || true)"
  if [[ "$actual" == "$expected" ]]; then
    printf '[OK] %-14s %s\n' "$topic" "$actual"
  else
    printf '[FAIL] %-12s expected %s, got %s\n' "$topic" "$expected" "${actual:-missing}" >&2
    fail=1
  fi
}

check_topic /scan sensor_msgs/msg/LaserScan
check_topic /icp/odom nav_msgs/msg/Odometry
check_topic /map nav_msgs/msg/OccupancyGrid

nodes="$(ros2 node list 2>/dev/null || true)"
for node in /cartographer_node /cartographer_occupancy_grid_node; do
  if grep -qx "$node" <<<"$nodes"; then
    echo "[OK] node $node"
  else
    echo "[FAIL] missing node $node" >&2
    fail=1
  fi
done

if timeout 4 ros2 run tf2_ros tf2_echo map odom >/dev/null 2>&1; then
  echo "[OK] TF map -> odom"
else
  echo "[FAIL] TF map -> odom unavailable" >&2
  fail=1
fi

if timeout 4 ros2 run tf2_ros tf2_echo odom base_link >/dev/null 2>&1; then
  echo "[OK] TF odom -> base_link"
else
  echo "[FAIL] TF odom -> base_link unavailable" >&2
  fail=1
fi

publishers="$(ros2 topic info /map --verbose 2>/dev/null | grep -c 'Node name:' || true)"
if [[ "$publishers" -eq 1 ]]; then
  echo "[OK] /map has one publisher"
else
  echo "[WARN] /map publisher count appears to be $publishers" >&2
fi

exit "$fail"
