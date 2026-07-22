#!/usr/bin/env bash
set -u

fail=0
check_node() {
  local node="$1"
  if ros2 node list 2>/dev/null | grep -qx "$node"; then
    echo "[OK] node $node"
  else
    echo "[FAIL] node $node" >&2
    fail=1
  fi
}
check_topic() {
  local topic="$1"
  if timeout 4 ros2 topic echo "$topic" --once >/dev/null 2>&1; then
    echo "[OK] topic $topic"
  else
    echo "[FAIL] topic $topic" >&2
    fail=1
  fi
}
check_service() {
  local service="$1"
  if ros2 service list 2>/dev/null | grep -qx "$service"; then
    echo "[OK] service $service"
  else
    echo "[FAIL] service $service" >&2
    fail=1
  fi
}

check_node /odom_sanitizer
check_node /maze_explorer
check_node /collision_monitor
check_node /cartographer_node
check_topic /scan
check_topic /icp/odom
check_topic /map
check_topic /maze/status
check_service /maze/start
check_service /maze/stop
check_service /maze/get_status

if timeout 4 ros2 run tf2_ros tf2_echo map base_link >/dev/null 2>&1; then
  echo "[OK] TF map -> base_link"
else
  echo "[FAIL] TF map -> base_link" >&2
  fail=1
fi

if ros2 lifecycle get /controller_server 2>/dev/null | grep -q "active \[3\]"; then
  echo "[OK] Nav2 controller active"
else
  echo "[FAIL] Nav2 controller is not active" >&2
  fail=1
fi

if python3 -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:8090/api/maze/status', timeout=3).read()" >/dev/null 2>&1; then
  echo "[OK] maze API :8090"
else
  echo "[FAIL] maze API :8090" >&2
  fail=1
fi

exit "$fail"
