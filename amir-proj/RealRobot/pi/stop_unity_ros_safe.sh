#!/usr/bin/env bash
set -u

CONTAINER_NAME="${GFSX_CONTAINER_NAME:-gfsx_unity_ros}"
CAMERA_PID_FILE="/tmp/gfsx_camera_stream.pid"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
  docker exec "$CONTAINER_NAME" bash -lc \
    "source /opt/ros/noetic/setup.bash && \
     rostopic pub -1 /gfsx/drive_enable std_msgs/Bool 'data: false' && \
     rostopic pub -1 /gfsx/servo_enable std_msgs/Bool 'data: false'" \
    >/dev/null 2>&1 || true
  sleep 1
fi

docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true

if [ -f "$CAMERA_PID_FILE" ]; then
  camera_pid="$(cat "$CAMERA_PID_FILE" 2>/dev/null || true)"
  if [ -n "$camera_pid" ]; then
    kill "$camera_pid" >/dev/null 2>&1 || true
  fi
  rm -f "$CAMERA_PID_FILE"
fi
pkill -f "$SCRIPT_DIR/camera_stream.py" >/dev/null 2>&1 || true

echo "GFS-X ROS control stopped; drive and servo gates were disabled first."
