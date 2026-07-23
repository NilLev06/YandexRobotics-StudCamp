#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${GFSX_CONTAINER_NAME:-gfsx_unity_ros}"
IMAGE_NAME="${GFSX_DOCKER_IMAGE:-ros_noetic_hardware_v2}"
ROBOT_IP="${GFSX_ROBOT_IP:-192.168.2.152}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEAM1_DIR="${GFSX_TEAM1_DIR:-/home/pi/team1}"
STATE_FILE="$SCRIPT_DIR/gfsx_servo_state.json"
CAMERA_PID_FILE="/tmp/gfsx_camera_stream.pid"
CAMERA_LOG="/tmp/gfsx_camera.log"

fail() {
  echo "ERROR: $*" >&2
  exit 1
}

command -v docker >/dev/null 2>&1 || fail "docker is not installed"
command -v python3 >/dev/null 2>&1 || fail "python3 is not installed"
docker image inspect "$IMAGE_NAME" >/dev/null 2>&1 ||
  fail "required Docker image '$IMAGE_NAME' is missing"

for required in \
  "$SCRIPT_DIR/gfsx_unity_safe_bridge.py" \
  "$SCRIPT_DIR/camera_stream.py" \
  "$STATE_FILE"; do
  [ -f "$required" ] || fail "missing $required"
done

# The state file is a commanded-angle estimate, not encoder feedback. Refuse
# malformed or out-of-range values before Docker can mount it.
python3 -c '
import json, math, sys
path = sys.argv[1]
limits = ((0, 180), (0, 180), (0, 180), (0, 50), (15, 160), (0, 180))
values = json.load(open(path, encoding="utf-8")).get("angles")
if not isinstance(values, list) or len(values) != 6:
    raise SystemExit("angles must be a six-number JSON array")
for index, (value, bounds) in enumerate(zip(values, limits), start=1):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SystemExit("S%d is not numeric" % index)
    if not math.isfinite(float(value)) or not bounds[0] <= value <= bounds[1]:
        raise SystemExit("S%d=%r is outside %s" % (index, value, bounds))
' "$STATE_FILE"

python3 -c 'import ast, pathlib, sys; ast.parse(pathlib.Path(sys.argv[1]).read_text())' \
  "$SCRIPT_DIR/gfsx_unity_safe_bridge.py"
python3 -c 'import ast, pathlib, sys; ast.parse(pathlib.Path(sys.argv[1]).read_text())' \
  "$SCRIPT_DIR/camera_stream.py"

sudo iw dev wlan0 set power_save off >/dev/null 2>&1 || true

# Only this read-only process owns the USB camera. It has no servo imports.
if [ -f "$CAMERA_PID_FILE" ]; then
  old_camera_pid="$(cat "$CAMERA_PID_FILE" 2>/dev/null || true)"
  if [ -n "$old_camera_pid" ]; then
    kill "$old_camera_pid" >/dev/null 2>&1 || true
  fi
fi
pkill -f "/home/pi/camera_servo_web.py" >/dev/null 2>&1 || true
pkill -f "/home/pi/camera_stream.py" >/dev/null 2>&1 || true
pkill -f "$SCRIPT_DIR/camera_stream.py" >/dev/null 2>&1 || true
nohup python3 "$SCRIPT_DIR/camera_stream.py" \
  --host 0.0.0.0 --port 8080 >"$CAMERA_LOG" 2>&1 &
camera_pid=$!
echo "$camera_pid" >"$CAMERA_PID_FILE"

cleanup_failed_start() {
  docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true
  kill "$camera_pid" >/dev/null 2>&1 || true
}
trap cleanup_failed_start EXIT

camera_ready=0
for _attempt in $(seq 1 30); do
  if python3 -c '
import sys
from urllib.request import urlopen
with urlopen("http://127.0.0.1:8080/healthz", timeout=1.0) as response:
    sys.exit(0 if response.status == 200 else 1)
' >/dev/null 2>&1; then
    camera_ready=1
    break
  fi
  kill -0 "$camera_pid" >/dev/null 2>&1 ||
    fail "camera streamer exited; inspect $CAMERA_LOG"
  sleep 0.2
done
[ "$camera_ready" -eq 1 ] ||
  fail "camera did not become healthy; inspect $CAMERA_LOG"

docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true

docker create \
  --name "$CONTAINER_NAME" \
  --restart unless-stopped \
  -e GFSX_COMMAND_PREFIX=/gfsx \
  --network host \
  --privileged \
  -v /dev:/dev \
  -v "$STATE_FILE:/root/gfsx_servo_state.json" \
  "$IMAGE_NAME" \
  bash -lc '
    set -e
    source /opt/ros/noetic/setup.bash

    # A stale value in an older image must not redirect commands away from
    # the exact topics published by the Fixed Unity scene.
    rm -f /root/gfsx_command_prefix.txt

    roscore >/tmp/roscore.log 2>&1 &
    ROSCORE_PID=$!

    ROSCORE_READY=0
    for _attempt in $(seq 1 30); do
      if rostopic list >/dev/null 2>&1; then
        ROSCORE_READY=1
        break
      fi
      sleep 0.5
    done
    if [ "$ROSCORE_READY" -ne 1 ]; then
      echo "roscore did not become ready" >&2
      exit 20
    fi

    source /root/catkin_ws/devel/setup.bash
    rosrun ros_tcp_endpoint default_server_endpoint.py \
      --tcp_ip 0.0.0.0 --tcp_port 10000 \
      >/tmp/endpoint.log 2>&1 &
    ENDPOINT_PID=$!

    python3 -u /root/gfsx_unity_safe_bridge.py \
      >/tmp/gfsx_bridge.log 2>&1 &
    BRIDGE_PID=$!

    terminate_children() {
      kill "$BRIDGE_PID" "$ENDPOINT_PID" "$ROSCORE_PID" 2>/dev/null || true
      wait "$BRIDGE_PID" "$ENDPOINT_PID" "$ROSCORE_PID" 2>/dev/null || true
    }
    trap terminate_children TERM INT EXIT

    # Restart the complete fail-closed ROS stack if a critical process exits.
    wait -n "$ROSCORE_PID" "$ENDPOINT_PID" "$BRIDGE_PID"
  ' >/dev/null

docker cp "$SCRIPT_DIR/gfsx_unity_safe_bridge.py" \
  "$CONTAINER_NAME:/root/gfsx_unity_safe_bridge.py"

for helper in smbus.py xr_car_light.py xr_music.py; do
  if [ -f "$TEAM1_DIR/$helper" ]; then
    docker cp "$TEAM1_DIR/$helper" \
      "$CONTAINER_NAME:/root/XiaoRGeek/$helper"
  fi
done

docker start "$CONTAINER_NAME" >/dev/null

ros_ready=0
for _attempt in $(seq 1 30); do
  if docker exec "$CONTAINER_NAME" bash -lc \
    "source /opt/ros/noetic/setup.bash && rostopic list" \
    >/dev/null 2>&1; then
    ros_ready=1
    break
  fi
  sleep 0.5
done
[ "$ros_ready" -eq 1 ] || fail "ROS did not become ready in the container"

for topic in \
  /gfsx/cmd_vel \
  /gfsx/drive_enable \
  /gfsx/servo_targets_degrees \
  /gfsx/servo_enable; do
  if ! docker exec "$CONTAINER_NAME" bash -lc \
    "source /opt/ros/noetic/setup.bash && rostopic info '$topic'" \
    2>/dev/null | grep -q "/gfsx_unity_safe_bridge"; then
    fail "bridge is not subscribed to $topic"
  fi
done

trap - EXIT

echo "GFS-X ROS bridge is ready and starts fully disarmed."
echo "ROS-TCP endpoint: ${ROBOT_IP}:10000"
echo "Camera stream:    http://${ROBOT_IP}:8080/"
echo "Bridge log:       docker exec $CONTAINER_NAME tail -f /tmp/gfsx_bridge.log"
echo "Camera log:       tail -f $CAMERA_LOG"
