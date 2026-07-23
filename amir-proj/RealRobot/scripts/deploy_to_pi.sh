#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PI_TARGET="${GFSX_PI_TARGET:-pi@192.168.2.152}"
REMOTE_DIR="${GFSX_PI_DIR:-/home/pi/gfsx-unity}"

command -v rsync >/dev/null 2>&1 || {
  echo "rsync is required" >&2
  exit 1
}

echo "Deploying the fail-closed Pi runtime to $PI_TARGET:$REMOTE_DIR"
echo "The live gfsx_servo_state.json file will never be overwritten."

ssh "$PI_TARGET" "mkdir -p '$REMOTE_DIR'"
rsync -av --delete \
  --exclude gfsx_servo_state.json \
  "$PACKAGE_DIR/pi/" "$PI_TARGET:$REMOTE_DIR/"
ssh "$PI_TARGET" \
  "chmod +x '$REMOTE_DIR/start_unity_ros_safe.sh' \
    '$REMOTE_DIR/stop_unity_ros_safe.sh' \
    '$REMOTE_DIR/camera_stream.py' \
    '$REMOTE_DIR/gfsx_unity_safe_bridge.py'"

echo "Deployment complete."
echo "On the Pi, create and verify the live servo-state estimate once:"
echo "  cd $REMOTE_DIR"
echo "  cp -n gfsx_servo_state.example.json gfsx_servo_state.json"
echo "  nano gfsx_servo_state.json"
