#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
UNITY_DIR="$(cd "$PACKAGE_DIR/.." && pwd)"

python3 -m py_compile \
  "$PACKAGE_DIR/mac/gfsx_real_vision.py" \
  "$PACKAGE_DIR/pi/camera_stream.py" \
  "$PACKAGE_DIR/pi/gfsx_unity_safe_bridge.py" \
  "$PACKAGE_DIR/tests/test_gfsx_bridge.py" \
  "$PACKAGE_DIR/tests/test_gfsx_bridge_obstacle_gate.py"

bash -n \
  "$PACKAGE_DIR/mac/run_real_vision.sh" \
  "$PACKAGE_DIR/pi/start_unity_ros_safe.sh" \
  "$PACKAGE_DIR/pi/stop_unity_ros_safe.sh" \
  "$PACKAGE_DIR/scripts/deploy_to_pi.sh"

python3 -m unittest discover \
  -s "$PACKAGE_DIR/tests" \
  -p "test_gfsx_bridge*.py"

(
  cd "$UNITY_DIR"
  shasum -a 256 -c RealRobot/CHECKSUMS.sha256
)

echo "Static checks, 25 bridge tests, and artifact checksums passed."
