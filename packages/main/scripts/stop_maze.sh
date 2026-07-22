#!/usr/bin/env bash
set -euo pipefail
ros2 service call /maze/stop std_srvs/srv/Trigger '{}'
# Belt-and-suspenders stop packet in case Nav2 is tearing down.
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null
