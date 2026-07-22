#!/usr/bin/env bash
set -euo pipefail
ros2 service call /maze/start std_srvs/srv/Trigger '{}'
