#!/usr/bin/env bash
set -eo pipefail

if [[ ! -d /src/install ]]; then
    make -C /src build-all
fi

cd /src
ROS_ROOT="/opt/ros/${ROS_DISTRO:-jazzy}"
source "${ROS_ROOT}/setup.bash"
source /src/install/setup.bash
export PATH="/root/venv/bin:${PATH}"
export LD_LIBRARY_PATH="${ROS_ROOT}/lib/aarch64-linux-gnu:${ROS_ROOT}/lib:${LD_LIBRARY_PATH:-}"

if [[ -f /src/.autostart ]]; then
    # A web terminal can leave a tmux server with a stale PATH/environment.
    # Recreate the server itself so every autostart pane inherits ROS setup.
    tmux kill-server 2>/dev/null || true
    tmuxp load -d /src/tmuxp.yaml
fi

trap : TERM INT
sleep infinity &
wait
