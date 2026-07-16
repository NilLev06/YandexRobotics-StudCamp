#!/usr/bin/env bash
# Rebuild an occupancy-grid map offline from an already-recorded telemetry
# session, using the same slam_toolbox that runs live on the rover.
#
# Why this is safe to run whenever, including while the rover is active:
# it spins up a throwaway container from the same image (slam_toolbox is
# already baked in, nothing gets rebuilt), on its own ROS_DOMAIN_ID and
# without joining the rover's Docker network, so it can never see or
# interfere with the live SLAM instance or the live robot's topics — it
# only ever talks to itself and the bag it is replaying.

set -uo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
REPO_DIR="$(dirname -- "$SCRIPT_DIR")"
IMAGE="${MAP_BUILDER_IMAGE:-registry.robotics-lab.ru/robomarvel:v6}"
CONTAINER_NAME="z-boys-map-builder"
PARAMS_FILE="$REPO_DIR/packages/main/config/params.yaml"

BAG_PATH=""
OUTPUT_DIR=""
MAP_NAME="map"
RATE="3.0"

usage() {
  cat <<'EOF'
Usage: build_map_from_bag.sh --bag PATH [options]

Required:
  --bag PATH           Path to a recorded rosbag2 directory, e.g.
                        ~/rover-logs/sessions/<session>/bag

Options:
  --output DIR          Where to write <name>.pgm/.yaml (default:
                        <bag's session dir>/map)
  --name NAME           Map base name (default: map)
  --rate N              Bag playback speed multiplier (default: 3.0;
                        use 1.0 if slam_toolbox struggles to keep up)
  -h, --help            Show this help
EOF
}

while (($#)); do
  case "$1" in
    --bag) BAG_PATH="${2:-}"; shift 2 ;;
    --output) OUTPUT_DIR="${2:-}"; shift 2 ;;
    --name) MAP_NAME="${2:-}"; shift 2 ;;
    --rate) RATE="${2:-}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 64 ;;
  esac
done

fail() { echo "ERROR: $*" >&2; exit 1; }

[[ -n "$BAG_PATH" ]] || { usage >&2; fail "--bag is required"; }
BAG_PATH="$(CDPATH= cd -- "$BAG_PATH" 2>/dev/null && pwd)" || fail "bag directory not found"
[[ -f "$BAG_PATH/metadata.yaml" ]] || fail "$BAG_PATH does not look like a rosbag2 directory (no metadata.yaml)"
[[ -r "$PARAMS_FILE" ]] || fail "missing $PARAMS_FILE"
command -v docker >/dev/null 2>&1 || fail "docker is not installed"

if [[ -z "$OUTPUT_DIR" ]]; then
  OUTPUT_DIR="$(dirname -- "$BAG_PATH")/map"
fi
mkdir -p -- "$OUTPUT_DIR" || fail "cannot create output directory: $OUTPUT_DIR"
OUTPUT_DIR="$(CDPATH= cd -- "$OUTPUT_DIR" && pwd)"

if docker inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
  fail "container $CONTAINER_NAME already exists; inspect/remove it before retrying"
fi

INNER_SCRIPT='
set -eu
source /opt/ros/jazzy/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/app/params.yaml use_sim_time:=false &
SLAM_PID=$!

trap "kill \$SLAM_PID 2>/dev/null || true" EXIT

echo "Waiting for slam_toolbox to come up..."
deadline=$(($(date +%s) + 30))
until ros2 service list 2>/dev/null | grep -q "/slam_toolbox/save_map"; do
  if [ "$(date +%s)" -ge "$deadline" ]; then
    echo "ERROR: slam_toolbox did not start within 30s" >&2
    exit 1
  fi
  sleep 1
done

echo "Replaying bag at ${RATE}x..."
ros2 bag play /bag --rate "${RATE}" --read-ahead-queue-size 2000

echo "Bag finished; letting slam_toolbox settle..."
sleep 3

mkdir -p /output
cd /output
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: \"${MAP_NAME}\"}}"

sleep 1
if [ ! -f "/output/${MAP_NAME}.yaml" ]; then
  echo "ERROR: save_map did not produce ${MAP_NAME}.yaml" >&2
  exit 1
fi
echo "Saved: /output/${MAP_NAME}.yaml, /output/${MAP_NAME}.pgm"
'

echo "Building map from $BAG_PATH -> $OUTPUT_DIR/${MAP_NAME}.{pgm,yaml}"
docker run --rm --init \
  --name "$CONTAINER_NAME" \
  --read-only \
  --security-opt no-new-privileges:true \
  --cap-drop ALL \
  --pids-limit 256 \
  --tmpfs /tmp:size=256m,mode=1777 \
  -e RATE="$RATE" \
  -e MAP_NAME="$MAP_NAME" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e ROS_DOMAIN_ID=42 \
  -e ROS_LOG_DIR=/tmp \
  -v "$PARAMS_FILE:/app/params.yaml:ro" \
  -v "$BAG_PATH:/bag:ro" \
  -v "$OUTPUT_DIR:/output" \
  --entrypoint /bin/bash \
  "$IMAGE" \
  -c "$INNER_SCRIPT"
STATUS=$?

if ((STATUS == 0)); then
  echo "Done. Map saved to $OUTPUT_DIR/${MAP_NAME}.yaml"
else
  echo "Map build failed with status $STATUS" >&2
fi
exit "$STATUS"
