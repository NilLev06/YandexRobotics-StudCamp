#!/usr/bin/env bash
# Launch the guarded bottle-search behavior without modifying the main rover tree.

set -uo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
DEFAULT_CALIBRATION="$SCRIPT_DIR/../config/bottle_search_calibration.json"
IMAGE="${BOTTLE_SEARCH_IMAGE:-registry.robotics-lab.ru/robomarvel:v6}"
CONTAINER_NAME="z-boys-bottle-search"
NETWORK="${BOTTLE_SEARCH_NETWORK:-robomarvel_default}"
WAV_FILE="${BOTTLE_SEARCH_WAV:-/tmp/bottle-not-found.wav}"
CALIBRATION_FILE="$DEFAULT_CALIBRATION"
BEHAVIOR_ARGS=()

usage() {
  cat <<'EOF'
Usage: run_bottle_search.sh [launcher options] [behavior options]

Launcher options:
  --wav FILE           WAV played only after a completed 360-degree no-find
  --calibration FILE   Calibration JSON mounted read-only into the behavior
  -h, --help           Show this help and the behavior options

Useful behavior options:
  --dry-run            Run every preflight without sending a motion goal
  --search-only        Stop safely after confirming a bottle; do not approach
  --step-deg N         Step-and-stare angle (default: 15)
  --min-clearance-m M  Required all-around spin clearance (default: 0.35)

All other arguments are passed to bottle_search.py.
EOF
}

while (($#)); do
  case "$1" in
    --wav)
      if (($# < 2)); then
        echo "ERROR: --wav needs a file path" >&2
        exit 64
      fi
      WAV_FILE="$2"
      shift 2
      ;;
    --calibration)
      if (($# < 2)); then
        echo "ERROR: --calibration needs a file path" >&2
        exit 64
      fi
      CALIBRATION_FILE="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      BEHAVIOR_ARGS+=("$1")
      shift
      ;;
  esac
done

fail() {
  echo "ERROR: $*" >&2
  exit 1
}

command -v docker >/dev/null 2>&1 || fail "docker is not installed"
[[ -r "$SCRIPT_DIR/bottle_search.py" ]] || fail "missing $SCRIPT_DIR/bottle_search.py"
[[ -r "$CALIBRATION_FILE" ]] || fail "missing calibration file: $CALIBRATION_FILE"

DASHBOARD_STATE="$(systemctl is-active rbm-web-tests.service 2>/dev/null || true)"
case "$DASHBOARD_STATE" in
  inactive|failed)
    ;;
  active|activating|reloading|deactivating)
    fail "rbm-web-tests.service is '$DASHBOARD_STATE' and can issue competing motion commands; stop it first"
    ;;
  *)
    fail "cannot verify rbm-web-tests.service state (reported '$DASHBOARD_STATE')"
    ;;
esac

[[ "$(docker inspect --format '{{.State.Running}}' ros 2>/dev/null || true)" == "true" ]] || \
  fail "main ROS container is not running"
[[ "$(docker inspect --format '{{.State.Running}}' z-boys-yolo-live 2>/dev/null || true)" == "true" ]] || \
  fail "z-boys-yolo-live is not running"

YOLO_HEALTH="$(docker inspect --format '{{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}}' z-boys-yolo-live 2>/dev/null || true)"
[[ "$YOLO_HEALTH" == "healthy" ]] || fail "YOLO sidecar health is '$YOLO_HEALTH', expected 'healthy'"

if docker inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
  fail "container $CONTAINER_NAME already exists; inspect it before retrying"
fi

if [[ ! -s "$WAV_FILE" ]]; then
  command -v ffmpeg >/dev/null 2>&1 || fail "missing $WAV_FILE and ffmpeg is unavailable to create it"
  mkdir -p -- "$(dirname -- "$WAV_FILE")" || fail "cannot create WAV directory"
  ffmpeg -hide_banner -loglevel error -y \
    -f lavfi -i "sine=frequency=660:duration=0.45" \
    -filter:a "volume=0.18" -ar 48000 -ac 1 "$WAV_FILE" || \
    fail "could not create $WAV_FILE"
fi

interrupted=0
stop_behavior() {
  interrupted=1
  echo "Stopping bottle-search container..." >&2
  docker stop --time 8 "$CONTAINER_NAME" >/dev/null 2>&1 || true
}
trap stop_behavior INT TERM HUP

echo "Launching guarded bottle search. Motion remains disabled unless every safety gate passes."
docker run --rm --init \
  --name "$CONTAINER_NAME" \
  --network "$NETWORK" \
  --read-only \
  --security-opt no-new-privileges:true \
  --cap-drop ALL \
  --pids-limit 128 \
  --tmpfs /tmp:size=32m,mode=1777 \
  -e PYTHONDONTWRITEBYTECODE=1 \
  -e PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages \
  -e LD_LIBRARY_PATH=/opt/ros/jazzy/lib/aarch64-linux-gnu:/opt/ros/jazzy/lib:/usr/local/lib/aarch64-linux-gnu \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e ROS_DOMAIN_ID=0 \
  -e ROS_LOG_DIR=/tmp \
  -v "$SCRIPT_DIR/bottle_search.py:/app/bottle_search.py:ro" \
  -v "$CALIBRATION_FILE:/app/bottle_search_calibration.json:ro" \
  --entrypoint /usr/bin/python3 \
  "$IMAGE" \
  -u /app/bottle_search.py \
  --calibration /app/bottle_search_calibration.json \
  "${BEHAVIOR_ARGS[@]}"
STATUS=$?

trap - INT TERM HUP
if ((interrupted)); then
  exit 130
fi

case "$STATUS" in
  0)
    echo "Bottle-search behavior completed successfully."
    ;;
  2)
    echo "A complete 360-degree search found no confirmed bottle; playing $WAV_FILE"
    if ! command -v aplay >/dev/null 2>&1; then
      echo "ERROR: aplay is unavailable; cannot play the no-find WAV" >&2
      exit 1
    fi
    if ! aplay -q -D plughw:0,0 "$WAV_FILE"; then
      echo "ERROR: search completed, but WAV playback failed" >&2
      exit 1
    fi
    ;;
  *)
    echo "Bottle search stopped with status $STATUS; no no-find audio was played." >&2
    ;;
esac

exit "$STATUS"
