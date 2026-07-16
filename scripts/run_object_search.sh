#!/usr/bin/env bash
# Launch the guarded object-search behavior without modifying the main rover
# tree. Pass --explore to use the frontier-driven exploration wrapper
# instead of a single 360-degree circle -- see OBJECT_SEARCH.md.

set -uo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
DEFAULT_CALIBRATION="$SCRIPT_DIR/../config/search_calibration.json"
DEFAULT_TARGET_FILE="$SCRIPT_DIR/../config/target.txt"
IMAGE="${OBJECT_SEARCH_IMAGE:-registry.robotics-lab.ru/robomarvel:v6}"
NETWORK="${OBJECT_SEARCH_NETWORK:-robomarvel_default}"
WAV_FILE="${OBJECT_SEARCH_WAV:-/tmp/object-not-found.wav}"
CALIBRATION_FILE="$DEFAULT_CALIBRATION"
TARGET_FILE="$DEFAULT_TARGET_FILE"
TARGET_CLASS=""
EXPLORE=0
BEHAVIOR_ARGS=()

usage() {
  cat <<'EOF'
Usage: run_object_search.sh [launcher options] [behavior options]

Launcher options:
  --explore             Use the frontier-driven exploration wrapper: if a
                         full 360-degree circle finds nothing, drive to the
                         nearest unexplored area of the map and search again
  --wav FILE             WAV played only after a completed no-find
  --calibration FILE     Calibration JSON mounted read-only into the behavior
  --target FILE          File containing the target class name (default:
                          config/target.txt); overridden by --target-class
  -h, --help              Show this help and the behavior options

Useful behavior options:
  --target-class NAME    YOLO/COCO class to search for (overrides --target file)
  --dry-run              Run every preflight without sending a motion goal
  --search-only          Stop safely after confirming the target; do not approach
  --step-deg N           Step-and-stare angle per search circle (default: 15)
  --min-clearance-m M    Required all-around spin clearance (default: 0.35)
  --max-cycles N         (--explore only) search+drive cycles before giving
                         up (default: 8)
  --max-runtime-s S      (--explore only) overall time budget in seconds
                         (default: 1200)

All other arguments are passed to the behavior script.
EOF
}

while (($#)); do
  case "$1" in
    --explore)
      EXPLORE=1
      shift
      ;;
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
    --target)
      if (($# < 2)); then
        echo "ERROR: --target needs a file path" >&2
        exit 64
      fi
      TARGET_FILE="$2"
      shift 2
      ;;
    --target-class)
      if (($# < 2)); then
        echo "ERROR: --target-class needs a value" >&2
        exit 64
      fi
      TARGET_CLASS="$2"
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

if ((EXPLORE)); then
  CONTAINER_NAME="z-boys-explore-search"
  ENTRY_SCRIPT="explore_search.py"
  LABEL="exploration search"
else
  CONTAINER_NAME="z-boys-object-search"
  ENTRY_SCRIPT="object_search.py"
  LABEL="search"
fi

fail() {
  echo "ERROR: $*" >&2
  exit 1
}

command -v docker >/dev/null 2>&1 || fail "docker is not installed"
[[ -r "$SCRIPT_DIR/object_search.py" ]] || fail "missing $SCRIPT_DIR/object_search.py"
if ((EXPLORE)); then
  [[ -r "$SCRIPT_DIR/explore_search.py" ]] || fail "missing $SCRIPT_DIR/explore_search.py"
  [[ -r "$SCRIPT_DIR/frontier.py" ]] || fail "missing $SCRIPT_DIR/frontier.py"
fi
[[ -r "$CALIBRATION_FILE" ]] || fail "missing calibration file: $CALIBRATION_FILE"

if [[ -z "$TARGET_CLASS" ]]; then
  [[ -r "$TARGET_FILE" ]] || fail "missing target file: $TARGET_FILE (or pass --target-class)"
  TARGET_CLASS="$(tr -d '[:space:]' < "$TARGET_FILE")"
  [[ -n "$TARGET_CLASS" ]] || fail "target file $TARGET_FILE is empty"
fi

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
  echo "Stopping $LABEL container..." >&2
  docker stop --time 8 "$CONTAINER_NAME" >/dev/null 2>&1 || true
}
trap stop_behavior INT TERM HUP

MOUNT_ARGS=(-v "$SCRIPT_DIR/object_search.py:/app/object_search.py:ro")
if ((EXPLORE)); then
  MOUNT_ARGS+=(
    -v "$SCRIPT_DIR/explore_search.py:/app/explore_search.py:ro"
    -v "$SCRIPT_DIR/frontier.py:/app/frontier.py:ro"
  )
fi

echo "Launching guarded $LABEL for '$TARGET_CLASS'. Motion remains disabled unless every safety gate passes."
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
  "${MOUNT_ARGS[@]}" \
  -v "$CALIBRATION_FILE:/app/search_calibration.json:ro" \
  --entrypoint /usr/bin/python3 \
  "$IMAGE" \
  -u "/app/$ENTRY_SCRIPT" \
  --calibration /app/search_calibration.json \
  --target-class "$TARGET_CLASS" \
  "${BEHAVIOR_ARGS[@]}"
STATUS=$?

trap - INT TERM HUP
if ((interrupted)); then
  exit 130
fi

case "$STATUS" in
  0)
    echo "$LABEL completed: '$TARGET_CLASS' was confirmed."
    ;;
  2)
    if ((EXPLORE)); then
      echo "Exploration exhausted the reachable area; no confirmed '$TARGET_CLASS'. Playing $WAV_FILE"
    else
      echo "A complete 360-degree search found no confirmed '$TARGET_CLASS'; playing $WAV_FILE"
    fi
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
    echo "$LABEL stopped with status $STATUS; no no-find audio was played." >&2
    ;;
esac

exit "$STATUS"
