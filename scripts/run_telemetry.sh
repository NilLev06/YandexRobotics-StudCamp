#!/usr/bin/env bash
# Всегда-включённая телеметрия ровера: крутит часовые сессии записи.
# Каждая сессия = rosbag со всеми топиками + видео-сегменты + JSONL-метрики.
# Запускается systemd-сервисом rbm-telemetry при загрузке (см. TELEMETRY.md).

set -uo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
REPO_DIR="$(dirname -- "$SCRIPT_DIR")"
ENV_FILE="$REPO_DIR/config/telemetry.env"

[[ -r "$ENV_FILE" ]] && source "$ENV_FILE"

TELEMETRY_ENABLED="${TELEMETRY_ENABLED:-1}"
TELEMETRY_DIR="${TELEMETRY_DIR:-/home/robomarvel/rover-logs}"
TELEMETRY_CAP_GB="${TELEMETRY_CAP_GB:-10}"
TELEMETRY_MIN_FREE_GB="${TELEMETRY_MIN_FREE_GB:-2}"
TELEMETRY_SESSION_SECONDS="${TELEMETRY_SESSION_SECONDS:-3600}"
TELEMETRY_YOLO_URL="${TELEMETRY_YOLO_URL:-http://127.0.0.1:8091/health}"

if [[ "$TELEMETRY_ENABLED" != "1" ]]; then
  echo "Telemetry is disabled (TELEMETRY_ENABLED=$TELEMETRY_ENABLED); exiting."
  exit 0
fi

command -v docker >/dev/null 2>&1 || { echo "ERROR: docker is not installed" >&2; exit 1; }

SESSIONS_DIR="$TELEMETRY_DIR/sessions"
mkdir -p "$SESSIONS_DIR"

COLLECTOR_PID=""
SESSION_DIR=""
shutting_down=0

stop_session() {
  if [[ -n "$COLLECTOR_PID" ]] && kill -0 "$COLLECTOR_PID" 2>/dev/null; then
    kill -TERM "$COLLECTOR_PID" 2>/dev/null
    wait "$COLLECTOR_PID" 2>/dev/null
  fi
  COLLECTOR_PID=""
  TELEMETRY_SESSION_DIR="$SESSION_DIR" \
    docker compose -f "$REPO_DIR/docker-compose.telemetry.yaml" down --timeout 25 \
    >/dev/null 2>&1
  if [[ -n "$SESSION_DIR" ]]; then
    rm -f "$SESSION_DIR/.active"
    date -u +"%Y-%m-%dT%H:%M:%SZ" > "$SESSION_DIR/finished_at.txt" 2>/dev/null
  fi
  SESSION_DIR=""
}

handle_term() {
  shutting_down=1
  stop_session
  exit 0
}
trap handle_term INT TERM

while (( ! shutting_down )); do
  SESSION_NAME="$(date -u +%Y-%m-%dT%H-%M-%SZ)"
  SESSION_DIR="$SESSIONS_DIR/$SESSION_NAME"
  mkdir -p "$SESSION_DIR/video"
  touch "$SESSION_DIR/.active"

  cat > "$SESSION_DIR/meta.json" <<EOF
{
  "session": "$SESSION_NAME",
  "started_at_utc": "$(date -u +"%Y-%m-%dT%H:%M:%SZ")",
  "hostname": "$(hostname)",
  "git_rev": "$(git -C "$REPO_DIR" rev-parse --short HEAD 2>/dev/null || echo unknown)",
  "session_seconds": $TELEMETRY_SESSION_SECONDS
}
EOF

  echo "Starting telemetry session $SESSION_NAME"
  TELEMETRY_SESSION_DIR="$SESSION_DIR" \
    docker compose -f "$REPO_DIR/docker-compose.telemetry.yaml" up -d

  python3 "$SCRIPT_DIR/telemetry_collector.py" \
    --session-dir "$SESSION_DIR" \
    --yolo-url "$TELEMETRY_YOLO_URL" \
    --cap-gb "$TELEMETRY_CAP_GB" \
    --min-free-gb "$TELEMETRY_MIN_FREE_GB" &
  COLLECTOR_PID=$!

  elapsed=0
  while (( elapsed < TELEMETRY_SESSION_SECONDS && ! shutting_down )); do
    sleep 5
    elapsed=$((elapsed + 5))
    # Коллектор умер — перезапускаем сессию целиком, не ждём конца часа
    if ! kill -0 "$COLLECTOR_PID" 2>/dev/null; then
      echo "WARN: collector exited early; rotating session" >&2
      break
    fi
  done

  echo "Closing telemetry session $SESSION_NAME"
  stop_session
done
