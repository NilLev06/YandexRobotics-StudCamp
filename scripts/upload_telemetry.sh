#!/usr/bin/env bash
# Выгрузка завершённых телеметрийных сессий на сервер (rsync по SSH) и
# удаление локальных копий после успешной передачи. Запускается systemd-таймером
# rbm-telemetry-upload каждые 10 минут. Активная сессия (.active) не трогается.

set -uo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
REPO_DIR="$(dirname -- "$SCRIPT_DIR")"
ENV_FILE="$REPO_DIR/config/telemetry.env"

[[ -r "$ENV_FILE" ]] && source "$ENV_FILE"

TELEMETRY_DIR="${TELEMETRY_DIR:-/home/robomarvel/rover-logs}"
TELEMETRY_REMOTE="${TELEMETRY_REMOTE:-}"
SESSIONS_DIR="$TELEMETRY_DIR/sessions"

if [[ -z "$TELEMETRY_REMOTE" ]]; then
  # Сервер не настроен — сессии остаются локально, ротацией занимается
  # сторож диска в telemetry_collector.py.
  exit 0
fi

command -v rsync >/dev/null 2>&1 || { echo "ERROR: rsync is not installed" >&2; exit 1; }
[[ -d "$SESSIONS_DIR" ]] || exit 0

status=0
for session in "$SESSIONS_DIR"/*/; do
  [[ -d "$session" ]] || continue
  [[ -e "$session/.active" ]] && continue

  name="$(basename "$session")"
  echo "Uploading session $name to $TELEMETRY_REMOTE"
  if rsync -a --partial --timeout 60 "$session" "$TELEMETRY_REMOTE/"; then
    rm -rf "$session"
    echo "Session $name uploaded and removed locally"
  else
    echo "WARN: upload of $name failed; keeping local copy" >&2
    status=1
  fi
done

exit "$status"
