#!/usr/bin/env bash
# Runs on the Pi host (not in a container) because docker/runc refuses to
# bind-mount anything under /proc into a container ("proc-safety" check),
# so /proc/net/wireless can't be shared with the dashboard directly. This
# reads it here and writes a plain file that mounts into containers fine.
set -uo pipefail

OUT_FILE="${1:-/run/rover-wifi-dbm}"
OUT_TMP="${OUT_FILE}.tmp"

parse_dbm() {
  awk 'NR > 2 { gsub(/\./, "", $4); print $4; exit }' /proc/net/wireless 2>/dev/null
}

value="$(parse_dbm)"
if [[ -n "$value" ]]; then
  echo "$value" > "$OUT_TMP" && mv -f "$OUT_TMP" "$OUT_FILE"
else
  # No wlan0 line (Wi-Fi down/disconnected): remove the file instead of
  # leaving the last-known value in place forever. dashboard.py's
  # read_wifi_dbm() treats a missing file as "no data" (shows "—"), which is
  # honest -- a stale number here would silently lie about the connection.
  rm -f "$OUT_TMP" "$OUT_FILE"
fi
