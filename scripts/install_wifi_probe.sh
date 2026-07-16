#!/usr/bin/env bash
# Installs a systemd timer that reads /proc/net/wireless on the host every
# 2 seconds and writes the signal level to /run/rover-wifi-dbm -- a plain
# file the dashboard container can bind-mount (docker/runc refuses to
# bind-mount anything under /proc directly). Only needed for the Wi-Fi card
# in docker-compose.dashboard.yaml; telemetry recording doesn't need this.
# Run on the rover: ./scripts/install_wifi_probe.sh

set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"

if [[ "$EUID" -eq 0 ]]; then
  echo "ERROR: запускайте от обычного пользователя с sudo, не от root" >&2
  exit 1
fi

install_unit() {
  local name="$1" content="$2"
  local file="/etc/systemd/system/${name}"
  echo "Installing $file"
  printf "%s\n" "$content" | sudo tee "$file" > /dev/null
}

install_unit "rbm-wifi-probe.service" "[Unit]
Description=RBM Wi-Fi signal probe (writes /run/rover-wifi-dbm)

[Service]
Type=oneshot
ExecStart=$SCRIPT_DIR/host_wifi_probe.sh"

install_unit "rbm-wifi-probe.timer" "[Unit]
Description=Run RBM Wi-Fi signal probe every 2 seconds

[Timer]
OnBootSec=2s
OnUnitActiveSec=2s
AccuracySec=1s

[Install]
WantedBy=timers.target"

sudo systemctl daemon-reload
sudo systemctl enable --now rbm-wifi-probe.timer

echo
echo "Готово. Проверка:"
echo "  cat /run/rover-wifi-dbm"
echo
echo "Выключить: sudo systemctl disable --now rbm-wifi-probe.timer"
