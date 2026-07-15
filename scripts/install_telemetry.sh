#!/usr/bin/env bash
# Установка телеметрии как systemd-сервисов: запись стартует при загрузке
# ровера, аплоадер выгружает завершённые сессии каждые 10 минут.
# Запускать на ровере: ./scripts/install_telemetry.sh

set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
REPO_DIR="$(dirname -- "$SCRIPT_DIR")"

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

install_unit "rbm-telemetry.service" "[Unit]
Description=RBM rover telemetry recorder (rosbag + video + metrics)
After=docker.service network.target
Requires=docker.service

[Service]
Type=simple
User=$USER
WorkingDirectory=$REPO_DIR
ExecStart=$REPO_DIR/scripts/run_telemetry.sh
Restart=always
RestartSec=15

[Install]
WantedBy=multi-user.target"

install_unit "rbm-telemetry-upload.service" "[Unit]
Description=RBM rover telemetry uploader (rsync finished sessions)

[Service]
Type=oneshot
User=$USER
WorkingDirectory=$REPO_DIR
ExecStart=$REPO_DIR/scripts/upload_telemetry.sh"

install_unit "rbm-telemetry-upload.timer" "[Unit]
Description=Run RBM telemetry uploader every 10 minutes

[Timer]
OnBootSec=5min
OnUnitActiveSec=10min

[Install]
WantedBy=timers.target"

sudo systemctl daemon-reload
sudo systemctl enable --now rbm-telemetry.service
sudo systemctl enable --now rbm-telemetry-upload.timer

echo
echo "Готово. Проверка:"
echo "  systemctl status rbm-telemetry"
echo "  ls $(grep -oP '(?<=^TELEMETRY_DIR=).*' "$REPO_DIR/config/telemetry.env" 2>/dev/null || echo /home/robomarvel/rover-logs)/sessions"
echo
echo "Выключить всё: sudo systemctl disable --now rbm-telemetry rbm-telemetry-upload.timer"
