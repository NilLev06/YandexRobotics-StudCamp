# Телеметрия ровера

Постоянная запись всего, что происходит с ровером — работает с момента
загрузки, независимо от того, запущен поиск или нет. Каждое записанное
сообщение/строка/видеосегмент имеет точный UTC-таймстамп.

## Что собирается

**В rosbag (формат MCAP, открывается в Foxglove):**

| Данные | Топики |
|---|---|
| Лидар 360° | `/scan` |
| Одометрия (колёса, ICP, EKF) | `/hardware/odom`, `/icp/odom`, `/ekf/*`, `/odometry*` |
| IMU (гироскоп) | `/hardware/imu` |
| Батарея | `/hardware/battery` |
| Скорости/углы колёс | `/hardware/status` |
| Команды движения | `/cmd_vel` |
| Позы и трансформы | `/tf`, `/tf_static` |
| Карта SLAM и планы Nav2 | `/map`, `/plan*` |
| Статусы действий Nav2 (spin, navigate) | `/spin/_action/status`, `/navigate_to_pose/_action/status` |
| Цели и фидбек | `/goal`, `/goal/*` |
| Кадры с YOLO-боксами (5 fps) | `/camera/yolo/image_annotated/compressed` |
| Голос | `/voice/*` |
| Диагностика | `/diagnostics*`, `/behavior_tree_log` |

**Отдельными файлами в каталоге сессии:**

- `video/cam_<UTC-время>.mp4` — видео с камеры сегментами по 2 минуты,
  H264 без перекодирования (пишет второй экземпляр mediamtx)
- `system.jsonl` — раз в секунду: загрузка CPU, память, температура,
  троттлинг Pi (`vcgencmd get_throttled`), свободный диск, сигнал Wi-Fi;
  раз в 15 секунд — CPU/память каждого контейнера
- `yolo_health.jsonl` — каждый новый кадр YOLO-инференса: список детекций
  с классами/уверенностью/bbox, fps, ошибки пайплайна
- `events.jsonl` — события самой телеметрии (старт/стоп, остановка записи
  из-за диска, удаление старых сессий)
- `meta.json` — время старта, hostname, git-коммит кода

## Где лежит

```
/home/robomarvel/rover-logs/sessions/
└── 2026-07-15T15-30-00Z/        # имя сессии = UTC-время старта
    ├── bag/                     # rosbag2 (.mcap, режется по 512 МБ)
    ├── video/cam_*.mp4
    ├── system.jsonl
    ├── yolo_health.jsonl
    ├── events.jsonl
    └── meta.json
```

Сессии режутся по 1 часу (настраивается). Закрытая сессия становится
доступной аплоадеру; новая открывается сразу же.

## Выгрузка на сервер

На ровере данные не хранятся дольше необходимого. Аплоадер каждые 10 минут
переносит завершённые сессии на сервер и удаляет локальные копии после
успешной передачи. Сервер задаётся в `config/telemetry.env`:

```sh
TELEMETRY_REMOTE=user@server:/path/to/rover-logs
```

Требуется SSH-доступ по ключу (без пароля): `ssh-copy-id user@server`.
Пока `TELEMETRY_REMOTE` пуст, сессии копятся локально в пределах капа
(`TELEMETRY_CAP_GB`, по умолчанию 10 ГБ) — старые удаляются автоматически.

**Защита диска:** если свободного места меньше `TELEMETRY_MIN_FREE_GB`
(по умолчанию 2 ГБ), запись останавливается — телеметрия никогда не
забьёт SD-карту до отказа ровера.

## Установка (один раз, на ровере)

```sh
cd /home/robomarvel/z_boys
./scripts/install_telemetry.sh
```

После этого запись стартует автоматически при каждой загрузке ровера.

## Управление

```sh
systemctl status rbm-telemetry            # что происходит
journalctl -u rbm-telemetry -f            # живой лог сервиса
sudo systemctl restart rbm-telemetry      # перезапуск (новая сессия)

# Полностью отказаться от телеметрии:
sudo systemctl disable --now rbm-telemetry rbm-telemetry-upload.timer
# или поставить TELEMETRY_ENABLED=0 в config/telemetry.env
```

## Как смотреть данные

- **Foxglove** (уже используется на проекте): File → Open local file →
  выбрать `.mcap` из `bag/` — вся сессия с таймлайном: лидар, картинка,
  команды, батарея синхронно.
- **Видео** — любой плеер (VLC, QuickTime).
- **JSONL** — построчный JSON, удобно грепать:
  `jq 'select(.cpu_temp_c > 70)' system.jsonl`,
  `jq 'select(.detection_count > 0) | {ts, detections}' yolo_health.jsonl`.
