# Rover M2M connector

Текстовый сценарий без голосовых команд:

1. Берёт один кадр с MediaMTX (`rtsp://127.0.0.1:8554/cam`).
2. Qwen3.6 в Yandex AI Studio определяет объект на изображении.
3. YandexGPT Pro 5.1 формирует две команды из `P10_yaRoverGuide.md`.
4. Локальный код строго проверяет контракт и неизменность `class_name`.
5. По умолчанию выполняет dry-run. Реальная отправка в Unity включается только
   флагом `--send`, при заданных `UNITY_API_URL` и `GFSX_ALLOWED_CLASSES`.

## Конфигурация Yandex Cloud

Все параметры облака находятся в отдельном файле
`/src/rover_m2m/yandex-cloud.env`. Реальный файл не входит в Git и должен быть
доступен только владельцу:

```bash
cd /home/robomarvel/robomarvel/rover_m2m
cp yandex-cloud.env.example yandex-cloud.env
nano yandex-cloud.env
chmod 600 yandex-cloud.env
```

Обязательные переменные:

- `YANDEX_FOLDER_ID` — идентификатор каталога Yandex Cloud;
- `YANDEX_API_KEY` — API-ключ сервисного аккаунта с доступом к моделям.

Настраиваемые переменные:

- `YANDEX_API_URL` — Responses API AI Studio;
- `YANDEX_VISION_MODEL` — мультимодальная модель распознавания;
- `YANDEX_COMMAND_MODEL` — модель формирования команд.

Путь можно переопределить переменной `YANDEX_CLOUD_ENV_FILE` или аргументом
`--env-file`. Ключи нельзя добавлять в `Dockerfile`, Compose или образ: один и
тот же образ разворачивается на всех роверах, а секреты подключаются отдельно.

Запуск на роботе:

```bash
docker exec ros python3 /src/rover_m2m/rover_dispatcher.py \
  --env-file /src/rover_m2m/yandex-cloud.env
```

Проверка на сохранённом изображении:

```bash
docker exec ros python3 /src/rover_m2m/rover_dispatcher.py \
  --env-file /src/rover_m2m/yandex-cloud.env \
  --image /src/rover_m2m/test.jpg
```

Реальная передача в Unity (только после проверки dry-run и списка классов):

```bash
docker exec ros python3 /src/rover_m2m/rover_dispatcher.py \
  --env-file /src/rover_m2m/yandex-cloud.env --send
```

Локальные тесты:

```bash
cd rover_m2m && python3 -m unittest -v
```

## API снимка камеры

Менеджер поиска предоставляет свежий JPEG без обращения к Yandex Cloud:

```text
GET http://ROVER_IP:8091/api/camera/snapshot.jpg
Content-Type: image/jpeg
Cache-Control: no-store
```

Пример:

```bash
curl --fail --output rover.jpg \
  http://192.168.2.37:8091/api/camera/snapshot.jpg
```

При недоступной камере API возвращает HTTP `503` и JSON с кодом
`camera_unavailable`. Одновременные запросы сериализуются, чтобы несколько
клиентов не открывали CSI-камеру параллельно.

Параметры камеры задаются отдельно от Yandex Cloud:

- `ROVER_CAMERA_URL` (по умолчанию `rtsp://127.0.0.1:8554/cam`);
- `CAMERA_SNAPSHOT_JPEG_QUALITY` (`1..100`, по умолчанию `85`);
- `CAMERA_SNAPSHOT_TIMEOUT_MS` (`250..30000`, по умолчанию `5000`).

## Сборка и развёртывание

Секреты не требуют пересборки. После изменения `yandex-cloud.env` достаточно
перезапустить задачу поиска или контейнер:

```bash
docker restart ros
```

После изменения ROS-кода менеджера:

```bash
docker exec ros bash -lc 'cd /src && make build packages=main'
docker restart ros
```

Полная пересборка образа нужна только при изменении `Dockerfile` или системных
зависимостей:

```bash
cd /home/robomarvel/robomarvel
docker compose build ros
docker compose up -d --force-recreate --no-build ros
```

После развёртывания обязательно проверить:

```bash
curl --fail --output /tmp/rover.jpg \
  http://127.0.0.1:8091/api/camera/snapshot.jpg
docker exec ros python3 /src/rover_m2m/rover_dispatcher.py --help
```

## Автономный поиск объекта

Однопроцессная задача принимает текстовое название, обследует свободную связную
область карты через Nav2, выполняет секторный обзор и требует два подтверждения
Qwen выше порога уверенности. Она не отправляет команды в Unity.

```bash
docker exec ros bash -c 'source /opt/ros/jazzy/setup.bash && \
  source /src/install/setup.bash && \
  export LD_LIBRARY_PATH=/opt/ros/jazzy/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH && \
  python3 /src/rover_m2m/autonomous_object_search.py "мяч"'
```

Только построить план без движения и запросов к модели:

```bash
docker exec ros bash -c 'source /opt/ros/jazzy/setup.bash && \
  source /src/install/setup.bash && \
  export LD_LIBRARY_PATH=/opt/ros/jazzy/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH && \
  python3 /src/rover_m2m/autonomous_object_search.py "мяч" --plan-only'
```

Статус и итог публикуются в `/object_search/status` и
`/object_search/result`. Подтверждающий кадр и JSON сохраняются в `findings/`.

## Защита аккумулятора

Штатный `hwnode` публикует быстрое напряжение в `/hardware/battery_raw` и
защёлкнутый сигнал `/hardware/low_battery`. На этом экземпляре моторы перестали
отвечать при 7.1 В, поэтому защита срабатывает при 7.2 В после выдержки 0.7 с и
снимается после устойчивого восстановления до 7.4 В.

При аварии `battery_guard`:

- отменяет активные Nav2 actions и `/goal`;
- удерживает нулевой `/cmd_vel`;
- публикует `DiagnosticStatus.ERROR` `RoboMarvel/Battery` в `/diagnostics`;
- отправляет в `/voice/speak` просьбу заменить аккумулятор не чаще раза в минуту.
