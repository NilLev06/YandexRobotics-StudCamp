# RoboMarvel rover — поиск объекта

Ровер на ROS 2 (Jazzy), который ищет заданный объект (пока — по фиксированному
слову, позже — по голосу) с помощью YOLO и safety-gated поведения
поиска/подъезда с лидаром. Это сайдкар-проект: работает рядом с основным
проектом `robomarvel`, не редактируя и не монтируя его, в той же Docker-сети.

## Структура

```
.
├── config/
│   ├── target.txt                 # текущая цель поиска, одно слово (например "bottle")
│   └── search_calibration.json    # калибровка камера<->лидар; подъезд
│                                   # заблокирован, пока это не измерено и не подтверждено
├── packages/                      # ROS 2 пакеты, собираются через colcon
│   ├── main/                      # bringup, launch/конфиг Nav2, goal_proxy
│   ├── hwnode/                    # serial-связь с контроллером моторов/IMU/батареей
│   ├── detection/                 # встроенная нода YOLO-детектора (топик сжатого изображения)
│   └── voice/                     # ноды STT/TTS (пока не подключены к поиску)
├── scripts/
│   ├── object_search.py           # guarded-поведение поиска и подъезда (см. ниже)
│   ├── run_object_search.sh       # хостовый лаунчер для object_search.py
│   ├── yolo_live_mjpeg.py         # сайдкар оверлея камеры (MJPEG + /health + ROS-топик)
│   └── setup.sh                   # первичная настройка Raspberry Pi
├── docker/                        # патчи, применяемые к сторонним ROS-пакетам при сборке
├── Dockerfile                     # основной образ `ros` (ROS 2 + OpenCV + Ultralytics + Nav2 + SLAM)
├── docker-compose.yaml            # полный привилегированный стек ровера (основной контейнер `ros`)
├── docker-compose.yolo-live.yaml  # только сайдкар YOLO-оверлея камеры
├── entrypoint.sh                  # старт основного контейнера (colcon build, tmuxp autostart)
├── OBJECT_SEARCH.md               # как запускать guarded-поиск, заметки по безопасности
└── YOLO_LIVE.md                   # как запускать/использовать стрим оверлея камеры
```

## Как это работает вместе

```
config/target.txt  ──►  run_object_search.sh  ──►  object_search.py
                                                          │
                                    читает /health из     │  Nav2 Spin / NavigateToPose
                                    yolo_live_mjpeg.py ◄──┘         │
                                    (любой класс COCO)              ▼
                                                                 hwnode.py
                                                            (serial → моторы)
```

- **`yolo_live_mjpeg.py`** работает постоянно в своём контейнере
  (`docker-compose.yolo-live.yaml`) и отдаёт все детекции, которые видит — не
  только текущую цель — через `/health` (JSON) и как MJPEG-стрим по адресу
  `http://<ip-ровера>:8091/`.
- **`config/target.txt`** содержит одно слово — что ровер сейчас ищет.
  Пока редактируется руками; в будущем сюда будет писать голосовая нода.
- **`object_search.py`** — короткоживущий изолированный контейнер
  (`run_object_search.sh`), который проходит preflight-проверки безопасности
  (батарея, клиренс лидара, свежесть данных сенсоров), крутит ровер малыми
  шагами в поиске целевого класса и — только если калибровка подтверждена —
  подъезжает к нему через Nav2. Полную модель безопасности и инструкции по
  запуску см. в [OBJECT_SEARCH.md](OBJECT_SEARCH.md).
- **`hwnode.py`** — единственное, что говорит с контроллером моторов по
  serial; всё остальное командует движением через ROS-топики/actions.

## Быстрый старт

```sh
# на ровере, внутри этой директории
docker start ros
docker compose -f docker-compose.yolo-live.yaml up -d   # оверлей камеры
echo "bottle" > config/target.txt
./scripts/run_object_search.sh --dry-run --search-only  # только preflight
./scripts/run_object_search.sh --search-only            # реальный поиск
```

Подробности, safety-гейты и коды завершения — в [OBJECT_SEARCH.md](OBJECT_SEARCH.md)
и [YOLO_LIVE.md](YOLO_LIVE.md).
