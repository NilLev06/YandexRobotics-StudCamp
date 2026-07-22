# Безопасная настройка и развёртывание роверов

## Конфигурация Yandex Cloud

Секреты не входят в прошивку или Docker-образ. На каждом ровере создаётся файл:

```text
/home/robomarvel/robomarvel/rover_m2m/yandex-cloud.env
```

Пример заполнения:

```bash
cd /home/robomarvel/robomarvel/rover_m2m
cp yandex-cloud.env.example yandex-cloud.env
nano yandex-cloud.env
chmod 600 yandex-cloud.env
```

Обязательны `YANDEX_FOLDER_ID` и `YANDEX_API_KEY`. Значения ключей нельзя
помещать в Git, документацию, журналы, Compose или Dockerfile. Изменение ключа
не требует пересборки образа.

## Получение фотографии

```bash
curl --fail --max-time 15 \
  --output /tmp/rover-camera.jpg \
  http://ROVER_IP:8091/api/camera/snapshot.jpg
```

Успешный ответ имеет `Content-Type: image/jpeg` и `Cache-Control: no-store`.
Ошибка камеры возвращает HTTP `503` и JSON `camera_unavailable`.

## Безопасное изменение кода

1. Выбрать один canary-ровер, обычно `.37`.
2. Записать hostname, время, image digest, состояние контейнера и сети.
3. Скопировать изменяемые файлы в каталог `backups/` на роботе.
4. Проверить Python и unit-тесты.
5. Для ROS-кода выполнить только целевую сборку:

   ```bash
   docker exec ros bash -lc \
     'cd /src && make PYTHON=/usr/bin/python3 build packages=main'
   ```

6. Перезапустить существующий контейнер командой `docker restart ros`.
7. Через 15 секунд проверить процессы ROS, Foxglove, TF и реальный JPEG.
8. Только после canary переходить к следующему роверу.

Не использовать `docker compose down`, `docker system prune`, полный `clean` или
слепой `docker pull`. Тег `v6` уже содержал разные образы на разных роверах.
Для старого образа `adaptive_mapping.launch.py` выбирает `slam_toolbox`, а при
наличии `cartographer_ros` — Cartographer. Не заменять этот выбор жёстким include
без проверки состава образа.

## Пакетный rollout

После успешной проверки canary `.37` разрешены группы не более четырёх роверов.
Оркестратор запускается с управляющей машины; секреты передаются только после
структурного preflight. При ошибке следующая группа не запускается. Нестандартный
студенческий проект нельзя форсировать: сначала согласовать миграцию и сохранить
его контейнеры для отката.

## Навигационный комплект

Навигационные параметры переносятся атомарно:

- `packages/main/config/params.yaml`;
- `packages/main/config/cartographer_rover_2d.lua`;
- `packages/main/launch/main.yaml`;
- `packages/main/scripts/odom_sanitizer.py`;
- связанная реализация аппаратной одометрии в `packages/hwnode`.

После сборки без движения проверить:

```bash
ros2 topic hz /scan
ros2 topic hz /icp/odom
ros2 run tf2_ros tf2_echo odom base_link
```

Отсутствующий или устаревший `odom -> base_link` блокирует Nav2 и является
причиной остановки rollout, а не поводом увеличивать пороги ICP вслепую.

## Wi-Fi watchdog

`rbm-wifi-recover.timer` запускает проверку каждые 20 секунд. При обрыве скрипт:

1. сканирует доступные сети и пишет результат в journal;
2. ничего не делает, если `robomarvel` не виден;
3. поднимает сохранённый профиль;
4. пересоздаёт профиль только если он отсутствует и SSID виден.

Проверка:

```bash
systemctl status rbm-wifi-recover.timer
journalctl -t rbm-wifi-recover --no-pager
nmcli -f connection.autoconnect,connection.autoconnect-retries,\
802-11-wireless.powersave connection show netplan-wlan0-robomarvel
```

## Откат

Восстановить файлы из созданного каталога `backups/<change>-<timestamp>`, снова
собрать только затронутые пакеты и перезапустить контейнер. Не удалять карты,
модели, результаты поиска или пользовательские файлы.
