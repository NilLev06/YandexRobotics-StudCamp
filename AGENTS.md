# RoboMarvel rover deployment rules

These rules apply to every file in this repository and to deployments on the
RoboMarvel rover fleet. Student operators and automated agents must follow them.

## Safety boundary

- Never issue movement goals, publish non-zero velocity, or start autonomous
  search as part of a configuration or deployment check.
- Keep the rover stationary with clear space around it. Treat motor, battery,
  navigation, and firmware changes as hardware-affecting operations.
- Rover `.37` is the canary unless the operator explicitly designates another
  unit. After the canary passes, use batches no larger than four rovers.
- Do not deploy to a rover whose hostname, IP address, project directory, ROS
  distribution, or Compose layout differs from the expected preflight result.

## Required preflight

Before changing a rover, record:

```bash
hostname
date --iso-8601=seconds
timedatectl show -p NTPSynchronized --value
nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status
docker inspect ros --format '{{.Image}} {{.State.Status}} {{.RestartCount}}'
docker image inspect registry.robotics-lab.ru/robomarvel:v6 \
  --format '{{.Id}} {{.Created}} {{json .RepoDigests}}'
docker compose config --quiet
```

Also verify that `/home/robomarvel/robomarvel` exists and inspect local changes.
Never assume that the mutable tag `v6` is identical across rovers.

## Backups and rollback

- Before overwriting files, copy the exact targets to
  `backups/<change>-<YYYYMMDD-HHMMSS>/` on that rover.
- Never delete user data, maps, findings, models, logs, or existing backups.
- Do not use `git reset --hard`, `git checkout --`, `docker system prune`,
  `docker compose down`, or a full workspace clean during routine deployment.
- If validation fails, restore the saved files, rebuild only the affected ROS
  packages, restart the existing container, and verify the rollback.

## Secrets and Yandex Cloud

- Real credentials belong only in `rover_m2m/yandex-cloud.env`, mode `0600`.
- Never commit, print, log, paste into `Dockerfile`, or bake API keys into an
  image. Repository files may contain only `yandex-cloud.env.example`.
- Required variables are `YANDEX_FOLDER_ID` and `YANDEX_API_KEY`. Optional
  variables are documented in `rover_m2m/README.md`.
- Validate presence by variable name only. Do not output values during checks.
- Rotate a key immediately if it appears in Git, CI output, shell tracing, or a
  public log.

## Docker and boot

- Never run `docker pull` or rebuild solely because a tag name matches. Compare
  image IDs/digests and verify `/usr/bin/mediamtx` is non-empty first.
- Prefer `docker restart ros` after a code-only symlink-install build.
- Use `docker compose up -d --force-recreate --no-build ros` only when Compose
  itself changed and after confirming the local image is valid.
- Full image builds are reserved for Dockerfile or system dependency changes.
- Where `rbm-ros.service` is installed, it is the sole boot owner and must start
  after network and clock synchronization; Compose restart policy must be `no`.

## Wi-Fi changes

- Inspect the active NetworkManager profile before modifying it.
- A recovery action must scan first. If SSID `robomarvel` is not visible, it
  must log and exit without forcing a connection.
- Do not delete active profiles remotely. The recovery watchdog may recreate a
  missing profile only after the target SSID is visible.
- Do not log the PSK. The recovery script must remain root-owned and mode `0700`.

## Navigation changes

- Treat `params.yaml`, `cartographer_rover_2d.lua`, `main.yaml`, hardware odometry,
  and `odom_sanitizer.py` as one versioned set. Do not copy only one of them.
- Run Python syntax checks and build package `main` (and `hwnode` if changed).
- After restart, verify `/scan`, `/icp/odom`, and TF `odom -> base_link` without
  commanding motion. Check lifecycle nodes and recent ROS launch errors.
- Never widen ICP jump limits or disable battery/velocity safety guards merely
  to make a failing test pass.

## Required validation

For the cloud/camera package:

```bash
cd /src/rover_m2m
/usr/bin/python3 -m unittest -v test_camera_snapshot.py test_rover_dispatcher.py
curl --fail --max-time 15 --output /tmp/rover-camera.jpg \
  http://127.0.0.1:8091/api/camera/snapshot.jpg
file /tmp/rover-camera.jpg
```

For ROS source changes:

```bash
cd /src
make PYTHON=/usr/bin/python3 build packages=main
```

After restarting, wait at least 15 seconds and verify the container remains up,
Foxglove is reachable, the camera endpoint returns a non-empty JPEG, and no new
process is in a restart loop. A port being open is not sufficient proof.

## Fleet rollout

- Validate `.37` first, then deploy in operator-approved batches (maximum four).
- Keep a per-rover result: preflight, backup path, build, restart, camera frame,
  ROS/TF health, Wi-Fi timer, and rollback status.
- Stop rollout when the same new failure appears twice. Diagnose before
  continuing to more devices.
- Offline or structurally different rovers are reported as skipped, not forced.
