# RoboMarvel rover — object search

A ROS 2 (Jazzy) rover that finds a named object (currently a fixed word, later
by voice) using YOLO and a lidar-guarded search/approach behavior. This is a
sidecar tree: it runs alongside the main `robomarvel` project without editing
or mounting it, on the same Docker network.

## Structure

```
rover/
├── config/
│   ├── target.txt                 # current search target, one word (e.g. "bottle")
│   └── search_calibration.json    # camera<->lidar calibration; approach is
│                                   # locked out until this is measured and validated
├── packages/                      # ROS 2 packages, built with colcon
│   ├── main/                      # bringup, Nav2 launch/config, goal_proxy
│   ├── hwnode/                    # serial link to the motor controller/IMU/battery
│   ├── detection/                 # in-tree YOLO detector node (compressed image topic)
│   └── voice/                     # STT/TTS nodes (not yet wired into search)
├── scripts/
│   ├── object_search.py           # guarded search-and-approach behavior (see below)
│   ├── run_object_search.sh       # host-side launcher for object_search.py
│   ├── yolo_live_mjpeg.py         # camera overlay sidecar (MJPEG + /health + ROS topic)
│   └── setup.sh                   # first-time Raspberry Pi provisioning
├── docker/                        # patches applied to upstream ROS packages at build time
├── Dockerfile                     # main `ros` image (ROS 2 + OpenCV + Ultralytics + Nav2 + SLAM)
├── docker-compose.yaml            # full privileged rover stack (main `ros` container)
├── docker-compose.yolo-live.yaml  # YOLO camera-overlay sidecar only
├── entrypoint.sh                  # main container's startup (colcon build, tmuxp autostart)
├── OBJECT_SEARCH.md               # how to run the guarded search, safety notes
└── YOLO_LIVE.md                   # how to run/use the camera overlay stream
```

## How it fits together

```
config/target.txt  ──►  run_object_search.sh  ──►  object_search.py
                                                          │
                                    reads /health from    │  Nav2 Spin / NavigateToPose
                                    yolo_live_mjpeg.py ◄──┘         │
                                    (any COCO class)                ▼
                                                                 hwnode.py
                                                            (serial → motors)
```

- **`yolo_live_mjpeg.py`** runs continuously in its own container
  (`docker-compose.yolo-live.yaml`) and exposes every detection it sees — not
  just the current target — over `/health` (JSON) and as an MJPEG stream at
  `http://<rover-ip>:8091/`.
- **`config/target.txt`** holds the one word the rover is currently looking
  for. Edit it by hand for now; a future voice node writes here instead.
- **`object_search.py`** is a short-lived, sandboxed container
  (`run_object_search.sh`) that runs preflight safety checks (battery, lidar
  clearance, fresh sensor data), spins the rover in small steps looking for
  the target class, and — only if calibration is validated — approaches it
  via Nav2. See [OBJECT_SEARCH.md](OBJECT_SEARCH.md) for the full safety
  model and run instructions.
- **`hwnode.py`** is the only thing that talks to the motor controller over
  serial; everything else commands motion through ROS topics/actions.

## Quick start

```sh
# on the rover, inside this directory
docker start ros
docker compose -f docker-compose.yolo-live.yaml up -d   # camera overlay
echo "bottle" > config/target.txt
./scripts/run_object_search.sh --dry-run --search-only  # preflight only
./scripts/run_object_search.sh --search-only            # actually search
```

See [OBJECT_SEARCH.md](OBJECT_SEARCH.md) and [YOLO_LIVE.md](YOLO_LIVE.md) for
details, safety gates, and exit codes.
