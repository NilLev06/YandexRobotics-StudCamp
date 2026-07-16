# Guarded object search

`scripts/run_object_search.sh` is the single host-side launcher for the rover's
object-search state machine. It runs a short-lived, unprivileged ROS 2 sidecar
on the main rover Docker network; it does not edit or mount the main
`/home/robomarvel/robomarvel` tree.

The target class (any YOLO/COCO label, e.g. `bottle`, `chair`, `cup`) comes
from `config/target.txt` by default, or `--target-class NAME` to override it.
This is a placeholder for voice input: later, a voice command node will write
the recognized word into `target.txt` instead of you editing it by hand — the
search behavior itself does not change.

The behavior is deliberately fail-closed:

1. Verify that the test dashboard is stopped and ROS, Nav2, YOLO, lidar,
   odometry, battery, and transforms are fresh.
2. Require conservative 360-degree lidar clearance before any rotation.
3. Search in small Nav2 `Spin` steps, stopping and waiting for multiple fresh
   YOLO inference sequences at each view.
4. Stop after a confirmed detection of the target class.
5. Approach only when an explicit, measured camera/lidar calibration file is
   enabled and the target can be paired unambiguously with a persistent lidar
   cluster. Nav2 receives a standoff goal, never the object's center.
6. Cancel and send zero velocity on stale data, a close obstacle, timeout,
   rejection, Ctrl-C, or any exception.

If all spin steps complete without a confirmed target, the behavior exits with
status `2`; only that status makes the launcher play the WAV file. Safety or
calibration failures exit with status `1` and never play the no-find sound.

## Current rover safety state

Do not bypass the defaults. The July 14 audit found all of the following:

- The nearest lidar return was about 0.15-0.19 m, below the launcher's default
  0.35 m spin-clearance requirement.
- The physical chassis swept radius has not been measured; Nav2 models only an
  0.08 m circular radius.
- The motor layer enforced a relatively fast minimum in-place turn despite a
  lower requested ROS angular speed. This has been reduced from 0.8 to
  0.3 rad/s (`hwnode/hwnode/hwnode.py: MIN_INPLACE_ANGULAR`), but the chassis
  swept radius still needs to be physically measured before that number is
  trusted for approach mode.
- Nav2 path-following collision detection is disabled.
- The camera has no `CameraInfo`, calibrated optical frame, or measured
  camera-to-base transform. A YOLO pixel therefore cannot safely be associated
  with a lidar obstacle yet.

The provided calibration JSON has `calibrated: false`, so autonomous approach
is intentionally locked out. `--search-only` does not need approach
calibration, but it still needs a physically clear spin area.

## Run

Set the target once:

```sh
echo "bottle" > config/target.txt
```

First test all non-motion preflights:

```sh
cd /home/robomarvel/z_boys
./scripts/run_object_search.sh --dry-run --search-only
```

After physically placing the rover in a clear area, a search-only run is:

```sh
./scripts/run_object_search.sh --search-only
```

The full search-and-approach command is the same launcher without
`--search-only`, but it will refuse to navigate until the calibration file is
measured, validated, and explicitly enabled:

```sh
./scripts/run_object_search.sh
```

Override the target without touching `target.txt`:

```sh
./scripts/run_object_search.sh --target-class chair --search-only
```

Use another WAV without changing the behavior:

```sh
./scripts/run_object_search.sh --wav /tmp/my-not-found.wav --search-only
```

If the default `/tmp/object-not-found.wav` is absent, the launcher creates a
quiet alert tone with `ffmpeg`. It plays through the Google Voice HAT using
`aplay -D plughw:0,0` only after a complete no-find search.

The root test dashboard must remain inactive because it can issue competing
motion commands:

```sh
sudo systemctl stop rbm-web-tests.service
```

Exit codes are `0` for success, `1` for an unsafe/error stop, `2` for a complete
no-find search (after audio playback), and `130` for an interrupted run.

## Exploring when a single 360° circle isn't enough

`scripts/run_object_search.sh --explore` wraps the same search behavior for
rooms larger than one spin can cover: if a full 360° circle finds nothing, it
picks the nearest unexplored frontier of the SLAM `/map` (a boundary between
known free space and the unknown) and drives there with Nav2 before searching
again, repeating until the target is confirmed, the reachable area is fully
mapped, or `--max-cycles` / `--max-runtime-s` is reached.

It changes nothing about the safety model above — `scripts/explore_search.py`
is a subclass of `object_search.py`'s `BottleSearchNode` that reuses every
preflight gate, the same Nav2 action wrappers, and the same `/search/found`,
`/search/sighted`, `/search/status` topics, so `mapping/map_view.py` shows
exploration runs with no changes on its side. `scripts/frontier.py` (pure
Python, no ROS dependency) does the frontier selection and is independently
testable with `python3 scripts/frontier.py`.

```sh
echo "bottle" > config/target.txt
./scripts/run_object_search.sh --explore --dry-run --search-only   # preflight only
./scripts/run_object_search.sh --explore --search-only              # explore, don't approach
./scripts/run_object_search.sh --explore --max-cycles 5              # full explore + approach
```

Exit codes match `run_object_search.sh`: `0` on a confirmed target, `1` on an
unsafe/error stop, `2` once the reachable area is fully explored (or the
cycle/time budget runs out) without a confirmed target — that status plays
the same no-find WAV as the single-circle search.

## Calibration required for approach

Approach mode must remain locked until these are physically measured and
validated:

- Camera intrinsics and distortion for the exact 1280x720 stream.
- `base_link` to `camera_optical_frame` translation and rotation.
- Lidar bearing orientation against the chassis.
- The lidar scan plane intersecting a floor-standing target object at useful
  ranges.
- Actual chassis swept radius and navigation footprint.

A known-bearing stationary target test and several short supervised standoff
tests should be completed before setting `calibrated` to `true`.
