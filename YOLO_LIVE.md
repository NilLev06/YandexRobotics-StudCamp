# YOLO live camera overlay

This is a sidecar for the existing rover camera service. It reads the main
container's internal RTSP feed, performs YOLO11n NCNN inference on the newest
frame, and serves a browser-friendly annotated MJPEG stream.

It does not open the CSI camera directly, mount `/dev`, run privileged, or
modify the main `/home/robomarvel/robomarvel` project.

## Start

The existing `ros` container must be running because it owns MediaMTX and the
raw camera feed:

```sh
docker start ros
cd /home/robomarvel/z_boys
docker compose -f docker-compose.yolo-live.yaml up -d
```

Port 8091 is already reserved (mapped but unused) by the main `ros` container's
`docker-compose.yaml`, so this sidecar binds 8092 instead — both inside the
rover network and on the host. All consumers (`object_search.py`, the
dashboard, and telemetry) talk to it as `z-boys-yolo-live:8092`; browser
links below use host port 8092. Change `HTTP_PORT`/`YOLO_LIVE_BIND_HOST` in
`docker-compose.yolo-live.yaml` and the port in the URLs below if 8092 is
also taken on your rover.

Open:

- Annotated viewer: `http://192.168.2.44:8092/`
- MJPEG only: `http://192.168.2.44:8092/stream.mjpg`
- One JPEG: `http://192.168.2.44:8092/snapshot.jpg`
- Health/metrics: `http://192.168.2.44:8092/health`
- Original WebRTC feed: `http://192.168.2.44:8889/cam/`

By default this runs the `rover_m2m/models/yolo11n-ball-cube-int8.onnx`
model (ball/cube/robot-claw, `imgsz=512`) instead of the stock COCO
`yolo11n`, via the `YOLO_MODEL`/`YOLO_IMAGE_SIZE` env vars and a read-only
mount of `rover_m2m/models`. To go back to the stock model, set `YOLO_MODEL`
to `/root/weights/yolo11n_ncnn_model` and drop `YOLO_IMAGE_SIZE` (defaults to
640).

**This image does not ship `onnx`/`onnxruntime`, and the container runs
`read_only: true` so Ultralytics' auto-install on first inference fails
silently (or worse, fails loud with `No space left on device` if `/tmp` is
too small).** Vendor both packages from a container that already has them
(e.g. `ros`, after `pip install onnx onnxruntime` there) into
`z_boys/vendor-python/`, mount it read-only at `/root/vendor-python`, and add
that path to `PYTHONPATH` — see `docker-compose.yolo-live.yaml` for the
working setup. This vendoring does not survive an image rebuild; if
`registry.robotics-lab.ru/robomarvel:v6` gets rebuilt, redo the `pip install`
+ `docker cp` steps.

## Foxglove

The same annotated JPEG is published to ROS 2 at up to 5 FPS without running
a second YOLO model:

```text
/camera/yolo/image_annotated/compressed
sensor_msgs/msg/CompressedImage
```

In Foxglove, connect to `ws://192.168.2.44:8765`, add an **Image** panel, and
select `/camera/yolo/image_annotated/compressed` as its topic.

YOLO updates detections at about 2 FPS while the compositor serves the newest
camera frame at up to 10 FPS. Old boxes expire instead of being drawn over a
later scene. The service accepts up to four simultaneous MJPEG viewers.

`/health` also exposes each fresh detection's class, confidence, pixel bounding
box, center, inference sequence, image size, and source/completion timestamps.
The guarded object-search sidecar uses those structured fields; the annotated
JPEG alone is not used for control.

See `OBJECT_SEARCH.md` for the launchable step-and-stare search workflow and
the calibration gates that intentionally block autonomous approach on the
current hardware setup.

## Operate

```sh
cd /home/robomarvel/z_boys
docker compose -f docker-compose.yolo-live.yaml ps
docker compose -f docker-compose.yolo-live.yaml logs -f
docker compose -f docker-compose.yolo-live.yaml restart
docker compose -f docker-compose.yolo-live.yaml down
```

Do **not** use the existing `z_boys/docker-compose.yaml` for this feature. That
file describes a second full privileged rover stack and conflicts with the
main `ros` container name, ports, and hardware access.

The port 8092 stream has no authentication. Keep it on the trusted rover LAN;
do not expose it directly to the public internet.
