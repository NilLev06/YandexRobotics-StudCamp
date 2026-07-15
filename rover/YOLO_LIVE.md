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

Open:

- Annotated viewer: `http://192.168.2.44:8091/`
- MJPEG only: `http://192.168.2.44:8091/stream.mjpg`
- One JPEG: `http://192.168.2.44:8091/snapshot.jpg`
- Health/metrics: `http://192.168.2.44:8091/health`
- Original WebRTC feed: `http://192.168.2.44:8889/cam/`

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

The port 8091 stream has no authentication. Keep it on the trusted rover LAN;
do not expose it directly to the public internet.
