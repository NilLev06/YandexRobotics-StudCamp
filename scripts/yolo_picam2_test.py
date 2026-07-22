from picamera2 import Picamera2
from ultralytics import YOLO
import time

SIZE = 640, 480
FPS = 2

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": "RGB888", "size": SIZE})
picam2.configure(config)
picam2.start()
time.sleep(1)

model = YOLO('~/weights/yolo11n_ncnn_model')

while True:
    t0 = time.perf_counter()
    frame = picam2.capture_array()
    results = model(frame)
    msg = [model.names[int(box.cls[0].item())] for r in results for box in r.boxes]
    print("FRAME", frame.shape, "OBJECTS", msg)
    dt = time.perf_counter() - t0
    time.sleep(max(0, 1 / FPS - dt))
