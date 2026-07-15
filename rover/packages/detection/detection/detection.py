import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO
import cv2
import numpy as np
import threading
from threading import Lock


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_detector")
        
        # Parameters
        self.declare_parameter("model", "yolo11s")
        self.declare_parameter("confidence", 0.5)
        self.declare_parameter("rtsp_url", "rtsp://localhost:8554/cam")
        self.declare_parameter("process_fps", 1.0)
        self.declare_parameter("jpeg_quality", 50)
        
        self.model_name = self.get_parameter("model").value
        self.confidence = self.get_parameter("confidence").value
        self.rtsp_url = self.get_parameter("rtsp_url").value
        self.process_fps = self.get_parameter("process_fps").value
        self.jpeg_quality = self.get_parameter("jpeg_quality").value
        
        # Load YOLO model
        self.get_logger().info(f"Loading model...")
        self.model = YOLO(f"/root/weights/{self.model_name}", task="detect")
        
        # Background thread to read RTSP frames
        self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open RTSP stream: {self.rtsp_url}")
            raise RuntimeError("Cannot connect to RTSP stream")
        self.get_logger().info(f"Connected to RTSP stream: {self.rtsp_url}")
        
        self.latest_frame = None
        self.running = True
        self.read_thread = threading.Thread(target=self._frame_reader, daemon=True)
        self.read_thread.start()
        
        # Publisher for compressed annotated image
        self.annotated_pub = self.create_publisher(
            CompressedImage,
            "/camera/image_annotated_compressed",
            10
        )
        
        # Timer for frame processing
        self.timer = self.create_timer(1.0 / self.process_fps, self.process_frame)
        self.get_logger().info("YOLO detector ready")

    def _frame_reader(self):
        frame_read_errors = 0
        max_errors = 5
        
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                frame_read_errors += 1
                self.get_logger().warn(f"Frame read error #{frame_read_errors}")
                if frame_read_errors >= max_errors:
                    self.get_logger().warn("Reconnecting to RTSP stream...")
                    self.cap.release()
                    self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
                    frame_read_errors = 0
                    if not self.cap.isOpened():
                        self.get_logger().error("Reconnection failed")
                continue
            
            frame_read_errors = 0
            
            # Store the latest frame
            self.latest_frame = frame

        self.cap.release()

    def process_frame(self):
        # Get the latest frame
        frame = self.latest_frame
        if frame is None:
            return
        
        # Run inference
        result = self.model(frame, conf=self.confidence, verbose=False)[0]
        annotated_frame = result.plot()
        
        # Build CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        msg.format = "jpeg"
        
        # Encode annotated image as JPEG
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        _, buffer = cv2.imencode(".jpg", annotated_frame, encode_param)
        msg.data = np.array(buffer).tobytes()
        self.annotated_pub.publish(msg)
        
        # Optional logging
        objects = []
        for box in result.boxes:
            label = self.model.names[int(box.cls.item())]
            objects.append({label: round(box.conf.item(), 2)})
        self.get_logger().info(f"Detected {len(objects)} objects: {objects}")

    def destroy_node(self):
        self.running = False
        if self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()


if __name__ == "__main__":
    main()
