import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

from rectangle_vision.detector import (
    RectangleDetection,
    RectangleTracker,
    draw_detection,
)


class RectangleVisionNode(Node):
    def __init__(self):
        super().__init__("rectangle_vision")
        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("framerate", 15.0)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("minimum_area_ratio", 0.08)
        self.declare_parameter("maximum_area_ratio", 0.95)
        self.declare_parameter("approximation_epsilon", 0.025)
        self.declare_parameter("line_thickness", 3)
        self.declare_parameter("center_radius", 7)
        self.declare_parameter("maximum_missed_frames", 10)
        self.declare_parameter("hold_frames", 3)
        self.declare_parameter("processing_scale", 0.5)
        self.declare_parameter("detection_interval", 2)
        self.declare_parameter("publish_raw", False)

        device = str(self.get_parameter("device").value)
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        self.framerate = float(self.get_parameter("framerate").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.processing_scale = float(self.get_parameter("processing_scale").value)
        self.processing_scale = min(1.0, max(0.3, self.processing_scale))
        self.detection_interval = max(
            1, int(self.get_parameter("detection_interval").value)
        )
        self.publish_raw = bool(self.get_parameter("publish_raw").value)
        self.frame_index = 0

        self.capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.CAP_PROP_FPS, self.framerate)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.capture.isOpened():
            raise RuntimeError(f"Unable to open camera {device}")

        self.tracker = RectangleTracker(
            maximum_missed_frames=int(
                self.get_parameter("maximum_missed_frames").value
            ),
            hold_frames=int(self.get_parameter("hold_frames").value),
        )

        self.raw_publisher = self.create_publisher(
            CompressedImage, "/camera/image_raw/compressed", 2
        )
        self.annotated_publisher = self.create_publisher(
            CompressedImage, "/rectangle/image/compressed", 2
        )
        self.detected_publisher = self.create_publisher(
            Bool, "/rectangle/detected", 5
        )
        self.center_publisher = self.create_publisher(
            PointStamped, "/rectangle/center", 5
        )
        self.polygon_publisher = self.create_publisher(
            PolygonStamped, "/rectangle/polygon", 5
        )
        self.timer = self.create_timer(1.0 / self.framerate, self.process_frame)
        self.get_logger().info(
            f"Reading {device} at {width}x{height}, {self.framerate:.1f} FPS"
        )

    def _compressed_message(self, frame, stamp):
        success, encoded = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if not success:
            return None
        message = CompressedImage()
        message.header.stamp = stamp
        message.header.frame_id = "camera"
        message.format = "jpeg"
        message.data = encoded.tobytes()
        return message

    def process_frame(self):
        success, frame = self.capture.read()
        if not success:
            self.get_logger().warning(
                "Camera frame read failed", throttle_duration_sec=5.0
            )
            return

        processing_frame = frame
        if self.processing_scale < 0.999:
            processing_frame = cv2.resize(
                frame,
                None,
                fx=self.processing_scale,
                fy=self.processing_scale,
                interpolation=cv2.INTER_AREA,
            )
        run_detection = (
            self.frame_index % self.detection_interval == 0
            or self.tracker.previous_corners is None
        )
        self.frame_index += 1
        detection = self.tracker.update(
            processing_frame,
            minimum_area_ratio=float(self.get_parameter("minimum_area_ratio").value),
            maximum_area_ratio=float(self.get_parameter("maximum_area_ratio").value),
            approximation_epsilon=float(
                self.get_parameter("approximation_epsilon").value
            ),
            run_detection=run_detection,
        )
        if detection is not None and self.processing_scale < 0.999:
            inverse_scale = 1.0 / self.processing_scale
            scaled_corners = detection.corners * inverse_scale
            detection = RectangleDetection(
                corners=scaled_corners,
                center=tuple(
                    np.rint(np.asarray(detection.center) * inverse_scale).astype(int)
                ),
                score=detection.score,
            )
        annotated = draw_detection(
            frame,
            detection,
            line_thickness=int(self.get_parameter("line_thickness").value),
            center_radius=int(self.get_parameter("center_radius").value),
        )
        stamp = self.get_clock().now().to_msg()
        if not rclpy.ok(context=self.context):
            return

        raw_message = self._compressed_message(frame, stamp) if self.publish_raw else None
        annotated_message = self._compressed_message(annotated, stamp)
        if raw_message is not None:
            self.raw_publisher.publish(raw_message)
        if annotated_message is not None:
            self.annotated_publisher.publish(annotated_message)

        detected_message = Bool()
        detected_message.data = detection is not None
        self.detected_publisher.publish(detected_message)
        if detection is None:
            return

        center_message = PointStamped()
        center_message.header.stamp = stamp
        center_message.header.frame_id = "camera_pixels"
        center_message.point.x = float(detection.center[0])
        center_message.point.y = float(detection.center[1])
        center_message.point.z = 0.0
        self.center_publisher.publish(center_message)

        polygon_message = PolygonStamped()
        polygon_message.header.stamp = stamp
        polygon_message.header.frame_id = "camera_pixels"
        polygon_message.polygon.points = [
            Point32(x=float(point[0]), y=float(point[1]), z=0.0)
            for point in detection.corners
        ]
        self.polygon_publisher.publish(polygon_message)

    def destroy_node(self):
        self.capture.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RectangleVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
