import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera")
        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("framerate", 15.0)
        self.declare_parameter("jpeg_quality", 75)

        device = self.get_parameter("device").value
        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        self.framerate = self.get_parameter("framerate").value
        self.jpeg_quality = self.get_parameter("jpeg_quality").value

        self.capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.CAP_PROP_FPS, self.framerate)
        if not self.capture.isOpened():
            raise RuntimeError(f"Unable to open camera {device}")

        self.publisher = self.create_publisher(
            CompressedImage, "/camera/image/compressed", 2
        )
        self.timer = self.create_timer(1.0 / self.framerate, self.publish_frame)
        self.get_logger().info(
            f"Publishing {width}x{height} camera frames at {self.framerate:.1f} FPS"
        )

    def publish_frame(self):
        success, frame = self.capture.read()
        if not success:
            self.get_logger().warning("Camera frame read failed", throttle_duration_sec=5.0)
            return

        success, encoded = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if not success:
            return

        message = CompressedImage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "camera"
        message.format = "jpeg"
        message.data = encoded.tobytes()
        self.publisher.publish(message)

    def destroy_node(self):
        self.capture.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
