import math
import re
import time

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None


ODO_RE = re.compile(
    r"\bODO\b.*\bL=(-?\d+)\b.*\bR=(-?\d+)\b", re.IGNORECASE
)


def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


class WheelOdomNode(Node):
    def __init__(self):
        super().__init__("wheel_odom_node")
        self.declare_parameter("port", "/dev/ttyAMA10")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("wheel_diameter_mm", 40.3)
        self.declare_parameter("wheel_base_mm", 129.0)
        self.declare_parameter("encoder_ppr", 11.0)
        self.declare_parameter("reduction_ratio", 20.0)
        self.declare_parameter("count_multiplier", 2.0)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("start_telemetry", False)

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.wheel_diameter_mm = float(
            self.get_parameter("wheel_diameter_mm").value
        )
        self.wheel_base_m = float(self.get_parameter("wheel_base_mm").value) / 1000.0
        encoder_ppr = float(self.get_parameter("encoder_ppr").value)
        reduction_ratio = float(self.get_parameter("reduction_ratio").value)
        count_multiplier = float(self.get_parameter("count_multiplier").value)
        counts_per_rev = encoder_ppr * reduction_ratio * count_multiplier
        wheel_circumference_m = (self.wheel_diameter_mm / 1000.0) * math.pi
        self.meters_per_count = wheel_circumference_m / counts_per_rev
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.start_telemetry = bool(self.get_parameter("start_telemetry").value)

        if serial is None:
            raise RuntimeError("python3-serial is required")

        self.odom_pub = self.create_publisher(Odometry, "odom", 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.serial = serial.Serial(self.port, self.baudrate, timeout=0.02)
        self.serial.reset_input_buffer()
        if self.start_telemetry:
            self.serial.write(b"TELE 1\n")

        self.buffer = bytearray()
        self.last_left = None
        self.last_right = None
        self.last_stamp = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.frame_count = 0
        self.last_log = time.monotonic()

        self.get_logger().info(
            f"wheel odom started port={self.port} baudrate={self.baudrate} "
            f"meters_per_count={self.meters_per_count:.7f} "
            f"wheel_base={self.wheel_base_m:.3f}m"
        )
        self.timer = self.create_timer(0.01, self.read_serial)

    def destroy_node(self):
        try:
            if hasattr(self, "serial") and self.serial and self.serial.is_open:
                self.serial.close()
        finally:
            super().destroy_node()

    def read_serial(self):
        waiting = getattr(self.serial, "in_waiting", 0)
        data = self.serial.read(max(1, min(waiting or 1, 512)))
        if not data:
            return

        self.buffer.extend(data)
        while True:
            newline = self.buffer.find(b"\n")
            if newline < 0:
                if len(self.buffer) > 256:
                    del self.buffer[:-128]
                return

            raw_line = bytes(self.buffer[:newline]).strip()
            del self.buffer[: newline + 1]
            if not raw_line:
                continue
            line = raw_line.decode("ascii", errors="ignore")
            self.handle_line(line)

    def handle_line(self, line):
        match = ODO_RE.search(line)
        if not match:
            return

        left = int(match.group(1))
        right = int(match.group(2))
        stamp = self.get_clock().now()
        if self.last_left is None or self.last_right is None:
            self.last_left = left
            self.last_right = right
            self.last_stamp = stamp
            self.publish_odom(stamp, 0.0, 0.0)
            return

        delta_left = (left - self.last_left) * self.meters_per_count
        delta_right = (right - self.last_right) * self.meters_per_count
        delta_center = (delta_left + delta_right) * 0.5
        delta_yaw = (delta_right - delta_left) / self.wheel_base_m
        mid_yaw = self.yaw + (delta_yaw * 0.5)

        self.x += delta_center * math.cos(mid_yaw)
        self.y += delta_center * math.sin(mid_yaw)
        self.yaw = math.atan2(math.sin(self.yaw + delta_yaw), math.cos(self.yaw + delta_yaw))

        dt = (stamp - self.last_stamp).nanoseconds / 1e9
        linear_velocity = delta_center / dt if dt > 0.0 else 0.0
        angular_velocity = delta_yaw / dt if dt > 0.0 else 0.0

        self.last_left = left
        self.last_right = right
        self.last_stamp = stamp
        self.publish_odom(stamp, linear_velocity, angular_velocity)

        self.frame_count += 1
        now = time.monotonic()
        if now - self.last_log >= 2.0:
            self.get_logger().info(
                f"odom frames={self.frame_count} x={self.x:.3f} "
                f"y={self.y:.3f} yaw={math.degrees(self.yaw):.1f}"
            )
            self.last_log = now

    def publish_odom(self, stamp, linear_velocity, angular_velocity):
        quat = yaw_to_quaternion(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.08
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[35] = 0.1
        self.odom_pub.publish(odom)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp.to_msg()
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.translation.z = 0.0
            transform.transform.rotation = quat
            self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
