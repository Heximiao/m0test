import struct
import time

import rclpy
import serial
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Bool


class GimbalUartNode(Node):
    def __init__(self):
        super().__init__("gimbal_uart")
        self.declare_parameter("port", "/dev/ttyAMA0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("deadband_pixels", 6.0)
        self.declare_parameter("yaw_gain", 0.25)
        self.declare_parameter("pitch_gain", 0.25)
        self.declare_parameter("maximum_rate_deg_s", 90.0)
        self.declare_parameter("yaw_direction", 1.0)
        self.declare_parameter("pitch_direction", 1.0)
        self.declare_parameter("yaw_limit_deg", 180.0)
        self.declare_parameter("pitch_limit_deg", 90.0)
        self.declare_parameter("target_timeout_sec", 0.30)
        self.declare_parameter("enable_motors", False)

        port = str(self.get_parameter("port").value)
        baudrate = int(self.get_parameter("baudrate").value)
        self.serial = serial.Serial(port, baudrate, timeout=0, write_timeout=0.05)
        self.detected = False
        self.center = None
        self.last_center_time = 0.0
        self.last_update_time = time.monotonic()
        self.sequence = 0
        self.yaw_target = 0.0
        self.pitch_target = 0.0
        self.rx_buffer = bytearray()

        self.create_subscription(Bool, "/rectangle/detected", self.on_detected, 5)
        self.create_subscription(PointStamped, "/rectangle/center", self.on_center, 5)
        self.timer = self.create_timer(0.05, self.control_tick)
        self.get_logger().info(
            f"Gimbal UART opened on {port} at {baudrate}; "
            f"enable_motors={bool(self.get_parameter('enable_motors').value)}"
        )

    def on_detected(self, message):
        self.detected = bool(message.data)

    def on_center(self, message):
        self.center = (float(message.point.x), float(message.point.y))
        self.last_center_time = time.monotonic()

    @staticmethod
    def clamp(value, minimum, maximum):
        return min(maximum, max(minimum, value))

    def update_target(self, now):
        elapsed = min(0.1, max(0.0, now - self.last_update_time))
        self.last_update_time = now
        timeout = float(self.get_parameter("target_timeout_sec").value)
        tracking_valid = (
            self.detected
            and self.center is not None
            and now - self.last_center_time <= timeout
        )
        if not tracking_valid:
            return False

        center_x, center_y = self.center
        error_x = center_x - float(self.get_parameter("image_width").value) * 0.5
        error_y = center_y - float(self.get_parameter("image_height").value) * 0.5
        deadband = float(self.get_parameter("deadband_pixels").value)
        if abs(error_x) <= deadband:
            error_x = 0.0
        if abs(error_y) <= deadband:
            error_y = 0.0

        maximum_rate = float(self.get_parameter("maximum_rate_deg_s").value)
        yaw_rate = self.clamp(
            error_x
            * float(self.get_parameter("yaw_gain").value)
            * float(self.get_parameter("yaw_direction").value),
            -maximum_rate,
            maximum_rate,
        )
        pitch_rate = self.clamp(
            error_y
            * float(self.get_parameter("pitch_gain").value)
            * float(self.get_parameter("pitch_direction").value),
            -maximum_rate,
            maximum_rate,
        )
        yaw_limit = float(self.get_parameter("yaw_limit_deg").value)
        pitch_limit = min(90.0, float(self.get_parameter("pitch_limit_deg").value))
        self.yaw_target = self.clamp(
            self.yaw_target + yaw_rate * elapsed, -yaw_limit, yaw_limit
        )
        self.pitch_target = self.clamp(
            self.pitch_target + pitch_rate * elapsed, -pitch_limit, pitch_limit
        )
        return True

    def build_packet(self, tracking_valid):
        flags = 0
        if tracking_valid:
            flags |= 0x01
        if bool(self.get_parameter("enable_motors").value):
            flags |= 0x02
        payload = struct.pack(
            ">BBBBhh",
            0xA5,
            0x5A,
            self.sequence,
            flags,
            round(self.yaw_target * 10.0),
            round(self.pitch_target * 10.0),
        )
        checksum = 0
        for value in payload:
            checksum ^= value
        return payload + bytes((checksum,))

    def read_status(self):
        waiting = self.serial.in_waiting
        if waiting:
            self.rx_buffer.extend(self.serial.read(waiting))
        while len(self.rx_buffer) >= 9:
            header_index = self.rx_buffer.find(b"\x5a\xa5")
            if header_index < 0:
                self.rx_buffer.clear()
                return
            if header_index:
                del self.rx_buffer[:header_index]
            if len(self.rx_buffer) < 9:
                return
            packet = bytes(self.rx_buffer[:9])
            del self.rx_buffer[:9]
            checksum = 0
            for value in packet[:8]:
                checksum ^= value
            if checksum != packet[8]:
                continue
            _, _, sequence, status, yaw, pitch = struct.unpack(">BBBBhh", packet[:8])
            self.get_logger().debug(
                f"MCU ACK seq={sequence} status={status} "
                f"yaw={yaw / 10.0:.1f} pitch={pitch / 10.0:.1f}"
            )

    def control_tick(self):
        now = time.monotonic()
        tracking_valid = self.update_target(now)
        packet = self.build_packet(tracking_valid)
        try:
            self.serial.write(packet)
            self.sequence = (self.sequence + 1) & 0xFF
            self.read_status()
        except serial.SerialException as error:
            self.get_logger().error(f"UART failure: {error}")

    def destroy_node(self):
        if self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GimbalUartNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
