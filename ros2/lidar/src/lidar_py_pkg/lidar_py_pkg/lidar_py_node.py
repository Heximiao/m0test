import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None


HEADER = b"\xAA\x55"
START_SCAN = b"\xA5\x60"
STOP_SCAN = b"\xA5\x65"
FORMAT_PHOTO_2B = "photo-2B"
FORMAT_ROS2_3B = "ros2-3B"


def angle_q6(raw):
    return (raw >> 1) / 64.0


def angle_correct(distance_mm):
    if distance_mm <= 0.0:
        return 0.0
    value = (21.8 * (155.3 - distance_mm)) / (155.3 * distance_mm)
    return math.degrees(math.atan(value))


class LidarPacketDecoder:
    def __init__(self, packet_format=FORMAT_ROS2_3B):
        self.buffer = bytearray()
        self.packet_format = packet_format
        self.bad_frames = 0

    def packet_len(self, lsn):
        if self.packet_format == FORMAT_ROS2_3B:
            return 8 + lsn * 3
        return 10 + lsn * 2

    def feed(self, data):
        if not data:
            return []

        self.buffer.extend(data)
        packets = []
        while True:
            start = self.buffer.find(HEADER)
            if start < 0:
                if len(self.buffer) > 1:
                    del self.buffer[:-1]
                return packets
            if start:
                del self.buffer[:start]

            min_header_len = 8 if self.packet_format == FORMAT_ROS2_3B else 10
            if len(self.buffer) < min_header_len:
                return packets

            lsn = self.buffer[3]
            if lsn == 0 or lsn > 120:
                self.bad_frames += 1
                del self.buffer[0]
                continue

            packet_len = self.packet_len(lsn)
            if len(self.buffer) < packet_len:
                return packets

            packet = bytes(self.buffer[:packet_len])
            del self.buffer[:packet_len]
            parsed = self.parse_packet(packet)
            if parsed is None:
                self.bad_frames += 1
                continue
            packets.append(parsed)

    def parse_packet(self, packet):
        min_header_len = 8 if self.packet_format == FORMAT_ROS2_3B else 10
        if len(packet) < min_header_len or packet[:2] != HEADER:
            return None

        ct = packet[2]
        lsn = packet[3]
        if lsn == 0:
            return None

        fsa = angle_q6(int.from_bytes(packet[4:6], "little"))
        lsa = angle_q6(int.from_bytes(packet[6:8], "little"))
        span = (lsa - fsa) % 360.0
        if lsn <= 1:
            base_angles = [fsa]
        else:
            base_angles = [(fsa + span * idx / (lsn - 1)) % 360.0 for idx in range(lsn)]

        samples = packet[8:] if self.packet_format == FORMAT_ROS2_3B else packet[10:]
        sample_size = 3 if self.packet_format == FORMAT_ROS2_3B else 2
        points = []
        for idx in range(lsn):
            offset = idx * sample_size
            raw = int.from_bytes(samples[offset : offset + 2], "little")
            distance_mm = raw / 4.0
            if distance_mm <= 0.0:
                continue
            angle_deg = (base_angles[idx] + angle_correct(distance_mm)) % 360.0
            points.append((angle_deg, distance_mm / 1000.0))

        return {"ct": ct, "points": points, "is_start": bool(ct & 0x01)}


class SerialLidarNode(Node):
    def __init__(self):
        super().__init__("lidar_py_node")
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("packet_format", FORMAT_ROS2_3B)
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("topic", "scan")
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 8.0)
        self.declare_parameter("scan_size", 720)

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.packet_format = self.get_parameter("packet_format").value
        self.frame_id = self.get_parameter("frame_id").value
        self.topic = self.get_parameter("topic").value
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.scan_size = int(self.get_parameter("scan_size").value)

        if serial is None:
            raise RuntimeError("python3-serial is required")

        self.decoder = LidarPacketDecoder(self.packet_format)
        self.scan_pub = self.create_publisher(LaserScan, self.topic, 10)
        self.current_points = []
        self.scan_start_time = self.get_clock().now()
        self.scan_count = 0
        self.point_count = 0
        self.last_log = time.monotonic()

        self.serial = serial.Serial(self.port, self.baudrate, timeout=0.02)
        self.serial.reset_input_buffer()
        self.serial.write(START_SCAN)
        self.get_logger().info(
            f"lidar started port={self.port} baudrate={self.baudrate} "
            f"format={self.packet_format} topic=/{self.topic}"
        )

        self.timer = self.create_timer(0.005, self.read_serial)

    def destroy_node(self):
        try:
            if hasattr(self, "serial") and self.serial and self.serial.is_open:
                self.serial.write(STOP_SCAN)
                self.serial.close()
        finally:
            super().destroy_node()

    def read_serial(self):
        waiting = getattr(self.serial, "in_waiting", 0)
        data = self.serial.read(max(1, min(waiting or 1, 4096)))
        for packet in self.decoder.feed(data):
            self.handle_packet(packet)

    def handle_packet(self, packet):
        points = packet["points"]
        if not points:
            return

        if packet["is_start"] and self.current_points:
            self.publish_scan()
            self.current_points = []
            self.scan_start_time = self.get_clock().now()

        self.current_points.extend(points)
        self.point_count += len(points)

        now = time.monotonic()
        if now - self.last_log >= 2.0:
            self.get_logger().info(
                f"scans={self.scan_count} points={self.point_count} "
                f"buffer={len(self.current_points)} bad_frames={self.decoder.bad_frames}"
            )
            self.last_log = now

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.scan_start_time.to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = 0.0
        scan.angle_max = 2.0 * math.pi
        scan.angle_increment = (2.0 * math.pi) / self.scan_size
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = [math.inf] * self.scan_size

        for angle_deg, distance_m in self.current_points:
            if distance_m < self.range_min or distance_m > self.range_max:
                continue
            angle_rad = math.radians(angle_deg % 360.0)
            index_angle = (2.0 * math.pi - angle_rad) % (2.0 * math.pi)
            index = int(index_angle / scan.angle_increment)
            if 0 <= index < self.scan_size and distance_m < scan.ranges[index]:
                scan.ranges[index] = float(distance_m)

        self.scan_pub.publish(scan)
        self.scan_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = SerialLidarNode()
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
