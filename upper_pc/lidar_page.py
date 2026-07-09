import math
import re
import time
import tkinter as tk
from tkinter import ttk


HEADER = b"\xAA\x55"
LIDAR_CMD_START_SCAN = b"\xA5\x60"
LIDAR_CMD_STOP_SCAN = b"\xA5\x65"
DEFAULT_SAMPLE_SIZE = 2
FORMAT_PHOTO_2B = "photo-2B"
FORMAT_ROS2_3B = "ros2-3B"
PACKET_FORMATS = (FORMAT_PHOTO_2B, FORMAT_ROS2_3B)
NUMBER_RE = re.compile(r"-?\d+(?:\.\d+)?")
NAMED_ANGLE_RE = re.compile(r"\b(?:ANGLE|A)\s*[:=]?\s*(-?\d+(?:\.\d+)?)", re.I)
NAMED_DIST_RE = re.compile(r"\b(?:DIST|D|R)\s*[:=]?\s*(-?\d+(?:\.\d+)?)", re.I)


def angle_q6(raw):
    return (raw >> 1) / 64.0


def angle_correct(distance_mm):
    if distance_mm <= 0.0:
        return 0.0
    value = (21.8 * (155.3 - distance_mm)) / (155.3 * distance_mm)
    return math.degrees(math.atan(value))


def checksum(packet):
    value = int.from_bytes(packet[0:2], "little")
    value ^= int.from_bytes(packet[2:4], "little")
    value ^= int.from_bytes(packet[4:6], "little")
    value ^= int.from_bytes(packet[6:8], "little")
    for offset in range(10, len(packet), 2):
        value ^= int.from_bytes(packet[offset : offset + 2], "little")
    return value


class LidarPacketDecoder:
    def __init__(self, packet_format=FORMAT_PHOTO_2B):
        self.buffer = bytearray()
        self.bad_frames = 0
        self.checksum_errors = 0
        self.sample_size = DEFAULT_SAMPLE_SIZE
        self.packet_format = packet_format
        self._apply_format()

    def set_format(self, packet_format):
        self.packet_format = packet_format
        self.buffer.clear()
        self._apply_format()

    def packet_len(self, lsn):
        if self.packet_format == FORMAT_ROS2_3B:
            return 8 + lsn * 3
        return 10 + lsn * 2

    def _apply_format(self):
        self.sample_size = 3 if self.packet_format == FORMAT_ROS2_3B else 2

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

        samples_info = []
        if self.packet_format == FORMAT_ROS2_3B:
            samples = packet[8:]
            sample_size = 3
        else:
            samples = packet[10:]
            sample_size = 2
        for idx in range(lsn):
            offset = idx * sample_size
            raw = int.from_bytes(samples[offset : offset + 2], "little")
            distance = raw / 4.0
            if distance <= 0.0:
                continue
            samples_info.append((base_angles[idx], distance))

        freq = (ct >> 1) / 10.0 if ct & 0x01 else (ct >> 1)
        return {
            "ct": ct,
            "lsn": lsn,
            "fsa": fsa,
            "lsa": lsa,
            "freq": freq,
            "sample_size": sample_size,
            "format": self.packet_format,
            "is_start": bool(ct & 0x01),
            "samples": samples_info,
        }


class LidarPage(ttk.Frame):
    def __init__(self, parent, send_line=None, send_bytes=None):
        super().__init__(parent)
        self.send_line = send_line
        self.send_bytes = send_bytes
        self.points = {}
        self.point_order = []
        self.point_keys = set()
        self.point_seq = 0
        self.scan_points = {}
        self.obstacles = {}
        self.max_display_points = 1800
        self.scan_count = 0
        self.packet_count = 0
        self.total_points = 0
        self.last_angle = None
        self.last_distance = None
        self.last_update_time = 0.0
        self.last_stats_time = 0.0
        self.paused = False
        self.range_max_var = tk.StringVar(value="4000")
        self.protocol_var = tk.StringVar(value=FORMAT_PHOTO_2B)
        self.status_var = tk.StringVar(value="等待雷达数据")
        self.scan_var = tk.StringVar(value="圈数: 0")
        self.point_var = tk.StringVar(value="当前点: 0")
        self.packet_var = tk.StringVar(value="包: 0  总点: 0")
        self.distance_var = tk.StringVar(value="距离: -")
        self.last_var = tk.StringVar(value="角度: -   距离: -")
        self.decoder = LidarPacketDecoder(self.protocol_var.get())
        self.min_distance = None
        self.max_distance = None
        self.last_freq = 0.0
        self.draw_interval_ms = 80
        self.draw_pending = False
        self.dirty = False
        self.draw_count = 0
        self.enable_second_level_var = tk.BooleanVar(value=True)
        self.show_obstacle_lines_var = tk.BooleanVar(value=False)
        self.filter_enabled_var = tk.BooleanVar(value=True)
        self.filter_level_var = tk.IntVar(value=4)
        self.filter_level_text_var = tk.StringVar(value="等级 4")
        self.filter_min_distance_mm = 30.0

        self._build_ui()

    def _build_ui(self):
        toolbar = ttk.Frame(self, padding=(12, 10, 12, 8))
        toolbar.pack(fill=tk.X)

        ttk.Label(toolbar, text="激光雷达").pack(side=tk.LEFT)
        ttk.Label(toolbar, textvariable=self.status_var).pack(side=tk.LEFT, padx=(12, 0))

        ttk.Button(toolbar, text="清空", command=self.clear).pack(side=tk.RIGHT, padx=(6, 0))
        self.pause_button = ttk.Button(toolbar, text="暂停", command=self.toggle_pause)
        self.pause_button.pack(side=tk.RIGHT, padx=(6, 0))
        ttk.Button(toolbar, text="停转", command=self.stop_scan).pack(side=tk.RIGHT, padx=(6, 0))
        ttk.Button(toolbar, text="启动", command=self.start_scan).pack(side=tk.RIGHT, padx=(6, 0))
        protocol_combo = ttk.Combobox(
            toolbar,
            textvariable=self.protocol_var,
            values=PACKET_FORMATS,
            width=10,
            state="readonly",
        )
        protocol_combo.pack(side=tk.RIGHT, padx=(6, 0))
        protocol_combo.bind("<<ComboboxSelected>>", lambda _event: self.set_protocol(self.protocol_var.get()))
        ttk.Label(toolbar, text="协议").pack(side=tk.RIGHT, padx=(12, 0))
        ttk.Checkbutton(toolbar, text="二级角修正", variable=self.enable_second_level_var).pack(
            side=tk.RIGHT, padx=(12, 0)
        )
        ttk.Checkbutton(toolbar, text="红线", variable=self.show_obstacle_lines_var, command=self._request_update).pack(
            side=tk.RIGHT, padx=(12, 0)
        )
        ttk.Checkbutton(toolbar, text="轻滤波", variable=self.filter_enabled_var, command=self._request_update).pack(
            side=tk.RIGHT, padx=(12, 0)
        )
        ttk.Scale(
            toolbar,
            from_=1,
            to=10,
            variable=self.filter_level_var,
            orient=tk.HORIZONTAL,
            length=90,
            command=self._on_filter_level_changed,
        ).pack(side=tk.RIGHT, padx=(4, 0))
        ttk.Label(toolbar, textvariable=self.filter_level_text_var, width=6).pack(side=tk.RIGHT)
        ttk.Label(toolbar, text="滤波").pack(side=tk.RIGHT, padx=(12, 0))

        ttk.Label(toolbar, text="最大距离 mm").pack(side=tk.RIGHT, padx=(12, 4))
        ttk.Entry(toolbar, textvariable=self.range_max_var, width=8).pack(side=tk.RIGHT)

        body = ttk.Frame(self, padding=(12, 0, 12, 12))
        body.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(body, width=180)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 12))
        left.pack_propagate(False)

        for label, var in (
            ("扫描", self.scan_var),
            ("点云", self.point_var),
            ("数据", self.packet_var),
            ("范围", self.distance_var),
            ("最新", self.last_var),
        ):
            box = ttk.Frame(left)
            box.pack(fill=tk.X, pady=(0, 8))
            ttk.Label(box, text=label).pack(anchor=tk.W)
            ttk.Label(box, textvariable=var).pack(anchor=tk.W)

        ttk.Separator(left).pack(fill=tk.X, pady=8)
        ttk.Label(left, text="彩色点云实时刷新，红色障碍线每圈更新。").pack(anchor=tk.W)

        radar_frame = ttk.LabelFrame(body, text="雷达图", padding=4)
        radar_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.radar_canvas = tk.Canvas(radar_frame, bg="#202326", highlightthickness=0)
        self.radar_canvas.pack(fill=tk.BOTH, expand=True)
        self.point_canvas = self.radar_canvas
        self.obstacle_canvas = self.radar_canvas

    def set_sender(self, send_line):
        self.send_line = send_line

    def set_byte_sender(self, send_bytes):
        self.send_bytes = send_bytes

    def has_pending_frame(self):
        return bool(self.decoder.buffer)

    def frame_length_for_lsn(self, lsn):
        return self.decoder.packet_len(lsn)

    def set_protocol(self, packet_format):
        if packet_format not in PACKET_FORMATS:
            return
        self.protocol_var.set(packet_format)
        self.decoder.set_format(packet_format)
        self.clear()

    def start_scan(self):
        self._send_lidar_command(LIDAR_CMD_START_SCAN, "启动扫描")

    def stop_scan(self):
        self._send_lidar_command(LIDAR_CMD_STOP_SCAN, "停止扫描")

    def _send_lidar_command(self, command, label):
        if self.send_bytes is not None:
            self.send_bytes(command, label)
        elif self.send_line is not None:
            self.send_line(label)

    def clear(self):
        self.points.clear()
        self.point_order.clear()
        self.point_keys.clear()
        self.point_seq = 0
        self.scan_points.clear()
        self.obstacles.clear()
        self.scan_count = 0
        self.packet_count = 0
        self.total_points = 0
        self.last_angle = None
        self.last_distance = None
        self.decoder = LidarPacketDecoder(self.protocol_var.get())
        self.min_distance = None
        self.max_distance = None
        self.last_freq = 0.0
        self.dirty = False
        self.status_var.set("已清空")
        self.scan_var.set("圈数: 0")
        self.point_var.set("当前点: 0")
        self.packet_var.set("包: 0  总点: 0")
        self.distance_var.set("距离: -")
        self.last_var.set("角度: -   距离: -")
        self._draw_now()

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_button.config(text="继续" if self.paused else "暂停")
        self.status_var.set("已暂停" if self.paused else "实时")

    def handle_serial_bytes(self, data):
        packets = self.decoder.feed(data)
        if not packets:
            return 0

        if self.paused:
            return len(packets)

        changed = False
        for packet in packets:
            points = self._packet_points(packet)
            if not points:
                continue
            self.packet_count += 1
            self.total_points += len(points)
            self.last_freq = packet["freq"]
            self._append_points(points)
            changed = True

        if changed:
            self._request_update()
        return len(packets)

    def _packet_points(self, packet):
        points = []
        use_second_level = self.enable_second_level_var.get()
        for base_angle, distance in packet.get("samples", ()):
            angle = base_angle
            if use_second_level:
                angle = (angle + angle_correct(distance)) % 360.0
            points.append((angle, distance))
        return points

    def handle_serial_line(self, line):
        if self.paused:
            return

        point = self._parse_point(line)
        if point is None:
            return

        self._append_points([point])
        self.total_points += 1
        self.packet_count += 1
        self._request_update()

    def _finish_scan(self):
        if self.scan_points:
            self._merge_obstacles(self.scan_points)
            self.scan_points.clear()
        self.scan_count += 1

    def _append_points(self, points):
        for angle, distance in points:
            angle = self._normalize_angle(angle)
            if self.last_angle is not None and angle < (self.last_angle - 180.0):
                self._finish_scan()

            distance = max(0.0, distance)
            key = self.point_seq
            self.point_seq += 1
            self.points[key] = (angle, distance)
            self.scan_points[key] = (angle, distance)
            if self._is_visible_distance(distance):
                self.obstacles[key] = (angle, distance)
            self.point_order.append(key)
            self.point_keys.add(key)
            self.last_angle = angle
            self.last_distance = distance
            self.min_distance = (
                distance if self.min_distance is None else min(self.min_distance, distance)
            )
            self.max_distance = (
                distance if self.max_distance is None else max(self.max_distance, distance)
            )

        if len(self.point_order) > self.max_display_points:
            stale = self.point_order[:-self.max_display_points]
            self.point_order = self.point_order[-self.max_display_points:]
            for key in stale:
                self.points.pop(key, None)
                self.point_keys.discard(key)
                self.obstacles.pop(key, None)

        self.last_update_time = time.time()

    def _request_update(self):
        now = time.time()
        if now - self.last_stats_time >= 0.10:
            self._update_stats()
            self.last_stats_time = now
        self.dirty = True
        self._request_draw()

    def _request_draw(self):
        if self.draw_pending:
            return
        self.draw_pending = True
        self.after(self.draw_interval_ms, self._draw_if_dirty)

    def _draw_if_dirty(self):
        self.draw_pending = False
        if not self.dirty:
            return
        self.dirty = False
        self._update_stats()
        self.last_stats_time = time.time()
        self._draw_now()

    def _merge_obstacles(self, scan_points):
        visible = {
            key: value
            for key, value in scan_points.items()
            if self._is_visible_distance(value[1])
        }
        if len(visible) >= 8:
            self.obstacles.update(visible)

    def _is_visible_distance(self, distance):
        return 0.0 < distance <= self._max_range()

    def _update_stats(self):
        self.scan_var.set(f"圈数: {self.scan_count}")
        self.point_var.set(f"当前点: {len(self.points)}")
        self.packet_var.set(f"包: {self.packet_count}  总点: {self.total_points}")
        if self.min_distance is None or self.max_distance is None:
            self.distance_var.set("距离: -")
        else:
            self.distance_var.set(
                f"距离: {self.min_distance:.1f}-{self.max_distance:.1f} mm"
            )
        if self.last_angle is None or self.last_distance is None:
            self.last_var.set("角度: -   距离: -")
        else:
            self.last_var.set(
                f"角度: {self.last_angle:.1f}   距离: {self.last_distance:.1f} mm"
            )
        self.status_var.set(
            f"实时  {self.last_freq:.1f} Hz  {self.decoder.packet_format}  S {self.decoder.sample_size}B  坏帧 {self.decoder.bad_frames}"
        )

    def _parse_point(self, line):
        upper = line.upper()
        angle = None
        distance = None

        match = NAMED_ANGLE_RE.search(line)
        if match:
            angle = float(match.group(1))
        match = NAMED_DIST_RE.search(line)
        if match:
            distance = float(match.group(1))

        if angle is not None and distance is not None:
            return angle, distance

        numbers = [float(value) for value in NUMBER_RE.findall(line)]
        if len(numbers) >= 2 and any(token in upper for token in ("SCAN", "LIDAR", "RADAR", "RANGE", "POINT", "DIST")):
            return numbers[0], numbers[1]

        return None

    def _normalize_angle(self, angle):
        angle = math.fmod(angle, 360.0)
        if angle < 0.0:
            angle += 360.0
        return angle

    def _now(self):
        try:
            return tk._default_root.tk.call("clock", "milliseconds")  # type: ignore[attr-defined]
        except Exception:
            return 0.0

    def _draw_now(self):
        self.draw_count += 1
        self._draw_radar()

    def _plot_geometry(self, canvas):
        width = max(canvas.winfo_width(), 2)
        height = max(canvas.winfo_height(), 2)
        cx = width * 0.5
        cy = height * 0.54
        outer = min(width, height) * 0.40
        return width, height, cx, cy, outer

    def _draw_radar(self):
        canvas = self.radar_canvas
        canvas.delete("all")
        width, height, cx, cy, outer = self._plot_geometry(canvas)
        max_range = self._max_range()
        self._draw_grid(canvas, cx, cy, outer)

        obstacle_points = []
        if self.show_obstacle_lines_var.get():
            obstacle_points = self._obstacle_points(max_range, cx, cy, outer)
            self._draw_obstacle_lines(canvas, obstacle_points)

        draw_items = self._visible_draw_items(max_range)
        visible_points = 0
        for _key, angle, distance in draw_items:
            x, y, ratio = self._polar_to_canvas(angle, distance, max_range, cx, cy, outer)
            color = self._point_color(ratio)
            radius = 3 if ratio < 0.55 else 2
            canvas.create_oval(
                x - radius,
                y - radius,
                x + radius,
                y + radius,
                fill=color,
                outline="",
            )
            visible_points += 1

        canvas.create_oval(cx - 6, cy - 6, cx + 6, cy + 6, fill="#f8fafc", outline="")
        if not self.points and not self.obstacles:
            canvas.create_text(cx, cy, fill="#cbd5e1", text="等待雷达数据")
        self._draw_overlay_text(canvas, visible_points, len(obstacle_points))

    def _visible_draw_items(self, max_range):
        items = []
        for key in self.point_order:
            if key not in self.points:
                continue
            angle, distance = self.points[key]
            if distance <= 0.0 or distance > max_range:
                continue
            if self.filter_enabled_var.get() and distance < self.filter_min_distance_mm:
                continue
            items.append((key, angle, distance))
        if not self.filter_enabled_var.get() or len(items) < 3:
            return items
        return self._neighbor_filter_items(items)

    def _on_filter_level_changed(self, value):
        level = max(1, min(10, int(round(float(value)))))
        if self.filter_level_var.get() != level:
            self.filter_level_var.set(level)
        self.filter_level_text_var.set(f"等级 {level}")
        self._request_update()

    def _neighbor_filter_items(self, items):
        kept = []
        level = max(1, min(10, int(round(self.filter_level_var.get()))))
        angle_window = max(0.6, 5.0 - (level * 0.38))
        distance_window = max(90.0, 760.0 - (level * 62.0))
        neighbor_radius = 1 + min(level // 2, 4)
        min_neighbors = 1 if level <= 4 else 2 if level <= 8 else 3
        for index, item in enumerate(items):
            _key, angle, distance = item
            neighbors = 0
            for offset in range(-neighbor_radius, neighbor_radius + 1):
                if offset == 0:
                    continue
                other_index = index + offset
                if other_index < 0 or other_index >= len(items):
                    continue
                _other_key, other_angle, other_distance = items[other_index]
                angle_gap = abs(self._angle_delta(angle, other_angle))
                if angle_gap <= angle_window and abs(distance - other_distance) <= distance_window:
                    neighbors += 1
                    if neighbors >= min_neighbors:
                        break
            if neighbors >= min_neighbors:
                kept.append(item)
        return kept

    def _angle_delta(self, first, second):
        delta = (first - second + 180.0) % 360.0 - 180.0
        return delta

    def _draw_overlay_text(self, canvas, visible_points, obstacle_points):
        lines = [
            f"序号: {self.packet_count}  角度: {self._fmt_value(self.last_angle)}  距离: {self._fmt_value(self.last_distance)}",
            self._distance_summary(),
            f"当前点: {visible_points}  障碍线: {obstacle_points}",
            "",
            f"扫描频率: {self.last_freq:.1f} Hz",
            f"帧格式: {self.decoder.packet_format}",
            f"采样格式: {self.decoder.sample_size}B",
            f"圈数: {self.scan_count}",
            f"总点数: {self.total_points}",
            f"坏帧: {self.decoder.bad_frames}",
        ]
        y = 16
        for line in lines:
            if line:
                canvas.create_text(16, y, anchor=tk.W, fill="#dbeafe", text=line)
            y += 24 if line else 14

    def _distance_summary(self):
        if self.min_distance is None or self.max_distance is None:
            return "距离范围: -"
        return f"距离范围: {self.min_distance:.1f}-{self.max_distance:.1f} mm"

    def _fmt_value(self, value):
        if value is None:
            return "-"
        return f"{value:.1f}"

    def _obstacle_points(self, max_range, cx, cy, outer):
        points = []
        for _key, (angle, distance) in sorted(self.obstacles.items()):
            if distance <= 0.0 or distance > max_range:
                continue
            x, y, _ratio = self._polar_to_canvas(angle, distance, max_range, cx, cy, outer)
            points.append((angle, distance, x, y))
        return points

    def _draw_obstacle_lines(self, canvas, obstacle_points):
        segment = []
        previous = None
        for item in obstacle_points:
            if previous is None:
                segment = [item]
            else:
                angle_gap = item[0] - previous[0]
                distance_gap = abs(item[1] - previous[1])
                if angle_gap <= 4.0 and distance_gap <= 900.0:
                    segment.append(item)
                else:
                    self._draw_obstacle_segment(canvas, segment)
                    segment = [item]
            previous = item
        self._draw_obstacle_segment(canvas, segment)

    def _draw_point_cloud(self):
        canvas = self.point_canvas
        canvas.delete("all")
        width, height, cx, cy, outer = self._plot_geometry(canvas)
        self._draw_grid(canvas, cx, cy, outer)

        if not self.points:
            canvas.create_text(
                cx,
                cy,
                fill="#9fb3c8",
                text="等待雷达点云",
            )
            return

        max_range = self._max_range()

        for key in self.point_order:
            if key not in self.points:
                continue
            angle, distance = self.points[key]
            if distance > max_range:
                continue
            x, y, ratio = self._polar_to_canvas(angle, distance, max_range, cx, cy, outer)
            color = self._point_color(ratio)
            canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill=color, outline="")

        canvas.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill="#e2e8f0", outline="")
        canvas.create_text(
            14,
            14,
            anchor=tk.W,
            fill="#dbe7f2",
            text=f"Packets: {self.packet_count}  Points: {self.total_points}",
        )

    def _draw_obstacles(self):
        canvas = self.obstacle_canvas
        canvas.delete("all")
        width, height, cx, cy, outer = self._plot_geometry(canvas)
        self._draw_grid(canvas, cx, cy, outer)

        if not self.obstacles:
            canvas.create_text(
                cx,
                cy,
                fill="#9fb3c8",
                text="等待障碍物轮廓",
            )
            return

        max_range = self._max_range()
        obstacle_points = self._obstacle_points(max_range, cx, cy, outer)
        self._draw_obstacle_lines(canvas, obstacle_points)

        canvas.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill="#e2e8f0", outline="")
        canvas.create_text(
            14,
            14,
            anchor=tk.W,
            fill="#fecaca",
            text=f"障碍线: {len(obstacle_points)} 点  每圈刷新",
        )

    def _draw_obstacle_segment(self, canvas, segment):
        if len(segment) < 2:
            return
        coords = []
        for _angle, _distance, x, y in segment:
            coords.extend((x, y))
        canvas.create_line(coords, fill="#ef4444", width=2, smooth=True)

    def _max_range(self):
        try:
            return max(1.0, float(self.range_max_var.get()))
        except ValueError:
            return 4000.0

    def _polar_to_canvas(self, angle, distance, max_range, cx, cy, outer):
        ratio = min(distance / max_range, 1.0)
        radius = ratio * outer
        theta = math.radians(angle - 90.0)
        x = cx + math.cos(theta) * radius
        y = cy + math.sin(theta) * radius
        return x, y, ratio

    def _draw_legacy(self):
        canvas = self.point_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 2)
        height = max(canvas.winfo_height(), 2)
        cx = width * 0.5
        cy = height * 0.54
        outer = min(width, height) * 0.42

        self._draw_grid(canvas, cx, cy, outer)

        if not self.points:
            canvas.create_text(
                cx,
                cy,
                fill="#9fb3c8",
                text="等待雷达点云",
            )
            return

        max_range = self._max_range()

        for angle in self.point_order:
            if angle not in self.points:
                continue
            _, distance = self.points[angle]
            ratio = min(distance / max_range, 1.0)
            radius = ratio * outer
            theta = math.radians(self.points[angle][0] - 90.0)
            x = cx + math.cos(theta) * radius
            y = cy + math.sin(theta) * radius
            color = self._point_color(ratio)
            canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill=color, outline="")

        canvas.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill="#e2e8f0", outline="")
        canvas.create_text(
            14,
            14,
            anchor=tk.W,
            fill="#dbe7f2",
            text=f"Packets: {self.packet_count}  Points: {self.total_points}",
        )

    def _draw_grid(self, canvas, cx, cy, outer):
        for fraction in (0.25, 0.5, 0.75, 1.0):
            radius = outer * fraction
            canvas.create_oval(
                cx - radius,
                cy - radius,
                cx + radius,
                cy + radius,
                outline="#1b2a3a",
            )
            label = int(self._max_range() * fraction)
            canvas.create_text(
                cx + 8,
                cy - radius + 14,
                anchor=tk.W,
                fill="#8b949e",
                text=f"{label}",
            )

        for angle in range(0, 360, 30):
            theta = math.radians(angle - 90.0)
            x = cx + math.cos(theta) * outer
            y = cy + math.sin(theta) * outer
            canvas.create_line(cx, cy, x, y, fill="#4b5563")
            lx = cx + math.cos(theta) * (outer + 12)
            ly = cy + math.sin(theta) * (outer + 12)
            canvas.create_text(lx, ly, fill="#8191a5", text=str(angle))

        canvas.create_line(cx - outer, cy, cx + outer, cy, fill="#6b7280")
        canvas.create_line(cx, cy - outer, cx, cy + outer, fill="#6b7280")
        canvas.create_text(cx, cy - outer - 16, fill="#d1d5db", text="前方")

    def _point_color(self, ratio):
        red = int(80 + (150 * (1.0 - ratio)))
        green = int(220 - (90 * ratio))
        blue = int(255 - (120 * ratio))
        return f"#{red:02x}{green:02x}{blue:02x}"
