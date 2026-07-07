import csv
import binascii
import math
import pathlib
import queue
import re
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from image_sender import image_to_rgb565, load_raw_rgb565

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


BAUD_RATES = ("9600", "19200", "38400", "57600", "115200", "230400", "460800")
PLOT_KEYS = (
    "L", "R", "TARGET", "LT", "RT", "LD", "RD", "ERR", "OUT", "LO", "RO",
    "KP", "KI", "KD", "PITCH", "ROLL", "YAW",
)
KEY_VALUE_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(-?\d+(?:\.\d+)?)")
SCALE_100_KEYS = {
    "BASE", "LT", "RT", "LD", "RD", "ERR", "OUT", "LO", "RO", "ML", "MR",
    "SPEED", "TARGET", "PITCH", "ROLL", "YAW",
}
SCALE_1000_KEYS = {"KP", "KI", "KD"}
CUBE_PITCH_SIGN = -1.0
CUBE_ROLL_SIGN = -1.0
CUBE_YAW_SIGN = 1.0


class SerialTunerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Car PID Serial Tuner")
        self.root.geometry("1080x720")

        self.serial_port = None
        self.reader_thread = None
        self.stop_event = threading.Event()
        self.rx_queue = queue.Queue()
        self.samples = []
        self.max_samples = 240
        self.max_log_lines = 400
        self.draw_interval_s = 0.10
        self.plot_min_span = 6.0
        self.pending_log_lines = []
        self.log_line_count = 0
        self.last_draw_time = 0.0
        self.last_rx_time = 0.0
        self.last_sample_time = 0.0
        self.rx_line_count = 0
        self.ui_line_count = 0
        self.sample_count = 0
        self.draw_count = 0
        self.cube_draw_count = 0
        self.attitude = {"PITCH": 0.0, "ROLL": 0.0, "YAW": 0.0}
        self.raw_attitude = {"PITCH": 0.0, "ROLL": 0.0, "YAW": 0.0}
        self.attitude_zero = None
        self.last_attitude_log_time = 0.0
        self.last_debug_time = time.time()
        self.last_debug_rx_count = 0
        self.last_debug_ui_count = 0
        self.last_debug_sample_count = 0
        self.last_debug_draw_count = 0
        self.draw_pending = False
        self.image_transfer = None
        self.image_writer_thread = None
        self.image_page_ack = threading.Event()
        self.image_manager = None
        self.image_list_active = False
        self.image_list_rows = []
        self.image_info_text = ""

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.image_slot_var = tk.StringVar(value="auto")
        self.image_show_after_send_var = tk.BooleanVar(value=True)
        self.base_var = tk.StringVar(value="20")
        self.kp_var = tk.StringVar(value="4.0")
        self.ki_var = tk.StringVar(value="0.2")
        self.kd_var = tk.StringVar(value="0.4")
        self.status_var = tk.StringVar(value="Disconnected")
        self.debug_var = tk.StringVar(value="RX 0/s  UI 0/s  Q 0  Draw 0/s  Last -")
        self.debug_log_enabled_var = tk.BooleanVar(value=False)

        self._build_ui()
        self.refresh_ports()
        self.root.after(100, self._draw_cube)
        self.root.after(50, self._poll_rx_queue)

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill=tk.X)

        ttk.Label(top, text="Port").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=18)
        self.port_combo.pack(side=tk.LEFT, padx=(4, 10))
        ttk.Button(top, text="Refresh", command=self.refresh_ports).pack(side=tk.LEFT)

        ttk.Label(top, text="Baud").pack(side=tk.LEFT, padx=(14, 0))
        ttk.Combobox(top, textvariable=self.baud_var, values=BAUD_RATES, width=10).pack(
            side=tk.LEFT, padx=(4, 10)
        )

        self.connect_button = ttk.Button(top, text="Connect", command=self.toggle_connection)
        self.connect_button.pack(side=tk.LEFT)
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.LEFT, padx=14)
        ttk.Checkbutton(
            top, text="Debug log", variable=self.debug_log_enabled_var
        ).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Label(top, textvariable=self.debug_var).pack(side=tk.LEFT, padx=14)

        controls = ttk.LabelFrame(self.root, text="PID Commands", padding=8)
        controls.pack(fill=tk.X, padx=8, pady=(0, 8))

        for label, var in (
            ("Base", self.base_var),
            ("Kp", self.kp_var),
            ("Ki", self.ki_var),
            ("Kd", self.kd_var),
        ):
            ttk.Label(controls, text=label).pack(side=tk.LEFT)
            ttk.Entry(controls, textvariable=var, width=8).pack(side=tk.LEFT, padx=(4, 12))

        ttk.Button(controls, text="Send PID", command=self.send_pid).pack(side=tk.LEFT, padx=4)
        ttk.Button(controls, text="Send Base", command=self.send_base).pack(side=tk.LEFT, padx=4)
        ttk.Button(controls, text="Request Status", command=lambda: self.send_line("GET")).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Label(controls, text="Image ID").pack(side=tk.LEFT, padx=(16, 0))
        ttk.Entry(controls, textvariable=self.image_slot_var, width=4).pack(
            side=tk.LEFT, padx=(4, 4)
        )
        ttk.Button(controls, text="Send Image", command=self.choose_image).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(controls, text="Image Files", command=self.open_image_manager).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Checkbutton(
            controls, text="Show", variable=self.image_show_after_send_var
        ).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(controls, text="Save CSV", command=self.save_csv).pack(side=tk.RIGHT, padx=4)
        ttk.Button(controls, text="Clear", command=self.clear_data).pack(side=tk.RIGHT, padx=4)

        middle = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        middle.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        left = ttk.Frame(middle)
        right = ttk.Frame(middle)
        middle.add(left, weight=3)
        middle.add(right, weight=2)

        self.canvas = tk.Canvas(left, height=320, bg="#101418", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        table_frame = ttk.LabelFrame(left, text="Latest Values", padding=6)
        table_frame.pack(fill=tk.X, pady=(8, 0))
        self.value_labels = {}
        for key in PLOT_KEYS:
            box = ttk.Frame(table_frame)
            box.pack(side=tk.LEFT, padx=8)
            ttk.Label(box, text=key).pack()
            value = ttk.Label(box, text="-", width=8, anchor=tk.CENTER)
            value.pack()
            self.value_labels[key] = value

        cube_frame = ttk.LabelFrame(right, text="MPU6050 Attitude", padding=6)
        cube_frame.pack(fill=tk.X, pady=(0, 8))
        ttk.Button(cube_frame, text="Zero MPU", command=self.zero_attitude).pack(
            anchor=tk.E, pady=(0, 4)
        )
        self.cube_canvas = tk.Canvas(cube_frame, height=240, bg="#0b1020", highlightthickness=0)
        self.cube_canvas.pack(fill=tk.X, expand=False)

        log_frame = ttk.LabelFrame(right, text="Serial Log", padding=6)
        log_frame.pack(fill=tk.BOTH, expand=True)
        self.log_text = tk.Text(log_frame, height=20, wrap=tk.NONE)
        self.log_text.pack(fill=tk.BOTH, expand=True)

        cmd_frame = ttk.Frame(right)
        cmd_frame.pack(fill=tk.X, pady=(8, 0))
        self.manual_cmd_var = tk.StringVar()
        ttk.Entry(cmd_frame, textvariable=self.manual_cmd_var).pack(
            side=tk.LEFT, fill=tk.X, expand=True
        )
        ttk.Button(cmd_frame, text="Send", command=self.send_manual).pack(side=tk.LEFT, padx=(8, 0))

    def refresh_ports(self):
        if serial is None:
            self.port_combo["values"] = []
            self.status_var.set("pyserial is not installed")
            return

        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def toggle_connection(self):
        if self.serial_port and self.serial_port.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        if serial is None:
            messagebox.showerror(
                "Missing dependency",
                "Install pyserial first: python -m pip install pyserial",
            )
            return

        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("No port", "Choose a COM port first.")
            return

        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=int(self.baud_var.get()),
                timeout=0.1,
                write_timeout=0.5,
            )
        except Exception as exc:
            messagebox.showerror("Open failed", str(exc))
            return

        self.stop_event.clear()
        self.reader_thread = threading.Thread(target=self._read_serial, daemon=True)
        self.reader_thread.start()
        self.connect_button.config(text="Disconnect")
        self.status_var.set(f"Connected to {port}")

    def disconnect(self):
        self.stop_event.set()
        if self.serial_port:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.connect_button.config(text="Connect")
        self.status_var.set("Disconnected")

    def _read_serial(self):
        while not self.stop_event.is_set():
            try:
                raw = self.serial_port.readline()
            except Exception as exc:
                self.rx_queue.put(("error", str(exc)))
                break

            if raw:
                line = raw.decode("utf-8", errors="replace").strip()
                self.rx_line_count += 1
                self.last_rx_time = time.time()
                self.rx_queue.put(("line", line))

    def _poll_rx_queue(self):
        latest_sample = None
        should_draw = False

        while True:
            try:
                kind, payload = self.rx_queue.get_nowait()
            except queue.Empty:
                break

            if kind == "line":
                self.ui_line_count += 1
                self._handle_image_transfer_line(payload)
                if self._should_log_line(payload):
                    self._queue_log(payload)
                sample = self._parse_sample(payload)
                if sample is not None:
                    latest_sample = sample
                    should_draw = True
            elif kind == "image_status":
                self._queue_log(payload)
            elif kind == "image_done":
                self._queue_log(payload)
                self.status_var.set(payload)
                if self.image_transfer is not None:
                    self.image_transfer["writing"] = False
            elif kind == "image_error":
                self._queue_log(f"[IMAGE ERROR] {payload}")
                self.status_var.set("Image send failed")
                self.image_transfer = None
            else:
                self._queue_log(f"[ERROR] {payload}")
                self.disconnect()

        if latest_sample is not None:
            self._append_sample(latest_sample)
        if self.pending_log_lines:
            self._flush_log()
        if should_draw:
            self._request_draw_plot()
        self._update_debug_status()

        self.root.after(50, self._poll_rx_queue)

    def _update_debug_status(self):
        now = time.time()
        elapsed = now - self.last_debug_time
        if elapsed < 1.0:
            return

        rx_rate = (self.rx_line_count - self.last_debug_rx_count) / elapsed
        ui_rate = (self.ui_line_count - self.last_debug_ui_count) / elapsed
        sample_rate = (self.sample_count - self.last_debug_sample_count) / elapsed
        draw_rate = (self.draw_count - self.last_debug_draw_count) / elapsed
        queue_size = self.rx_queue.qsize()
        last_age = (now - self.last_rx_time) if self.last_rx_time > 0.0 else -1.0
        last_text = f"{last_age:.1f}s" if last_age >= 0.0 else "-"

        debug_text = (
            f"RX {rx_rate:.0f}/s  UI {ui_rate:.0f}/s  S {sample_rate:.0f}/s  "
            f"Q {queue_size}  Draw {draw_rate:.0f}/s  Last {last_text}"
        )
        self.debug_var.set(debug_text)
        if self.debug_log_enabled_var.get():
            self._queue_log(f"[DBG] {debug_text}")
            self._flush_log()

        self.last_debug_time = now
        self.last_debug_rx_count = self.rx_line_count
        self.last_debug_ui_count = self.ui_line_count
        self.last_debug_sample_count = self.sample_count
        self.last_debug_draw_count = self.draw_count

    def _queue_log(self, line):
        timestamp = time.strftime("%H:%M:%S")
        self.pending_log_lines.append(f"{timestamp}  {line}\n")

    def _should_log_line(self, line):
        if not line.startswith("ATT "):
            return True

        now = time.time()
        if (now - self.last_attitude_log_time) >= 1.0:
            self.last_attitude_log_time = now
            return True
        return False

    def _flush_log(self):
        line_count = len(self.pending_log_lines)
        self.log_text.insert(tk.END, "".join(self.pending_log_lines))
        self.pending_log_lines.clear()
        self.log_line_count += line_count
        if self.log_line_count > self.max_log_lines:
            delete_count = self.log_line_count - self.max_log_lines
            self.log_text.delete("1.0", f"{delete_count + 1}.0")
            self.log_line_count = self.max_log_lines
        self.log_text.see(tk.END)

    def _parse_sample(self, line):
        pairs = self._parse_key_values(line)
        if not pairs:
            return

        for key, label in self.value_labels.items():
            if key in pairs:
                label.config(text=f"{pairs[key]:.3g}")

        if all(key in pairs for key in ("PITCH", "ROLL", "YAW")):
            self._update_attitude(pairs)
            self._draw_cube()

        if not (("LD" in pairs) or ("RD" in pairs)):
            return None

        pairs["TIME"] = time.time()
        self.last_sample_time = pairs["TIME"]
        return pairs

    def _update_attitude(self, pairs):
        self.raw_attitude["PITCH"] = pairs["PITCH"]
        self.raw_attitude["ROLL"] = pairs["ROLL"]
        self.raw_attitude["YAW"] = pairs["YAW"]

        if self.attitude_zero is None:
            self.attitude_zero = dict(self.raw_attitude)

        self.attitude["PITCH"] = self._wrap_angle(
            self.raw_attitude["PITCH"] - self.attitude_zero["PITCH"]
        )
        self.attitude["ROLL"] = self._wrap_angle(
            self.raw_attitude["ROLL"] - self.attitude_zero["ROLL"]
        )
        self.attitude["YAW"] = self._wrap_angle(
            self.raw_attitude["YAW"] - self.attitude_zero["YAW"]
        )

    def _wrap_angle(self, value):
        while value > 180.0:
            value -= 360.0
        while value < -180.0:
            value += 360.0
        return value

    def _append_sample(self, pairs):
        self.samples.append(pairs)
        self.sample_count += 1
        if len(self.samples) > self.max_samples:
            self.samples = self.samples[-self.max_samples :]

    def _request_draw_plot(self):
        now = time.time()
        if (now - self.last_draw_time) >= self.draw_interval_s:
            self.last_draw_time = now
            self._draw_plot()
            return

        if not self.draw_pending:
            self.draw_pending = True
            delay_ms = int((self.draw_interval_s - (now - self.last_draw_time)) * 1000)
            self.root.after(max(delay_ms, 1), self._draw_plot_if_pending)

    def _draw_plot_if_pending(self):
        if not self.draw_pending:
            return

        self.draw_pending = False
        self.last_draw_time = time.time()
        self._draw_plot()

    def _parse_key_values(self, line):
        pairs = {}
        for key, value in KEY_VALUE_RE.findall(line):
            key = key.upper()
            number = float(value)
            if key in SCALE_100_KEYS:
                number /= 100.0
            elif key in SCALE_1000_KEYS:
                number /= 1000.0
            pairs[key] = number

        if "BASE" in pairs:
            pairs["TARGET"] = pairs["BASE"]
        if ("LT" in pairs) and ("RT" in pairs):
            pairs["TARGET"] = (pairs["LT"] + pairs["RT"]) * 0.5

        return pairs

    def _draw_plot(self):
        self.draw_count += 1
        self.canvas.delete("all")
        width = max(self.canvas.winfo_width(), 2)
        height = max(self.canvas.winfo_height(), 2)
        pad = 32

        series = {
            "TARGET": "#d0d7de",
            "LT": "#8ecae6",
            "RT": "#ffafcc",
            "LD": "#4cc9f0",
            "RD": "#f72585",
            "ERR": "#f9c74f",
        }
        values = []
        for sample in self.samples:
            for key in series:
                if key in sample:
                    values.append(sample[key])

        if not values:
            self.canvas.create_text(
                width // 2, height // 2, fill="#d0d7de", text="Waiting for serial data"
            )
            return

        min_v = min(values)
        max_v = max(values)
        if (max_v - min_v) < self.plot_min_span:
            center = (max_v + min_v) * 0.5
            min_v = center - (self.plot_min_span * 0.5)
            max_v = center + (self.plot_min_span * 0.5)

        self.canvas.create_line(pad, height - pad, width - pad, height - pad, fill="#35424d")
        self.canvas.create_line(pad, pad, pad, height - pad, fill="#35424d")

        for idx, (key, color) in enumerate(series.items()):
            points = []
            keyed_samples = [sample for sample in self.samples if key in sample]
            if len(keyed_samples) < 2:
                continue
            for i, sample in enumerate(keyed_samples):
                x = pad + (width - 2 * pad) * i / (len(keyed_samples) - 1)
                y = height - pad - (height - 2 * pad) * (sample[key] - min_v) / (max_v - min_v)
                points.extend((x, y))
            self.canvas.create_line(points, fill=color, width=2)
            self.canvas.create_text(
                pad + 50 * idx, 14, anchor=tk.W, fill=color, text=key
            )

    def _draw_cube(self):
        self.cube_draw_count += 1
        canvas = self.cube_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 2)
        height = max(canvas.winfo_height(), 2)
        cx = width * 0.5
        cy = height * 0.52
        size = min(width, height) * 0.34
        distance = 4.0

        pitch = math.radians(self.attitude["PITCH"] * CUBE_PITCH_SIGN)
        roll = math.radians(self.attitude["ROLL"] * CUBE_ROLL_SIGN)
        yaw = math.radians(self.attitude["YAW"] * CUBE_YAW_SIGN)

        corners = [
            (-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
            (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1),
        ]
        edges = (
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        )
        faces = (
            ((0, 1, 2, 3), "#1d4ed8"),
            ((4, 5, 6, 7), "#0f766e"),
            ((0, 1, 5, 4), "#7c3aed"),
            ((2, 3, 7, 6), "#b45309"),
            ((1, 2, 6, 5), "#be123c"),
            ((0, 3, 7, 4), "#047857"),
        )

        projected = []
        rotated = []
        for x, y, z in corners:
            x1 = x
            y1 = (y * math.cos(pitch)) - (z * math.sin(pitch))
            z1 = (y * math.sin(pitch)) + (z * math.cos(pitch))

            x2 = (x1 * math.cos(yaw)) + (z1 * math.sin(yaw))
            y2 = y1
            z2 = (-x1 * math.sin(yaw)) + (z1 * math.cos(yaw))

            x3 = (x2 * math.cos(roll)) - (y2 * math.sin(roll))
            y3 = (x2 * math.sin(roll)) + (y2 * math.cos(roll))
            z3 = z2
            rotated.append((x3, y3, z3))

            perspective = distance / (distance - z3)
            projected.append((cx + x3 * size * perspective, cy - y3 * size * perspective))

        face_depths = []
        for indices, color in faces:
            depth = sum(rotated[index][2] for index in indices) / len(indices)
            face_depths.append((depth, indices, color))

        for _, indices, color in sorted(face_depths):
            points = []
            for index in indices:
                points.extend(projected[index])
            canvas.create_polygon(points, fill=color, outline="", stipple="gray50")

        for start, end in edges:
            canvas.create_line(
                projected[start][0], projected[start][1],
                projected[end][0], projected[end][1],
                fill="#e5e7eb", width=2,
            )

        canvas.create_text(
            12, 14, anchor=tk.W, fill="#e5e7eb",
            text=(
                f"PITCH {self.attitude['PITCH']:.1f}  "
                f"ROLL {self.attitude['ROLL']:.1f}  "
                f"YAW {self.attitude['YAW']:.1f}"
            ),
        )

    def send_pid(self):
        self.send_line(f"PID {self.kp_var.get()} {self.ki_var.get()} {self.kd_var.get()}")

    def send_base(self):
        self.send_line(f"BASE {self.base_var.get()}")

    def choose_image(self):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Connect to a COM port first.")
            return
        if self.image_transfer is not None:
            messagebox.showinfo("Busy", "An image transfer is already running.")
            return

        path = filedialog.askopenfilename(
            title="Choose image",
            filetypes=(
                ("Images", "*.png;*.jpg;*.jpeg;*.bmp"),
                ("Raw RGB565", "*.bin"),
                ("All files", "*.*"),
            ),
        )
        if not path:
            return

        try:
            slot_text = self.image_slot_var.get().strip().lower()
            slot = -1 if slot_text in ("", "auto") else int(slot_text)
            if slot < -1:
                raise ValueError("image id must be auto or non-negative")
            if path.lower().endswith(".bin"):
                width, height, payload = load_raw_rgb565(path, 320, 170)
            else:
                width, height, payload = image_to_rgb565(path, 320, 170)
        except Exception as exc:
            messagebox.showerror("Image conversion failed", str(exc))
            return

        crc = binascii.crc32(payload) & 0xFFFFFFFF
        size_kb = len(payload) / 1024.0
        confirmed = messagebox.askokcancel(
            "Send image",
            (
                f"File: {path}\n"
                f"Image ID: {'auto' if slot < 0 else slot}\n"
                f"Size: {width} x {height}\n"
                f"RGB565 bytes: {len(payload)} ({size_kb:.1f} KB)\n"
                f"CRC32: {crc:08X}\n\n"
                "Send to W25Q64 now?"
            ),
        )
        if not confirmed:
            return

        self.image_transfer = {
            "slot": slot,
            "name": self._image_name_from_path(path),
            "width": width,
            "height": height,
            "payload": payload,
            "crc": crc,
            "writing": False,
            "sent": 0,
            "acked": 0,
            "show_after_send": self.image_show_after_send_var.get(),
        }
        self.image_page_ack.clear()
        self.image_list_active = False
        self.status_var.set("Preparing image send")
        if slot >= 0:
            self.send_line(f"IMG_WRITE {slot} {width} {height} {len(payload)} {crc:08X}")
        else:
            self.send_line(
                f"IMG_SAVE {self.image_transfer['name']} {width} {height} "
                f"{len(payload)} {crc:08X}"
            )

    def _handle_image_transfer_line(self, line):
        if self._handle_image_manager_line(line):
            return

        if self.image_transfer is None:
            return

        if line.startswith("OK IMG_READY"):
            if not self.image_transfer["writing"]:
                self.image_transfer["writing"] = True
                self.image_writer_thread = threading.Thread(
                    target=self._write_image_payload, daemon=True
                )
                self.image_writer_thread.start()
            return

        if line.startswith("OK IMG_PAGE "):
            try:
                self.image_transfer["acked"] = int(line.split()[2])
                self.image_page_ack.set()
            except (IndexError, ValueError):
                pass
            return

        if line.startswith("OK IMG_DONE"):
            slot = self.image_transfer["slot"]
            show_after_send = self.image_transfer["show_after_send"]
            self.image_transfer = None
            self.status_var.set("Image sent")
            if show_after_send:
                image_id = self._image_id_from_done_line(line, slot)
                self.send_line(f"IMG_SHOW {image_id}")
            if self.image_manager:
                self.request_image_list()
            messagebox.showinfo("Image sent", "Image written successfully.")
            return

        if line.startswith("ERR IMG") or line.startswith("ERR FLASH"):
            self.image_transfer = None
            self.status_var.set("Image send failed")
            messagebox.showerror("Image send failed", line)

    def open_image_manager(self):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Connect to a COM port first.")
            return

        if self.image_manager and self.image_manager["window"].winfo_exists():
            self.image_manager["window"].lift()
            self.request_image_list()
            return

        window = tk.Toplevel(self.root)
        window.title("Image File Manager")
        window.geometry("760x420")

        toolbar = ttk.Frame(window, padding=8)
        toolbar.pack(fill=tk.X)
        ttk.Button(toolbar, text="Refresh", command=self.request_image_list).pack(
            side=tk.LEFT, padx=(0, 6)
        )
        ttk.Button(toolbar, text="Upload", command=self.choose_image).pack(
            side=tk.LEFT, padx=6
        )
        ttk.Button(toolbar, text="Show", command=self.show_selected_image).pack(
            side=tk.LEFT, padx=6
        )
        ttk.Button(toolbar, text="Delete", command=self.delete_selected_image).pack(
            side=tk.LEFT, padx=6
        )
        info_var = tk.StringVar(value="No storage info yet")
        ttk.Label(toolbar, textvariable=info_var).pack(side=tk.RIGHT)

        columns = ("id", "name", "size", "resolution", "crc", "address")
        tree = ttk.Treeview(window, columns=columns, show="headings", selectmode="browse")
        tree.heading("id", text="ID")
        tree.heading("name", text="Name")
        tree.heading("size", text="Size")
        tree.heading("resolution", text="Resolution")
        tree.heading("crc", text="CRC32")
        tree.heading("address", text="Address")
        tree.column("id", width=54, anchor=tk.CENTER)
        tree.column("name", width=160)
        tree.column("size", width=96, anchor=tk.E)
        tree.column("resolution", width=110, anchor=tk.CENTER)
        tree.column("crc", width=110, anchor=tk.CENTER)
        tree.column("address", width=90, anchor=tk.CENTER)
        tree.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))
        tree.bind("<Double-1>", lambda _event: self.show_selected_image())

        status_var = tk.StringVar(value="Ready")
        ttk.Label(window, textvariable=status_var, padding=(8, 0, 8, 8)).pack(
            fill=tk.X
        )

        self.image_manager = {
            "window": window,
            "tree": tree,
            "status": status_var,
            "info": info_var,
        }
        window.protocol("WM_DELETE_WINDOW", self.close_image_manager)
        self.request_image_list()

    def close_image_manager(self):
        if self.image_manager and self.image_manager["window"].winfo_exists():
            self.image_manager["window"].destroy()
        self.image_manager = None
        self.image_list_active = False

    def request_image_list(self):
        if not self.image_manager:
            return
        self.image_list_rows = []
        self.image_list_active = True
        self.image_manager["status"].set("Refreshing image list...")
        self.send_line("IMG_INFO")
        self.send_line("IMG_LIST")

    def show_selected_image(self):
        image_id = self._selected_image_id()
        if image_id is None:
            messagebox.showinfo("No selection", "Choose an image first.")
            return
        self.send_line(f"IMG_SHOW {image_id}")
        if self.image_manager:
            self.image_manager["status"].set(f"Showing image {image_id}")

    def delete_selected_image(self):
        image_id = self._selected_image_id()
        if image_id is None:
            messagebox.showinfo("No selection", "Choose an image first.")
            return
        if not messagebox.askokcancel("Delete image", f"Delete image {image_id}?"):
            return
        self.send_line(f"IMG_DELETE {image_id}")
        if self.image_manager:
            self.image_manager["status"].set(f"Deleting image {image_id}")

    def _selected_image_id(self):
        if not self.image_manager:
            return None
        tree = self.image_manager["tree"]
        selection = tree.selection()
        if not selection:
            return None
        values = tree.item(selection[0], "values")
        if not values:
            return None
        try:
            return int(values[0])
        except ValueError:
            return None

    def _handle_image_manager_line(self, line):
        if line.startswith("IMG_INFO "):
            self.image_info_text = self._format_image_info(line)
            if self.image_manager:
                self.image_manager["info"].set(self.image_info_text)
            return True

        if line.startswith("IMG_LIST_BEGIN"):
            self.image_list_active = True
            self.image_list_rows = []
            return True

        if line.startswith("IMG_FILE "):
            row = self._parse_image_file_line(line)
            if row is not None:
                self.image_list_rows.append(row)
            return True

        if line.startswith("IMG_LIST_END"):
            self.image_list_active = False
            self._update_image_manager_rows()
            return True

        if line.startswith("OK IMG_DELETE"):
            if self.image_manager and self.image_transfer is None:
                self.image_manager["status"].set(line)
                self.request_image_list()
            return True

        if line.startswith("OK IMG_SHOW"):
            if self.image_manager:
                self.image_manager["status"].set("Image shown on LCD")
            return False

        if line.startswith("ERR IMG") and self.image_manager:
            self.image_manager["status"].set(line)
            return False

        return self.image_list_active

    def _parse_image_file_line(self, line):
        values = {}
        for part in line.split()[1:]:
            if "=" not in part:
                continue
            key, value = part.split("=", 1)
            values[key] = value
        try:
            size = int(values.get("SIZE", "0"))
            width = int(values.get("W", "0"))
            height = int(values.get("H", "0"))
            return {
                "id": int(values["ID"]),
                "name": values.get("NAME", ""),
                "size": size,
                "resolution": f"{width} x {height}",
                "crc": values.get("CRC", ""),
                "address": values.get("ADDR", ""),
            }
        except (KeyError, ValueError):
            return None

    def _update_image_manager_rows(self):
        if not self.image_manager:
            return
        tree = self.image_manager["tree"]
        for item in tree.get_children():
            tree.delete(item)
        for row in sorted(self.image_list_rows, key=lambda item: item["id"]):
            tree.insert(
                "",
                tk.END,
                values=(
                    row["id"],
                    row["name"],
                    self._format_bytes(row["size"]),
                    row["resolution"],
                    row["crc"],
                    row["address"],
                ),
            )
        self.image_manager["status"].set(f"{len(self.image_list_rows)} image(s)")

    def _format_image_info(self, line):
        values = {}
        for part in line.split()[1:]:
            if "=" in part:
                key, value = part.split("=", 1)
                values[key] = value
        try:
            used = int(values.get("USED", "0"))
            free = int(values.get("FREE", "0"))
            total = int(values.get("TOTAL", "0"))
            count = int(values.get("COUNT", "0"))
            return (
                f"{count} files  "
                f"{self._format_bytes(used)} used  "
                f"{self._format_bytes(free)} free  "
                f"{self._format_bytes(total)} total"
            )
        except ValueError:
            return line

    def _format_bytes(self, value):
        if value >= 1024 * 1024:
            return f"{value / (1024 * 1024):.2f} MB"
        if value >= 1024:
            return f"{value / 1024:.1f} KB"
        return f"{value} B"

    def _write_image_payload(self):
        transfer = self.image_transfer
        if transfer is None:
            return

        payload = transfer["payload"]
        chunk_size = 256
        page_timeout_s = 3.0

        try:
            sent = 0
            while sent < len(payload):
                if self.stop_event.is_set():
                    return
                chunk = payload[sent : sent + chunk_size]
                self.image_page_ack.clear()
                self.serial_port.write(chunk)
                sent += len(chunk)
                transfer["sent"] = sent
                if not self.image_page_ack.wait(page_timeout_s):
                    raise TimeoutError(f"timeout waiting for IMG_PAGE {sent}")
                if transfer.get("acked", 0) != sent:
                    raise RuntimeError(
                        f"unexpected IMG_PAGE ack {transfer.get('acked')} expected {sent}"
                    )
                percent = sent * 100.0 / len(payload)
                self.rx_queue.put(("image_status", f"[IMAGE] {sent}/{len(payload)} {percent:.1f}%"))
            self.rx_queue.put(("image_status", "[IMAGE] payload sent, waiting for MCU CRC"))
        except Exception as exc:
            self.rx_queue.put(("image_error", str(exc)))

    def _image_name_from_path(self, path):
        stem = pathlib.Path(path).stem[:15]
        clean = "".join(ch if ch.isalnum() or ch in ("_", "-", ".") else "_" for ch in stem)
        return clean or "image"

    def _image_id_from_done_line(self, line, fallback):
        match = re.search(r"ID=(\d+)", line)
        if match:
            return int(match.group(1))
        return fallback

    def send_manual(self):
        command = self.manual_cmd_var.get().strip()
        if command:
            self.send_line(command)
            self.manual_cmd_var.set("")

    def send_line(self, line):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Connect to a COM port first.")
            return

        try:
            self.serial_port.write((line.strip() + "\n").encode("utf-8"))
            self._queue_log(f"> {line.strip()}")
            self._flush_log()
        except Exception as exc:
            messagebox.showerror("Send failed", str(exc))

    def save_csv(self):
        if not self.samples:
            messagebox.showinfo("No data", "No parsed samples to save.")
            return

        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*")),
        )
        if not path:
            return

        keys = ["TIME"] + list(PLOT_KEYS)
        with open(path, "w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=keys, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(self.samples)

    def clear_data(self):
        self.samples.clear()
        self.log_text.delete("1.0", tk.END)
        self.pending_log_lines.clear()
        self.log_line_count = 0
        self.draw_pending = False
        for label in self.value_labels.values():
            label.config(text="-")
        self.attitude = {"PITCH": 0.0, "ROLL": 0.0, "YAW": 0.0}
        self.raw_attitude = {"PITCH": 0.0, "ROLL": 0.0, "YAW": 0.0}
        self.attitude_zero = None
        self.last_attitude_log_time = 0.0
        self._draw_plot()
        self._draw_cube()

    def zero_attitude(self):
        self.attitude_zero = dict(self.raw_attitude)
        self.attitude = {"PITCH": 0.0, "ROLL": 0.0, "YAW": 0.0}
        self._draw_cube()
        if self.serial_port and self.serial_port.is_open:
            self.send_line("MPUZERO")


def main():
    root = tk.Tk()
    app = SerialTunerApp(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.disconnect(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
