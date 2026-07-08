import csv
import binascii
import json
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
SOURCE_RE = re.compile(r"\bSRC\s*=\s*([A-Za-z0-9_]+)")
SCALE_100_KEYS = {
    "BASE", "LT", "RT", "LD", "RD", "ERR", "OUT", "LO", "RO", "ML", "MR",
    "SPEED", "TARGET", "PITCH", "ROLL", "YAW", "RAWP", "RAWR", "RAWY",
}
SCALE_1000_KEYS = {"KP", "KI", "KD"}
CUBE_PITCH_SIGN = -1.0
CUBE_ROLL_SIGN = -1.0
CUBE_YAW_SIGN = 1.0
SETTINGS_PATH = pathlib.Path(__file__).with_name("serial_tuner_settings.json")
SENSOR_AXIS_OPTIONS = ("+X", "-X", "+Y", "-Y", "+Z", "-Z")
SENSOR_AXIS_VECTORS = {
    "+X": (1.0, 0.0, 0.0),
    "-X": (-1.0, 0.0, 0.0),
    "+Y": (0.0, 1.0, 0.0),
    "-Y": (0.0, -1.0, 0.0),
    "+Z": (0.0, 0.0, 1.0),
    "-Z": (0.0, 0.0, -1.0),
}


class SerialTunerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("小车串口调参上位机")
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
        self.mounted_attitude = {"PITCH": 0.0, "ROLL": 0.0, "YAW": 0.0}
        self.raw_attitude_matrix = self._identity_matrix()
        self.mounted_attitude_matrix = self._identity_matrix()
        self.attitude_matrix = self._identity_matrix()
        self.attitude_zero_matrix = None
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
        self.status_var = tk.StringVar(value="未连接")
        self.debug_var = tk.StringVar(value="RX 0/s  UI 0/s  Q 0  Draw 0/s  Last -")
        self.debug_log_enabled_var = tk.BooleanVar(value=False)
        self.page_var = tk.StringVar(value="monitor")
        self.mount_forward_var = tk.StringVar(value="-X")
        self.mount_up_var = tk.StringVar(value="+Y")
        self.mount_status_var = tk.StringVar(value="")
        self.raw_attitude_var = tk.StringVar(value="Raw: P 0.0  R 0.0  Y 0.0")
        self.mounted_attitude_var = tk.StringVar(value="Body: P 0.0  R 0.0  Y 0.0")
        self.zeroed_attitude_var = tk.StringVar(value="Zeroed: P 0.0  R 0.0  Y 0.0")
        self.nav_buttons = {}
        self.monitor_cube_canvas = None
        self.cal_cube_canvas = None

        self._load_settings()
        self._build_ui()
        self._update_mount_status()
        self.refresh_ports()
        self.root.after(100, self._draw_cube)
        self.root.after(50, self._poll_rx_queue)

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill=tk.X)

        ttk.Label(top, text="端口").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=18)
        self.port_combo.pack(side=tk.LEFT, padx=(4, 10))
        ttk.Button(top, text="刷新", command=self.refresh_ports).pack(side=tk.LEFT)

        ttk.Label(top, text="波特率").pack(side=tk.LEFT, padx=(14, 0))
        ttk.Combobox(top, textvariable=self.baud_var, values=BAUD_RATES, width=10).pack(
            side=tk.LEFT, padx=(4, 10)
        )

        self.connect_button = ttk.Button(top, text="连接", command=self.toggle_connection)
        self.connect_button.pack(side=tk.LEFT)
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.LEFT, padx=14)
        ttk.Checkbutton(
            top, text="调试日志", variable=self.debug_log_enabled_var
        ).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Label(top, textvariable=self.debug_var).pack(side=tk.LEFT, padx=14)

        body = ttk.Frame(self.root)
        body.pack(fill=tk.BOTH, expand=True)

        sidebar = tk.Frame(body, width=176, bg="#2f3133")
        sidebar.pack(side=tk.LEFT, fill=tk.Y)
        sidebar.pack_propagate(False)

        tk.Label(
            sidebar,
            text="小车地面站",
            bg="#373a3c",
            fg="#f2f5f7",
            anchor=tk.W,
            padx=14,
            pady=14,
            font=("Segoe UI", 12),
        ).pack(fill=tk.X)
        self._add_nav_button(sidebar, "monitor", "PID 监视")
        self._add_nav_button(sidebar, "accel", "姿态校准")

        self.content_frame = ttk.Frame(body)
        self.content_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.monitor_page = ttk.Frame(self.content_frame)
        self.accel_page = ttk.Frame(self.content_frame)

        self._build_monitor_page(self.monitor_page)
        self._build_accel_page(self.accel_page)
        self._show_page("monitor")

    def _add_nav_button(self, parent, page, text):
        button = tk.Button(
            parent,
            text=text,
            anchor=tk.W,
            padx=18,
            pady=12,
            bd=0,
            relief=tk.FLAT,
            bg="#2f3133",
            fg="#c7c9cc",
            activebackground="#3d4144",
            activeforeground="#ffffff",
            command=lambda: self._show_page(page),
        )
        button.pack(fill=tk.X)
        self.nav_buttons[page] = button

    def _show_page(self, page):
        self.page_var.set(page)
        for frame in (self.monitor_page, self.accel_page):
            frame.pack_forget()
        if page == "accel":
            self.accel_page.pack(fill=tk.BOTH, expand=True)
        else:
            self.monitor_page.pack(fill=tk.BOTH, expand=True)
        for name, button in self.nav_buttons.items():
            if name == page:
                button.config(bg="#454a4e", fg="#ffffff")
            else:
                button.config(bg="#2f3133", fg="#c7c9cc")
        self._draw_cube()

    def _build_monitor_page(self, parent):
        controls = ttk.LabelFrame(parent, text="PID 指令", padding=8)
        controls.pack(fill=tk.X, padx=8, pady=(8, 8))

        for label, var in (
            ("Base", self.base_var),
            ("Kp", self.kp_var),
            ("Ki", self.ki_var),
            ("Kd", self.kd_var),
        ):
            ttk.Label(controls, text=label).pack(side=tk.LEFT)
            ttk.Entry(controls, textvariable=var, width=8).pack(side=tk.LEFT, padx=(4, 12))

        ttk.Button(controls, text="发送 PID", command=self.send_pid).pack(side=tk.LEFT, padx=4)
        ttk.Button(controls, text="发送基础速度", command=self.send_base).pack(side=tk.LEFT, padx=4)
        ttk.Button(controls, text="读取状态", command=lambda: self.send_line("GET")).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Label(controls, text="图片 ID").pack(side=tk.LEFT, padx=(16, 0))
        ttk.Entry(controls, textvariable=self.image_slot_var, width=4).pack(
            side=tk.LEFT, padx=(4, 4)
        )
        ttk.Button(controls, text="发送图片", command=self.choose_image).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(controls, text="图片文件", command=self.open_image_manager).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Checkbutton(
            controls, text="发送后显示", variable=self.image_show_after_send_var
        ).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(controls, text="保存 CSV", command=self.save_csv).pack(side=tk.RIGHT, padx=4)
        ttk.Button(controls, text="清空", command=self.clear_data).pack(side=tk.RIGHT, padx=4)

        middle = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        middle.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        left = ttk.Frame(middle)
        right = ttk.Frame(middle)
        middle.add(left, weight=3)
        middle.add(right, weight=2)

        self.canvas = tk.Canvas(left, height=320, bg="#101418", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        table_frame = ttk.LabelFrame(left, text="最新数据", padding=6)
        table_frame.pack(fill=tk.X, pady=(8, 0))
        self.value_labels = {}
        for key in PLOT_KEYS:
            box = ttk.Frame(table_frame)
            box.pack(side=tk.LEFT, padx=8)
            ttk.Label(box, text=key).pack()
            value = ttk.Label(box, text="-", width=8, anchor=tk.CENTER)
            value.pack()
            self.value_labels[key] = value

        cube_frame = ttk.LabelFrame(right, text="JY61 车体姿态", padding=6)
        cube_frame.pack(fill=tk.X, pady=(0, 8))
        ttk.Button(cube_frame, text="姿态置零", command=self.zero_attitude).pack(
            anchor=tk.E, pady=(0, 4)
        )
        self.monitor_cube_canvas = tk.Canvas(
            cube_frame, height=240, bg="#0b1020", highlightthickness=0
        )
        self.monitor_cube_canvas.pack(fill=tk.X, expand=False)

        log_frame = ttk.LabelFrame(right, text="串口日志", padding=6)
        log_frame.pack(fill=tk.BOTH, expand=True)
        self.log_text = tk.Text(log_frame, height=20, wrap=tk.NONE)
        self.log_text.pack(fill=tk.BOTH, expand=True)

        cmd_frame = ttk.Frame(right)
        cmd_frame.pack(fill=tk.X, pady=(8, 0))
        self.manual_cmd_var = tk.StringVar()
        ttk.Entry(cmd_frame, textvariable=self.manual_cmd_var).pack(
            side=tk.LEFT, fill=tk.X, expand=True
        )
        ttk.Button(cmd_frame, text="发送", command=self.send_manual).pack(side=tk.LEFT, padx=(8, 0))

    def _build_accel_page(self, parent):
        header = ttk.Frame(parent, padding=(18, 16, 18, 8))
        header.pack(fill=tk.X)
        ttk.Label(header, text="姿态安装方向校准", font=("Segoe UI", 16)).pack(
            side=tk.LEFT
        )
        ttk.Button(header, text="姿态置零", command=self.zero_attitude).pack(
            side=tk.RIGHT, padx=(8, 0)
        )
        ttk.Button(header, text="恢复默认安装方向", command=self.reset_mount).pack(side=tk.RIGHT)

        body = ttk.Frame(parent, padding=(18, 0, 18, 18))
        body.pack(fill=tk.BOTH, expand=True)

        controls = ttk.LabelFrame(body, text="传感器安装方向", padding=12)
        controls.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 12))

        ttk.Label(controls, text="车头对应传感器轴").pack(anchor=tk.W)
        front_combo = ttk.Combobox(
            controls,
            textvariable=self.mount_forward_var,
            values=SENSOR_AXIS_OPTIONS,
            width=8,
            state="readonly",
        )
        front_combo.pack(fill=tk.X, pady=(4, 12))
        front_combo.bind("<<ComboboxSelected>>", lambda _event: self.on_mount_changed())

        ttk.Label(controls, text="车顶对应传感器轴").pack(anchor=tk.W)
        up_combo = ttk.Combobox(
            controls,
            textvariable=self.mount_up_var,
            values=SENSOR_AXIS_OPTIONS,
            width=8,
            state="readonly",
        )
        up_combo.pack(fill=tk.X, pady=(4, 12))
        up_combo.bind("<<ComboboxSelected>>", lambda _event: self.on_mount_changed())

        ttk.Label(controls, textvariable=self.mount_status_var, foreground="#555555").pack(
            anchor=tk.W, pady=(0, 14)
        )
        ttk.Separator(controls).pack(fill=tk.X, pady=(0, 12))
        ttk.Label(controls, textvariable=self.raw_attitude_var).pack(anchor=tk.W, pady=2)
        ttk.Label(controls, textvariable=self.mounted_attitude_var).pack(anchor=tk.W, pady=2)
        ttk.Label(controls, textvariable=self.zeroed_attitude_var).pack(anchor=tk.W, pady=2)

        preview = ttk.LabelFrame(body, text="车体姿态预览", padding=8)
        preview.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.cal_cube_canvas = tk.Canvas(preview, bg="#f8fafc", highlightthickness=0)
        self.cal_cube_canvas.pack(fill=tk.BOTH, expand=True)

    def refresh_ports(self):
        if serial is None:
            self.port_combo["values"] = []
            self.status_var.set("未安装 pyserial")
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
                "缺少依赖",
                "请先安装 pyserial: python -m pip install pyserial",
            )
            return

        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("未选择端口", "请先选择 COM 端口。")
            return

        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=int(self.baud_var.get()),
                timeout=0.1,
                write_timeout=0.5,
            )
        except Exception as exc:
            messagebox.showerror("打开失败", str(exc))
            return

        self.stop_event.clear()
        self.reader_thread = threading.Thread(target=self._read_serial, daemon=True)
        self.reader_thread.start()
        self.connect_button.config(text="断开")
        self.status_var.set(f"已连接 {port}")

    def disconnect(self):
        self.stop_event.set()
        if self.serial_port:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.connect_button.config(text="连接")
        self.status_var.set("未连接")

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
                self.status_var.set("图片发送失败")
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
            self._update_attitude(pairs, self._parse_source(line))
            self._draw_cube()

        if not (("LD" in pairs) or ("RD" in pairs)):
            return None

        pairs["TIME"] = time.time()
        self.last_sample_time = pairs["TIME"]
        return pairs

    def _update_attitude(self, pairs, source):
        if source == "JY61BODY":
            self.raw_attitude["PITCH"] = pairs.get("RAWP", pairs["PITCH"])
            self.raw_attitude["ROLL"] = pairs.get("RAWR", pairs["ROLL"])
            self.raw_attitude["YAW"] = pairs.get("RAWY", pairs["YAW"])
            self.raw_attitude_matrix = self._euler_to_matrix(
                self.raw_attitude["PITCH"],
                self.raw_attitude["ROLL"],
                self.raw_attitude["YAW"],
            )
            self.mounted_attitude["PITCH"] = pairs["PITCH"]
            self.mounted_attitude["ROLL"] = pairs["ROLL"]
            self.mounted_attitude["YAW"] = pairs["YAW"]
            self.mounted_attitude_matrix = self._euler_to_matrix(
                self.mounted_attitude["PITCH"],
                self.mounted_attitude["ROLL"],
                self.mounted_attitude["YAW"],
            )
            self.attitude_matrix = self.mounted_attitude_matrix
            self.attitude = dict(self.mounted_attitude)
            self.attitude_zero_matrix = self._identity_matrix()
            self._update_attitude_labels()
            return

        self.raw_attitude["PITCH"] = pairs["PITCH"]
        self.raw_attitude["ROLL"] = pairs["ROLL"]
        self.raw_attitude["YAW"] = pairs["YAW"]
        self.raw_attitude_matrix = self._euler_to_matrix(
            self.raw_attitude["PITCH"],
            self.raw_attitude["ROLL"],
            self.raw_attitude["YAW"],
        )
        self.mounted_attitude_matrix = self._apply_mount_to_matrix(
            self.raw_attitude_matrix
        )
        self.mounted_attitude = self._matrix_to_euler(self.mounted_attitude_matrix)

        if self.attitude_zero_matrix is None:
            self.attitude_zero_matrix = self.mounted_attitude_matrix

        self.attitude_matrix = self._mat_mul(
            self.mounted_attitude_matrix,
            self._mat_transpose(self.attitude_zero_matrix),
        )
        self.attitude = self._matrix_to_euler(self.attitude_matrix)
        self._update_attitude_labels()

    def _apply_mount_to_attitude(self, attitude):
        matrix = self._euler_to_matrix(
            attitude["PITCH"],
            attitude["ROLL"],
            attitude["YAW"],
        )
        mount = self._mount_sensor_to_body_matrix()
        if mount is None:
            return dict(attitude)
        body_matrix = self._mat_mul(mount, matrix)
        return self._matrix_to_euler(body_matrix)

    def _apply_mount_to_matrix(self, matrix):
        mount = self._mount_sensor_to_body_matrix()
        if mount is None:
            return matrix
        return self._mat_mul(mount, matrix)

    def _mount_sensor_to_body_matrix(self):
        forward = SENSOR_AXIS_VECTORS.get(self.mount_forward_var.get())
        up = SENSOR_AXIS_VECTORS.get(self.mount_up_var.get())
        if forward is None or up is None:
            return None
        if abs(self._dot(forward, up)) > 0.001:
            return None
        right = self._cross(up, forward)
        if self._norm(right) < 0.001:
            return None
        right = self._normalize(right)
        up = self._normalize(up)
        forward = self._normalize(forward)
        return (
            (right[0], right[1], right[2]),
            (up[0], up[1], up[2]),
            (forward[0], forward[1], forward[2]),
        )

    def _euler_to_matrix(self, pitch_deg, roll_deg, yaw_deg):
        pitch = math.radians(pitch_deg * CUBE_PITCH_SIGN)
        roll = math.radians(roll_deg * CUBE_ROLL_SIGN)
        yaw = math.radians(yaw_deg * CUBE_YAW_SIGN)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)
        cy, sy = math.cos(yaw), math.sin(yaw)

        pitch_m = ((1.0, 0.0, 0.0), (0.0, cp, -sp), (0.0, sp, cp))
        yaw_m = ((cy, 0.0, sy), (0.0, 1.0, 0.0), (-sy, 0.0, cy))
        roll_m = ((cr, -sr, 0.0), (sr, cr, 0.0), (0.0, 0.0, 1.0))
        return self._mat_mul(roll_m, self._mat_mul(yaw_m, pitch_m))

    def _identity_matrix(self):
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

    def _matrix_to_euler(self, matrix):
        pitch = math.asin(max(-1.0, min(1.0, matrix[2][1])))
        cp = math.cos(pitch)
        if abs(cp) > 0.0001:
            yaw = math.atan2(-matrix[2][0], matrix[2][2])
            roll = math.atan2(-matrix[0][1], matrix[1][1])
        else:
            yaw = math.atan2(matrix[0][2], matrix[0][0])
            roll = 0.0
        return {
            "PITCH": self._wrap_angle(math.degrees(pitch) / CUBE_PITCH_SIGN),
            "ROLL": self._wrap_angle(math.degrees(roll) / CUBE_ROLL_SIGN),
            "YAW": self._wrap_angle(math.degrees(yaw) / CUBE_YAW_SIGN),
        }

    def _mat_mul(self, left, right):
        return tuple(
            tuple(
                sum(left[row][idx] * right[idx][col] for idx in range(3))
                for col in range(3)
            )
            for row in range(3)
        )

    def _mat_transpose(self, matrix):
        return tuple(tuple(matrix[row][col] for row in range(3)) for col in range(3))

    def _dot(self, left, right):
        return sum(left[idx] * right[idx] for idx in range(3))

    def _cross(self, left, right):
        return (
            left[1] * right[2] - left[2] * right[1],
            left[2] * right[0] - left[0] * right[2],
            left[0] * right[1] - left[1] * right[0],
        )

    def _norm(self, vector):
        return math.sqrt(self._dot(vector, vector))

    def _normalize(self, vector):
        length = self._norm(vector)
        if length <= 0.0:
            return vector
        return tuple(value / length for value in vector)

    def _update_attitude_labels(self):
        self.raw_attitude_var.set(
            "Raw: P {PITCH:.1f}  R {ROLL:.1f}  Y {YAW:.1f}".format(**self.raw_attitude)
        )
        self.mounted_attitude_var.set(
            "Body: P {PITCH:.1f}  R {ROLL:.1f}  Y {YAW:.1f}".format(
                **self.mounted_attitude
            )
        )
        self.zeroed_attitude_var.set(
            "Zeroed: P {PITCH:.1f}  R {ROLL:.1f}  Y {YAW:.1f}".format(**self.attitude)
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

    def _parse_source(self, line):
        match = SOURCE_RE.search(line)
        if match:
            return match.group(1).upper()
        return ""

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
                width // 2, height // 2, fill="#d0d7de", text="等待串口数据"
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
        canvases = [
            canvas for canvas in (self.monitor_cube_canvas, self.cal_cube_canvas) if canvas
        ]
        for canvas in canvases:
            self._draw_attitude_cube(canvas)

    def _draw_attitude_cube(self, canvas):
        canvas.delete("all")
        width = max(canvas.winfo_width(), 2)
        height = max(canvas.winfo_height(), 2)
        cx = width * 0.5
        cy = height * 0.52
        size = min(width, height) * 0.34
        distance = 4.0

        attitude_matrix = self.attitude_matrix

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
            x3 = (
                attitude_matrix[0][0] * x +
                attitude_matrix[0][1] * y +
                attitude_matrix[0][2] * z
            )
            y3 = (
                attitude_matrix[1][0] * x +
                attitude_matrix[1][1] * y +
                attitude_matrix[1][2] * z
            )
            z3 = (
                attitude_matrix[2][0] * x +
                attitude_matrix[2][1] * y +
                attitude_matrix[2][2] * z
            )
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
            messagebox.showwarning("未连接", "请先连接 COM 端口。")
            return
        if self.image_transfer is not None:
            messagebox.showinfo("正在忙", "已有图片正在传输。")
            return

        path = filedialog.askopenfilename(
            title="选择图片",
            filetypes=(
                ("图片", "*.png;*.jpg;*.jpeg;*.bmp"),
                ("原始 RGB565", "*.bin"),
                ("所有文件", "*.*"),
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
            messagebox.showerror("图片转换失败", str(exc))
            return

        crc = binascii.crc32(payload) & 0xFFFFFFFF
        size_kb = len(payload) / 1024.0
        confirmed = messagebox.askokcancel(
            "发送图片",
            (
                f"文件: {path}\n"
                f"图片 ID: {'自动' if slot < 0 else slot}\n"
                f"尺寸: {width} x {height}\n"
                f"RGB565 字节数: {len(payload)} ({size_kb:.1f} KB)\n"
                f"CRC32: {crc:08X}\n\n"
                "现在写入 W25Q64 吗？"
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
        self.status_var.set("准备发送图片")
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
            self.status_var.set("图片已发送")
            if show_after_send:
                image_id = self._image_id_from_done_line(line, slot)
                self.send_line(f"IMG_SHOW {image_id}")
            if self.image_manager:
                self.request_image_list()
            messagebox.showinfo("图片已发送", "图片写入成功。")
            return

        if line.startswith("ERR IMG") or line.startswith("ERR FLASH"):
            self.image_transfer = None
            self.status_var.set("图片发送失败")
            messagebox.showerror("图片发送失败", line)

    def open_image_manager(self):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("未连接", "请先连接 COM 端口。")
            return

        if self.image_manager and self.image_manager["window"].winfo_exists():
            self.image_manager["window"].lift()
            self.request_image_list()
            return

        window = tk.Toplevel(self.root)
        window.title("图片文件管理")
        window.geometry("760x420")

        toolbar = ttk.Frame(window, padding=8)
        toolbar.pack(fill=tk.X)
        ttk.Button(toolbar, text="刷新", command=self.request_image_list).pack(
            side=tk.LEFT, padx=(0, 6)
        )
        ttk.Button(toolbar, text="上传", command=self.choose_image).pack(
            side=tk.LEFT, padx=6
        )
        ttk.Button(toolbar, text="显示", command=self.show_selected_image).pack(
            side=tk.LEFT, padx=6
        )
        ttk.Button(toolbar, text="删除", command=self.delete_selected_image).pack(
            side=tk.LEFT, padx=6
        )
        info_var = tk.StringVar(value="暂无存储信息")
        ttk.Label(toolbar, textvariable=info_var).pack(side=tk.RIGHT)

        columns = ("id", "name", "size", "resolution", "crc", "address")
        tree = ttk.Treeview(window, columns=columns, show="headings", selectmode="browse")
        tree.heading("id", text="ID")
        tree.heading("name", text="名称")
        tree.heading("size", text="大小")
        tree.heading("resolution", text="分辨率")
        tree.heading("crc", text="CRC32")
        tree.heading("address", text="地址")
        tree.column("id", width=54, anchor=tk.CENTER)
        tree.column("name", width=160)
        tree.column("size", width=96, anchor=tk.E)
        tree.column("resolution", width=110, anchor=tk.CENTER)
        tree.column("crc", width=110, anchor=tk.CENTER)
        tree.column("address", width=90, anchor=tk.CENTER)
        tree.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))
        tree.bind("<Double-1>", lambda _event: self.show_selected_image())

        status_var = tk.StringVar(value="就绪")
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
        self.image_manager["status"].set("正在刷新图片列表...")
        self.send_line("IMG_INFO")
        self.send_line("IMG_LIST")

    def show_selected_image(self):
        image_id = self._selected_image_id()
        if image_id is None:
            messagebox.showinfo("未选择", "请先选择一张图片。")
            return
        self.send_line(f"IMG_SHOW {image_id}")
        if self.image_manager:
            self.image_manager["status"].set(f"正在显示图片 {image_id}")

    def delete_selected_image(self):
        image_id = self._selected_image_id()
        if image_id is None:
            messagebox.showinfo("未选择", "请先选择一张图片。")
            return
        if not messagebox.askokcancel("删除图片", f"删除图片 {image_id}？"):
            return
        self.send_line(f"IMG_DELETE {image_id}")
        if self.image_manager:
            self.image_manager["status"].set(f"正在删除图片 {image_id}")

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
                self.image_manager["status"].set("图片已显示到 LCD")
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
            messagebox.showwarning("未连接", "请先连接 COM 端口。")
            return

        try:
            self.serial_port.write((line.strip() + "\n").encode("utf-8"))
            self._queue_log(f"> {line.strip()}")
            self._flush_log()
        except Exception as exc:
            messagebox.showerror("发送失败", str(exc))

    def save_csv(self):
        if not self.samples:
            messagebox.showinfo("没有数据", "暂无可保存的解析数据。")
            return

        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=(("CSV 文件", "*.csv"), ("所有文件", "*.*")),
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
        self.mounted_attitude = {"PITCH": 0.0, "ROLL": 0.0, "YAW": 0.0}
        self.raw_attitude_matrix = self._identity_matrix()
        self.mounted_attitude_matrix = self._identity_matrix()
        self.attitude_matrix = self._identity_matrix()
        self.attitude_zero_matrix = None
        self.last_attitude_log_time = 0.0
        self._update_attitude_labels()
        self._draw_plot()
        self._draw_cube()

    def zero_attitude(self):
        self.attitude_zero_matrix = self.mounted_attitude_matrix
        self.attitude = {"PITCH": 0.0, "ROLL": 0.0, "YAW": 0.0}
        self.attitude_matrix = self._identity_matrix()
        self._update_attitude_labels()
        self._draw_cube()
        if self.serial_port and self.serial_port.is_open:
            self.send_line("MPUZERO")

    def reset_mount(self):
        self.mount_forward_var.set("-X")
        self.mount_up_var.set("+Y")
        self.on_mount_changed()

    def on_mount_changed(self):
        self.mounted_attitude_matrix = self._apply_mount_to_matrix(
            self.raw_attitude_matrix
        )
        self.mounted_attitude = self._matrix_to_euler(self.mounted_attitude_matrix)
        self.attitude_zero_matrix = self.mounted_attitude_matrix
        self.attitude = {"PITCH": 0.0, "ROLL": 0.0, "YAW": 0.0}
        self.attitude_matrix = self._identity_matrix()
        self._update_mount_status()
        self._update_attitude_labels()
        self._save_settings()
        self._draw_cube()

    def _update_mount_status(self):
        if self._mount_sensor_to_body_matrix() is None:
            self.mount_status_var.set("车头轴和车顶轴不能相同。")
        else:
            self.mount_status_var.set(
                f"车头 {self.mount_forward_var.get()}  车顶 {self.mount_up_var.get()}"
            )

    def _load_settings(self):
        try:
            with open(SETTINGS_PATH, "r", encoding="utf-8") as file:
                settings = json.load(file)
        except (OSError, ValueError):
            return
        forward = settings.get("mount_forward")
        up = settings.get("mount_up")
        if forward in SENSOR_AXIS_OPTIONS:
            self.mount_forward_var.set(forward)
        if up in SENSOR_AXIS_OPTIONS:
            self.mount_up_var.set(up)

    def _save_settings(self):
        settings = {
            "mount_forward": self.mount_forward_var.get(),
            "mount_up": self.mount_up_var.get(),
        }
        try:
            with open(SETTINGS_PATH, "w", encoding="utf-8") as file:
                json.dump(settings, file, indent=2)
        except OSError:
            pass


def main():
    root = tk.Tk()
    app = SerialTunerApp(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.disconnect(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
