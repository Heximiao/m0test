import csv
import queue
import re
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


BAUD_RATES = ("9600", "19200", "38400", "57600", "115200", "230400", "460800")
PLOT_KEYS = (
    "L", "R", "TARGET", "LT", "RT", "LD", "RD", "ERR", "OUT", "LO", "RO",
    "KP", "KI", "KD",
)
KEY_VALUE_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(-?\d+(?:\.\d+)?)")
SCALE_100_KEYS = {
    "BASE", "LT", "RT", "LD", "RD", "ERR", "OUT", "LO", "RO", "ML", "MR",
    "SPEED", "TARGET",
}
SCALE_1000_KEYS = {"KP", "KI", "KD"}


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

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.base_var = tk.StringVar(value="8.0")
        self.kp_var = tk.StringVar(value="0.35")
        self.ki_var = tk.StringVar(value="0.02")
        self.kd_var = tk.StringVar(value="0.0")
        self.status_var = tk.StringVar(value="Disconnected")

        self._build_ui()
        self.refresh_ports()
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
                self.rx_queue.put(("line", line))

    def _poll_rx_queue(self):
        while True:
            try:
                kind, payload = self.rx_queue.get_nowait()
            except queue.Empty:
                break

            if kind == "line":
                self._append_log(payload)
                self._parse_sample(payload)
            else:
                self._append_log(f"[ERROR] {payload}")
                self.disconnect()

        self.root.after(50, self._poll_rx_queue)

    def _append_log(self, line):
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"{timestamp}  {line}\n")
        self.log_text.see(tk.END)

    def _parse_sample(self, line):
        pairs = self._parse_key_values(line)
        if not pairs:
            return

        for key, label in self.value_labels.items():
            if key in pairs:
                label.config(text=f"{pairs[key]:.3g}")

        if not (("LD" in pairs) or ("RD" in pairs)):
            return

        pairs["TIME"] = time.time()
        self.samples.append(pairs)
        if len(self.samples) > self.max_samples:
            self.samples = self.samples[-self.max_samples :]

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

        return pairs

    def _draw_plot(self):
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
        if abs(max_v - min_v) < 1e-6:
            max_v += 1.0
            min_v -= 1.0

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

    def send_pid(self):
        self.send_line(f"PID {self.kp_var.get()} {self.ki_var.get()} {self.kd_var.get()}")

    def send_base(self):
        self.send_line(f"BASE {self.base_var.get()}")

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
            self._append_log(f"> {line.strip()}")
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
        for label in self.value_labels.values():
            label.config(text="-")
        self._draw_plot()


def main():
    root = tk.Tk()
    app = SerialTunerApp(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.disconnect(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
