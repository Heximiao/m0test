import argparse
import datetime
import sys
import threading
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("缺少 pyserial，请先运行：python -m pip install pyserial", file=sys.stderr)
    raise SystemExit(1)


def timestamp():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def print_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("未发现串口设备")
        return
    for port in ports:
        print(f"{port.device:8s}  {port.description}  {port.hwid}")


class SerialTester:
    def __init__(self, port, baudrate, show_hex):
        self.serial = serial.Serial(port, baudrate, timeout=0.05)
        self.show_hex = show_hex
        self.running = True
        self.buffer = bytearray()
        self.reader = threading.Thread(target=self.read_loop, daemon=True)

    def start(self):
        self.reader.start()
        print(
            f"[{timestamp()}] 已打开 {self.serial.port} @ {self.serial.baudrate}，"
            "按 Ctrl-C 退出"
        )

    def close(self):
        self.running = False
        try:
            if self.serial.is_open:
                self.send("STOP", quiet=True)
                time.sleep(0.05)
                self.serial.close()
        finally:
            self.reader.join(timeout=0.2)

    def send(self, command, quiet=False):
        data = (command.strip() + "\n").encode("ascii")
        self.serial.write(data)
        self.serial.flush()
        if not quiet:
            print(f"[{timestamp()}] TX {data.decode('ascii').rstrip()}")

    def read_loop(self):
        while self.running:
            try:
                data = self.serial.read(self.serial.in_waiting or 1)
            except serial.SerialException as exc:
                print(f"\n[{timestamp()}] 串口读取失败：{exc}")
                self.running = False
                return
            if not data:
                continue
            if self.show_hex:
                print(f"[{timestamp()}] RX HEX {data.hex(' ')}")
            self.buffer.extend(data)
            while b"\n" in self.buffer:
                raw_line, _, remainder = self.buffer.partition(b"\n")
                self.buffer = bytearray(remainder)
                text = raw_line.rstrip(b"\r").decode("utf-8", errors="replace")
                print(f"[{timestamp()}] RX {text}")


def listen(test, duration):
    deadline = None if duration <= 0 else time.monotonic() + duration
    while test.running and (deadline is None or time.monotonic() < deadline):
        time.sleep(0.05)


def drive_test(test, linear_mm_s, angular_deg_s, duration, rate):
    deadline = time.monotonic() + duration
    period = 1.0 / rate
    print(
        f"即将发送 NAV {linear_mm_s:g} {angular_deg_s:g}，持续 {duration:g} 秒；"
        "车轮必须架空。"
    )
    input("确认安全后按回车开始，Ctrl-C 取消：")
    try:
        while test.running and time.monotonic() < deadline:
            started = time.monotonic()
            test.send(f"NAV {linear_mm_s:.3f} {angular_deg_s:.3f}")
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    finally:
        test.send("STOP")
        print("测试结束，已发送 STOP")


def interactive(test):
    print("可输入 NAV 80 0、WHEELS 10 10、STOP；直接回车不发送。")
    while test.running:
        command = input("> ").strip()
        if command:
            test.send(command)


def parse_args():
    parser = argparse.ArgumentParser(description="树莓派/MCU 导航 UART 诊断工具")
    parser.add_argument("--list", action="store_true", help="列出电脑串口")
    parser.add_argument("--port", help="USB-TTL 串口，例如 COM5")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--hex", action="store_true", help="同时显示十六进制数据")
    subparsers = parser.add_subparsers(dest="mode")

    listen_parser = subparsers.add_parser("listen", help="只监听串口输出")
    listen_parser.add_argument("--duration", type=float, default=0.0)

    drive_parser = subparsers.add_parser("drive", help="周期发送 NAV 测试")
    drive_parser.add_argument("--linear", type=float, default=80.0)
    drive_parser.add_argument("--angular", type=float, default=0.0)
    drive_parser.add_argument("--duration", type=float, default=2.0)
    drive_parser.add_argument("--rate", type=float, default=10.0)

    subparsers.add_parser("interactive", help="交互发送任意命令")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.list:
        print_ports()
        return
    if not args.port or not args.mode:
        print("请指定 --port 和模式；使用 --help 查看示例。", file=sys.stderr)
        raise SystemExit(2)

    tester = SerialTester(args.port, args.baudrate, args.hex)
    tester.start()
    try:
        if args.mode == "listen":
            listen(tester, args.duration)
        elif args.mode == "drive":
            drive_test(tester, args.linear, args.angular, args.duration, args.rate)
        else:
            interactive(tester)
    except KeyboardInterrupt:
        print("\n用户终止")
    finally:
        tester.close()
        print("串口已关闭，已尝试发送 STOP")


if __name__ == "__main__":
    main()
