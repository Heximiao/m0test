import argparse
import binascii
import pathlib
import struct
import sys
import time

try:
    import serial
except ImportError:
    serial = None

try:
    from PIL import Image
except ImportError:
    Image = None


DEFAULT_BAUD = 115200
DEFAULT_CHUNK = 256


def image_to_rgb565(path, width=None, height=None):
    if Image is None:
        raise RuntimeError("Pillow is required for PNG/JPG input: python -m pip install pillow")

    image = Image.open(path).convert("RGB")
    if width and height:
        image.thumbnail((width, height), Image.Resampling.LANCZOS)
        canvas = Image.new("RGB", (width, height), (255, 255, 255))
        x = (width - image.width) // 2
        y = (height - image.height) // 2
        canvas.paste(image, (x, y))
        image = canvas
    width, height = image.size

    payload = bytearray()
    for red, green, blue in image.getdata():
        value = ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3)
        payload.extend(struct.pack(">H", value))

    return width, height, bytes(payload)


def load_raw_rgb565(path, width, height):
    if not width or not height:
        raise RuntimeError("--width and --height are required for .bin input")
    payload = pathlib.Path(path).read_bytes()
    expected = width * height * 2
    if len(payload) != expected:
        raise RuntimeError(f"raw RGB565 size mismatch: got {len(payload)}, expected {expected}")
    return width, height, payload


def read_line(port, timeout_s):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        line = port.readline()
        if line:
            return line.decode("utf-8", errors="replace").strip()
    return ""


def wait_for(port, prefix, timeout_s=20.0):
    while True:
        line = read_line(port, timeout_s)
        if not line:
            raise TimeoutError(f"timeout waiting for {prefix}")
        print(f"< {line}")
        if line.startswith(prefix):
            return line
        if line.startswith("ERR "):
            raise RuntimeError(line)


def send_image(args):
    if serial is None:
        raise RuntimeError("pyserial is required: python -m pip install pyserial")

    path = pathlib.Path(args.image)
    if path.suffix.lower() == ".bin":
        width, height, payload = load_raw_rgb565(path, args.width, args.height)
    else:
        width, height, payload = image_to_rgb565(path, args.width, args.height)

    crc = binascii.crc32(payload) & 0xFFFFFFFF
    command = f"IMG_WRITE {args.slot} {width} {height} {len(payload)} {crc:08X}\n"

    with serial.Serial(args.port, args.baud, timeout=0.2, write_timeout=2.0) as port:
        port.reset_input_buffer()
        print(f"> FLASHID")
        port.write(b"FLASHID\n")
        try:
            wait_for(port, "FLASH", timeout_s=3.0)
        except Exception as exc:
            print(f"warning: {exc}")

        print(f"> {command.strip()}")
        port.write(command.encode("ascii"))
        wait_for(port, "OK IMG_READY", timeout_s=30.0)

        sent = 0
        while sent < len(payload):
            chunk = payload[sent : sent + args.chunk]
            port.write(chunk)
            sent += len(chunk)
            line = wait_for(port, "OK IMG_PAGE", timeout_s=5.0)
            try:
                acked = int(line.split()[2])
            except (IndexError, ValueError) as exc:
                raise RuntimeError(f"bad page ack: {line}") from exc
            if acked != sent:
                raise RuntimeError(f"unexpected page ack {acked}, expected {sent}")
            if args.progress:
                percent = sent * 100.0 / len(payload)
                print(f"\r{sent}/{len(payload)} bytes {percent:5.1f}%", end="", flush=True)
            if args.delay > 0.0:
                time.sleep(args.delay)

        if args.progress:
            print()
        wait_for(port, "OK IMG_DONE", timeout_s=60.0)

        if args.show:
            show_cmd = f"IMG_SHOW {args.slot}\n"
            print(f"> {show_cmd.strip()}")
            port.write(show_cmd.encode("ascii"))
            wait_for(port, "OK IMG_SHOW", timeout_s=20.0)


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Send an image to W25Q64 over UART.")
    parser.add_argument("image", help="PNG/JPG image or raw RGB565 .bin")
    parser.add_argument("--port", required=True, help="COM port, for example COM6")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--slot", type=int, default=0)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=170)
    parser.add_argument("--chunk", type=int, default=DEFAULT_CHUNK)
    parser.add_argument("--delay", type=float, default=0.0)
    parser.add_argument("--show", action="store_true", help="Ask MCU to display the image after writing")
    parser.add_argument("--no-progress", dest="progress", action="store_false")
    parser.set_defaults(progress=True)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    send_image(args)


if __name__ == "__main__":
    main()
