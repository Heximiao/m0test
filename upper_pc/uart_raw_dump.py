#!/usr/bin/env python3
import argparse
import time


def printable(data):
    chars = []
    for byte in data:
        if byte in (10, 13):
            chars.append(chr(byte))
        elif 32 <= byte <= 126:
            chars.append(chr(byte))
        else:
            chars.append(".")
    return "".join(chars)


def main():
    parser = argparse.ArgumentParser(
        description="Dump raw UART bytes as ASCII and hex."
    )
    parser.add_argument("--port", default="/dev/ttyAMA10")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--chunk", type=int, default=128)
    args = parser.parse_args()

    try:
        import serial
    except ImportError as exc:  # pragma: no cover
        raise SystemExit("pyserial is required: pip install pyserial") from exc

    deadline = time.monotonic() + args.seconds
    total = 0
    line_buffer = bytearray()

    with serial.Serial(args.port, args.baudrate, timeout=0.1) as ser:
        print(f"opened {args.port} @ {args.baudrate}, listening {args.seconds:.1f}s")
        while time.monotonic() < deadline:
            data = ser.read(args.chunk)
            if not data:
                continue

            total += len(data)
            hex_text = " ".join(f"{byte:02X}" for byte in data)
            print(f"+{len(data):03d} bytes  hex={hex_text}  ascii={printable(data)!r}")

            line_buffer.extend(data)
            while True:
                newline = line_buffer.find(b"\n")
                if newline < 0:
                    break
                raw_line = bytes(line_buffer[:newline]).strip()
                del line_buffer[: newline + 1]
                if raw_line:
                    print(f"LINE {raw_line.decode('ascii', errors='replace')!r}")

    print(f"done, total={total} bytes")


if __name__ == "__main__":
    main()
