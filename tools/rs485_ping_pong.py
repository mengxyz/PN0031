#!/usr/bin/env python3
import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial", file=sys.stderr)
    sys.exit(2)


def read_line(ser: serial.Serial, timeout_s: float) -> bytes:
    deadline = time.time() + timeout_s
    out = bytearray()
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        out.extend(b)
        if b == b"\n":
            return bytes(out)
    return bytes(out)


def main():
    ap = argparse.ArgumentParser(description="Simple RS485 ping/pong tester")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=1.0)
    ap.add_argument("--message", default="ping", help="Text to send")
    args = ap.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        ser.reset_input_buffer()
        msg = (args.message + "\r\n").encode("utf-8")
        ser.write(msg)
        ser.flush()
        line = read_line(ser, args.timeout)
        if not line:
            print("timeout")
            sys.exit(1)
        print(line.decode("utf-8", errors="replace").rstrip())


if __name__ == "__main__":
    main()
