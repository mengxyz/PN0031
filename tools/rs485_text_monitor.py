#!/usr/bin/env python3
import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial", file=sys.stderr)
    sys.exit(2)


def main():
    ap = argparse.ArgumentParser(description="Plain RS485/UART text monitor")
    ap.add_argument("--port", required=True, help="Serial port, for example /dev/tty.usbserial-14720")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate")
    ap.add_argument("--timeout", type=float, default=0.2, help="Serial read timeout in seconds")
    ap.add_argument("--raw", action="store_true", help="Print raw bytes repr instead of decoded text")
    ap.add_argument("--timestamp", action="store_true", help="Prefix each chunk with local time")
    args = ap.parse_args()

    with serial.Serial(port=args.port, baudrate=args.baud, timeout=args.timeout) as ser:
        print(f"open port={args.port} baud={args.baud}", file=sys.stderr)
        while True:
            data = ser.read(256)
            if not data:
                continue
            if args.timestamp:
                stamp = time.strftime("%H:%M:%S")
                prefix = f"[{stamp}] "
            else:
                prefix = ""
            if args.raw:
                print(prefix + repr(data))
            else:
                text = data.decode("utf-8", errors="replace")
                print(prefix + text, end="")
            sys.stdout.flush()


if __name__ == "__main__":
    main()
