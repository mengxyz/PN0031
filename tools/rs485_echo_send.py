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
    ap = argparse.ArgumentParser(description="Send raw text to rs485_echo_test and print reply")
    ap.add_argument("--port", required=True, help="Serial port, for example /dev/tty.wchusbserialXXXX")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--message", default="ping", help="ASCII text to send")
    ap.add_argument("--settle-ms", type=int, default=500, help="Wait time before reading reply")
    ap.add_argument("--read-bytes", type=int, default=64, help="Max reply bytes to read")
    ap.add_argument("--newline", action="store_true", help="Append CRLF")
    args = ap.parse_args()

    payload = args.message.encode("utf-8")
    if args.newline:
      payload += b"\r\n"

    with serial.Serial(args.port, args.baud, timeout=1.0) as ser:
        ser.reset_input_buffer()
        ser.write(payload)
        ser.flush()
        time.sleep(args.settle_ms / 1000.0)
        data = ser.read(args.read_bytes)
        if not data:
            print("timeout")
            sys.exit(1)
        print(data.decode("utf-8", errors="replace"))
        print(f"raw={data!r}")


if __name__ == "__main__":
    main()
