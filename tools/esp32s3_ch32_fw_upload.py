#!/usr/bin/env python3
import argparse
import hashlib
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial", file=sys.stderr)
    sys.exit(2)


def read_line(ser: serial.Serial, timeout_s: float) -> str:
    deadline = time.time() + timeout_s
    buf = bytearray()
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b == b"\n":
            return buf.decode("utf-8", errors="replace").strip()
        if b != b"\r":
            buf += b
    raise TimeoutError("timeout waiting line")


def send_cmd(ser: serial.Serial, line: str, timeout_s: float = 3.0) -> str:
    ser.write((line + "\n").encode("utf-8"))
    ser.flush()
    while True:
        resp = read_line(ser, timeout_s)
        if not resp:
            continue
        if resp.startswith("["):
            continue
        return resp


def expect_ok(resp: str):
    if not resp.startswith("OK "):
        raise RuntimeError(resp)


def parse_ok_count(resp: str, prefix: str) -> int:
    expect_ok(resp)
    if not resp.startswith(prefix):
        raise RuntimeError(f"unexpected response: {resp}")
    return int(resp[len(prefix):].strip())


def cmd_upload(args):
    image = Path(args.image).read_bytes()
    if not image:
        raise RuntimeError("image is empty")

    ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    try:
        ser.reset_input_buffer()
        resp = send_cmd(ser, f"fwb {len(image)}", timeout_s=5.0)
        expect_ok(resp)
        print(resp)

        chunk_size = args.chunk
        sent = 0
        for off in range(0, len(image), chunk_size):
            part = image[off:off + chunk_size]
            resp = send_cmd(ser, f"fwc {part.hex().upper()}", timeout_s=5.0)
            sent += len(part)
            written = parse_ok_count(resp, "OK chunk ")
            if written != sent:
                raise RuntimeError(f"chunk mismatch wrote={written} expected={sent}")
            if args.progress:
                print(f"upload {sent}/{len(image)}")
            if args.pace_ms > 0:
                time.sleep(args.pace_ms / 1000.0)

        resp = send_cmd(ser, "fwe", timeout_s=5.0)
        final_size = parse_ok_count(resp, "OK end ")
        if final_size != len(image):
            raise RuntimeError(f"final size mismatch wrote={final_size} expected={len(image)}")
        print(resp)

        if args.flash:
            ser.write(b"fwf\n")
            ser.flush()
            while True:
                line = read_line(ser, args.flash_timeout)
                if not line:
                    continue
                print(line)
                if line == "FLASH done":
                    break
                if line.startswith("ERR "):
                    raise RuntimeError(line)
    finally:
        ser.close()


def cmd_info(args):
    ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    try:
        ser.reset_input_buffer()
        print(send_cmd(ser, "fwi"))
    finally:
        ser.close()


def parse_fw_size(resp: str) -> int:
    expect_ok(resp)
    prefix = "OK size="
    if not resp.startswith(prefix):
        raise RuntimeError(f"unexpected response: {resp}")
    return int(resp[len(prefix):].strip())


def read_fw_chunk(ser: serial.Serial, offset: int, length: int) -> bytes:
    resp = send_cmd(ser, f"fwr {offset} {length}", timeout_s=5.0)
    prefix = "OK data "
    if not resp.startswith(prefix):
        raise RuntimeError(resp)
    return bytes.fromhex(resp[len(prefix):].strip())


def pull_firmware(ser: serial.Serial, chunk_size: int, progress: bool = False) -> bytes:
    size = parse_fw_size(send_cmd(ser, "fwi", timeout_s=5.0))
    out = bytearray()
    while len(out) < size:
        want = min(chunk_size, size - len(out))
        part = read_fw_chunk(ser, len(out), want)
        if len(part) != want:
            raise RuntimeError(f"short read offset={len(out)} got={len(part)} expected={want}")
        out += part
        if progress:
            print(f"download {len(out)}/{size}")
    return bytes(out)


def sha256_hex(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def cmd_download(args):
    ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    try:
        ser.reset_input_buffer()
        image = pull_firmware(ser, args.chunk, args.progress)
        Path(args.output).write_bytes(image)
        print(f"saved {args.output} size={len(image)} sha256={sha256_hex(image)}")
    finally:
        ser.close()


def cmd_compare(args):
    local = Path(args.image).read_bytes()
    ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    try:
        ser.reset_input_buffer()
        remote = pull_firmware(ser, args.chunk, args.progress)
    finally:
        ser.close()

    local_hash = sha256_hex(local)
    remote_hash = sha256_hex(remote)
    print(f"local  size={len(local)} sha256={local_hash}")
    print(f"remote size={len(remote)} sha256={remote_hash}")
    if local == remote:
        print("MATCH")
        return

    first = next((i for i, (a, b) in enumerate(zip(local, remote)) if a != b), None)
    if first is None and len(local) != len(remote):
        first = min(len(local), len(remote))
    raise RuntimeError(f"MISMATCH first_diff={first}")


def cmd_delete(args):
    ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    try:
        ser.reset_input_buffer()
        print(send_cmd(ser, "fwx"))
    finally:
        ser.close()


def cmd_fs_status(args):
    ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    try:
        ser.reset_input_buffer()
        resp = send_cmd(ser, "fws")
        expect_ok(resp)
        parts = resp.split()
        values = {}
        for part in parts[1:]:
            if "=" not in part:
                continue
            k, v = part.split("=", 1)
            values[k] = int(v)
        total = values.get("total", 0)
        used = values.get("used", 0)
        free = values.get("free", 0)
        mb = 1024.0 * 1024.0
        print(
            f"total={total / mb:.2f} MB ({total} bytes) "
            f"used={used / mb:.2f} MB ({used} bytes) "
            f"free={free / mb:.2f} MB ({free} bytes)"
        )
    finally:
        ser.close()


def cmd_flash(args):
    ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    try:
        ser.reset_input_buffer()
        ser.write(b"fwf\n")
        ser.flush()
        while True:
            line = read_line(ser, args.flash_timeout)
            if not line or line.startswith("["):
                continue
            print(line)
            if line == "FLASH done":
                break
            if line.startswith("ERR "):
                raise RuntimeError(line)
    finally:
        ser.close()


def main():
    ap = argparse.ArgumentParser(description="Upload CH32 firmware to ESP32-S3 LittleFS and optionally flash over RS485")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=0.5)

    sp = ap.add_subparsers(dest="cmd", required=True)

    p_up = sp.add_parser("upload")
    p_up.add_argument("image")
    p_up.add_argument("--chunk", type=int, default=64)
    p_up.add_argument("--pace-ms", type=int, default=3)
    p_up.add_argument("--flash", action="store_true")
    p_up.add_argument("--flash-timeout", type=float, default=20.0)
    p_up.add_argument("--progress", action="store_true")

    sp.add_parser("info")
    p_dl = sp.add_parser("download")
    p_dl.add_argument("output")
    p_dl.add_argument("--chunk", type=int, default=128)
    p_dl.add_argument("--progress", action="store_true")
    p_cmp = sp.add_parser("compare")
    p_cmp.add_argument("image")
    p_cmp.add_argument("--chunk", type=int, default=128)
    p_cmp.add_argument("--progress", action="store_true")
    sp.add_parser("fs-status")
    sp.add_parser("delete")
    p_flash = sp.add_parser("flash")
    p_flash.add_argument("--flash-timeout", type=float, default=20.0)

    args = ap.parse_args()
    if args.cmd == "upload":
        cmd_upload(args)
    elif args.cmd == "info":
        cmd_info(args)
    elif args.cmd == "download":
        cmd_download(args)
    elif args.cmd == "compare":
        cmd_compare(args)
    elif args.cmd == "fs-status":
        cmd_fs_status(args)
    elif args.cmd == "delete":
        cmd_delete(args)
    elif args.cmd == "flash":
        cmd_flash(args)


if __name__ == "__main__":
    main()
