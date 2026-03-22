#!/usr/bin/env python3
import argparse
import struct
import sys
import time
import zlib

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial", file=sys.stderr)
    sys.exit(2)

SOF0 = 0x55
SOF1 = 0xAA

CMD_PING = 0x01
CMD_INFO = 0x02
CMD_ERASE = 0x10
CMD_WRITE = 0x11
CMD_CRC32 = 0x12
CMD_RUN = 0x20
CMD_RESET = 0x21

ST_OK = 0x00


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class BootClient:
    def __init__(self, port: str, baud: int, timeout: float, debug: bool = False):
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        self.seq = 0
        self.debug = debug

    def close(self):
        self.ser.close()

    def _next_seq(self) -> int:
        self.seq = (self.seq + 1) & 0xFF
        return self.seq

    def _send_frame(self, cmd: int, payload: bytes) -> int:
        seq = self._next_seq()
        hdr = bytes([cmd, seq]) + struct.pack("<H", len(payload))
        crc = crc16_ccitt(hdr + payload)
        frame = bytes([SOF0, SOF1]) + hdr + payload + struct.pack("<H", crc)
        if self.debug:
            print("TX", frame.hex().upper())
        self.ser.write(frame)
        self.ser.flush()
        return seq

    def _recv_frame(self, timeout_s: float):
        t0 = time.time()
        while True:
            if time.time() - t0 > timeout_s:
                raise TimeoutError("Timeout waiting response")
            b = self.ser.read(1)
            if not b:
                continue
            if b[0] != SOF0:
                continue
            b2 = self.ser.read(1)
            if not b2 or b2[0] != SOF1:
                continue
            hdr = self.ser.read(4)
            if len(hdr) != 4:
                continue
            cmd, seq, ln = hdr[0], hdr[1], struct.unpack("<H", hdr[2:4])[0]
            payload = self.ser.read(ln)
            if len(payload) != ln:
                continue
            crc_raw = self.ser.read(2)
            if len(crc_raw) != 2:
                continue
            crc_rx = struct.unpack("<H", crc_raw)[0]
            crc_calc = crc16_ccitt(hdr + payload)
            if crc_rx != crc_calc:
                continue
            frame = bytes([SOF0, SOF1]) + hdr + payload + crc_raw
            if self.debug:
                print("RX", frame.hex().upper())
            return cmd, seq, payload

    def request(self, cmd: int, payload: bytes = b"", timeout_s: float = 1.5) -> bytes:
        seq = self._send_frame(cmd, payload)
        rc_cmd, rc_seq, rc_payload = self._recv_frame(timeout_s)
        if rc_cmd != (cmd | 0x80):
            raise RuntimeError(f"Unexpected response cmd: 0x{rc_cmd:02X}")
        if rc_seq != seq:
            raise RuntimeError(f"Unexpected sequence: {rc_seq} != {seq}")
        if not rc_payload:
            raise RuntimeError("Empty response")
        status = rc_payload[0]
        if status != ST_OK:
            raise RuntimeError(f"Device status error: 0x{status:02X}")
        return rc_payload[1:]

    def ping(self):
        p = self.request(CMD_PING)
        if len(p) < 3:
            raise RuntimeError("Invalid ping payload")
        ver_maj, ver_min, app_valid = p[0], p[1], p[2]
        return ver_maj, ver_min, bool(app_valid)

    def info(self):
        p = self.request(CMD_INFO)
        if len(p) < 10:
            raise RuntimeError("Invalid info payload")
        app_base = struct.unpack("<I", p[0:4])[0]
        app_max = struct.unpack("<I", p[4:8])[0]
        page = struct.unpack("<H", p[8:10])[0]
        return app_base, app_max, page

    def erase(self, addr: int, length: int):
        self.request(CMD_ERASE, struct.pack("<II", addr, length), timeout_s=3.0)

    def write(self, addr: int, chunk: bytes):
        payload = struct.pack("<IH", addr, len(chunk)) + chunk
        self.request(CMD_WRITE, payload, timeout_s=2.0)

    def crc32(self, addr: int, length: int) -> int:
        p = self.request(CMD_CRC32, struct.pack("<II", addr, length), timeout_s=2.0)
        if len(p) < 4:
            raise RuntimeError("Invalid CRC response")
        return struct.unpack("<I", p[:4])[0]

    def run(self):
        self.request(CMD_RUN, b"")

    def reset(self):
        self.request(CMD_RESET, b"")


def align_up(v: int, a: int) -> int:
    return (v + (a - 1)) & ~(a - 1)


def cmd_ping(cli: BootClient, _args):
    maj, min_, valid = cli.ping()
    print(f"bootloader={maj}.{min_} app_valid={int(valid)}")


def cmd_info(cli: BootClient, _args):
    base, max_size, page = cli.info()
    print(f"app_base=0x{base:08X} app_max={max_size} page={page}")


def cmd_run(cli: BootClient, _args):
    cli.run()
    print("run=ok")

def cmd_reset(cli: BootClient, _args):
    cli.reset()
    print("reset=ok")


def cmd_upload(cli: BootClient, args):
    with open(args.bin, "rb") as f:
        image = f.read()

    maj, min_, valid = cli.ping()
    base, app_max, page = cli.info()

    padded_len = align_up(len(image), page)
    if padded_len > app_max:
        raise RuntimeError(f"Image too large: {padded_len} > {app_max}")

    padded = image + (b"\xFF" * (padded_len - len(image)))

    print(f"bootloader={maj}.{min_} app_valid={int(valid)}")
    print(f"erase addr=0x{base:08X} len={padded_len}")
    cli.erase(base, padded_len)

    for off in range(0, padded_len, page):
        chunk = padded[off:off + page]
        cli.write(base + off, chunk)
        if args.progress:
            print(f"write {off + page}/{padded_len}")

    dev_crc = cli.crc32(base, padded_len)
    host_crc = zlib.crc32(padded) & 0xFFFFFFFF
    print(f"crc host=0x{host_crc:08X} dev=0x{dev_crc:08X}")
    if host_crc != dev_crc:
        raise RuntimeError("CRC mismatch after upload")

    if args.run:
        cli.run()
        print("run=ok")


def main():
    ap = argparse.ArgumentParser(description="CH32V006 RS485 bootloader tool")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=0.2)
    ap.add_argument("--debug", action="store_true")

    sp = ap.add_subparsers(dest="cmd", required=True)
    sp.add_parser("ping")
    sp.add_parser("info")
    sp.add_parser("run")
    sp.add_parser("reset")
    p_up = sp.add_parser("upload")
    p_up.add_argument("bin", help="Application firmware .bin linked for APP_BASE")
    p_up.add_argument("--run", action="store_true")
    p_up.add_argument("--progress", action="store_true")

    args = ap.parse_args()

    cli = BootClient(args.port, args.baud, args.timeout, args.debug)
    try:
        if args.cmd == "ping":
            cmd_ping(cli, args)
        elif args.cmd == "info":
            cmd_info(cli, args)
        elif args.cmd == "run":
            cmd_run(cli, args)
        elif args.cmd == "reset":
            cmd_reset(cli, args)
        elif args.cmd == "upload":
            cmd_upload(cli, args)
        else:
            raise RuntimeError("Unknown command")
    finally:
        cli.close()


if __name__ == "__main__":
    main()
