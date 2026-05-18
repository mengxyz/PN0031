#!/usr/bin/env python3
import argparse
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial", file=sys.stderr)
    sys.exit(2)


SYNC1 = 0xAA
SYNC2 = 0x55

CMD_SCAN       = 0x01
CMD_IAP_PROM   = 0x80
CMD_IAP_ERASE  = 0x81
CMD_IAP_VERIFY = 0x82  # legacy per-chunk verify
CMD_IAP_END    = 0x83
CMD_IAP_CRC    = 0x85  # flush+verify: 4-byte payload [size_l size_h crc16_l crc16_h]

BROADCAST_ADDR = 0xFF

ERR_SUCCESS = 0x00
ERR_ERROR   = 0x01

DEV_TYPE_NAMES = {
    0x00: "heater",
    0x01: "rewinder",
}


def decode_iap_version(ext: int) -> tuple[int, int, int]:
    return ((ext >> 4) & 0x0F, (ext >> 2) & 0x03, ext & 0x03)


def checksum16(dest: int, cmd: int, length: int, data: bytes) -> int:
    return (dest + cmd + length + sum(data)) & 0xFFFF


class IapClient:
    def __init__(self, port: str, baud: int, timeout: float, addr: int, debug: bool = False):
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        self.addr = addr
        self.debug = debug

    def close(self):
        self.ser.close()

    def _build_frame(self, cmd: int, data: bytes = b"", dest: int | None = None) -> bytes:
        if len(data) > 128:
            raise ValueError("data too long")
        d = self.addr if dest is None else dest
        length = len(data)
        csum = checksum16(d, cmd, length, data)
        return (
            bytes([SYNC1, SYNC2, d, cmd, length]) +
            data +
            bytes([csum & 0xFF, (csum >> 8) & 0xFF, SYNC2, SYNC1])
        )

    def _read_ack(self, timeout: float = 1.0) -> tuple[int, int]:
        """Return (err, ext) from a 7-byte response frame."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            if b[0] != SYNC1:
                continue
            rest = self.ser.read(6)
            if len(rest) != 6:
                continue
            frame = b + rest
            if self.debug:
                print("RX", frame.hex().upper())
            if frame[1] != SYNC2 or frame[5] != SYNC2 or frame[6] != SYNC1:
                continue
            return frame[3], frame[4]   # err, ext
        raise TimeoutError("Timeout waiting for IAP ACK")

    def _send(self, frame: bytes):
        if self.debug:
            print("TX", frame.hex().upper())
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()

    # ------------------------------------------------------------------ #

    def scan(self, collect_s: float = 1.5) -> list[tuple[int, int]]:
        """Broadcast CMD_SCAN; return list of (dev_addr, dev_type) responses."""
        frame = self._build_frame(CMD_SCAN, b"", dest=BROADCAST_ADDR)
        if self.debug:
            print("TX", frame.hex().upper())
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()

        found = []
        buf = b""
        deadline = time.time() + collect_s
        while time.time() < deadline:
            chunk = self.ser.read(16)
            if chunk:
                buf += chunk
            # parse all complete 7-byte frames from buf
            while len(buf) >= 7:
                idx = buf.find(bytes([SYNC1, SYNC2]))
                if idx < 0:
                    buf = b""
                    break
                if idx > 0:
                    buf = buf[idx:]
                if len(buf) < 7:
                    break
                frame = buf[:7]
                buf = buf[7:]
                if self.debug:
                    print("RX", frame.hex().upper())
                if frame[5] == SYNC2 and frame[6] == SYNC1 and frame[3] == ERR_SUCCESS:
                    found.append((frame[2], frame[4]))  # dev_addr, dev_type
        return found

    def erase(self) -> float:
        t0 = time.perf_counter()
        self._send(self._build_frame(CMD_IAP_ERASE))
        err, _ = self._read_ack(timeout=15.0)
        if err != ERR_SUCCESS:
            raise RuntimeError(f"ERASE failed status=0x{err:02X}")
        return time.perf_counter() - t0

    def program_chunk(self, chunk: bytes) -> float:
        t0 = time.perf_counter()
        self._send(self._build_frame(CMD_IAP_PROM, chunk))
        err, _ = self._read_ack(timeout=1.0)
        if err != ERR_SUCCESS:
            raise RuntimeError(f"PROGRAM failed status=0x{err:02X}")
        return time.perf_counter() - t0

    def verify_chunk(self, chunk: bytes) -> float:
        t0 = time.perf_counter()
        self._send(self._build_frame(CMD_IAP_VERIFY, chunk))
        err, _ = self._read_ack(timeout=1.0)
        if err != ERR_SUCCESS:
            raise RuntimeError(f"VERIFY failed status=0x{err:02X}")
        return time.perf_counter() - t0

    def end(self) -> float:
        t0 = time.perf_counter()
        self._send(self._build_frame(CMD_IAP_END))
        time.sleep(0.3)
        return time.perf_counter() - t0

    def verify_cksum(self, cksum16: int) -> float:
        """Send CMD_IAP_CRC: 2-byte payload = sum16 of all PROM bytes sent."""
        import struct
        payload = struct.pack("<H", cksum16 & 0xFFFF)
        t0 = time.perf_counter()
        self._send(self._build_frame(CMD_IAP_CRC, payload))
        err, _ = self._read_ack(timeout=2.0)
        if err != ERR_SUCCESS:
            raise RuntimeError(f"cksum verify failed status=0x{err:02X}")
        return time.perf_counter() - t0

def chunks(buf: bytes, n: int):
    for i in range(0, len(buf), n):
        yield buf[i:i + n]


def page_chunks(buf: bytes, page_size: int = 256):
    for i in range(0, len(buf), page_size):
        yield i, buf[i:i + page_size]


def cksum16(data: bytes) -> int:
    """16-bit additive sum — matches bootloader g_prog_cksum accumulation."""
    return sum(data) & 0xFFFF


def cmd_upload(args):
    image = Path(args.image).read_bytes()
    if len(image) == 0:
        raise RuntimeError("image is empty")

    cli = IapClient(args.port, args.baud, args.timeout, args.addr, args.debug)
    try:
        if not args.skip_erase:
            print(f"erase addr=0x{args.addr:02X} image_len={len(image)}")
            dt = cli.erase()
            print(f"erase_ack_ms={dt * 1000:.2f}")
        else:
            print(f"skip erase addr=0x{args.addr:02X} image_len={len(image)}")

        total = len(image)
        chunk_n = 0
        for part in chunks(image, args.chunk):
            dt_prog = cli.program_chunk(part)
            chunk_n += 1
            if args.progress:
                done = min(chunk_n * args.chunk, total)
                print(f"program {done}/{total} chunk={chunk_n} ack_ms={dt_prog * 1000:.2f}")

        if not args.no_verify:
            # Fast end-to-end additive checksum verify (CMD_IAP_CRC)
            cs = cksum16(image)
            dt_cs = cli.verify_cksum(cs)
            if args.progress:
                print(f"cksum16=0x{cs:04X} ack_ms={dt_cs * 1000:.2f}")
            print(f"cksum ok (0x{cs:04X})")

        print("end")
        dt = cli.end()
        print(f"end_wait_ms={dt * 1000:.2f}")
        print("done")
    finally:
        cli.close()


def cmd_scan(args):
    cli = IapClient(args.port, args.baud, args.timeout, BROADCAST_ADDR, args.debug)
    try:
        print(f"scanning bus on {args.port} (collect={args.collect}s)...")
        devices = cli.scan(collect_s=args.collect)
        if not devices:
            print("no devices found in IAP mode")
        else:
            print(f"found {len(devices)} device(s):")
            for dev_addr, ext in sorted(devices):
                if ext in DEV_TYPE_NAMES:
                    type_name = DEV_TYPE_NAMES.get(ext, f"unknown(0x{ext:02X})")
                    print(f"  addr=0x{dev_addr:02X}  type=0x{ext:02X} ({type_name})")
                else:
                    maj, min_, patch = decode_iap_version(ext)
                    print(f"  addr=0x{dev_addr:02X}  iap_version={maj}.{min_}.{patch} ext=0x{ext:02X}")
    finally:
        cli.close()


def main():
    ap = argparse.ArgumentParser(description="CH32V006 RS485 IAP uploader")
    ap.add_argument("--port",    required=True)
    ap.add_argument("--baud",    type=int,   default=460800)
    ap.add_argument("--timeout", type=float, default=0.2)
    ap.add_argument("--addr",    type=lambda x: int(x, 0), default=0x01,
                    help="device address 0x01–0xFE (default 0x01)")
    ap.add_argument("--chunk",   type=int,   default=128,
                    help="bytes per PROM packet, 1..128 (default 128)")
    ap.add_argument("--debug",   action="store_true")
    ap.add_argument("--progress", action="store_true")

    sp = ap.add_subparsers(dest="cmd", required=True)

    up = sp.add_parser("upload", help="flash a firmware image")
    up.add_argument("image")
    up.add_argument("--no-verify",   action="store_true",
                    help="skip end-to-end checksum (fastest, no safety net)")
    up.add_argument("--skip-erase", action="store_true")

    sc = sp.add_parser("scan", help="discover devices in IAP mode on the bus")
    sc.add_argument("--collect", type=float, default=1.5,
                    help="seconds to collect scan responses (default 1.5)")

    args = ap.parse_args()

    if args.chunk < 1 or args.chunk > 128:
        raise SystemExit("--chunk must be 1..128")
    if not (0x01 <= args.addr <= 0xFF):
        raise SystemExit("--addr must be 0x01..0xFF")

    if args.cmd == "upload":
        cmd_upload(args)
    elif args.cmd == "scan":
        cmd_scan(args)
    else:
        raise SystemExit("unknown command")


if __name__ == "__main__":
    main()
