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
CMD_IAP_VERIFY = 0x82
CMD_IAP_END    = 0x83
CMD_JUMP_APP   = 0x84

BROADCAST_ADDR = 0xFF

ERR_SUCCESS = 0x00
ERR_ERROR   = 0x01

DEV_TYPE_NAMES = {
    0x00: "heater",
    0x01: "rewinder",
}


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
        if len(data) > 64:
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

    def jump_app(self) -> bool:
        """Send CMD_JUMP_APP; returns True if bootloader had a valid app and jumped."""
        self._send(self._build_frame(CMD_JUMP_APP))
        try:
            err, _ = self._read_ack(timeout=1.0)
            return err == ERR_SUCCESS
        except TimeoutError:
            # No response = bootloader already jumped (expected on success too)
            return True


def chunks(buf: bytes, n: int):
    for i in range(0, len(buf), n):
        yield buf[i:i + n]


def page_chunks(buf: bytes, page_size: int = 256):
    for i in range(0, len(buf), page_size):
        yield i, buf[i:i + page_size]


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
        for page_off, page in page_chunks(image, 256):
            attempt = 0
            page_ok = False
            page_padded = page + (b"\xFF" * (256 - len(page)))

            while attempt < args.retries and not page_ok:
                attempt += 1

                for chunk_idx, part in enumerate(chunks(page_padded, args.chunk), start=1):
                    dt_prog = cli.program_chunk(part)
                    if args.progress:
                        prog_off = page_off + min(chunk_idx * args.chunk, len(page))
                        print(
                            f"program {prog_off}/{total} page=0x{page_off:04X} "
                            f"chunk={chunk_idx}/4 try={attempt} ack_ms={dt_prog * 1000:.2f}"
                        )

                if args.no_verify:
                    page_ok = True
                    break

                try:
                    for chunk_idx, part in enumerate(chunks(page, args.chunk), start=1):
                        dt_verify = cli.verify_chunk(part)
                        if args.progress:
                            ver_off = page_off + min(chunk_idx * args.chunk, len(page))
                            print(
                                f"verify  {ver_off}/{total} page=0x{page_off:04X} "
                                f"chunk={chunk_idx}/{(len(page) + args.chunk - 1) // args.chunk} "
                                f"try={attempt} ack_ms={dt_verify * 1000:.2f}"
                            )
                    page_ok = True
                except Exception as exc:
                    if args.progress:
                        print(f"verify  page=0x{page_off:04X} try={attempt} failed={exc}")

            if not page_ok:
                raise RuntimeError(
                    f"page verify failed after {args.retries} tries at offset {page_off} size {len(page)}"
                )

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
            for dev_addr, dev_type in sorted(devices):
                type_name = DEV_TYPE_NAMES.get(dev_type, f"unknown(0x{dev_type:02X})")
                print(f"  addr=0x{dev_addr:02X}  type=0x{dev_type:02X} ({type_name})")
    finally:
        cli.close()


def cmd_jump_app(args):
    cli = IapClient(args.port, args.baud, args.timeout, args.addr, args.debug)
    try:
        ok = cli.jump_app()
        print("jumped to app" if ok else "ERROR: no valid app present")
    finally:
        cli.close()


def main():
    ap = argparse.ArgumentParser(description="CH32V006 RS485 IAP uploader")
    ap.add_argument("--port",    required=True)
    ap.add_argument("--baud",    type=int,   default=115200)
    ap.add_argument("--timeout", type=float, default=0.2)
    ap.add_argument("--addr",    type=lambda x: int(x, 0), default=0x01,
                    help="device address 0x01–0xFE (default 0x01)")
    ap.add_argument("--chunk",   type=int,   default=64)
    ap.add_argument("--retries", type=int,   default=3)
    ap.add_argument("--debug",   action="store_true")
    ap.add_argument("--progress", action="store_true")

    sp = ap.add_subparsers(dest="cmd", required=True)

    up = sp.add_parser("upload", help="flash a firmware image")
    up.add_argument("image")
    up.add_argument("--no-verify",   action="store_true")
    up.add_argument("--skip-erase",  action="store_true")

    sc = sp.add_parser("scan", help="discover devices in IAP mode on the bus")
    sc.add_argument("--collect", type=float, default=1.5,
                    help="seconds to collect scan responses (default 1.5)")

    sp.add_parser("jump-app", help="jump to existing app without flashing")

    args = ap.parse_args()

    if args.chunk < 1 or args.chunk > 64:
        raise SystemExit("--chunk must be 1..64")
    if args.retries < 1:
        raise SystemExit("--retries must be >= 1")
    if not (0x01 <= args.addr <= 0xFF):
        raise SystemExit("--addr must be 0x01..0xFF")

    if args.cmd == "upload":
        cmd_upload(args)
    elif args.cmd == "scan":
        cmd_scan(args)
    elif args.cmd == "jump-app":
        cmd_jump_app(args)
    else:
        raise SystemExit("unknown command")


if __name__ == "__main__":
    main()
