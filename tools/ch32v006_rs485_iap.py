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

CMD_IAP_PROM = 0x80
CMD_IAP_ERASE = 0x81
CMD_IAP_VERIFY = 0x82
CMD_IAP_END = 0x83
CMD_JUMP_IAP = 0x84

ERR_SUCCESS = 0x00
ERR_ERROR = 0x01


def checksum16(cmd: int, length: int, extra: bytes, data: bytes) -> int:
    total = cmd + length
    total += sum(extra)
    total += sum(data)
    return total & 0xFFFF


class IapClient:
    def __init__(self, port: str, baud: int, timeout: float, debug: bool = False):
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        self.debug = debug

    def close(self):
        self.ser.close()

    def _build_frame(self, cmd: int, data: bytes = b"", extra4: bytes = b"") -> bytes:
        if len(extra4) not in (0, 4):
            raise ValueError("extra4 must be 0 or 4 bytes")
        if len(data) > 64:
            raise ValueError("data too long")
        length = len(data)
        csum = checksum16(cmd, length, extra4, data)
        return (
            bytes([SYNC1, SYNC2, cmd, length]) +
            extra4 +
            data +
            bytes([csum & 0xFF, (csum >> 8) & 0xFF, SYNC2, SYNC1])
        )

    def _read_ack(self, timeout: float = 1.0) -> int:
        deadline = time.time() + timeout
        while time.time() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            if b[0] != SYNC1:
                continue
            rest = self.ser.read(5)
            if len(rest) != 5:
                continue
            frame = b + rest
            if self.debug:
                print("RX", frame.hex().upper())
            if frame[1] != SYNC2 or frame[2] != 0x00 or frame[4] != SYNC2 or frame[5] != SYNC1:
                continue
            return frame[3]
        raise TimeoutError("Timeout waiting IAP ACK")

    def _send(self, frame: bytes):
        if self.debug:
            print("TX", frame.hex().upper())
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()

    def erase(self):
        t0 = time.perf_counter()
        self._send(self._build_frame(CMD_IAP_ERASE, b"", b"\x00\x00\x00\x00"))
        err = self._read_ack(timeout=15.0)
        if err != ERR_SUCCESS:
            raise RuntimeError(f"ERASE failed status=0x{err:02X}")
        return time.perf_counter() - t0

    def program_chunk(self, chunk: bytes):
        t0 = time.perf_counter()
        self._send(self._build_frame(CMD_IAP_PROM, chunk))
        err = self._read_ack(timeout=1.0)
        if err != ERR_SUCCESS:
            raise RuntimeError(f"PROGRAM failed status=0x{err:02X}")
        return time.perf_counter() - t0

    def verify_chunk(self, chunk: bytes):
        t0 = time.perf_counter()
        self._send(self._build_frame(CMD_IAP_VERIFY, chunk, b"\x00\x00\x00\x00"))
        err = self._read_ack(timeout=1.0)
        if err != ERR_SUCCESS:
            raise RuntimeError(f"VERIFY failed status=0x{err:02X}")
        return time.perf_counter() - t0

    def end(self):
        t0 = time.perf_counter()
        self._send(self._build_frame(CMD_IAP_END))
        time.sleep(0.3)
        return time.perf_counter() - t0

    def probe(self):
        t0 = time.perf_counter()
        self._send(self._build_frame(CMD_JUMP_IAP))
        err = self._read_ack(timeout=1.0)
        if err != ERR_SUCCESS:
            raise RuntimeError(f"PROBE failed status=0x{err:02X}")
        return time.perf_counter() - t0


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

    cli = IapClient(args.port, args.baud, args.timeout, args.debug)
    try:
        if not args.skip_erase:
            print(f"erase image_len={len(image)}")
            dt = cli.erase()
            print(f"erase_ack_ms={dt * 1000:.2f}")
        else:
            print(f"skip erase image_len={len(image)}")

        total = len(image)
        for page_off, page in page_chunks(image, 256):
            attempt = 0
            page_ok = False
            page_end = min(page_off + len(page), total)
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


def main():
    ap = argparse.ArgumentParser(description="CH32V006 RS485 WCH-IAP uploader")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=0.2)
    ap.add_argument("--chunk", type=int, default=64)
    ap.add_argument("--retries", type=int, default=3)
    ap.add_argument("--debug", action="store_true")
    ap.add_argument("--progress", action="store_true")

    sp = ap.add_subparsers(dest="cmd", required=True)
    up = sp.add_parser("upload")
    up.add_argument("image")
    up.add_argument("--no-verify", action="store_true")
    up.add_argument("--skip-erase", action="store_true")
    sp.add_parser("probe")

    args = ap.parse_args()

    if args.chunk < 1 or args.chunk > 64:
        raise SystemExit("--chunk must be 1..64")
    if args.retries < 1:
        raise SystemExit("--retries must be >= 1")

    if args.cmd == "upload":
        cmd_upload(args)
    elif args.cmd == "probe":
        cli = IapClient(args.port, args.baud, args.timeout, args.debug)
        try:
            dt = cli.probe()
            print(f"probe=OK ack_ms={dt * 1000:.2f}")
        finally:
            cli.close()
    else:
        raise SystemExit("unknown command")


if __name__ == "__main__":
    main()
