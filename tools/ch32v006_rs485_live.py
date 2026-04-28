#!/usr/bin/env python3
"""
CH32V006 RS485 live REPL.
Usage: python3 ch32v006_rs485_live.py --port /dev/tty.xxx [--baud 460800] [--addr 0x01] [--debug]

Stays connected. Background thread prints incoming heartbeat frames.
Type commands at the prompt; type 'help' for the list.
"""
import argparse
import queue
import readline  # noqa: F401 — enables arrow-key history in input()
import struct
import sys
import threading
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial", file=sys.stderr)
    sys.exit(2)

SOF0 = 0x55
SOF1 = 0xAA
BROADCAST_ADDR = 0xFF

CMD_DISCOVER          = 0x00
CMD_PING              = 0x01
CMD_ENTER_BOOT        = 0x22
CMD_GET_VERSION       = 0x30
CMD_READ_UID          = 0x32
CMD_READ_FILAMENT     = 0x33
CMD_MOTOR_CTRL        = 0x40
CMD_GET_SWITCH_STATUS = 0x41
CMD_HB                = 0x70

ST_OK        = 0x00
ST_BAD_CMD   = 0x01
ST_BAD_ARG   = 0x02
ST_IO_ERR    = 0x04
ST_NO_TAG    = 0x05
ST_NOT_READY = 0x06

DEV_TYPE_NAMES = {0x00: "heater", 0x01: "rewinder"}


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def status_name(st: int) -> str:
    return {ST_OK: "OK", ST_BAD_CMD: "BAD_CMD", ST_BAD_ARG: "BAD_ARG",
            ST_IO_ERR: "IO_ERR", ST_NO_TAG: "NO_TAG", ST_NOT_READY: "NOT_READY",
            }.get(st, f"0x{st:02X}")


class LiveClient:
    def __init__(self, port: str, baud: int, addr: int, debug: bool = False, no_hb: bool = False):
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.05)
        self.addr = addr
        self.seq = 0
        self.debug = debug
        self.no_hb = no_hb
        self._rx_queue: queue.Queue = queue.Queue()
        self._stop = threading.Event()
        self._keepalive_addrs: list = []
        self._reader  = threading.Thread(target=self._reader_thread,    daemon=True)
        self._keepalive = threading.Thread(target=self._keepalive_thread, daemon=True)
        self._reader.start()
        self._keepalive.start()

    def register_device(self, addr: int):
        if addr not in self._keepalive_addrs:
            self._keepalive_addrs.append(addr)

    def _keepalive_thread(self):
        while not self._stop.is_set():
            for addr in list(self._keepalive_addrs):
                try:
                    self._send_frame(CMD_HB, b"", dest=addr)
                except Exception:
                    pass
            self._stop.wait(2.0)

    def close(self):
        self._stop.set()
        self._reader.join(timeout=1.0)
        self.ser.close()

    def _next_seq(self) -> int:
        self.seq = (self.seq + 1) & 0xFF
        return self.seq

    def _reader_thread(self):
        buf = bytearray()
        while not self._stop.is_set():
            data = self.ser.read(64)
            if not data:
                continue
            buf.extend(data)
            while True:
                # Find SOF
                idx = -1
                for i in range(len(buf) - 1):
                    if buf[i] == SOF0 and buf[i + 1] == SOF1:
                        idx = i
                        break
                if idx < 0:
                    buf = buf[-1:] if len(buf) > 1 else buf
                    break
                if idx > 0:
                    buf = buf[idx:]
                if len(buf) < 9:
                    break
                plen = struct.unpack("<H", buf[5:7])[0]
                total = 9 + plen
                if len(buf) < total:
                    break
                frame = bytes(buf[:total])
                buf = buf[total:]
                hdr     = frame[2:7]
                payload = frame[7:7 + plen]
                crc_rx   = struct.unpack("<H", frame[7 + plen:9 + plen])[0]
                crc_calc = crc16_ccitt(hdr + payload)
                if crc_rx != crc_calc:
                    if self.debug:
                        print(f"\n[rx] CRC mismatch {frame.hex().upper()}")
                    continue
                if self.debug:
                    print(f"\nRX {frame.hex().upper()}")
                dev_addr, cmd = frame[2], frame[3]
                if cmd == CMD_HB:
                    self._on_hb(dev_addr, payload)
                else:
                    self._rx_queue.put((dev_addr, cmd, frame[4], payload))

    def _on_hb(self, dev_addr: int, p: bytes):
        pass  # runs in background, no output

    def _send_frame(self, cmd: int, payload: bytes, dest: int | None = None) -> int:
        d   = self.addr if dest is None else dest
        seq = self._next_seq()
        hdr = bytes([d, cmd, seq]) + struct.pack("<H", len(payload))
        crc = crc16_ccitt(hdr + payload)
        frame = bytes([SOF0, SOF1]) + hdr + payload + struct.pack("<H", crc)
        if self.debug:
            print(f"TX {frame.hex().upper()}")
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()
        return seq

    def _drain(self):
        while not self._rx_queue.empty():
            try:
                self._rx_queue.get_nowait()
            except queue.Empty:
                break

    def request(self, cmd: int, payload: bytes = b"", timeout_s: float = 1.0,
                dest: int | None = None) -> bytes:
        self._drain()
        seq = self._send_frame(cmd, payload, dest=dest)
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            try:
                _addr, rc_cmd, rc_seq, rc_payload = self._rx_queue.get(timeout=0.1)
                if rc_cmd == (cmd | 0x80) and rc_seq == seq:
                    return rc_payload
            except queue.Empty:
                continue
        raise TimeoutError(f"no response to cmd=0x{cmd:02X}")

    def discover(self, collect_s: float = 2.0):
        self._drain()
        self._send_frame(CMD_DISCOVER, b"", dest=BROADCAST_ADDR)
        found = []
        deadline = time.time() + collect_s
        while time.time() < deadline:
            try:
                dev_addr, rc_cmd, _seq, p = self._rx_queue.get(timeout=0.15)
                if rc_cmd == (CMD_DISCOVER | 0x80) and len(p) >= 14 and p[0] == ST_OK:
                    found.append({
                        "dev_addr": dev_addr,
                        "type":     p[1],
                        "addr":     p[2],
                        "version":  (p[3], p[4], p[5]),
                        "uid":      p[6:14].hex().upper(),
                    })
            except queue.Empty:
                break
        return found


# ------------------------------------------------------------------ #
# Command handlers
# ------------------------------------------------------------------ #

def do_discover(cli: LiveClient, _tokens):
    print("discovering on bus (broadcast)...")
    devices = cli.discover()
    if not devices:
        print("no devices found")
        return
    print(f"found {len(devices)} device(s):")
    for d in devices:
        v     = d["version"]
        tname = DEV_TYPE_NAMES.get(d["type"], f"0x{d['type']:02X}")
        print(f"  addr=0x{d['dev_addr']:02X}  type={tname}  "
              f"version={v[0]}.{v[1]}.{v[2]}  uid={d['uid']}")
        cli.register_device(d["dev_addr"])


def do_ping(cli: LiveClient, _tokens):
    p   = cli.request(CMD_PING)
    uid = p[6:14].hex().upper() if len(p) >= 14 else ""
    print(f"status={status_name(p[0])} type={DEV_TYPE_NAMES.get(p[1], f'0x{p[1]:02X}')} "
          f"addr=0x{p[2]:02X} version={p[3]}.{p[4]}.{p[5]} uid={uid}")


def do_version(cli: LiveClient, _tokens):
    p   = cli.request(CMD_GET_VERSION)
    uid = p[6:14].hex().upper() if len(p) >= 14 else ""
    print(f"status={status_name(p[0])} type={DEV_TYPE_NAMES.get(p[1], f'0x{p[1]:02X}')} "
          f"addr=0x{p[2]:02X} version={p[3]}.{p[4]}.{p[5]} uid={uid}")


def do_enter_boot(cli: LiveClient, _tokens):
    confirm = input("enter bootloader? this will reset the device [y/N] ").strip().lower()
    if confirm != "y":
        print("cancelled")
        return
    p = cli.request(CMD_ENTER_BOOT, timeout_s=1.0)
    print(f"status={status_name(p[0])}")


def do_motor(cli: LiveClient, tokens):
    if len(tokens) != 2:
        print("usage: motor <channel 1-4> <speed 0-255>"); return
    try:
        ch, speed = int(tokens[0]), int(tokens[1])
    except ValueError:
        print("usage: motor <channel 1-4> <speed 0-255>"); return
    if not (1 <= ch <= 4) or not (0 <= speed <= 255):
        print("channel 1-4, speed 0-255"); return
    p     = cli.request(CMD_MOTOR_CTRL, bytes([ch, speed]))
    label = "stop" if speed == 0 else f"speed={speed}"
    print(f"status={status_name(p[0])} channel={ch} {label}")


def do_switch(cli: LiveClient, _tokens):
    p = cli.request(CMD_GET_SWITCH_STATUS)
    if p[0] == ST_OK and len(p) >= 3:
        raw, mask = p[1], p[2]
        sw = "  ".join(f"sw{i+1}={'ON' if (mask >> i) & 1 else 'off'}" for i in range(4))
        print(f"status=OK  raw=0x{raw:02X}  online=0x{mask:02X}  {sw}")
    else:
        print(f"status={status_name(p[0])}")


def do_read_uid(cli: LiveClient, tokens):
    if len(tokens) != 1:
        print("usage: read-uid <antenna 1-4>"); return
    try:
        ant = int(tokens[0])
    except ValueError:
        print("usage: read-uid <antenna 1-4>"); return
    if not (1 <= ant <= 4):
        print("antenna 1-4"); return
    p = cli.request(CMD_READ_UID, bytes([ant]))
    if p[0] == ST_OK and len(p) >= 4:
        uid = p[4:4 + p[3]]
        print(f"status=OK antenna={p[1]} pn=0x{p[2]:02X} uid={uid.hex().upper()}")
    else:
        print(f"status={status_name(p[0])}")


def do_read_filament(cli: LiveClient, tokens):
    if len(tokens) != 1:
        print("usage: read-filament <antenna 1-4>"); return
    try:
        ant = int(tokens[0])
    except ValueError:
        print("usage: read-filament <antenna 1-4>"); return
    if not (1 <= ant <= 4):
        print("antenna 1-4"); return
    p = cli.request(CMD_READ_FILAMENT, bytes([ant]), timeout_s=3.0)
    if p[0] != ST_OK or len(p) < 4:
        print(f"status={status_name(p[0])}"); return
    uid_len = p[3]; pos = 4 + uid_len
    uid           = p[4:pos]
    rgba          = p[pos:pos+4];   pos += 4
    drying_temp_c = struct.unpack("<h", p[pos:pos+2])[0]; pos += 2
    drying_time_h = struct.unpack("<H", p[pos:pos+2])[0]; pos += 2
    nozzle_min_c  = struct.unpack("<h", p[pos:pos+2])[0]; pos += 2
    nozzle_max_c  = struct.unpack("<h", p[pos:pos+2])[0]; pos += 2
    def ns(b): return b.split(b"\x00", 1)[0].decode("ascii", errors="ignore")
    variant         = ns(p[pos:pos+8]);  pos += 8
    material_id     = ns(p[pos:pos+8]);  pos += 8
    filament_type   = ns(p[pos:pos+16]); pos += 16
    detailed_type   = ns(p[pos:pos+16]); pos += 16
    production_time = ns(p[pos:pos+16])
    print(f"status=OK antenna={p[1]} pn=0x{p[2]:02X}")
    print(f"uid={uid.hex().upper()}")
    print(f"variant={variant}  material_id={material_id}")
    print(f"filament_type={filament_type}  detailed_type={detailed_type}")
    print(f"color_rgba=#{rgba.hex().upper()}")
    print(f"drying_temp={drying_temp_c}°C  drying_time={drying_time_h}h")
    print(f"nozzle={nozzle_min_c}-{nozzle_max_c}°C")
    print(f"production_time={production_time}")


COMMANDS = {
    "discover":      (do_discover,      "broadcast discover all devices"),
    "ping":          (do_ping,          "ping device"),
    "version":       (do_version,       "get firmware version"),
    "enter-boot":    (do_enter_boot,    "reboot into IAP bootloader"),
    "switch":        (do_switch,        "read limit switch states"),
    "motor":         (do_motor,         "motor <ch 1-4> <speed 0-255>"),
    "read-uid":      (do_read_uid,      "read-uid <antenna 1-4>"),
    "read-filament": (do_read_filament, "read-filament <antenna 1-4>"),
}


def do_help(_cli, _tokens):
    print("commands:")
    for name, (_, desc) in COMMANDS.items():
        print(f"  {name:<18} {desc}")
    print("  hb                 toggle heartbeat display on/off")
    print("  help               show this help")
    print("  quit / exit        exit")


def main():
    ap = argparse.ArgumentParser(description="CH32V006 RS485 live REPL")
    ap.add_argument("--port",  required=True)
    ap.add_argument("--baud",  type=int, default=460800)
    ap.add_argument("--addr",  type=lambda x: int(x, 0), default=0x01,
                    help="device address (default 0x01)")
    ap.add_argument("--debug", action="store_true")
    ap.add_argument("--hb",    action="store_true", help="show heartbeats on startup")
    args = ap.parse_args()

    cli = LiveClient(args.port, args.baud, args.addr, args.debug, no_hb=not args.hb)
    print(f"connected  port={args.port}  baud={args.baud}  addr=0x{args.addr:02X}")
    print(f"type 'help' for commands, 'quit' to exit  (type 'hb' to show heartbeats)\n")

    try:
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break
            if not line:
                continue
            tokens = line.split()
            cmd    = tokens[0].lower()
            rest   = tokens[1:]
            if cmd in ("quit", "exit"):
                break
            elif cmd == "hb":
                cli.no_hb = not cli.no_hb
                print(f"heartbeat display {'OFF' if cli.no_hb else 'ON'}")
            elif cmd == "help":
                do_help(cli, rest)
            elif cmd in COMMANDS:
                try:
                    COMMANDS[cmd][0](cli, rest)
                except TimeoutError as e:
                    print(f"error: {e}")
                except Exception as e:
                    print(f"error: {e}")
            else:
                print(f"unknown command '{cmd}' — type 'help'")
    except KeyboardInterrupt:
        print()
    finally:
        cli.close()
        print("disconnected")


if __name__ == "__main__":
    main()
