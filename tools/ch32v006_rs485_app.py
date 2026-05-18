#!/usr/bin/env python3
import argparse
import struct
import sys
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


class AppClient:
    def __init__(self, port: str, baud: int, timeout: float, addr: int, debug: bool = False):
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        self.addr = addr
        self.seq = 0
        self.debug = debug

    def close(self):
        self.ser.close()

    def _next_seq(self) -> int:
        self.seq = (self.seq + 1) & 0xFF
        return self.seq

    def _send_frame(self, cmd: int, payload: bytes, dest: int | None = None) -> int:
        d = self.addr if dest is None else dest
        seq = self._next_seq()
        # Frame: SOF0 SOF1 DEST CMD SEQ PLEN_L PLEN_H [PAYLOAD] CRC_L CRC_H
        hdr = bytes([d, cmd, seq]) + struct.pack("<H", len(payload))
        crc = crc16_ccitt(hdr + payload)
        frame = bytes([SOF0, SOF1]) + hdr + payload + struct.pack("<H", crc)
        if self.debug:
            print("TX", frame.hex().upper())
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()
        return seq

    def _recv_frame(self, timeout_s: float):
        """Return (dev_addr, cmd, seq, payload). Response format:
           SOF0 SOF1 DEV_ADDR CMD SEQ PLEN_L PLEN_H [PAYLOAD] CRC_L CRC_H
           CRC covers bytes [DEV_ADDR..end of PAYLOAD].
        """
        t0 = time.time()
        while True:
            if time.time() - t0 > timeout_s:
                raise TimeoutError("Timeout waiting for response")
            b = self.ser.read(1)
            if not b or b[0] != SOF0:
                continue
            b2 = self.ser.read(1)
            if not b2 or b2[0] != SOF1:
                continue
            hdr = self.ser.read(5)   # DEV_ADDR CMD SEQ PLEN_L PLEN_H
            if len(hdr) != 5:
                continue
            dev_addr, cmd, seq = hdr[0], hdr[1], hdr[2]
            plen = struct.unpack("<H", hdr[3:5])[0]
            payload = self.ser.read(plen)
            if len(payload) != plen:
                continue
            crc_raw = self.ser.read(2)
            if len(crc_raw) != 2:
                continue
            crc_rx   = struct.unpack("<H", crc_raw)[0]
            crc_calc = crc16_ccitt(hdr + payload)
            if crc_rx != crc_calc:
                if self.debug:
                    frame = bytes([SOF0, SOF1]) + hdr + payload + crc_raw
                    print(f"RX CRC mismatch {frame.hex().upper()}")
                continue
            if self.debug:
                frame = bytes([SOF0, SOF1]) + hdr + payload + crc_raw
                print("RX", frame.hex().upper())
            return dev_addr, cmd, seq, payload

    def request(self, cmd: int, payload: bytes = b"", timeout_s: float = 1.0,
                dest: int | None = None):
        seq = self._send_frame(cmd, payload, dest=dest)
        _dev_addr, rc_cmd, rc_seq, rc_payload = self._recv_frame(timeout_s)
        if rc_cmd != (cmd | 0x80):
            raise RuntimeError(f"Unexpected cmd 0x{rc_cmd:02X} (expected 0x{cmd|0x80:02X})")
        if rc_seq != seq:
            raise RuntimeError(f"Seq mismatch {rc_seq} != {seq}")
        if len(rc_payload) < 1:
            raise RuntimeError("Empty response")
        return rc_payload

    # ------------------------------------------------------------------ #

    def discover(self, collect_s: float = 2.0):
        """Broadcast CMD_DISCOVER; collect all responses."""
        self._send_frame(CMD_DISCOVER, b"", dest=BROADCAST_ADDR)
        found = []
        deadline = time.time() + collect_s
        while time.time() < deadline:
            try:
                dev_addr, rc_cmd, _seq, p = self._recv_frame(timeout_s=0.15)
                if rc_cmd == (CMD_DISCOVER | 0x80) and len(p) >= 14 and p[0] == ST_OK:
                    found.append({
                        "dev_addr": dev_addr,
                        "type":     p[1],
                        "addr":     p[2],
                        "version":  (p[3], p[4], p[5]),
                        "uid":      p[6:14].hex().upper(),
                    })
            except TimeoutError:
                break
        return found

    def ping(self):
        p = self.request(CMD_PING)
        uid = p[6:14].hex().upper() if len(p) >= 14 else ""
        return p[0], p[1], p[2], p[3], p[4], p[5], uid

    def get_version(self):
        p = self.request(CMD_GET_VERSION)
        uid = p[6:14].hex().upper() if len(p) >= 14 else ""
        return p[0], p[1], p[2], p[3], p[4], p[5], uid

    def enter_boot(self):
        return self.request(CMD_ENTER_BOOT, timeout_s=1.0)[0]

    def motor_ctrl(self, channel: int, speed: int):
        """speed: 0=stop, 1-255=PWM duty."""
        return self.request(CMD_MOTOR_CTRL, bytes([channel, speed]))[0]

    def get_switch_status(self):
        p = self.request(CMD_GET_SWITCH_STATUS)
        if p[0] != ST_OK or len(p) < 3:
            return p[0], None
        return p[0], (p[1], p[2])

    def read_uid(self, antenna: int):
        p = self.request(CMD_READ_UID, bytes([antenna]))
        if p[0] != ST_OK or len(p) < 4:
            return p[0], None
        uid = p[4:4 + p[3]]
        return p[0], {"antenna": p[1], "pn_status": p[2], "uid": uid}

    def read_filament(self, antenna: int):
        p = self.request(CMD_READ_FILAMENT, bytes([antenna]), timeout_s=3.0)
        if p[0] != ST_OK or len(p) < 4:
            return p[0], None
        uid_len = p[3]
        pos = 4 + uid_len
        uid = p[4:pos]
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
        return p[0], {
            "antenna": p[1], "pn_status": p[2], "uid": uid,
            "color_rgba": rgba, "drying_temp_c": drying_temp_c,
            "drying_time_h": drying_time_h, "nozzle_min_c": nozzle_min_c,
            "nozzle_max_c": nozzle_max_c, "variant": variant,
            "material_id": material_id, "filament_type": filament_type,
            "detailed_type": detailed_type, "production_time": production_time,
        }

    def listen_hb(self, duration_s: float = 10.0):
        """Passively receive unsolicited CMD_HB frames."""
        results = []
        deadline = time.time() + duration_s
        while time.time() < deadline:
            try:
                dev_addr, rc_cmd, _seq, p = self._recv_frame(timeout_s=0.2)
                if rc_cmd == CMD_HB and len(p) >= 8:
                    results.append({
                        "dev_addr":   dev_addr,
                        "type":       p[0],
                        "addr":       p[1],
                        "version":    (p[2], p[3], p[4]),
                        "motor_mask": p[5],
                        "sw_mask":    p[6],
                        "pcf_raw":    p[7],
                    })
            except TimeoutError:
                pass
        return results


# ------------------------------------------------------------------ #
# CLI command handlers
# ------------------------------------------------------------------ #

def cmd_discover(cli: AppClient, _args):
    print("discovering on bus (broadcast)...")
    devices = cli.discover()
    if not devices:
        print("no devices found")
        return
    print(f"found {len(devices)} device(s):")
    for d in devices:
        v = d["version"]
        tname = DEV_TYPE_NAMES.get(d["type"], f"0x{d['type']:02X}")
        print(f"  addr=0x{d['dev_addr']:02X}  type={tname}  "
              f"version={v[0]}.{v[1]}.{v[2]}  uid={d['uid']}")


def cmd_ping(cli: AppClient, _args):
    st, typ, addr, maj, mi, patch, uid = cli.ping()
    print(f"status={status_name(st)} type={DEV_TYPE_NAMES.get(typ, f'0x{typ:02X}')} "
          f"addr=0x{addr:02X} version={maj}.{mi}.{patch} uid={uid}")


def cmd_version(cli: AppClient, _args):
    st, typ, addr, maj, mi, patch, uid = cli.get_version()
    print(f"status={status_name(st)} type={DEV_TYPE_NAMES.get(typ, f'0x{typ:02X}')} "
          f"addr=0x{addr:02X} version={maj}.{mi}.{patch} uid={uid}")


def cmd_enter_boot(cli: AppClient, _args):
    st = cli.enter_boot()
    print(f"status={status_name(st)}")


def cmd_motor(cli: AppClient, args):
    st = cli.motor_ctrl(args.channel, args.speed)
    label = "stop" if args.speed == 0 else f"speed={args.speed}"
    print(f"status={status_name(st)} channel={args.channel} {label}")


def cmd_get_switch(cli: AppClient, _args):
    st, data = cli.get_switch_status()
    if st == ST_OK and data:
        raw, mask = data
        switches = "  ".join(f"sw{i+1}={'ON' if (mask>>i)&1 else 'off'}" for i in range(4))
        print(f"status=OK  raw=0x{raw:02X}  online=0x{mask:02X}  {switches}")
    else:
        print(f"status={status_name(st)}")


def cmd_read_uid(cli: AppClient, args):
    st, data = cli.read_uid(args.antenna)
    if st == ST_OK and data:
        print(f"status=OK antenna={data['antenna']} pn=0x{data['pn_status']:02X} "
              f"uid={data['uid'].hex().upper()}")
    else:
        print(f"status={status_name(st)}")


def cmd_read_filament(cli: AppClient, args):
    st, d = cli.read_filament(args.antenna)
    if st == ST_OK and d:
        print(f"status=OK antenna={d['antenna']} pn=0x{d['pn_status']:02X}")
        print(f"uid={d['uid'].hex().upper()}")
        print(f"variant={d['variant']}")
        print(f"material_id={d['material_id']}")
        print(f"filament_type={d['filament_type']}")
        print(f"detailed_type={d['detailed_type']}")
        print(f"color_rgba=#{d['color_rgba'].hex().upper()}")
        print(f"drying_temp_c={d['drying_temp_c']}")
        print(f"drying_time_h={d['drying_time_h']}")
        print(f"nozzle_min_c={d['nozzle_min_c']}")
        print(f"nozzle_max_c={d['nozzle_max_c']}")
        print(f"production_time={d['production_time']}")
    else:
        print(f"status={status_name(st)}")


def cmd_listen(cli: AppClient, args):
    print(f"listening for heartbeats ({args.duration}s)...")
    hbs = cli.listen_hb(duration_s=args.duration)
    if not hbs:
        print("no heartbeats received")
        return
    for h in hbs:
        v = h["version"]
        motors   = "  ".join(f"m{i+1}={'ON' if (h['motor_mask']>>i)&1 else 'off'}" for i in range(4))
        switches = "  ".join(f"sw{i+1}={'ON' if (h['sw_mask']>>i)&1 else 'off'}" for i in range(4))
        tname = DEV_TYPE_NAMES.get(h["type"], f"0x{h['type']:02X}")
        print(f"HB addr=0x{h['dev_addr']:02X}  type={tname}  "
              f"v={v[0]}.{v[1]}.{v[2]}  {motors}  {switches}  pcf=0x{h['pcf_raw']:02X}")


def main():
    ap = argparse.ArgumentParser(description="CH32V006 rewinder app RS485 tool")
    ap.add_argument("--port",    required=True)
    ap.add_argument("--baud",    type=int,   default=460800)
    ap.add_argument("--timeout", type=float, default=0.3)
    ap.add_argument("--addr",    type=lambda x: int(x, 0), default=0x01,
                    help="device address 0x01-0xFE (default 0x01)")
    ap.add_argument("--debug",   action="store_true")

    sp = ap.add_subparsers(dest="cmd", required=True)

    sp.add_parser("discover",          help="broadcast discover all devices")
    sp.add_parser("ping",              help="ping device")
    sp.add_parser("version",           help="get firmware version")
    sp.add_parser("enter-boot",        help="reboot into IAP bootloader")
    sp.add_parser("get-switch-status", help="read limit switch states")

    p_motor = sp.add_parser("motor",   help="set motor speed")
    p_motor.add_argument("channel",    type=int, choices=[1, 2, 3, 4])
    p_motor.add_argument("speed",      type=int, metavar="SPEED",
                         help="0=stop, 1-255=PWM speed")

    p_uid = sp.add_parser("read-uid",  help="read RFID UID")
    p_uid.add_argument("antenna",      type=int, choices=[1, 2, 3, 4])

    p_fil = sp.add_parser("read-filament", help="read filament tag data")
    p_fil.add_argument("antenna",      type=int, choices=[1, 2, 3, 4])

    p_listen = sp.add_parser("listen", help="listen for heartbeat frames")
    p_listen.add_argument("--duration", type=float, default=10.0)

    args = ap.parse_args()

    if not (0x01 <= args.addr <= 0xFF):
        raise SystemExit("--addr must be 0x01..0xFF")
    if args.cmd == "motor" and not (0 <= args.speed <= 255):
        raise SystemExit("speed must be 0-255")

    cli = AppClient(args.port, args.baud, args.timeout, args.addr, args.debug)
    try:
        {
            "discover":          cmd_discover,
            "ping":              cmd_ping,
            "version":           cmd_version,
            "enter-boot":        cmd_enter_boot,
            "get-switch-status": cmd_get_switch,
            "motor":             cmd_motor,
            "read-uid":          cmd_read_uid,
            "read-filament":     cmd_read_filament,
            "listen":            cmd_listen,
        }[args.cmd](cli, args)
    finally:
        cli.close()


if __name__ == "__main__":
    main()
