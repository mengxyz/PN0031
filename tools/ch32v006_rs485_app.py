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

CMD_PING = 0x01
CMD_ENTER_BOOT = 0x22
CMD_GET_VERSION = 0x30
CMD_READ_UID = 0x32
CMD_READ_FILAMENT = 0x33
CMD_MOTOR_CTRL = 0x40
CMD_GET_SWITCH_STATUS = 0x41

ST_OK = 0x00
ST_BAD_CMD = 0x01
ST_BAD_ARG = 0x02
ST_IO_ERR = 0x04
ST_NO_TAG = 0x05
ST_NOT_READY = 0x06


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


class AppClient:
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
        self.ser.reset_input_buffer()
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

    def request(self, cmd: int, payload: bytes = b"", timeout_s: float = 1.0):
        seq = self._send_frame(cmd, payload)
        rc_cmd, rc_seq, rc_payload = self._recv_frame(timeout_s)
        if rc_cmd != (cmd | 0x80):
            raise RuntimeError(f"Unexpected response cmd: 0x{rc_cmd:02X}")
        if rc_seq != seq:
            raise RuntimeError(f"Unexpected sequence: {rc_seq} != {seq}")
        if len(rc_payload) < 1:
            raise RuntimeError("Empty response")
        return rc_payload

    def ping(self):
        payload = self.request(CMD_PING)
        if len(payload) < 4:
            raise RuntimeError("Invalid ping payload")
        status, maj, min_, patch = payload[:4]
        return status, maj, min_, patch

    def get_version(self):
        payload = self.request(CMD_GET_VERSION)
        if len(payload) < 4:
            raise RuntimeError("Invalid version payload")
        status, maj, min_, patch = payload[:4]
        return status, maj, min_, patch

    def enter_boot(self):
        payload = self.request(CMD_ENTER_BOOT, b"", timeout_s=1.0)
        status = payload[0]
        return status

    def read_uid(self, antenna: int):
        payload = self.request(CMD_READ_UID, bytes([antenna]))
        status = payload[0]
        if status != ST_OK:
            extra = payload[1:] if len(payload) > 1 else b""
            return status, extra
        if len(payload) < 4:
            raise RuntimeError("Invalid READ_UID payload")
        ant = payload[1]
        pn_status = payload[2]
        uid_len = payload[3]
        uid = payload[4:4 + uid_len]
        if len(uid) != uid_len:
            raise RuntimeError("Short UID payload")
        return status, (ant, pn_status, uid)

    def read_filament(self, antenna: int):
        payload = self.request(CMD_READ_FILAMENT, bytes([antenna]), timeout_s=3.0)
        status = payload[0]
        if status != ST_OK:
            extra = payload[1:] if len(payload) > 1 else b""
            return status, extra
        if len(payload) < 4:
            raise RuntimeError(f"Invalid READ_FILAMENT payload len={len(payload)}")
        ant = payload[1]
        pn_status = payload[2]
        uid_len = payload[3]
        min_len = 4 + uid_len + 4 + 2 + 2 + 2 + 2 + 8 + 8 + 16 + 16 + 16
        if len(payload) < min_len:
            raise RuntimeError(f"Invalid READ_FILAMENT payload len={len(payload)} expected>={min_len}")
        pos = 4
        uid = payload[pos:pos + uid_len]
        pos += uid_len
        rgba = payload[pos:pos + 4]
        pos += 4
        drying_temp_c = struct.unpack("<h", payload[pos:pos + 2])[0]
        pos += 2
        drying_time_h = struct.unpack("<H", payload[pos:pos + 2])[0]
        pos += 2
        nozzle_min_c = struct.unpack("<h", payload[pos:pos + 2])[0]
        pos += 2
        nozzle_max_c = struct.unpack("<h", payload[pos:pos + 2])[0]
        pos += 2
        variant = payload[pos:pos + 8].split(b"\x00", 1)[0].decode("ascii", errors="ignore")
        pos += 8
        material_id = payload[pos:pos + 8].split(b"\x00", 1)[0].decode("ascii", errors="ignore")
        pos += 8
        filament_type = payload[pos:pos + 16].split(b"\x00", 1)[0].decode("ascii", errors="ignore")
        pos += 16
        detailed_type = payload[pos:pos + 16].split(b"\x00", 1)[0].decode("ascii", errors="ignore")
        pos += 16
        production_time = payload[pos:pos + 16].split(b"\x00", 1)[0].decode("ascii", errors="ignore")
        return status, {
            "antenna": ant,
            "pn_status": pn_status,
            "uid": uid,
            "color_rgba": rgba,
            "drying_temp_c": drying_temp_c,
            "drying_time_h": drying_time_h,
            "nozzle_min_c": nozzle_min_c,
            "nozzle_max_c": nozzle_max_c,
            "variant": variant,
            "material_id": material_id,
            "filament_type": filament_type,
            "detailed_type": detailed_type,
            "production_time": production_time,
        }

    def get_switch_status(self):
        payload = self.request(CMD_GET_SWITCH_STATUS)
        status = payload[0]
        if status != ST_OK:
            extra = payload[1:] if len(payload) > 1 else b""
            return status, extra
        if len(payload) < 3:
            raise RuntimeError("Invalid GET_SWITCH_STATUS payload")
        return status, (payload[1], payload[2])

    def motor_ctrl(self, channel: int, action: int):
        payload = self.request(CMD_MOTOR_CTRL, bytes([channel, action]))
        if len(payload) < 1:
            raise RuntimeError("Invalid MOTOR_CTRL payload")
        return payload[0]


def status_name(status: int) -> str:
    return {
        ST_OK: "OK",
        ST_BAD_CMD: "BAD_CMD",
        ST_BAD_ARG: "BAD_ARG",
        ST_IO_ERR: "IO_ERR",
        ST_NO_TAG: "NO_TAG",
        ST_NOT_READY: "NOT_READY",
    }.get(status, f"0x{status:02X}")


def cmd_ping(cli: AppClient, _args):
    status, maj, min_, patch = cli.ping()
    print(f"status={status_name(status)} version={maj}.{min_}.{patch}")


def cmd_version(cli: AppClient, _args):
    status, maj, min_, patch = cli.get_version()
    print(f"status={status_name(status)} version={maj}.{min_}.{patch}")


def cmd_enter_boot(cli: AppClient, _args):
    status = cli.enter_boot()
    print(f"status={status_name(status)}")


def cmd_read_uid(cli: AppClient, args):
    status, data = cli.read_uid(args.antenna)
    if status == ST_OK:
        ant, pn_status, uid = data
        print(f"status=OK antenna={ant} pn_status=0x{pn_status:02X} uid={uid.hex().upper()}")
    elif status == ST_NO_TAG:
        pn_status = data[0] if data else 0xFF
        print(f"status=NO_TAG pn_status=0x{pn_status:02X}")
    else:
        print(f"status={status_name(status)}")


def cmd_get_switch_status(cli: AppClient, _args):
    status, data = cli.get_switch_status()
    if status == ST_OK:
        raw_state, online_mask = data
        print(
            f"status=OK raw=0x{raw_state:02X} "
            f"online_mask=0x{online_mask:01X} "
            f"sw1={'ON' if (online_mask & 0x1) else 'OFF'} "
            f"sw2={'ON' if (online_mask & 0x2) else 'OFF'} "
            f"sw3={'ON' if (online_mask & 0x4) else 'OFF'} "
            f"sw4={'ON' if (online_mask & 0x8) else 'OFF'}"
        )
    else:
        print(f"status={status_name(status)}")


def cmd_read_filament(cli: AppClient, args):
    status, data = cli.read_filament(args.antenna)
    if status == ST_OK:
        rgba = data["color_rgba"]
        print(f"status=OK antenna={data['antenna']} pn_status=0x{data['pn_status']:02X}")
        print(f"uid={data['uid'].hex().upper()}")
        print(f"variant={data['variant']}")
        print(f"material_id={data['material_id']}")
        print(f"filament_type={data['filament_type']}")
        print(f"detailed_type={data['detailed_type']}")
        print(f"color_rgba=#{rgba.hex().upper()}")
        print(f"drying_temp_c={data['drying_temp_c']}")
        print(f"drying_time_h={data['drying_time_h']}")
        print(f"nozzle_min_c={data['nozzle_min_c']}")
        print(f"nozzle_max_c={data['nozzle_max_c']}")
        print(f"production_time={data['production_time']}")
    elif status == ST_NO_TAG:
        pn_status = data[0] if data else 0xFF
        print(f"status=NO_TAG pn_status=0x{pn_status:02X}")
    else:
        print(f"status={status_name(status)}")


def cmd_motor(cli: AppClient, args):
    action = {
        "stop": 0x00,
        "forward": 0x01,
        "reverse": 0x02,
    }[args.action]
    status = cli.motor_ctrl(args.channel, action)
    print(f"status={status_name(status)} channel={args.channel} action={args.action}")


def main():
    ap = argparse.ArgumentParser(description="CH32V006 app RS485 tool")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--timeout", type=float, default=0.2)
    ap.add_argument("--debug", action="store_true")

    sp = ap.add_subparsers(dest="cmd", required=True)
    sp.add_parser("ping")
    sp.add_parser("version")
    sp.add_parser("enter-boot")
    sp.add_parser("get-switch-status")
    p_uid = sp.add_parser("read-uid")
    p_uid.add_argument("antenna", type=int)
    p_fil = sp.add_parser("read-filament")
    p_fil.add_argument("antenna", type=int)
    p_motor = sp.add_parser("motor")
    p_motor.add_argument("channel", type=int, choices=[1, 2, 3, 4])
    p_motor.add_argument("action", choices=["stop", "forward", "reverse"])

    args = ap.parse_args()

    cli = AppClient(args.port, args.baud, args.timeout, args.debug)
    try:
        if args.cmd == "ping":
            cmd_ping(cli, args)
        elif args.cmd == "version":
            cmd_version(cli, args)
        elif args.cmd == "enter-boot":
            cmd_enter_boot(cli, args)
        elif args.cmd == "get-switch-status":
            cmd_get_switch_status(cli, args)
        elif args.cmd == "read-uid":
            cmd_read_uid(cli, args)
        elif args.cmd == "read-filament":
            cmd_read_filament(cli, args)
        elif args.cmd == "motor":
            cmd_motor(cli, args)
        else:
            raise RuntimeError("Unknown command")
    finally:
        cli.close()


if __name__ == "__main__":
    main()
