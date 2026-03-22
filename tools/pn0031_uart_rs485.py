#!/usr/bin/env python3
"""
PN0031 UART direct test tool.

Requires:
    pip install pyserial
"""

import argparse
import hashlib
import hmac
import os
import struct
import sys
import time
from typing import List, Tuple

import serial

STX = 0xA0
ETX = 0x0D
FT_COMMAND = 0x02
FT_SET_ADDRESS = 0x00

FC_SET_ADDRESS = 0x00
FC_READ_CARD = 0x20
FC_READ_ALL = 0x21
FC_READ_BLOCK = 0x22
FC_WRITE_BLOCK = 0x23


def hkdf_sha256(ikm: bytes, salt: bytes, info: bytes, length: int) -> bytes:
    if length <= 0:
        return b""
    prk = hmac.new(salt, ikm, hashlib.sha256).digest()
    out = bytearray()
    t = b""
    counter = 1
    while len(out) < length:
        t = hmac.new(prk, t + info + bytes([counter]), hashlib.sha256).digest()
        out.extend(t)
        counter += 1
    return bytes(out[:length])


def derive_bambu_keys(uid: bytes, info: bytes) -> List[bytes]:
    # Matches RFID-Tag-Guide deriveKeys.py / Android app (HKDF + info).
    master = bytes(
        [0x9A, 0x75, 0x9C, 0xF2, 0xC4, 0xF7, 0xCA, 0xFF, 0x22, 0x2C, 0xB9, 0x76, 0x9B, 0x41, 0xBC, 0x96]
    )
    raw = hkdf_sha256(uid, master, info, 16 * 6)
    return [raw[i * 6 : (i + 1) * 6] for i in range(16)]


def xor_checksum(data: bytes) -> int:
    out = 0
    for b in data:
        out ^= b
    return out


def build_frame(address: int, ft: int, fc: int, data: bytes) -> bytes:
    payload = bytes(
        [STX, address & 0xFF, ft & 0xFF, fc & 0xFF, len(data) & 0xFF, (len(data) >> 8) & 0xFF]
    ) + data
    bcc = xor_checksum(payload)
    return payload + bytes([bcc, ETX])


def read_exact(ser: serial.Serial, count: int, timeout_s: float) -> bytes:
    deadline = time.time() + timeout_s
    out = bytearray()
    while len(out) < count:
        remaining = max(0.0, deadline - time.time())
        if remaining <= 0:
            break
        ser.timeout = remaining
        chunk = ser.read(count - len(out))
        if not chunk:
            continue
        out.extend(chunk)
    return bytes(out)


def read_frame(ser: serial.Serial, timeout_s: float) -> Tuple[int, int, int, bytes]:
    deadline = time.time() + timeout_s

    while True:
        remaining = max(0.0, deadline - time.time())
        if remaining <= 0:
            raise TimeoutError("Timeout waiting STX")
        ser.timeout = remaining
        b = ser.read(1)
        if not b:
            continue
        if b[0] == STX:
            break

    header = read_exact(ser, 5, max(0.0, deadline - time.time()))
    if len(header) != 5:
        raise TimeoutError("Timeout reading header")

    addr, ft, fc, len_lo, len_hi = header
    data_len = len_lo | (len_hi << 8)
    data = read_exact(ser, data_len, max(0.0, deadline - time.time()))
    if len(data) != data_len:
        raise TimeoutError("Timeout reading data")

    tail = read_exact(ser, 2, max(0.0, deadline - time.time()))
    if len(tail) != 2:
        raise TimeoutError("Timeout reading tail")
    rx_bcc, rx_etx = tail

    if rx_etx != ETX:
        raise ValueError(f"Bad ETX: 0x{rx_etx:02X}")

    check = bytes([STX]) + header + data
    expected_bcc = xor_checksum(check)
    if rx_bcc != expected_bcc:
        raise ValueError(f"Bad BCC: got 0x{rx_bcc:02X}, expected 0x{expected_bcc:02X}")

    return addr, ft, fc, data


def parse_hex_bytes(hex_string: str) -> bytes:
    cleaned = hex_string.replace(" ", "").replace(":", "").replace("-", "")
    if len(cleaned) % 2 != 0:
        raise ValueError("Hex string length must be even")
    return bytes.fromhex(cleaned)


def bytes_to_hex(data: bytes) -> str:
    return "".join(f"{b:02X}" for b in data)


def ascii_or_hex(data: bytes) -> str:
    if not data:
        return ""
    if all((b == 0) or (32 <= b <= 126) for b in data):
        return data.rstrip(b"\x00").decode("ascii", errors="ignore")
    return bytes_to_hex(data)


def to_u16le(data: bytes, offset: int) -> int:
    if offset + 2 > len(data):
        return 0
    return data[offset] | (data[offset + 1] << 8)


def to_f32le(data: bytes, offset: int) -> float:
    if offset + 4 > len(data):
        return 0.0
    return struct.unpack("<f", data[offset : offset + 4])[0]


def parse_bambu_summary(raw_blocks: List[bytes]) -> List[str]:
    lines: List[str] = []
    b1 = raw_blocks[1]
    b2 = raw_blocks[2]
    b4 = raw_blocks[4]
    b5 = raw_blocks[5]
    b6 = raw_blocks[6]
    b9 = raw_blocks[9]
    b12 = raw_blocks[12]

    variant = ascii_or_hex(b1[0:8])
    material_id = ascii_or_hex(b1[8:16])
    filament_type = ascii_or_hex(b2[0:16])
    detailed_type = ascii_or_hex(b4[0:16])
    color_rgba = "#" + bytes_to_hex(b5[0:4])
    spool_weight_g = to_u16le(b5, 4)
    diameter = to_f32le(b5, 8)
    drying_temp_c = to_u16le(b6, 0)
    drying_hour = to_u16le(b6, 2)
    bed_temp_c = to_u16le(b6, 6)
    nozzle_min_c = to_u16le(b6, 8)
    nozzle_max_c = to_u16le(b6, 10)
    tray_uid = bytes_to_hex(b9[0:16])
    prod_time = ascii_or_hex(b12[0:16])

    lines.append(f"variant={variant}")
    lines.append(f"material_id={material_id}")
    lines.append(f"filament_type={filament_type}")
    lines.append(f"detailed_type={detailed_type}")
    lines.append(f"color_rgba={color_rgba}")
    lines.append(f"spool_weight_g={spool_weight_g}")
    lines.append(f"diameter_mm={diameter:.3f}")
    lines.append(f"drying_temp_c={drying_temp_c}")
    lines.append(f"drying_time_h={drying_hour}")
    lines.append(f"bed_temp_c={bed_temp_c}")
    lines.append(f"nozzle_min_c={nozzle_min_c}")
    lines.append(f"nozzle_max_c={nozzle_max_c}")
    lines.append(f"tray_uid_block9={tray_uid}")
    lines.append(f"production_time={prod_time}")
    return lines


def load_keys_file(path: str) -> List[Tuple[bytes, bytes]]:
    keys: List[Tuple[bytes, bytes]] = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            toks = [t.replace(":", "").replace("-", "") for t in s.split() if t]
            try:
                if len(toks) >= 2 and len(toks[0]) == 12 and len(toks[1]) == 12:
                    ka = bytes.fromhex(toks[0])
                    kb = bytes.fromhex(toks[1])
                    keys.append((ka, kb))
                elif len(toks) >= 1 and len(toks[0]) == 12:
                    # Backward compatible: single key means same A/B.
                    k = bytes.fromhex(toks[0])
                    keys.append((k, k))
            except ValueError:
                pass
    if not keys:
        raise ValueError(f"No valid keys found in {path} (expect KEYA KEYB or KEY)")
    return keys


def transact(
    ser: serial.Serial,
    address: int,
    ft: int,
    fc: int,
    req_data: bytes,
    timeout_s: float,
    tx_via_rts: bool,
    rts_active_high: bool,
    guard_us: int,
    debug: bool,
) -> Tuple[int, int, int, bytes]:
    frame = build_frame(address, ft, fc, req_data)
    ser.reset_input_buffer()

    if debug:
        print(f"TX: {bytes_to_hex(frame)}")

    if tx_via_rts:
        ser.rts = bool(rts_active_high)
        if guard_us > 0:
            time.sleep(guard_us / 1_000_000.0)

    ser.write(frame)
    ser.flush()

    if tx_via_rts:
        if guard_us > 0:
            time.sleep(guard_us / 1_000_000.0)
        ser.rts = not bool(rts_active_high)

    resp_addr, resp_ft, resp_fc, resp_data = read_frame(ser, timeout_s)
    if debug:
        print(
            "RX: %s"
            % bytes_to_hex(
                build_frame(resp_addr, resp_ft, resp_fc, resp_data)
            )
        )
    if resp_ft != ft or resp_fc != fc:
        raise ValueError(
            f"Unexpected response: ft=0x{resp_ft:02X}, fc=0x{resp_fc:02X}, expected ft=0x{ft:02X}, fc=0x{fc:02X}"
        )

    if len(resp_data) < 1:
        raise ValueError("Response data too short")

    return resp_addr, resp_ft, resp_fc, resp_data


def decode_status(resp_data: bytes) -> int:
    return resp_data[0]


def cmd_set_address(args: argparse.Namespace, ser: serial.Serial) -> int:
    _, _, _, data = transact(
        ser,
        args.address,
        FT_SET_ADDRESS,
        FC_SET_ADDRESS,
        bytes([args.new_address & 0xFF]),
        args.timeout_ms / 1000.0,
        args.tx_via_rts,
        args.rts_active_high,
        args.guard_us,
        args.debug,
    )
    status = decode_status(data)
    print(f"status=0x{status:02X}")
    return 0 if status == 0 else 2


def cmd_read_card(args: argparse.Namespace, ser: serial.Serial) -> int:
    _, _, _, data = transact(
        ser,
        args.address,
        FT_COMMAND,
        FC_READ_CARD,
        bytes([args.antenna & 0xFF]),
        args.timeout_ms / 1000.0,
        args.tx_via_rts,
        args.rts_active_high,
        args.guard_us,
        args.debug,
    )

    status = decode_status(data)
    if status != 0:
        print(f"status=0x{status:02X}")
        return 2

    if len(data) < 3:
        raise ValueError("Response too short for read-card")

    antenna = data[1]
    uid_len = data[2]
    uid = data[3 : 3 + uid_len]
    if len(uid) != uid_len:
        raise ValueError("UID length mismatch")

    print(f"status=0x{status:02X} antenna={antenna} uid_len={uid_len} uid={bytes_to_hex(uid)}")
    return 0


def cmd_read_all(args: argparse.Namespace, ser: serial.Serial) -> int:
    _, _, _, data = transact(
        ser,
        args.address,
        FT_COMMAND,
        FC_READ_ALL,
        b"",
        args.timeout_ms / 1000.0,
        args.tx_via_rts,
        args.rts_active_high,
        args.guard_us,
        args.debug,
    )

    status = decode_status(data)
    if status != 0:
        print(f"status=0x{status:02X}")
        return 2

    idx = 1
    cards: List[Tuple[int, bytes]] = []
    while idx < len(data):
        if idx + 2 > len(data):
            raise ValueError("Malformed read-all response")
        antenna = data[idx]
        uid_len = data[idx + 1]
        idx += 2
        uid = data[idx : idx + uid_len]
        if len(uid) != uid_len:
            raise ValueError("Malformed UID in read-all response")
        idx += uid_len
        cards.append((antenna, uid))

    print(f"status=0x{status:02X} cards={len(cards)}")
    for i, (antenna, uid) in enumerate(cards, start=1):
        print(f"  {i}. antenna={antenna} uid={bytes_to_hex(uid)}")
    return 0


def cmd_read_block(args: argparse.Namespace, ser: serial.Serial) -> int:
    key = parse_hex_bytes(args.key_hex)
    if len(key) != 6:
        raise ValueError("key_hex must be exactly 6 bytes (12 hex chars)")

    req = bytes(
        [
            args.antenna & 0xFF,
            args.block & 0xFF,
            args.key_pos & 0xFF,
            args.key_type & 0xFF,
        ]
    ) + key

    _, _, _, data = transact(
        ser,
        args.address,
        FT_COMMAND,
        FC_READ_BLOCK,
        req,
        args.timeout_ms / 1000.0,
        args.tx_via_rts,
        args.rts_active_high,
        args.guard_us,
        args.debug,
    )

    status = decode_status(data)
    if status != 0:
        print(f"status=0x{status:02X}")
        return 2
    if len(data) != 19:
        raise ValueError(f"Unexpected read-block response length: {len(data)}")

    print(
        "status=0x%02X antenna=%d block=%d data=%s"
        % (status, data[1], data[2], bytes_to_hex(data[3:19]))
    )
    return 0


def cmd_write_block(args: argparse.Namespace, ser: serial.Serial) -> int:
    key = parse_hex_bytes(args.key_hex)
    payload = parse_hex_bytes(args.data_hex)
    if len(key) != 6:
        raise ValueError("key_hex must be exactly 6 bytes (12 hex chars)")
    if len(payload) != 16:
        raise ValueError("data_hex must be exactly 16 bytes (32 hex chars)")

    req = bytes(
        [
            args.antenna & 0xFF,
            args.block & 0xFF,
            args.key_pos & 0xFF,
            args.key_type & 0xFF,
        ]
    ) + key + payload

    _, _, _, data = transact(
        ser,
        args.address,
        FT_COMMAND,
        FC_WRITE_BLOCK,
        req,
        args.timeout_ms / 1000.0,
        args.tx_via_rts,
        args.rts_active_high,
        args.guard_us,
        args.debug,
    )

    status = decode_status(data)
    if status != 0:
        print(f"status=0x{status:02X}")
        return 2
    if len(data) != 3:
        raise ValueError(f"Unexpected write-block response length: {len(data)}")

    print(f"status=0x{status:02X} antenna={data[1]} block={data[2]}")
    return 0


def cmd_dump_mifare(args: argparse.Namespace, ser: serial.Serial) -> int:
    if args.keys_file:
        keys_ab = load_keys_file(args.keys_file)
    elif args.key_hex:
        one = parse_hex_bytes(args.key_hex)
        if len(one) != 6:
            raise ValueError("key_hex must be exactly 6 bytes (12 hex chars)")
        keys_ab = [(one, one)] * 16
    else:
        raise ValueError("Provide --keys-file or --key-hex")

    if len(keys_ab) < 16:
        # Repeat keys if file has fewer entries.
        rep = (16 + len(keys_ab) - 1) // len(keys_ab)
        keys_ab = (keys_ab * rep)[:16]

    def do_transact(fc: int, req: bytes, ft: int = FT_COMMAND):
        if args.reconnect_per_block:
            with serial.Serial(args.port, args.baud, timeout=args.timeout_ms / 1000.0) as s2:
                if args.tx_via_rts:
                    s2.rts = not bool(args.rts_active_high)
                return transact(
                    s2,
                    args.address,
                    ft,
                    fc,
                    req,
                    args.timeout_ms / 1000.0,
                    args.tx_via_rts,
                    args.rts_active_high,
                    args.guard_us,
                    args.debug,
                )
        return transact(
            ser,
            args.address,
            ft,
            fc,
            req,
            args.timeout_ms / 1000.0,
            args.tx_via_rts,
            args.rts_active_high,
            args.guard_us,
            args.debug,
        )

    key_types = [0]
    if args.try_keyb:
        key_types = [0, 1]

    # MIFARE Classic 1K: 16 sectors, 4 blocks/sector.
    dump_rows: List[str] = []
    for sector in range(16):
        key_a, key_b = keys_ab[sector]
        for block_in_sector in range(4):
            block = sector * 4 + block_in_sector
            success = False
            last_status = 0xFF
            for key_type in key_types:
                key = key_a if key_type == 0 else key_b
                req = bytes([args.antenna & 0xFF, block & 0xFF, 0x00, key_type & 0xFF]) + key
                last_exc = None
                for attempt in range(max(1, args.retries)):
                    try:
                        _, _, _, data = do_transact(FC_READ_BLOCK, req, FT_COMMAND)
                        status = decode_status(data)
                        last_status = status
                        if status == 0:
                            row = data[3:19]
                            line = f"{block:02d}:{bytes_to_hex(row)} keyType={key_type}"
                            dump_rows.append(line)
                            print(line)
                            success = True
                        break
                    except Exception as exc:
                        last_exc = exc
                        if attempt + 1 < max(1, args.retries):
                            time.sleep(max(0, args.inter_cmd_ms) / 1000.0)
                            continue
                if success:
                    break
                if last_exc is not None:
                    print(
                        f"sector={sector:02d} block={block:02d} error={str(last_exc)} "
                        f"key={bytes_to_hex(key)} keyType={key_type}"
                    )

            if not success:
                keytype_note = "A|B" if args.try_keyb else "A"
                print(
                    f"sector={sector:02d} block={block:02d} status=0x{last_status:02X} "
                    f"keyA={bytes_to_hex(key_a)} keyB={bytes_to_hex(key_b)} keyType={keytype_note}"
                )
            if args.inter_cmd_ms > 0:
                time.sleep(args.inter_cmd_ms / 1000.0)

    if args.out and dump_rows:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write("\n".join(dump_rows) + "\n")
        print(f"saved={os.path.abspath(args.out)}")

    return 0


def cmd_derive_bambu_keys(args: argparse.Namespace, _ser: serial.Serial) -> int:
    uid = parse_hex_bytes(args.uid_hex)
    if len(uid) != 4:
        raise ValueError("uid_hex must be exactly 4 bytes (8 hex chars)")

    keys_a = derive_bambu_keys(uid, b"RFID-A\x00")
    keys_b = derive_bambu_keys(uid, b"RFID-B\x00")
    for ka, kb in zip(keys_a, keys_b):
        print(f"{bytes_to_hex(ka)} {bytes_to_hex(kb)}")

    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            for ka, kb in zip(keys_a, keys_b):
                f.write(f"{bytes_to_hex(ka)} {bytes_to_hex(kb)}\n")
        print(f"saved={os.path.abspath(args.out)}", file=sys.stderr)
    return 0


def cmd_read_bambu(args: argparse.Namespace, ser: serial.Serial) -> int:
    # 1) Detect card and UID on selected antenna.
    _, _, _, data = transact(
        ser,
        args.address,
        FT_COMMAND,
        FC_READ_CARD,
        bytes([args.antenna & 0xFF]),
        args.timeout_ms / 1000.0,
        args.tx_via_rts,
        args.rts_active_high,
        args.guard_us,
        args.debug,
    )
    status = decode_status(data)
    if status != 0:
        print(f"ERROR: tag not found on antenna {args.antenna} (status=0x{status:02X})")
        return 2
    if len(data) < 3:
        raise ValueError("read-card response too short")

    uid_len = data[2]
    uid = data[3 : 3 + uid_len]
    if len(uid) != uid_len:
        raise ValueError("UID length mismatch")
    uid_hex = bytes_to_hex(uid)
    print(f"uid={uid_hex} antenna={data[1]} uid_len={uid_len}")

    # 2) Derive sector keys in memory.
    keys_a = derive_bambu_keys(uid, b"RFID-A\x00")
    keys_b = derive_bambu_keys(uid, b"RFID-B\x00")
    keys_ab = list(zip(keys_a, keys_b))

    # 3) Dump all blocks.
    raw_blocks: List[bytes] = [bytes([0] * 16) for _ in range(64)]

    def do_transact(fc: int, req: bytes, ft: int = FT_COMMAND):
        if args.reconnect_per_block:
            with serial.Serial(args.port, args.baud, timeout=args.timeout_ms / 1000.0) as s2:
                if args.tx_via_rts:
                    s2.rts = not bool(args.rts_active_high)
                return transact(
                    s2,
                    args.address,
                    ft,
                    fc,
                    req,
                    args.timeout_ms / 1000.0,
                    args.tx_via_rts,
                    args.rts_active_high,
                    args.guard_us,
                    args.debug,
                )
        return transact(
            ser,
            args.address,
            ft,
            fc,
            req,
            args.timeout_ms / 1000.0,
            args.tx_via_rts,
            args.rts_active_high,
            args.guard_us,
            args.debug,
        )

    failed = 0
    out_rows: List[str] = []
    key_types = [0, 1]

    for sector in range(16):
        key_a, key_b = keys_ab[sector]
        for block_in_sector in range(4):
            block = sector * 4 + block_in_sector
            success = False
            last_status = 0xFF
            for key_type in key_types:
                key = key_a if key_type == 0 else key_b
                req = bytes([args.antenna & 0xFF, block & 0xFF, 0x00, key_type]) + key
                for _ in range(max(1, args.retries)):
                    try:
                        _, _, _, rb = do_transact(FC_READ_BLOCK, req, FT_COMMAND)
                        st = decode_status(rb)
                        last_status = st
                        if st == 0 and len(rb) == 19:
                            payload = rb[3:19]
                            raw_blocks[block] = payload
                            line = f"{block:02d}:{bytes_to_hex(payload)} keyType={key_type}"
                            out_rows.append(line)
                            print(line)
                            success = True
                        break
                    except Exception:
                        continue
                if success:
                    break

            if not success:
                failed += 1
                print(
                    f"sector={sector:02d} block={block:02d} status=0x{last_status:02X} "
                    f"keyType=A|B"
                )
            if args.inter_cmd_ms > 0:
                time.sleep(args.inter_cmd_ms / 1000.0)

    if args.out and out_rows:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write("\n".join(out_rows) + "\n")
        print(f"saved={os.path.abspath(args.out)}")

    # 4) Parsed summary.
    if failed < 64:
        print("---- parsed ----")
        for line in parse_bambu_summary(raw_blocks):
            print(line)
    else:
        print("ERROR: dump failed for all blocks (check tag/key compatibility)")
        return 3

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="PN0031 normal UART protocol direct test tool")
    parser.add_argument("--port", help="Serial port, e.g. /dev/ttyUSB0 or COM3")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate (default: 115200)")
    parser.add_argument("--address", type=lambda x: int(x, 0), default=0x00, help="PN0031 address")
    parser.add_argument("--timeout-ms", type=int, default=200, help="Response timeout in ms")
    parser.add_argument("--debug", action="store_true", help="Print raw TX/RX hex frames")

    # Optional for manual RS485 direction adapters. Auto-direction bridges do not need these.
    parser.add_argument(
        "--tx-via-rts",
        action="store_true",
        help="Use RTS pin for RS485 direction control (advanced)",
    )
    parser.add_argument(
        "--rts-active-high",
        action="store_true",
        help="Set RTS high for TX. If not set, RTS low is TX.",
    )
    parser.add_argument(
        "--guard-us",
        type=int,
        default=120,
        help="Direction guard delay in microseconds before/after TX (default: 120)",
    )

    sub = parser.add_subparsers(dest="cmd", required=True)

    p_set = sub.add_parser("set-address", help="Set module address")
    p_set.add_argument("new_address", type=lambda x: int(x, 0))
    p_set.set_defaults(handler=cmd_set_address)

    p_card = sub.add_parser("read-card", help="Read card from specified antenna")
    p_card.add_argument("antenna", type=lambda x: int(x, 0))
    p_card.set_defaults(handler=cmd_read_card)

    p_all = sub.add_parser("read-all", help="Read cards from all antennas")
    p_all.set_defaults(handler=cmd_read_all)

    p_rb = sub.add_parser("read-block", help="Read block data")
    p_rb.add_argument("antenna", type=lambda x: int(x, 0))
    p_rb.add_argument("block", type=lambda x: int(x, 0))
    p_rb.add_argument("key_pos", type=lambda x: int(x, 0), help="0=transport, 1=EEPROM")
    p_rb.add_argument("key_type", type=lambda x: int(x, 0), help="0=KEYA, 1=KEYB")
    p_rb.add_argument("key_hex", help="6-byte key hex, e.g. FFFFFFFFFFFF")
    p_rb.set_defaults(handler=cmd_read_block)

    p_wb = sub.add_parser("write-block", help="Write block data")
    p_wb.add_argument("antenna", type=lambda x: int(x, 0))
    p_wb.add_argument("block", type=lambda x: int(x, 0))
    p_wb.add_argument("key_pos", type=lambda x: int(x, 0), help="0=transport, 1=EEPROM")
    p_wb.add_argument("key_type", type=lambda x: int(x, 0), help="0=KEYA, 1=KEYB")
    p_wb.add_argument("key_hex", help="6-byte key hex, e.g. FFFFFFFFFFFF")
    p_wb.add_argument("data_hex", help="16-byte data hex, e.g. 001122...EEFF")
    p_wb.set_defaults(handler=cmd_write_block)

    p_dump = sub.add_parser("dump-mifare", help="Dump all 64 blocks (MIFARE Classic 1K)")
    p_dump.add_argument("antenna", type=lambda x: int(x, 0), help="Antenna number 1..4")
    p_dump.add_argument("--keys-file", help="Text file with per-sector keys (from deriveKeys.py)")
    p_dump.add_argument("--key-hex", help="Fallback single 6-byte key for all sectors")
    p_dump.add_argument("--out", help="Output text file for dumped blocks")
    p_dump.add_argument("--try-keyb", action="store_true", help="Try KEYB if KEYA fails")
    p_dump.add_argument("--retries", type=int, default=2, help="Retries per key attempt (default: 2)")
    p_dump.add_argument("--inter-cmd-ms", type=int, default=15, help="Delay between block reads in ms")
    p_dump.add_argument(
        "--reconnect-per-block",
        action="store_true",
        help="Reopen serial port for each block read (slower, but often more stable)",
    )
    p_dump.set_defaults(handler=cmd_dump_mifare)

    p_derive = sub.add_parser("derive-bambu-keys", help="Derive 16 Bambu sector keys from UID")
    p_derive.add_argument("uid_hex", help="Tag UID hex (8 hex chars), e.g. 13CD7E38")
    p_derive.add_argument("--out", help="Write keys to file (keys.dic)")
    p_derive.set_defaults(handler=cmd_derive_bambu_keys)

    p_read_bambu = sub.add_parser(
        "read-bambu",
        help="One-shot: read UID -> derive keys -> dump 64 blocks -> print parsed filament data",
    )
    p_read_bambu.add_argument("antenna", type=lambda x: int(x, 0), help="Antenna number 1..4")
    p_read_bambu.add_argument("--out", help="Output text file for dumped blocks")
    p_read_bambu.add_argument("--retries", type=int, default=2, help="Retries per block read")
    p_read_bambu.add_argument("--inter-cmd-ms", type=int, default=15, help="Delay between block reads")
    p_read_bambu.add_argument(
        "--reconnect-per-block",
        action="store_true",
        help="Reopen serial port for each block read (slower, but often more stable)",
    )
    p_read_bambu.set_defaults(handler=cmd_read_bambu)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    try:
        if args.cmd == "derive-bambu-keys":
            return int(args.handler(args, None))

        if not args.port:
            raise ValueError("--port is required for serial commands")

        with serial.Serial(args.port, args.baud, timeout=args.timeout_ms / 1000.0) as ser:
            if args.tx_via_rts:
                ser.rts = not bool(args.rts_active_high)
            return int(args.handler(args, ser))
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
