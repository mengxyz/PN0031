#!/usr/bin/env python3
import argparse
import struct
import sys
from dataclasses import dataclass
from datetime import datetime

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial", file=sys.stderr)
    sys.exit(2)


PN_STX = 0xA0
PN_ETX = 0x0D
PN_ADDR = 0x00
PN_FT_CMD = 0x02
PN_FC_READ_CARD = 0x20
PN_FC_READ_BLOCK = 0x22
PN_FC_WRITE_BLOCK = 0x23

KEY_DEFAULT = bytes([0xFF] * 6)
ACCESS_BAMBU = bytes.fromhex("87878769")
ACCESS_REWRITABLE = bytes.fromhex("FF078069")


def xor_checksum(data: bytes) -> int:
    x = 0
    for b in data:
        x ^= b
    return x & 0xFF


def ascii_pad(text: str, size: int) -> bytes:
    raw = text.encode("ascii", errors="ignore")[:size]
    return raw + bytes(size - len(raw))


def u16le(v: int) -> bytes:
    return struct.pack("<H", v & 0xFFFF)


def f32le(v: float) -> bytes:
    return struct.pack("<f", float(v))


def rotr(x: int, n: int) -> int:
    return ((x >> n) | ((x << (32 - n)) & 0xFFFFFFFF)) & 0xFFFFFFFF


K_SHA = [
    0x428A2F98,0x71374491,0xB5C0FBCF,0xE9B5DBA5,0x3956C25B,0x59F111F1,0x923F82A4,0xAB1C5ED5,
    0xD807AA98,0x12835B01,0x243185BE,0x550C7DC3,0x72BE5D74,0x80DEB1FE,0x9BDC06A7,0xC19BF174,
    0xE49B69C1,0xEFBE4786,0x0FC19DC6,0x240CA1CC,0x2DE92C6F,0x4A7484AA,0x5CB0A9DC,0x76F988DA,
    0x983E5152,0xA831C66D,0xB00327C8,0xBF597FC7,0xC6E00BF3,0xD5A79147,0x06CA6351,0x14292967,
    0x27B70A85,0x2E1B2138,0x4D2C6DFC,0x53380D13,0x650A7354,0x766A0ABB,0x81C2C92E,0x92722C85,
    0xA2BFE8A1,0xA81A664B,0xC24B8B70,0xC76C51A3,0xD192E819,0xD6990624,0xF40E3585,0x106AA070,
    0x19A4C116,0x1E376C08,0x2748774C,0x34B0BCB5,0x391C0CB3,0x4ED8AA4A,0x5B9CCA4F,0x682E6FF3,
    0x748F82EE,0x78A5636F,0x84C87814,0x8CC70208,0x90BEFFFA,0xA4506CEB,0xBEF9A3F7,0xC67178F2,
]


def sha256(msg: bytes) -> bytes:
    h = [
        0x6A09E667, 0xBB67AE85, 0x3C6EF372, 0xA54FF53A,
        0x510E527F, 0x9B05688C, 0x1F83D9AB, 0x5BE0CD19,
    ]
    data = bytearray(msg)
    bit_len = len(data) * 8
    data.append(0x80)
    while (len(data) % 64) != 56:
        data.append(0x00)
    data.extend(bit_len.to_bytes(8, "big"))

    for chunk_off in range(0, len(data), 64):
        chunk = data[chunk_off:chunk_off + 64]
        w = [0] * 64
        for i in range(16):
            w[i] = int.from_bytes(chunk[i * 4:(i + 1) * 4], "big")
        for i in range(16, 64):
            s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3)
            s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10)
            w[i] = (w[i - 16] + s0 + w[i - 7] + s1) & 0xFFFFFFFF

        a, b, c, d, e, f, g, hh = h
        for i in range(64):
            s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25)
            ch = (e & f) ^ ((~e) & g)
            t1 = (hh + s1 + ch + K_SHA[i] + w[i]) & 0xFFFFFFFF
            s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22)
            maj = (a & b) ^ (a & c) ^ (b & c)
            t2 = (s0 + maj) & 0xFFFFFFFF
            hh = g
            g = f
            f = e
            e = (d + t1) & 0xFFFFFFFF
            d = c
            c = b
            b = a
            a = (t1 + t2) & 0xFFFFFFFF

        h = [
            (h[0] + a) & 0xFFFFFFFF,
            (h[1] + b) & 0xFFFFFFFF,
            (h[2] + c) & 0xFFFFFFFF,
            (h[3] + d) & 0xFFFFFFFF,
            (h[4] + e) & 0xFFFFFFFF,
            (h[5] + f) & 0xFFFFFFFF,
            (h[6] + g) & 0xFFFFFFFF,
            (h[7] + hh) & 0xFFFFFFFF,
        ]

    return b"".join(v.to_bytes(4, "big") for v in h)


def hmac_sha256(key: bytes, msg: bytes) -> bytes:
    if len(key) > 64:
        key = sha256(key)
    key = key + bytes(64 - len(key))
    kipad = bytes((b ^ 0x36) for b in key)
    kopad = bytes((b ^ 0x5C) for b in key)
    return sha256(kopad + sha256(kipad + msg))


def hkdf_expand_sha256(prk: bytes, info: bytes, out_len: int) -> bytes:
    t = b""
    okm = b""
    ctr = 1
    while len(okm) < out_len:
        t = hmac_sha256(prk, t + info + bytes([ctr]))
        okm += t
        ctr += 1
    return okm[:out_len]


def derive_sector_keys(uid: bytes):
    salt = bytes.fromhex("9A759CF2C4F7CAFF222CB9769B41BC96")
    prk = hmac_sha256(salt, uid)
    okm_a = hkdf_expand_sha256(prk, b"RFID-A\x00", 16 * 6)
    okm_b = hkdf_expand_sha256(prk, b"RFID-B\x00", 16 * 6)
    keys_a = [okm_a[i * 6:(i + 1) * 6] for i in range(16)]
    keys_b = [okm_b[i * 6:(i + 1) * 6] for i in range(16)]
    return keys_a, keys_b


@dataclass
class MockFilament:
    variant: str = "A00-G1"
    material_id: str = "GFA00"
    filament_type: str = "PLA"
    detailed_type: str = "PLA Basic"
    color_rgba: str = "00AE42FF"
    spool_weight_g: int = 250
    diameter_mm: float = 1.75
    drying_temp_c: int = 55
    drying_time_h: int = 8
    bed_temp_type: int = 0
    bed_temp_c: int = 0
    nozzle_max_c: int = 230
    nozzle_min_c: int = 190
    production_time: str = "2025_04_14_12_47"

    def blocks(self) -> dict[int, bytes]:
        color = bytes.fromhex(self.color_rgba)
        out = {}
        out[1] = ascii_pad(self.variant, 8) + ascii_pad(self.material_id, 8)
        out[2] = ascii_pad(self.filament_type, 16)
        out[4] = ascii_pad(self.detailed_type, 16)
        out[5] = color + u16le(self.spool_weight_g) + b"\x00\x00" + f32le(self.diameter_mm) + b"\x00\x00\x00\x00"
        out[6] = (
            u16le(self.drying_temp_c) +
            u16le(self.drying_time_h) +
            u16le(self.bed_temp_type) +
            u16le(self.bed_temp_c) +
            u16le(self.nozzle_max_c) +
            u16le(self.nozzle_min_c) +
            b"\x00\x00\x00\x00"
        )
        out[12] = ascii_pad(self.production_time, 16)
        return out


class PN0031:
    def __init__(self, port: str, baud: int, timeout: float, debug: bool = False):
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        self.debug = debug

    def close(self):
        self.ser.close()

    def _send(self, fc: int, data: bytes) -> None:
        frame = bytes([PN_STX, PN_ADDR, PN_FT_CMD, fc]) + struct.pack("<H", len(data)) + data
        frame += bytes([xor_checksum(frame), PN_ETX])
        if self.debug:
            print("TX", frame.hex().upper())
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()

    def _recv(self, expect_fc: int) -> bytes:
        while True:
            b = self.ser.read(1)
            if not b:
                raise TimeoutError("Timeout waiting STX")
            if b[0] != PN_STX:
                continue
            hdr = self.ser.read(5)
            if len(hdr) != 5:
                raise TimeoutError("Short header")
            ad, ft, fc, ln_lo, ln_hi = hdr
            ln = ln_lo | (ln_hi << 8)
            payload = self.ser.read(ln)
            tail = self.ser.read(2)
            if len(payload) != ln or len(tail) != 2:
                raise TimeoutError("Short payload")
            bcc, etx = tail
            frame_wo_tail = bytes([PN_STX]) + hdr + payload
            if etx != PN_ETX:
                raise RuntimeError("Bad ETX")
            if xor_checksum(frame_wo_tail) != bcc:
                raise RuntimeError("Bad BCC")
            if ft != PN_FT_CMD or fc != expect_fc:
                raise RuntimeError(f"Unexpected FT/FC 0x{ft:02X}/0x{fc:02X}")
            frame = bytes([PN_STX]) + hdr + payload + tail
            if self.debug:
                print("RX", frame.hex().upper())
            return payload

    def read_uid(self, antenna: int) -> bytes:
        self._send(PN_FC_READ_CARD, bytes([antenna]))
        payload = self._recv(PN_FC_READ_CARD)
        if len(payload) < 1:
            raise RuntimeError("Short READ_CARD response")
        status = payload[0]
        if status != 0x00:
            raise RuntimeError(f"READ_CARD status=0x{status:02X}")
        if len(payload) < 3:
            raise RuntimeError("Short READ_CARD OK response")
        uid_len = payload[2]
        uid = payload[3:3 + uid_len]
        if len(uid) != uid_len:
            raise RuntimeError("Short UID")
        return uid

    def read_block(self, antenna: int, block: int, key_type: int, key: bytes) -> bytes:
        req = bytes([antenna, block, 0x00, key_type]) + key
        self._send(PN_FC_READ_BLOCK, req)
        payload = self._recv(PN_FC_READ_BLOCK)
        if len(payload) < 1:
            raise RuntimeError("Short READ_BLOCK response")
        status = payload[0]
        if status != 0x00:
            raise RuntimeError(f"READ_BLOCK block={block} status=0x{status:02X}")
        if len(payload) != 19:
            raise RuntimeError(f"READ_BLOCK block={block} bad len={len(payload)}")
        return payload[3:19]

    def write_block(self, antenna: int, block: int, key_type: int, key: bytes, data16: bytes) -> None:
        if len(key) != 6 or len(data16) != 16:
            raise ValueError("key must be 6 bytes and data must be 16 bytes")
        req = bytes([antenna, block, 0x00, key_type]) + key + data16
        self._send(PN_FC_WRITE_BLOCK, req)
        payload = self._recv(PN_FC_WRITE_BLOCK)
        if len(payload) < 1:
            raise RuntimeError("Short WRITE_BLOCK response")
        status = payload[0]
        if status != 0x00:
            raise RuntimeError(f"WRITE_BLOCK block={block} status=0x{status:02X}")


def read_block_fallback(dev: PN0031, antenna: int, block: int, key_a: bytes, key_b: bytes) -> bytes:
    last_err = None
    for key_type, key in (
        (0, KEY_DEFAULT),
        (1, KEY_DEFAULT),
        (0, key_a),
        (1, key_b),
    ):
        try:
            return dev.read_block(antenna, block, key_type, key)
        except Exception as e:
            last_err = e
    raise RuntimeError(f"READ fallback failed block={block}: {last_err}")


def write_block_fallback(dev: PN0031, antenna: int, block: int, key_a: bytes, key_b: bytes, data16: bytes) -> None:
    last_err = None
    for key_type, key in (
        (0, KEY_DEFAULT),
        (1, KEY_DEFAULT),
        (0, key_a),
        (1, key_b),
    ):
        try:
            dev.write_block(antenna, block, key_type, key, data16)
            return
        except Exception as e:
            last_err = e
    raise RuntimeError(f"WRITE fallback failed block={block}: {last_err}")


def probe_block_auth(dev: PN0031, antenna: int, block: int, key_a: bytes, key_b: bytes):
    attempts = (
        ("default-A", 0, KEY_DEFAULT),
        ("default-B", 1, KEY_DEFAULT),
        ("derived-A", 0, key_a),
        ("derived-B", 1, key_b),
    )
    results = []
    for name, key_type, key in attempts:
        try:
            data = dev.read_block(antenna, block, key_type, key)
            results.append((name, True, data.hex().upper()))
        except Exception as e:
            results.append((name, False, str(e)))
    return results


def build_sector_trailer(key_a: bytes, access_bits: bytes, key_b: bytes) -> bytes:
    return key_a + access_bits + key_b


def write_mock_tag(dev: PN0031, antenna: int, mock: MockFilament, verify: bool, access_bits: bytes) -> None:
    uid = dev.read_uid(antenna)
    keys_a, keys_b = derive_sector_keys(uid)
    blocks = mock.blocks()

    print(f"uid={uid.hex().upper()}")
    print("probing sector-0 block-1 auth...")
    for name, ok, detail in probe_block_auth(dev, antenna, 1, keys_a[0], keys_b[0]):
        print(f"auth {name}: {'OK' if ok else 'FAIL'} {detail}")
    print("writing data blocks...")

    sectors_with_data = {}
    for block, data in blocks.items():
        sec = block // 4
        sectors_with_data.setdefault(sec, []).append((block, data))

    for sec, items in sorted(sectors_with_data.items()):
        for block, data in sorted(items):
            write_block_fallback(dev, antenna, block, keys_a[sec], keys_b[sec], data)
            print(f"write block={block:02d} ok")
        trailer_block = sec * 4 + 3
        trailer = build_sector_trailer(keys_a[sec], access_bits, keys_b[sec])
        write_block_fallback(dev, antenna, trailer_block, keys_a[sec], keys_b[sec], trailer)
        print(f"write trailer sector={sec:02d} block={trailer_block:02d} ok")

    print("writing remaining trailers...")
    for sec in range(16):
        if sec in sectors_with_data:
            continue
        trailer_block = sec * 4 + 3
        trailer = build_sector_trailer(keys_a[sec], access_bits, keys_b[sec])
        write_block_fallback(dev, antenna, trailer_block, keys_a[sec], keys_b[sec], trailer)
        print(f"write trailer sector={sec:02d} block={trailer_block:02d} ok")

    if verify:
        print("verifying...")
        for block, data in sorted(blocks.items()):
            sec = block // 4
            got = read_block_fallback(dev, antenna, block, keys_a[sec], keys_b[sec])
            if got != data:
                raise RuntimeError(
                    f"verify failed block={block:02d} expect={data.hex().upper()} got={got.hex().upper()}"
                )
            print(f"verify block={block:02d} ok")
        for sec in range(16):
            trailer_block = sec * 4 + 3
            got = read_block_fallback(dev, antenna, trailer_block, keys_a[sec], keys_b[sec])
            if got[6:10] != access_bits:
                raise RuntimeError(
                    f"verify trailer failed sector={sec:02d} access expect={access_bits.hex().upper()} got={got.hex().upper()}"
                )
            print(f"verify trailer sector={sec:02d} ok")


def main() -> None:
    ap = argparse.ArgumentParser(description="Write mock Bambu-style filament data to a MIFARE Classic tag via PN0031 UART")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=0.5)
    ap.add_argument("--debug", action="store_true")
    ap.add_argument("--antenna", type=int, default=1)
    ap.add_argument("--no-verify", action="store_true")

    ap.add_argument("--variant", default="A00-G1")
    ap.add_argument("--material-id", default="GFA00")
    ap.add_argument("--filament-type", default="PLA")
    ap.add_argument("--detailed-type", default="PLA Basic")
    ap.add_argument("--color-rgba", default="00AE42FF")
    ap.add_argument("--spool-weight-g", type=int, default=250)
    ap.add_argument("--diameter-mm", type=float, default=1.75)
    ap.add_argument("--drying-temp-c", type=int, default=55)
    ap.add_argument("--drying-time-h", type=int, default=8)
    ap.add_argument("--bed-temp-type", type=int, default=0)
    ap.add_argument("--bed-temp-c", type=int, default=0)
    ap.add_argument("--nozzle-max-c", type=int, default=230)
    ap.add_argument("--nozzle-min-c", type=int, default=190)
    ap.add_argument("--production-time", default=datetime.now().strftime("%Y_%m_%d_%H_%M"))
    ap.add_argument("--rewritable", action="store_true", help="use rewritable trailer access bits FF078069 instead of Bambu 87878769")

    args = ap.parse_args()
    mock = MockFilament(
        variant=args.variant,
        material_id=args.material_id,
        filament_type=args.filament_type,
        detailed_type=args.detailed_type,
        color_rgba=args.color_rgba,
        spool_weight_g=args.spool_weight_g,
        diameter_mm=args.diameter_mm,
        drying_temp_c=args.drying_temp_c,
        drying_time_h=args.drying_time_h,
        bed_temp_type=args.bed_temp_type,
        bed_temp_c=args.bed_temp_c,
        nozzle_max_c=args.nozzle_max_c,
        nozzle_min_c=args.nozzle_min_c,
        production_time=args.production_time,
    )

    access_bits = ACCESS_REWRITABLE if args.rewritable else ACCESS_BAMBU
    print(f"access_bits={access_bits.hex().upper()}")
    dev = PN0031(args.port, args.baud, args.timeout, args.debug)
    try:
        write_mock_tag(dev, args.antenna, mock, verify=not args.no_verify, access_bits=access_bits)
    finally:
        dev.close()


if __name__ == "__main__":
    main()
