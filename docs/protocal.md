# CH32V006 Protocol Reference

Two independent protocols run on the CH32V006F8U6:

1. **RS485 IAP** — bootloader firmware update, `env:rs485_iap`
2. **Bus + PN0031** — application command interface, `env:bus_pn0031_test`

---

## 1. RS485 IAP Bootloader Protocol

Source: `app/ch32_v006_bootloader/src/main_rs485_iap.c`

### MCU Pins

| Pin | Function |
|-----|----------|
| PD6 | USART1 TX (AF, partial remap 1) |
| PD5 | USART1 RX (floating input) |
| PD4 | RS485 DE / direction control (output) |
| PD0 | Status LED (output) |

UART: **115200 8N1**, half-duplex RS485.

### Boot Decision (on power-on / reset)

```
if app present (flash[0x08000000] != 0xFFFFFFFF)
   AND reset was NOT a software reset (SFTRST flag clear)
   → jump immediately to app at 0x08000000
else
   → stay in IAP mode, wait for commands
```

The app triggers reflash by calling `SystemReset_StartMode(BOOT)` + `NVIC_SystemReset()`, which sets the SFTRST flag so the bootloader stays resident on the next boot.

### IAP Request Frame

```
AA 55 DEST CMD LEN [DATA×LEN] CSUM_L CSUM_H 55 AA
```

| Field   | Size | Notes |
|---------|------|-------|
| Header  | 2    | Fixed `0xAA 0x55` |
| DEST    | 1    | Target device address (`0x01`–`0xFE`), or `0xFF` broadcast |
| CMD     | 1    | See table below |
| LEN     | 1    | Byte count of DATA field (0 for ERASE/END/SCAN) |
| DATA    | LEN  | Present only for CMD_IAP_PROM and CMD_IAP_VERIFY |
| CSUM    | 2    | 16-bit sum of (DEST + CMD + LEN + DATA bytes), little-endian |
| Trailer | 2    | Fixed `0x55 0xAA` |

Devices ignore frames where `DEST != DEVICE_ADDR && DEST != 0xFF`.

### IAP Response Frame (7 bytes, always)

```
AA 55 DEV_ADDR ERR EXT 55 AA
```

| Field    | Notes |
|----------|-------|
| DEV_ADDR | Responding device's own address |
| ERR      | `0x00` = success, `0x01` = error |
| EXT      | `DEVICE_TYPE` for CMD_SCAN; `0x00` otherwise |

No response is sent for CMD_IAP_END (device resets immediately after flash write).

### IAP Commands

| CMD  | Name           | Description |
|------|----------------|-------------|
| 0x01 | CMD_SCAN       | Discover devices in IAP mode. Each responds with its type in EXT. Broadcast responses are staggered: device N waits N × ~4 ms before replying to avoid RS485 collision. |
| 0x80 | CMD_IAP_PROM   | Append LEN bytes to 256-byte page buffer; flush to flash when full, auto-advance address. |
| 0x81 | CMD_IAP_ERASE  | Erase all flash pages from `0x08000000` to `IAP_CAL_ADDR` (`0x08003FFC`). |
| 0x82 | CMD_IAP_VERIFY | Compare LEN bytes of DATA against flash at verify pointer; pointer auto-advances by LEN. |
| 0x83 | CMD_IAP_END    | Flush remaining buffer (pad 0xFF), write magic `0x5AA55AA5` at `IAP_CAL_ADDR`, reset to app. |

### Device Types

| Value | Name              |
|-------|-------------------|
| 0x00  | DEV_TYPE_HEATER   |
| 0x01  | DEV_TYPE_REWINDER |

### Device Address

Set `DEVICE_ADDR` at compile time in `main_rs485_iap.c` (range `0x01`–`0xFE`). Each device on the bus must have a unique address for scan collision avoidance to work correctly. `0xFF` is reserved for host broadcast.

### Flash Layout

```
0x08000000  APP_FLASH_BASE   ← app programmed here by IAP
...
0x08003FFC  IAP_CAL_ADDR     ← magic word 0x5AA55AA5 written on successful flash
0x08004000  (end of app region; bootloader occupies upper flash)
```

### Typical Firmware Update Sequence

```
Host → ERASE  (to DEST or broadcast)
Host → PROM ×N
Host → VERIFY ×N
Host → END
```

---

## 2. Bus + PN0031 Application Protocol

Source: `app/ch32_v006_rewinder/src/main_bus_pn0031_test.c`

The CH32V006 bridges two buses:
- **BUS side** (RS485): receives commands from the host (e.g. ESP32)
- **PN0031 side** (UART): communicates with the RFID reader module

### MCU Pins

| Pin  | Function |
|------|----------|
| PD6  | USART1 TX — BUS RS485 (AF, partial remap 1) |
| PD5  | USART1 RX — BUS RS485 (floating input) |
| PD4  | RS485 DE — BUS direction control (output) |
| PD2  | USART2 TX — PN0031 UART (AF, partial remap 3) |
| PD3  | USART2 RX — PN0031 UART (floating input) |
| PC1  | PCF8574 SDA — software I2C |
| PC2  | PCF8574 SCL — software I2C |
| PC4  | PCF8574 INT — falling-edge EXTI interrupt |
| PC6  | LED OK (output) |
| PC7  | LED RX (output) |

USART1 (BUS): **460800 8N1** half-duplex RS485
USART2 (PN0031): **115200 8N1** full-duplex UART

---

### Bus Frame Format

**Request (host → CH32):**

```
55 AA CMD SEQ PLEN_L PLEN_H [PAYLOAD × PLEN] CRC16_L CRC16_H
```

**Response (CH32 → host):**

```
55 AA (CMD|0x80) SEQ PLEN_L PLEN_H [PAYLOAD × PLEN] CRC16_L CRC16_H
```

| Field   | Size   | Notes |
|---------|--------|-------|
| SOF     | 2      | Fixed `0x55 0xAA` |
| CMD     | 1      | Command byte; responses set bit 7 |
| SEQ     | 1      | Sequence number echoed back |
| PLEN    | 2      | Payload length, little-endian |
| PAYLOAD | PLEN   | Command-specific |
| CRC16   | 2      | CRC16-CCITT over bytes [CMD..end of PAYLOAD], little-endian |

CRC16-CCITT: init `0xFFFF`, poly `0x1021`, MSB-first.

---

### Bus Commands

| CMD  | Name                  | Request payload | Response payload |
|------|-----------------------|-----------------|-----------------|
| 0x01 | CMD_PING              | —               | `ST, MAJOR, MINOR, PATCH` |
| 0x30 | CMD_GET_VERSION       | —               | `ST, MAJOR, MINOR, PATCH` |
| 0x22 | CMD_ENTER_BOOT        | —               | `ST_OK` then soft-reset to bootloader |
| 0x32 | CMD_READ_UID          | `ant[1]`        | `ST, ant, pn_status, uid_len, uid[uid_len]` |
| 0x33 | CMD_READ_FILAMENT     | `ant[1]`        | `ST, ant, pn_status, uid_len, uid[], color_rgba[4], dry_temp[2], dry_time[2], nozzle_min[2], nozzle_max[2], variant[8], material_id[8], filament_type[16], detailed_type[16], production_time[16]` |
| 0x41 | CMD_GET_SWITCH_STATUS | —               | `ST, raw_port[1], online_mask[1]` |
| 0x40 | CMD_MOTOR_CTRL        | `ch[1], op[1]`  | `ST` |

`ant` = antenna number 1–4. Numeric filament fields are little-endian signed/unsigned 16-bit.

### Status Codes

| Value | Name         |
|-------|--------------|
| 0x00  | ST_OK        |
| 0x01  | ST_BAD_CMD   |
| 0x02  | ST_BAD_ARG   |
| 0x04  | ST_IO_ERR    |
| 0x05  | ST_NO_TAG    |
| 0x06  | ST_NOT_READY |

### Motor Control

- `ch`: 1–4 (motor channel)
- `op`: `0x00`=stop, `0x01`=forward, `0x02`=reverse

Motors are driven by TC118S H-bridges on PCF8574 device 0 (I2C addr `0x21`):

| Channel | A bit | B bit |
|---------|-------|-------|
| 1       | 6     | 7     |
| 2       | 5     | 4     |
| 3       | 0     | 1     |
| 4       | 3     | 2     |

A=high B=low → forward. A=low B=high → reverse. Both low → stop.

Limit switches on PCF8574 device 1 (I2C addr `0x20`), bits 4–7 (active low, one per channel). PC4 EXTI interrupt auto-stops a reverse-running motor when its limit switch opens.

---

### PN0031 RFID Sub-Protocol

CH32V006 acts as host to PN0031 (see `docs/PN0031_protocol.md`):

```
A0 AD FT FC LEN_L LEN_H [DATA × LEN] BCC 0D
```

BCC = XOR of all bytes from `0xA0` through the last DATA byte.

**Key derivation for block reads:**

1. `PRK = HMAC-SHA256(salt=<16-byte static>, msg=UID)`
2. `key_a = HKDF-Expand(PRK, info="RFID-A\x00", 96 bytes)` → 16 × 6-byte keys
3. `key_b = HKDF-Expand(PRK, info="RFID-B\x00", 96 bytes)` → 16 × 6-byte keys

`CMD_READ_FILAMENT` reads blocks 1, 2, 4, 5, 6, 12 — tries key_a first then key_b per sector.

**Filament data block layout:**

| Block | Content |
|-------|---------|
| 1     | variant[8], material_id[8] |
| 2     | filament_type[16] |
| 4     | detailed_type[16] |
| 5     | color RGBA[4] |
| 6     | dry_temp °C (s16le), dry_time h (u16le), …, nozzle_min (s16le), nozzle_max (s16le) |
| 12    | production_time[16] ASCII |

---

### CMD_ENTER_BOOT Flow

1. Host sends `CMD_ENTER_BOOT` (CMD=0x22, no payload)
2. CH32 erases IAP_CAL_ADDR page (invalidates app-valid marker)
3. CH32 sends `ST_OK` response, waits for TX complete
4. CH32 calls `SystemReset_StartMode(BOOT)` + `NVIC_SystemReset()`
5. Bootloader starts with SFTRST set → stays in IAP mode
6. Host scans bus (`CMD_SCAN`) to confirm device is in bootloader, then sends ERASE → PROM → VERIFY → END
