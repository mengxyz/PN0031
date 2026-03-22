# CH32V006 RS485 Bootloader

This repository now contains two updater paths for `CH32V006F8U6`:

- older custom RS485 updater: `env:bootloader` in `app/ch32_v006_bootloader`
- WCH-style RS485 IAP updater: `env:rs485_iap` in `app/ch32_v006_bootloader`

## Projects

- Bootloader: `app/ch32_v006_bootloader`
- Application (normal): `app/ch32_v006_rewinder` env `rewinder`

## Current status

The custom split layout with:

- bootloader at `0x08000000`
- application at `0x08002000`

was tested and is not reliable on this target. Flashing and readback worked, but the application never executed after handoff.

For now, use the normal application layout only:

- `app/ch32_v006_rewinder`
- env `rewinder`
- linker `src/Link.ld`

The old app-slot environments were removed from `app/ch32_v006_rewinder/platformio.ini`.

## Recommended updater path

Use the WCH-style IAP flow:

- app runs from normal user flash
- app receives a bus command to enter boot mode
- app erases the IAP marker page and calls `SystemReset_StartMode(Start_Mode_BOOT)`
- reset enters the boot section updater
- updater writes/verifies normal app flash
- updater finishes with `SystemReset_StartMode(Start_Mode_USER)`

This avoids the failed custom `0x08002000` split-app handoff.

## RS485 IAP target

Target:

- `app/ch32_v006_bootloader`
- env `rs485_iap`

Build:

```bash
cd app/ch32_v006_bootloader
pio run -e rs485_iap
```

Flash:

```bash
cd app/ch32_v006_bootloader
pio run -e rs485_iap -t upload
```

### RS485 IAP pin mapping

- `USART1 remap1`
- `PD6` = TX
- `PD5` = RX
- `PD4` = RS485 DE (`HIGH=TX`, `LOW=RX`)

### RS485 IAP LEDs

- `PC7` = activity
  - slow blink while updater is waiting
  - pulse on RX/TX traffic
- `PC6` = error/status
  - off normally
  - on after protocol/flash error

### RS485 IAP protocol

This target keeps the WCH `USART_IAP` frame format:

- header: `AA 55`
- `CMD`
- `LEN`
- optional command payload
- 16-bit additive checksum low/high
- tail: `55 AA`

Command IDs:

- `0x80` program chunk
- `0x81` erase
- `0x82` verify
- `0x83` end
- `0x84` jump IAP / enter IAP marker flow

Ack frame:

- `AA 55 00 00 55 AA` = success
- `AA 55 00 01 55 AA` = error

### App-side enter boot command

The normal rewinder app now supports bus command `0x22` to enter WCH boot mode.

Tool support:

```bash
python3 tools/ch32v006_rs485_app.py --port /dev/tty.usbserial-XXXX enter-boot
```

Behavior:

1. rewinder app replies `status=OK`
2. rewinder app erases the WCH IAP marker page at `0x08003F00`
3. rewinder app resets with `Start_Mode_BOOT`
4. next reset boot enters `rs485_iap`

Marker note:

- current marker address is `0x08003FFC`
- this reserves the flash page `0x08003F00..0x08003FFF`
- current app size is below this page, so it is safe now
- if app grows past 16 KB later, move this marker and reserve it in the linker

## Experimental split layout

- Bootloader region: `0x08000000` .. `0x08001FFF` (8 KB)
- App region: `0x08002000` .. `0x0800F7FF` (54 KB)

## UART/RS485 mapping

Bootloader uses `USART1` default mapping:

- `PD5` = USART1 TX
- `PD6` = USART1 RX
- `PD4` = RS485 DE/RE control (HIGH TX, LOW RX)
- `PC7` = activity LED (blink during boot/update)
- `PC6` = error LED (invalid app / update error)

Baud default: `115200` 8N1.

## Build

Bootloader:

```bash
cd app/ch32_v006_bootloader
pio run -e bootloader
```

Application:

```bash
cd app/ch32_v006_rewinder
pio run -e rewinder
```

## Flash update tool

Tool: `tools/ch32v006_rs485_boot.py`

Install dependency:

```bash
pip install pyserial
```

Check bootloader:

```bash
python3 tools/ch32v006_rs485_boot.py --port /dev/tty.usbserial-XXXX ping
python3 tools/ch32v006_rs485_boot.py --port /dev/tty.usbserial-XXXX info
python3 tools/ch32v006_rs485_boot.py --port /dev/tty.usbserial-XXXX reset
```

Upload app image:

```bash
python3 tools/ch32v006_rs485_boot.py \
  --port /dev/tty.usbserial-XXXX \
  --baud 115200 \
  upload app/ch32_v006_rewinder/.pio/build/rewinder_app/firmware.bin --run --progress
```

The tool performs:

1. `PING`
2. `INFO`
3. `ERASE` app area used by image
4. `WRITE` pages (256 bytes each)
5. `CRC32` verify
6. optional `RUN`

## Protocol summary

Frame:

- `SOF0=0x55`
- `SOF1=0xAA`
- `CMD` (1)
- `SEQ` (1)
- `LEN` (2, little-endian)
- `PAYLOAD` (`LEN` bytes)
- `CRC16-CCITT` over `CMD..PAYLOAD` (2, little-endian)

Commands:

- `0x01` PING
- `0x02` INFO
- `0x10` ERASE (`addr:u32`, `len:u32`, both 256-byte aligned)
- `0x11` WRITE (`addr:u32`, `len:u16`, `data`, len is 256-byte aligned)
- `0x12` CRC32 (`addr:u32`, `len:u32`)
- `0x20` RUN
- `0x21` RESET

Response command is `CMD | 0x80`, first payload byte is status (`0x00` = OK).

## Application mode bus protocol (rewinder app)

Application firmware in `app/ch32_v006_rewinder` now listens on bus UART (`USART1`, `PD5/PD6`) using the same frame format as bootloader.

This lets BMCU use one parser for both bootloader mode and application mode.

### Bus/UART routing in app mode

- Bus side: `USART1` (`PD5` TX, `PD6` RX) at `115200 8N1`, `PD4` as RS485 DE/RE
- PN0031 side: `USART2 remap3` (`PD2` TX, `PD3` RX) at `115200 8N1`
- PCF8574 software-I2C: `PC1` = SDA, `PC2` = SCL
- PCF8574 interrupt input: `PC4` (EXTI line 4, falling edge, pull-up)

### PCF8574 behavior in app

- I2C access is implemented in software bit-bang mode on `PC1/PC2`.
- Driver is split into reusable module:
  - `app/ch32_v006_rewinder/src/lib/pcf8574_sw_i2c.h`
  - `app/ch32_v006_rewinder/src/lib/pcf8574_sw_i2c.c`
- Current app instantiates 2 devices on the same bus:
  - `0x20` and `0x21` (7-bit addresses; change in `main.c` if needed)
- Device role split in app:
  - `0x20` = motor output expander (TC118S control)
  - `0x21` = switch input/interrupt expander
- TC118S mapping on PCF8574 `0x20`:
  - `P0=ch3A`, `P1=ch3B`
  - `P2=ch4A`, `P3=ch4B`
  - `P4=ch2B`, `P5=ch2A`
  - `P6=ch1A`, `P7=ch1B` (assumed from duplicated `ch2B` in note)
- Switch mapping on PCF8574 `0x21`:
  - `P4=SW1`, `P5=SW2`, `P6=SW3`, `P7=SW4` (active-low inputs)
- On boot:
  - motor expander (`0x20`) is initialized from `g_motor_port_shadow` (currently `0x00`)
  - input expander (`0x21`) is written `0xFF` to keep pins released/high
- On `PC4` interrupt, app reads one byte from PCF8574 and updates internal state/change mask.
- Current default hooks:
  - On SW1..SW4 change to offline while corresponding channel is running in reverse (rewind), that channel is stopped immediately in local MCU.
  - This stop path is local and does not wait for BMCU command round-trip.

### Frame format (same as bootloader)

- `SOF0` = `0x55`
- `SOF1` = `0xAA`
- `CMD` (1 byte)
- `SEQ` (1 byte)
- `LEN` (2 bytes, little-endian)
- `PAYLOAD` (`LEN` bytes)
- `CRC16-CCITT` over `CMD..PAYLOAD` (2 bytes, little-endian)

Response frame uses `CMD | 0x80`.

### Supported application commands

- `0x30` `GET_VERSION`
- `0x01` `PING` (returns same version payload for compatibility)
- `0x40` `MOTOR_CTRL` (TC118S via PCF8574 `0x20`)

Unknown commands return status `0x01` (`ST_BAD_CMD`).

### MOTOR_CTRL payload

Request payload length: `2`

- `payload[0]`: channel (`1..4`)
- `payload[1]`: operation
  - `0x00` stop
  - `0x01` forward (A=1, B=0)
  - `0x02` reverse (A=0, B=1)

Response payload length: `1`

- `payload[0]`: status
  - `0x00` OK
  - `0x02` bad arg (invalid channel/op/length)
  - `0x06` not ready / switch offline (command rejected)

### Version response payload

Payload length: `4`

- `payload[0]`: status (`0x00` = OK)
- `payload[1]`: app major
- `payload[2]`: app minor
- `payload[3]`: app patch

Current app version constants:

- major `0x01`
- minor `0x00`
- patch `0x00`

### Example transaction (`GET_VERSION`)

Request (from BMCU):

- `CMD=0x30`, `SEQ=0x01`, `LEN=0`
- raw bytes before CRC: `55 AA 30 01 00 00`

Response (from app):

- command byte will be `0xB0` (`0x30 | 0x80`)
- payload will be `00 01 00 00` for version `1.0.0`

### Runtime behavior

- App polls bus parser in main loop.
- App also polls bus parser between PN0031 antenna reads.
- This allows version query while filament polling is active (not only when idle).
