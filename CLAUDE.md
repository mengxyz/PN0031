# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Multi-target embedded firmware project for the PN0031 RFID module. Two hardware platforms:
- **ESP32** (C6/C3/S3) — host side, Arduino framework, in `src/` and `lib/`
- **CH32V006F8U6** — peripheral side, native C/RISC-V, in `app/`

## Build Commands

All builds use PlatformIO. Run from the directory containing the relevant `platformio.ini`.

**Root (ESP32 targets):**
```sh
pio run -e esp32-c6-devkitc-1          # main PN0031 UART example
pio run -e esp32-c3-proxy              # C3 proxy/bridge firmware
pio run -e esp32-c3-rs485-master-test  # RS485 master test
pio run -e esp32-s3-bus-test           # S3 bus test (LittleFS)
pio run -e <env> -t upload             # flash
pio monitor -e <env>                   # serial at 115200
```

**CH32V006 bootloader (`app/ch32_v006_bootloader/`):**
```sh
pio run -e bootloader      # standard bootloader
pio run -e rs485_iap       # RS485 IAP bootloader
pio run -e <env> -t upload # flash via minichlink
```

**CH32V006 rewinder app (`app/ch32_v006_rewinder/`):**
```sh
pio run -e rewinder            # main rewinder firmware
pio run -e pn0031_poll_test    # PN0031 polling test
pio run -e bus_pn0031_test     # bus + PN0031 integration
pio run -e rs485_iap_app_test  # IAP app test
# also: blink_led_test, pcf8574_test, bus_test, uart_bus_test,
#        uart_hello_test, rs485_hello_test, rs485_ping_pong_test, rs485_echo_test
```

CH32V006 requires **minichlink** programmer (WCH-LinkE or compatible).

## Architecture

### PN0031 UART Protocol (`lib/PN0031/`)

Frame format: `STX(0xA0) AD FT FC LEN_L LEN_H DATA... BCC XOR ETX(0x0D)`
- `LEN` is `DATA` length only, little-endian
- `BCC` is XOR of all bytes from `STX` through the byte before `BCC`
- UART: 115200 8N1

The `PN0031` Arduino library (`lib/PN0031/src/`) wraps this protocol. Commands use `FT=0x02` with FC values: `0x00` setAddress, `0x20` readCard, `0x21` readAllCards, `0x22` readBlock, `0x23` writeBlock.

### ESP32 src/ — Multiple entry points

`platformio.ini` uses `build_src_filter` to select one `main_*.cpp` per environment. The default environment (`esp32-c6-devkitc-1`) compiles `main.cpp`; other envs exclude it and include their own file.

### CH32BusHost (`lib/CH32BusHost/`)

Arduino-side library for the ESP32 host to communicate with a CH32V006 peripheral over a secondary UART/RS485 bus. Abstracts the framing layer between the two MCUs.

### CH32V006 app structure

Both `app/ch32_v006_bootloader/` and `app/ch32_v006_rewinder/` are independent PlatformIO projects with their own `boards/` directory containing the `genericCH32V006F8U6.json` board definition. The bootloader uses `BootLink.ld`; the app uses `Link.ld`.

IAP (In-Application Programming) is done over RS485: the bootloader receives a new app binary and programs it to flash, rather than using a split-memory layout.

### PCF8574 I2C expander (`app/ch32_v006_rewinder/src/lib/`)

Software I2C driver for the PCF8574 8-bit I/O expander, used to extend the CH32V006's limited GPIO. Included only in environments that need it (`rewinder`, `pcf8574_test`, `bus_pn0031_test`).

## Key docs

- `docs/PN0031_protocol.md` — full frame/command reference
- `docs/ch32v006-rs485-bootloader.md` — bootloader and IAP architecture
