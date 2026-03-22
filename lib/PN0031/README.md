# PN0031 Arduino Library

Library for PN0031 RFID module UART protocol.

## Protocol Summary
- UART: `115200 8N1`
- Frame: `STX(0xA0) AD FT FC LEN_L LEN_H DATA... BCC XOR ETX(0x0D)`
- `LEN` is `DATA` length only (little-endian)
- `BCC` is XOR of all bytes before `BCC`, starting at `STX`

## Supported Commands
- `setAddress()` (`FT=0x02, FC=0x00`)
- `readCard()` (`FT=0x02, FC=0x20`)
- `readAllCards()` (`FT=0x02, FC=0x21`)
- `readBlock()` (`FT=0x02, FC=0x22`)
- `writeBlock()` (`FT=0x02, FC=0x23`)

## Quick Start (PlatformIO)
- Put library under `lib/PN0031`
- Build with `pio run -e esp32-c6-devkitc-1`
- Flash with `pio run -e esp32-c6-devkitc-1 -t upload`

Example sketch: `examples/pn0031_read_uid/pn0031_read_uid.ino`
