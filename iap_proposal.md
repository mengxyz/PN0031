Goal:
Optimize CH32V003 custom IAP bootloader firmware update over RS485. Current IAP code space is very limited, around 3KB, so avoid large features.

Context:
- MCU: CH32V003
- IAP/bootloader sector is protected and cannot normally be erased.
- Bootloader jumps only to normal user area/startup sector.
- Firmware update currently sends data in chunks over RS485.
- Current flow appears to send 4 chunks, then verify those 4 chunks, repeatedly.
- Updating only a few KB including verify takes about 300 seconds, which is too slow.
- Need a plan that fits a tiny bootloader and improves speed/reliability.

Main idea to evaluate:
Use two user flash areas:
- App A: normal startup/user app area
- App B: staging area for new firmware

Proposed flow:
1. Running App A receives firmware over RS485 and writes it to App B/staging.
2. App A verifies App B using CRC/checksum.
3. App A writes update flag with size, CRC, source address, destination address.
4. Reset into protected IAP bootloader.
5. IAP sees update pending.
6. IAP copies App B to App A internally, page by page.
7. IAP verifies App A CRC.
8. IAP clears update flag.
9. IAP jumps to App A.

Questions to answer:
1. Is staging App B then IAP copy to App A practical on CH32V003?
2. Will it reduce downtime and/or total update time compared with direct RS485 flashing?
3. What is the smallest reliable protocol change to reduce the current 300s update time?
4. Can verify be changed from read-back-over-RS485 every 4 chunks to local CRC/checksum inside MCU?
5. What packet size is realistic for CH32V003 RAM and flash write constraints?
6. What minimal update flag/state machine is needed to survive power loss during copy?
7. What flash page layout should be used to avoid erasing bootloader, staging, or flags?
8. What should be implemented inside 3KB bootloader vs inside App A?

Preferred constraints:
- Keep bootloader simple.
- Avoid read-back verify over RS485.
- ACK per packet/block, not per word/byte.
- Use CRC16 per packet if cheap.
- Use CRC32 or simple checksum for full firmware if CRC32 is too large.
- Bootloader should only handle pending flag, copy B→A, verify, clear flag, jump.
- App A can handle slower/larger logic like RS485 receiving and staging writes.

Deliverable:
Give a concrete implementation plan, flash layout, state machine, packet protocol, and pseudocode for bootloader copy/verify.