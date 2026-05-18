# WS2812B ARGB on CH32V006 — Known Quirks

Driver: `app/ch32_v006_rewinder/src/lib/ws2812b_dma_spi_led_driver.h`  
Output pin: **PC6** (SPI1 MOSI, AF push-pull)  
Method: SPI1 TX → DMA1_Channel3, circular mode, half/complete interrupts

---

## Quirk 1 — Do not enable DMA channel at init (CNTR=0 lockup)

The original driver called `DMA1_Channel3->CFGR |= DMA_CFGR1_EN` at the end of `WS2812BDMAInit()`.

On CH32V006, enabling a DMA channel with `CNTR=0` immediately fires a Transfer Complete interrupt. The ISR then calls `WS2812FillBuffSec()`, which reads `WS2812BLEDInUse` and tries to take the DMA out of circular mode — before any transfer was ever started. This locks up the CPU in a continuous interrupt loop before `main()` even reaches `bus_uart_init()`.

**Fix:** Remove `DMA_CFGR1_EN` from `WS2812BDMAInit()`. Enable the channel only inside `WS2812BDMAStart()`, after setting `CNTR` and `MADDR`.

```c
/* WS2812BDMAInit — channel configured but NOT enabled */
/* comment at end of function: */
/* DMA IRQ enabled/disabled in argb_flush(), not here, to prevent stray
   interrupts from interfering with the main bus polling loop. */

/* WS2812BDMAStart — disable, reconfigure, then enable */
DMA1_Channel3->CFGR &= ~(DMA_Mode_Circular | DMA_CFGR1_EN);
DMA1_Channel3->CNTR  = 0;
DMA1_Channel3->MADDR = (uint32_t)WS2812dmabuff;
// ... fill buffer ...
DMA1_Channel3->CNTR = DMA_BUFFER_LEN;
DMA1_Channel3->CFGR |= DMA_Mode_Circular | DMA_CFGR1_EN;
```

---

## Quirk 2 — SysTick SR must be cleared in the handler

CH32V006 SysTick is a WCH custom peripheral at `0xE000F000`, not the ARM Cortex-M SysTick. Its `SR` (status register) is **level-triggered**: if not cleared in the ISR, the interrupt re-asserts immediately on exit and fires continuously.

When SysTick fires at MHz rates it starves all lower-priority IRQs, including `DMA1_Channel3_IRQn`. The DMA half/complete callbacks never run, `WS2812BLEDInUse` stays `1`, and `argb_flush()` hangs forever.

**Fix:** Clear `SysTick->SR` in the handler, and mark it with the WCH fast-interrupt attribute:

```c
void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void SysTick_Handler(void)
{
    g_tick++;
    SysTick->SR = 0;
}
```

---

## Quirk 3 — Disable SysTick during DMA transfer (argb_flush)

Even with SR cleared correctly, SysTick can still preempt `DMA1_Channel3_IRQn` mid-transfer if both share the same NVIC priority level. This causes LED jitter or a hung flush.

**Fix:** Save and clear `SysTick->CTLR` before starting the transfer; restore it after.  
Also gate `NVIC_EnableIRQ(DMA1_Channel3_IRQn)` to the flush window so stray DMA interrupts don't interfere with the RS485 bus poll loop between flushes.

```c
static void argb_flush(void)
{
    uint32_t ctlr = SysTick->CTLR;
    SysTick->CTLR = 0U;
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    while (WS2812BLEDInUse) { }
    WS2812BDMAStart(ARGB_LED_COUNT);
    while (WS2812BLEDInUse) { }
    NVIC_DisableIRQ(DMA1_Channel3_IRQn);
    SysTick->CTLR = ctlr;
}
```

---

## Quirk 4 — delay_ms must not depend on g_tick during DMA

If `delay_ms()` is implemented as a busy-wait on `g_tick` (SysTick-based), it breaks whenever SysTick is disabled (Quirk 3) or firing too fast (Quirk 2). Calls inside `poll_filament()` between PN0031 block reads will return instantly or never return.

**Fix:** Use a pure busy-loop calibrated to the system clock:

```c
static void delay_ms(uint32_t ms)
{
    while (ms-- != 0U) {
        for (volatile uint32_t d = 0; d < 12000U; d++) {}  /* ~1ms @ 48MHz, 4 cycles/iter */
    }
}
```

---

## Quirk 5 — WSGRB internal packing is BGR, not RGB

With `#define WSGRB`, the driver maps the 24-bit LED value to the wire as:
- bits\[15:8\] → G channel (wire byte 1)
- bits\[7:0\]  → R channel (wire byte 2)
- bits\[23:16\] → B channel (wire byte 3)

So the 24-bit value must be packed as **BGR** (blue in high bits) for R/G/B calls to produce the correct colour. If you store as RGB (red in high bits), R and B appear swapped — red shows as blue and vice versa.

**Fix:** In `argb_set`, swap r and b when storing:

```c
static void argb_set(uint8_t idx, uint8_t r, uint8_t g, uint8_t b)
{
    /* WSGRB driver maps bits[23:16]->B, bits[15:8]->G, bits[7:0]->R on wire */
    g_argb[idx] = ((uint32_t)b << 16) | ((uint32_t)g << 8) | r;
}
```

---

## Register map note

| Register | Address | Note |
|---|---|---|
| `SysTick->CTLR` | `0xE000F000` | Enable/clock-source/interrupt-enable |
| `SysTick->SR`   | `0xE000F004` | **Must clear by software in ISR** |
| `SysTick->CNT`  | `0xE000F008` | Current counter |
| `SysTick->CMP`  | `0xE000F010` | Compare (reload) value |

Standard ARM `SysTick->CTRL` at `0xE000E010` does **not** exist on CH32V006.
