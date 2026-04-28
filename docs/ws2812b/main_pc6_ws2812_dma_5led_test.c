#include <stdint.h>
#include <ch32v00X.h>
#include <ch32v00X_dma.h>
#include <ch32v00X_spi.h>

#define AHBPCENR HBPCENR
#define APB2PCENR PB2PCENR
#define RCC_AHBPeriph_DMA1 RCC_HBPeriph_DMA1
#define RCC_APB2Periph_GPIOC RCC_PB2Periph_GPIOC
#define RCC_APB2Periph_SPI1 RCC_PB2Periph_SPI1
#define GPIO_Speed_10MHz GPIO_Speed_30MHz
#define GPIO_CNF_OUT_PP_AF 0x08
#define CTLR1_SPE_Set ((uint16_t)0x0040)

#define WS_LED_COUNT 5
#define WS2812DMA_IMPLEMENTATION
#define DMALEDS 8
#define WSGRB
#include "../../../docs/ws2812b/ws2812b_dma_spi_led_driver.h"

static volatile uint32_t g_led_colors[WS_LED_COUNT] = {0U, 0U, 0U, 0U, 0U};
static const uint32_t k_bg_white = 0x0D0D0D;
static const uint32_t k_move_colors[] = {
    0xB30000U,
    0x00B300U,
    0x0000B3U,
};
static const uint32_t k_step_delay_ms = 25U;

static void delay_cycles(volatile uint32_t cycles)
{
    while (cycles-- != 0U) { }
}

static void delay_ms(uint32_t ms)
{
    while (ms-- != 0U) {
        delay_cycles(12000U);
    }
}

uint32_t WS2812BLEDCallback(int ledno)
{
    if (ledno < WS_LED_COUNT) {
        return g_led_colors[ledno];
    }
    return 0U;
}

static void ws_fill_all(uint32_t color)
{
    uint8_t i;
    for (i = 0U; i < WS_LED_COUNT; i++) {
        g_led_colors[i] = color;
    }
}

static void ws_start_blocking(void)
{
    while (WS2812BLEDInUse) { }
    WS2812BDMAStart(WS_LED_COUNT);
    while (WS2812BLEDInUse) { }
}

static void ws_show_moving_pixel(uint8_t index, uint32_t color)
{
    uint8_t i;
    for (i = 0U; i < WS_LED_COUNT; i++) {
        g_led_colors[i] = k_bg_white;
    }
    if (index < WS_LED_COUNT) {
        g_led_colors[index] = color;
    }
    ws_start_blocking();
}

int main(void)
{
    uint8_t color_idx;
    uint8_t led_idx;

    SystemCoreClockUpdate();
    delay_ms(100U);
    WS2812BDMAInit();
    delay_ms(100U);

    ws_fill_all(0x000000U);
    ws_start_blocking();
    delay_ms(100U);

    while (1) {
        for (color_idx = 0U; color_idx < (uint8_t)(sizeof(k_move_colors) / sizeof(k_move_colors[0])); color_idx++) {
            for (led_idx = 0U; led_idx < WS_LED_COUNT; led_idx++) {
                ws_show_moving_pixel(led_idx, k_move_colors[color_idx]);
                delay_ms(k_step_delay_ms);
            }
        }
    }
}
