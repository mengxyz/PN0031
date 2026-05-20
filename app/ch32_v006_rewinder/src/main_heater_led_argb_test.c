/* Heater PCB LED/ARGB smoke test.
 *
 * PC4: LED 1
 * PC5: LED 2
 * PC0: ARGB data, SPI1 partial remap 3
 *
 * Pattern:
 *   1. PC4 on,  PC5 off, ARGB red
 *   2. PC4 off, PC5 on,  ARGB green
 *   3. PC4 on,  PC5 on,  ARGB blue
 *   4. PC4 off, PC5 off, ARGB white
 *   5. PC4/PC5 blink together, ARGB off
 */

#include <stdint.h>

#include <ch32v00X.h>
#include "ch32v00X_dma.h"
#include "ch32v00X_gpio.h"
#include "ch32v00X_rcc.h"
#include "ch32v00X_spi.h"

#define AHBPCENR               HBPCENR
#define APB2PCENR              PB2PCENR
#define RCC_AHBPeriph_DMA1     RCC_HBPeriph_DMA1
#define RCC_APB2Periph_GPIOC   RCC_PB2Periph_GPIOC
#define RCC_APB2Periph_AFIO    RCC_PB2Periph_AFIO
#define RCC_APB2Periph_SPI1    RCC_PB2Periph_SPI1
#define GPIO_Speed_10MHz       GPIO_Speed_30MHz
#define GPIO_CNF_OUT_PP_AF     0x08
#define CTLR1_SPE_Set          ((uint16_t)0x0040)

#define ARGB_LED_COUNT         1U
#define WS_LED_COUNT           ARGB_LED_COUNT
#define WS2812_GPIO_PORT       GPIOC
#define WS2812_GPIO_PORT_CLOCK RCC_PB2Periph_GPIOC
#define WS2812_GPIO_PIN_INDEX  0
#define WS2812_SPI_REMAP       GPIO_PartialRemap3_SPI1
#define WS2812DMA_IMPLEMENTATION
#define DMALEDS                8
#define WSGRB
#include "lib/ws2812b_dma_spi_led_driver.h"

#define LED1_PIN GPIO_Pin_4
#define LED2_PIN GPIO_Pin_5

static volatile uint32_t g_argb[ARGB_LED_COUNT];
static uint8_t g_argb_fault;

uint32_t WS2812BLEDCallback(int ledno)
{
    return (ledno == 0) ? g_argb[0] : 0U;
}

static void delay_ms(uint32_t ms)
{
    volatile uint32_t n;
    while (ms--) {
        for (n = 0; n < 8000U; n++) {
            __asm__ volatile ("nop");
        }
    }
}

static void leds_set(uint8_t led1, uint8_t led2)
{
    if (led1) GPIO_SetBits(GPIOC, LED1_PIN);
    else GPIO_ResetBits(GPIOC, LED1_PIN);

    if (led2) GPIO_SetBits(GPIOC, LED2_PIN);
    else GPIO_ResetBits(GPIOC, LED2_PIN);
}

static void argb_set(uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t guard;

    if (g_argb_fault) {
        return;
    }

    g_argb[0] = ((uint32_t)b << 16) | ((uint32_t)g << 8) | r;

    guard = 60000U;
    while (WS2812BLEDInUse && --guard) {}
    if (!guard) {
        g_argb_fault = 1U;
        return;
    }

    WS2812BDMAStart(ARGB_LED_COUNT);

    guard = 60000U;
    while (WS2812BLEDInUse && --guard) {}
    if (!guard) {
        g_argb_fault = 1U;
    }
}

static void fault_blink(void)
{
    leds_set(1, 0);
    delay_ms(80);
    leds_set(0, 1);
    delay_ms(80);
    leds_set(0, 0);
    delay_ms(80);
}

static void gpio_init_test(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_PB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

    gpio.GPIO_Pin = LED1_PIN | LED2_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOC, &gpio);

    leds_set(0, 0);
}

int main(void)
{
    SystemCoreClockUpdate();
    gpio_init_test();
    WS2812BDMAInit();

    argb_set(0, 0, 0);

    while (1) {
        if (g_argb_fault) {
            fault_blink();
            continue;
        }

        leds_set(1, 0);
        argb_set(64, 0, 0);
        delay_ms(700);

        leds_set(0, 1);
        argb_set(0, 64, 0);
        delay_ms(700);

        leds_set(1, 1);
        argb_set(0, 0, 64);
        delay_ms(700);

        leds_set(0, 0);
        argb_set(40, 40, 40);
        delay_ms(700);

        leds_set(1, 1);
        argb_set(0, 0, 0);
        delay_ms(120);
        leds_set(0, 0);
        delay_ms(120);
        leds_set(1, 1);
        delay_ms(120);
        leds_set(0, 0);
        delay_ms(700);
    }
}
