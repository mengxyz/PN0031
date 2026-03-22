#include <stdint.h>
#include <stdbool.h>
#include <ch32v00X.h>
#include "lib/pcf8574_sw_i2c.h"

#define TEST_LED_PORT GPIOC
#define TEST_LED_OK_PIN GPIO_Pin_6
#define TEST_LED_AUX_PIN GPIO_Pin_7

#define TEST_I2C_PORT GPIOC
#define TEST_SDA_PIN GPIO_Pin_1
#define TEST_SCL_PIN GPIO_Pin_2

#define TEST_PCF0_ADDR 0x21U
#define TEST_PCF1_ADDR 0x20U

static volatile uint32_t g_tick = 0;

void SysTick_Handler(void)
{
    g_tick++;
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = g_tick;
    while ((g_tick - start) < ms) { }
}

static void led_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC, ENABLE);
    gpio.GPIO_Pin = (uint16_t)(TEST_LED_OK_PIN | TEST_LED_AUX_PIN);
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(TEST_LED_PORT, &gpio);
    GPIO_ResetBits(TEST_LED_PORT, (uint16_t)(TEST_LED_OK_PIN | TEST_LED_AUX_PIN));
}

static void led_ok_set(bool on)
{
    if (on) GPIO_SetBits(TEST_LED_PORT, TEST_LED_OK_PIN);
    else GPIO_ResetBits(TEST_LED_PORT, TEST_LED_OK_PIN);
}

static void led_aux_set(bool on)
{
    if (on) GPIO_SetBits(TEST_LED_PORT, TEST_LED_AUX_PIN);
    else GPIO_ResetBits(TEST_LED_PORT, TEST_LED_AUX_PIN);
}

int main(void)
{
    Pcf8574Bus bus = {TEST_I2C_PORT, TEST_SDA_PIN, TEST_SCL_PIN, 20U};
    Pcf8574Device dev0;
    Pcf8574Device dev1;
    bool ok0_write;
    bool ok1_write;
    bool ok0_read;
    bool ok1_read;
    uint8_t v0 = 0xFFU;
    uint8_t v1 = 0xFFU;
    uint32_t last_blink = 0;
    bool blink_state = false;
    bool all_ok;

    SystemCoreClockUpdate();

    SysTick->SR = 0;
    SysTick->CNT = 0;
    SysTick->CMP = (SystemCoreClock / 1000U) - 1U;
    SysTick->CTLR = 0x0FU;

    led_init();

    pcf8574_bus_init(&bus);
    pcf8574_device_init(&dev0, TEST_PCF0_ADDR);
    pcf8574_device_init(&dev1, TEST_PCF1_ADDR);

    ok0_write = pcf8574_write_port(&bus, &dev0, 0xFFU);
    ok1_write = pcf8574_write_port(&bus, &dev1, 0xFFU);
    ok0_read = pcf8574_read_port(&bus, &dev0, &v0);
    ok1_read = pcf8574_read_port(&bus, &dev1, &v1);

    all_ok = ok0_write && ok1_write && ok0_read && ok1_read;
    led_ok_set(all_ok);

    while (1) {
        if (all_ok) {
            // Mirror switch expander P4 on PC7: low input turns LED on.
            if (pcf8574_read_port(&bus, &dev1, &v1)) {
                led_aux_set((v1 & (1U << 4)) == 0U);
            }
            delay_ms(50);
        } else {
            if ((g_tick - last_blink) > 300U) {
                last_blink = g_tick;
                blink_state = !blink_state;
                led_ok_set(blink_state);
            }
        }
    }
}
