#include <stdint.h>
#include <ch32v00X.h>

#define LED_PORT GPIOC
#define LED_PIN  GPIO_Pin_6

static void delay_cycles(volatile uint32_t cycles)
{
    while (cycles-- != 0U) { }
}

static void delay_us(uint32_t us)
{
    while (us-- != 0U) {
        delay_cycles(12U);
    }
}

static void delay_ms(uint32_t ms)
{
    while (ms-- != 0U) {
        delay_cycles(12000U);
    }
}

static void led_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC, ENABLE);

    gpio.GPIO_Pin = LED_PIN;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LED_PORT, &gpio);

    GPIO_ResetBits(LED_PORT, LED_PIN);
}

int main(void)
{
    uint16_t duty;
    uint16_t cycle;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    led_init();

    while (1) {
        for (duty = 0U; duty <= 1000U; duty += 20U) {
            for (cycle = 0U; cycle < 10U; cycle++) {
                GPIO_SetBits(LED_PORT, LED_PIN);
                delay_us(duty);
                GPIO_ResetBits(LED_PORT, LED_PIN);
                delay_us(1000U - duty);
            }
        }

        for (duty = 1000U; duty >= 20U; duty -= 20U) {
            for (cycle = 0U; cycle < 10U; cycle++) {
                GPIO_SetBits(LED_PORT, LED_PIN);
                delay_us(duty);
                GPIO_ResetBits(LED_PORT, LED_PIN);
                delay_us(1000U - duty);
            }
        }

        delay_ms(30U);
    }
}
