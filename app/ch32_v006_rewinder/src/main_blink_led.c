#include <stdint.h>
#include <ch32v00X.h>

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

static void delay_us(uint32_t us)
{
    while (us-- != 0U) {
        delay_cycles(12U);
    }
}

static void led_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC | RCC_PB2Periph_GPIOD, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_4;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &gpio);

    GPIO_ResetBits(GPIOD, GPIO_Pin_4);
    GPIO_ResetBits(GPIOC, GPIO_Pin_6 | GPIO_Pin_7);
}

int main(void)
{
    uint16_t duty = 0U;
    uint16_t cycle = 0U;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();

    led_gpio_init();

    while (1) {
        GPIO_ResetBits(GPIOD, GPIO_Pin_4);
        GPIO_ResetBits(GPIOC, GPIO_Pin_6);

        for (duty = 0U; duty <= 1000U; duty += 20U) {
            for (cycle = 0U; cycle < 10U; cycle++) {
                GPIO_SetBits(GPIOC, GPIO_Pin_7);
                delay_us(duty);
                GPIO_ResetBits(GPIOC, GPIO_Pin_7);
                delay_us(1000U - duty);
            }
        }

        for (duty = 1000U; duty >= 20U; duty -= 20U) {
            for (cycle = 0U; cycle < 10U; cycle++) {
                GPIO_SetBits(GPIOC, GPIO_Pin_7);
                delay_us(duty);
                GPIO_ResetBits(GPIOC, GPIO_Pin_7);
                delay_us(1000U - duty);
            }
        }

        delay_ms(30U);
    }
}
