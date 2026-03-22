#include <stdint.h>
#include <ch32v00X.h>

static void delay_cycles(volatile uint32_t cycles)
{
    while (cycles-- != 0U) { }
}

static void delay_ms(uint32_t ms)
{
    while (ms-- != 0U) {
        delay_cycles(8000U);
    }
}

static void leds_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &gpio);

    GPIO_ResetBits(GPIOC, GPIO_Pin_6 | GPIO_Pin_7);
}

int main(void)
{
    leds_init();

    while (1) {
        GPIO_SetBits(GPIOC, GPIO_Pin_7);
        GPIO_ResetBits(GPIOC, GPIO_Pin_6);
        delay_ms(150U);

        GPIO_ResetBits(GPIOC, GPIO_Pin_7);
        GPIO_SetBits(GPIOC, GPIO_Pin_6);
        delay_ms(150U);
    }
}
