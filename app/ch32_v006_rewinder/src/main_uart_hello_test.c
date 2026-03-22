#include <stdint.h>
#include <ch32v00X.h>

#define UART_BAUD 115200U
#define LED_PORT GPIOC
#define LED_PIN  GPIO_Pin_6

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

static void leds_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC, ENABLE);
    gpio.GPIO_Pin = LED_PIN;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LED_PORT, &gpio);
    GPIO_ResetBits(LED_PORT, LED_PIN);
}

static void uart_init(uint32_t baud)
{
    GPIO_InitTypeDef gpio = {0};
    USART_InitTypeDef uart = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD | RCC_PB2Periph_AFIO | RCC_PB2Periph_USART1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_5;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &gpio);

    USART_StructInit(&uart);
    uart.USART_BaudRate = baud;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &uart);
    USART_Cmd(USART1, ENABLE);
}

static void uart_write_byte(uint8_t b)
{
    while ((USART1->STATR & USART_FLAG_TXE) == 0) { }
    USART1->DATAR = b;
}

static void uart_write_str(const char *s)
{
    while (*s != '\0') {
        uart_write_byte((uint8_t)*s++);
    }
    while ((USART1->STATR & USART_FLAG_TC) == 0) { }
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    leds_init();
    uart_init(UART_BAUD);

    while (1) {
        GPIO_SetBits(LED_PORT, LED_PIN);
        uart_write_str("hello world\r\n");
        delay_ms(500U);
        GPIO_ResetBits(LED_PORT, LED_PIN);
        delay_ms(500U);
    }
}
