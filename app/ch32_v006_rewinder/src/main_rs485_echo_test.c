#include <stdbool.h>
#include <stdint.h>
#include <ch32v00X.h>

#define UART_BAUD 115200U
#define RS485_DE_PORT GPIOD
#define RS485_DE_PIN  GPIO_Pin_4
#define LED_PORT GPIOC
#define LED_HB_PIN GPIO_Pin_6
#define LED_RX_PIN GPIO_Pin_7

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
    gpio.GPIO_Pin = (uint16_t)(LED_HB_PIN | LED_RX_PIN);
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LED_PORT, &gpio);
    GPIO_ResetBits(LED_PORT, (uint16_t)(LED_HB_PIN | LED_RX_PIN));
}

static void rs485_uart_init(uint32_t baud)
{
    GPIO_InitTypeDef gpio = {0};
    USART_InitTypeDef uart = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD | RCC_PB2Periph_AFIO | RCC_PB2Periph_USART1, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap1_USART1, ENABLE);

    gpio.GPIO_Pin = RS485_DE_PIN;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(RS485_DE_PORT, &gpio);
    GPIO_ResetBits(RS485_DE_PORT, RS485_DE_PIN);

    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_5;
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

static bool uart_read_byte(uint8_t *out)
{
    if ((USART1->STATR & USART_FLAG_RXNE) != 0U) {
        *out = (uint8_t)(USART1->DATAR & 0xFFU);
        return true;
    }
    return false;
}

static void uart_write_byte(uint8_t b)
{
    while ((USART1->STATR & USART_FLAG_TXE) == 0U) { }
    USART1->DATAR = b;
}

int main(void)
{
    uint8_t rx_buf[64];
    uint8_t rx_len = 0U;
    bool hb = false;
    uint32_t idle = 0U;
    uint32_t rx_idle = 0U;
    uint16_t led_rx_ticks = 0U;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    leds_init();
    rs485_uart_init(UART_BAUD);

    while (1) {
        uint8_t b;
        if (uart_read_byte(&b)) {
            idle = 0U;
            rx_idle = 0U;
            led_rx_ticks = 20U;
            GPIO_SetBits(LED_PORT, LED_RX_PIN);
            if (rx_len < sizeof(rx_buf)) {
                rx_buf[rx_len++] = b;
            } else {
                rx_len = 0U;
            }
        } else {
            idle++;
            if (rx_len > 0U) {
                rx_idle++;
                if (rx_idle >= 5000U) {
                    uint8_t i;
                    delay_ms(50U);
                    GPIO_SetBits(RS485_DE_PORT, RS485_DE_PIN);
                    delay_ms(1U);
                    for (i = 0U; i < rx_len; i++) {
                        uart_write_byte(rx_buf[i]);
                    }
                    while ((USART1->STATR & USART_FLAG_TC) == 0U) { }
                    GPIO_ResetBits(RS485_DE_PORT, RS485_DE_PIN);
                    rx_len = 0U;
                    rx_idle = 0U;
                }
            }
            if (idle >= 200000U) {
                idle = 0U;
                hb = !hb;
                if (hb) GPIO_SetBits(LED_PORT, LED_HB_PIN);
                else GPIO_ResetBits(LED_PORT, LED_HB_PIN);
            }
            if (led_rx_ticks > 0U) {
                led_rx_ticks--;
                if (led_rx_ticks == 0U) {
                    GPIO_ResetBits(LED_PORT, LED_RX_PIN);
                }
            }
        }
    }
}
