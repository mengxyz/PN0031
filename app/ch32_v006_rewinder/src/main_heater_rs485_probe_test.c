/* Heater RS485 bus smoke test.
 *
 * Cycles through common CH32V006 USART1/RS485 combinations at 460800:
 *   remap1:  PD6 = TX, PD5 = RX
 *   default: PD5 = TX, PD6 = RX
 *   DE active high and active low
 *
 * PC4 blinks 1..4 times to show the current mode.
 * PC5 pulses when any byte is received.
 */

#include <stdbool.h>
#include <stdint.h>
#include <ch32v00X.h>

#define UART_BAUD 460800U
#define RS485_DE_PORT GPIOD
#define RS485_DE_PIN  GPIO_Pin_4
#define LED_TX_PORT   GPIOC
#define LED_TX_PIN    GPIO_Pin_4
#define LED_RX_PORT   GPIOC
#define LED_RX_PIN    GPIO_Pin_5

static void delay_cycles(volatile uint32_t cycles)
{
    while (cycles-- != 0U) {}
}

static void delay_ms(uint32_t ms)
{
    while (ms-- != 0U) {
        delay_cycles(12000U);
    }
}

static void led_set(GPIO_TypeDef *port, uint16_t pin, bool on)
{
    if (on) GPIO_SetBits(port, pin);
    else GPIO_ResetBits(port, pin);
}

static void leds_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC, ENABLE);
    gpio.GPIO_Pin = LED_TX_PIN | LED_RX_PIN;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &gpio);
    led_set(LED_TX_PORT, LED_TX_PIN, false);
    led_set(LED_RX_PORT, LED_RX_PIN, false);
}

static void de_tx_begin(bool active_high)
{
    if (active_high) GPIO_SetBits(RS485_DE_PORT, RS485_DE_PIN);
    else GPIO_ResetBits(RS485_DE_PORT, RS485_DE_PIN);
    delay_ms(1U);
}

static void de_rx_begin(bool active_high)
{
    if (active_high) GPIO_ResetBits(RS485_DE_PORT, RS485_DE_PIN);
    else GPIO_SetBits(RS485_DE_PORT, RS485_DE_PIN);
}

static void uart_pin_init(bool remap1)
{
    GPIO_InitTypeDef gpio = {0};

    GPIO_PinRemapConfig(GPIO_PartialRemap1_USART1, remap1 ? ENABLE : DISABLE);

    gpio.GPIO_Pin = RS485_DE_PIN;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(RS485_DE_PORT, &gpio);

    gpio.GPIO_Pin = remap1 ? GPIO_Pin_6 : GPIO_Pin_5;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &gpio);

    gpio.GPIO_Pin = remap1 ? GPIO_Pin_5 : GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &gpio);
}

static void rs485_uart_init(bool remap1)
{
    USART_InitTypeDef uart = {0};

    USART_Cmd(USART1, DISABLE);
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD | RCC_PB2Periph_AFIO | RCC_PB2Periph_USART1, ENABLE);
    uart_pin_init(remap1);

    USART_StructInit(&uart);
    uart.USART_BaudRate = UART_BAUD;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &uart);
    USART_Cmd(USART1, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap1_USART1, remap1 ? ENABLE : DISABLE);
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
    while ((USART1->STATR & USART_FLAG_TXE) == 0U) {}
    USART1->DATAR = b;
}

static void rs485_write_bytes(const uint8_t *buf, uint8_t len, bool de_active_high)
{
    uint8_t i;
    de_tx_begin(de_active_high);
    for (i = 0U; i < len; i++) {
        uart_write_byte(buf[i]);
    }
    while ((USART1->STATR & USART_FLAG_TC) == 0U) {}
    de_rx_begin(de_active_high);
}

static void rs485_write_str(const char *s, bool de_active_high)
{
    de_tx_begin(de_active_high);
    while (*s != '\0') {
        uart_write_byte((uint8_t)*s++);
    }
    while ((USART1->STATR & USART_FLAG_TC) == 0U) {}
    de_rx_begin(de_active_high);
}

static void mode_blink(uint8_t count)
{
    uint8_t i;
    for (i = 0U; i < count; i++) {
        led_set(LED_TX_PORT, LED_TX_PIN, true);
        delay_ms(90U);
        led_set(LED_TX_PORT, LED_TX_PIN, false);
        delay_ms(120U);
    }
}

int main(void)
{
    uint8_t rx[64];
    uint8_t rx_len = 0U;
    uint16_t rx_idle = 0U;
    uint16_t tx_tick = 0U;
    uint8_t mode = 0U;
    bool remap1 = true;
    bool de_active_high = true;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    leds_init();
    rs485_uart_init(remap1);
    de_rx_begin(de_active_high);

    while (1) {
        uint8_t b;
        if (uart_read_byte(&b)) {
            led_set(LED_RX_PORT, LED_RX_PIN, true);
            rx_idle = 0U;
            if (rx_len < sizeof(rx)) {
                rx[rx_len++] = b;
            } else {
                rx_len = 0U;
            }
        } else {
            delay_ms(1U);
            if (rx_len > 0U && ++rx_idle >= 20U) {
                rs485_write_bytes(rx, rx_len, de_active_high);
                led_set(LED_RX_PORT, LED_RX_PIN, false);
                rx_len = 0U;
                rx_idle = 0U;
            }

            if (++tx_tick >= 1000U) {
                tx_tick = 0U;
                mode = (uint8_t)((mode + 1U) & 3U);
                remap1 = (mode < 2U);
                de_active_high = ((mode & 1U) == 0U);
                rs485_uart_init(remap1);
                de_rx_begin(de_active_high);
                mode_blink((uint8_t)(mode + 1U));
                if (mode == 0U) rs485_write_str("mode1 remap1 PD6TX PD5RX DE_HIGH\r\n", de_active_high);
                else if (mode == 1U) rs485_write_str("mode2 remap1 PD6TX PD5RX DE_LOW\r\n", de_active_high);
                else if (mode == 2U) rs485_write_str("mode3 default PD5TX PD6RX DE_HIGH\r\n", de_active_high);
                else rs485_write_str("mode4 default PD5TX PD6RX DE_LOW\r\n", de_active_high);
            }
        }
    }
}
