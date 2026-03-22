#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ch32v00X.h>

#define BUS_UART_BAUD 115200U

#define LED_PORT GPIOC
#define LED_OK_PIN GPIO_Pin_6
#define LED_RX_PIN GPIO_Pin_7

#define SOF0 0x55
#define SOF1 0xAA

#define CMD_PING        0x01
#define CMD_GET_VERSION 0x30
#define CMD_LED_SET     0x31

#define ST_OK      0x00
#define ST_BAD_CMD 0x01
#define ST_BAD_ARG 0x02

#define APP_VERSION_MAJOR 0x01
#define APP_VERSION_MINOR 0x00
#define APP_VERSION_PATCH 0x00

#define BUS_RX_MAX_PAYLOAD 32

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
    gpio.GPIO_Pin = (uint16_t)(LED_OK_PIN | LED_RX_PIN);
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LED_PORT, &gpio);
    GPIO_ResetBits(LED_PORT, (uint16_t)(LED_OK_PIN | LED_RX_PIN));
}

static void led_ok_set(bool on)
{
    if (on) GPIO_SetBits(LED_PORT, LED_OK_PIN);
    else GPIO_ResetBits(LED_PORT, LED_OK_PIN);
}

static void led_rx_pulse(void)
{
    GPIO_SetBits(LED_PORT, LED_RX_PIN);
    delay_ms(20);
    GPIO_ResetBits(LED_PORT, LED_RX_PIN);
}

static void bus_uart_init(uint32_t baud)
{
    GPIO_InitTypeDef gpio = {0};
    USART_InitTypeDef uart = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD | RCC_PB2Periph_AFIO | RCC_PB2Periph_USART1, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap1_USART1, ENABLE);

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

static bool bus_uart_read_byte(uint8_t *out)
{
    if ((USART1->STATR & USART_FLAG_RXNE) != 0) {
        *out = (uint8_t)(USART1->DATAR & 0xFFU);
        return true;
    }
    return false;
}

static void bus_uart_write(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++) {
        while ((USART1->STATR & USART_FLAG_TXE) == 0) { }
        USART1->DATAR = buf[i];
    }
    while ((USART1->STATR & USART_FLAG_TC) == 0) { }
}

static uint16_t get_u16le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void put_u16le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    for (i = 0; i < len; i++) {
        uint8_t b = data[i];
        uint8_t j;
        crc ^= (uint16_t)b << 8;
        for (j = 0; j < 8; j++) {
            if (crc & 0x8000U) crc = (uint16_t)((crc << 1) ^ 0x1021U);
            else crc <<= 1;
        }
    }
    return crc;
}

static void bus_send_resp(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint16_t len)
{
    uint8_t frame[6 + BUS_RX_MAX_PAYLOAD + 2];
    uint16_t crc;

    frame[0] = SOF0;
    frame[1] = SOF1;
    frame[2] = (uint8_t)(cmd | 0x80U);
    frame[3] = seq;
    put_u16le(&frame[4], len);
    if (len > 0U) memcpy(&frame[6], payload, len);
    crc = crc16_ccitt(&frame[2], (uint16_t)(4U + len));
    put_u16le(&frame[6 + len], crc);
    bus_uart_write(frame, (uint16_t)(8U + len));
}

static void bus_handle_cmd(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint16_t len)
{
    uint8_t out[BUS_RX_MAX_PAYLOAD];
    uint16_t out_len = 0;

    led_rx_pulse();

    if (cmd == CMD_PING || cmd == CMD_GET_VERSION) {
        out[0] = ST_OK;
        out[1] = APP_VERSION_MAJOR;
        out[2] = APP_VERSION_MINOR;
        out[3] = APP_VERSION_PATCH;
        out_len = 4;
    } else if (cmd == CMD_LED_SET) {
        if (len != 1U) {
            out[0] = ST_BAD_ARG;
            out_len = 1;
        } else {
            led_ok_set(payload[0] != 0U);
            out[0] = ST_OK;
            out_len = 1;
        }
    } else {
        out[0] = ST_BAD_CMD;
        out_len = 1;
    }

    bus_send_resp(cmd, seq, out, out_len);
}

static void bus_poll_protocol(void)
{
    static uint8_t rx_buf[6 + BUS_RX_MAX_PAYLOAD + 2];
    static uint16_t rx_len = 0;
    uint8_t b;

    while (bus_uart_read_byte(&b)) {
        if (rx_len >= sizeof(rx_buf)) rx_len = 0;
        rx_buf[rx_len++] = b;
    }

    while (rx_len >= 2U) {
        uint16_t plen;
        uint16_t frame_len;
        uint16_t crc_rx;
        uint16_t crc_calc;

        if (rx_buf[0] != SOF0 || rx_buf[1] != SOF1) {
            memmove(&rx_buf[0], &rx_buf[1], (size_t)(rx_len - 1U));
            rx_len--;
            continue;
        }

        if (rx_len < 6U) return;

        plen = get_u16le(&rx_buf[4]);
        if (plen > BUS_RX_MAX_PAYLOAD) {
            memmove(&rx_buf[0], &rx_buf[1], (size_t)(rx_len - 1U));
            rx_len--;
            continue;
        }

        frame_len = (uint16_t)(8U + plen);
        if (rx_len < frame_len) return;

        crc_rx = get_u16le(&rx_buf[6U + plen]);
        crc_calc = crc16_ccitt(&rx_buf[2], (uint16_t)(4U + plen));
        if (crc_calc == crc_rx) {
            bus_handle_cmd(rx_buf[2], rx_buf[3], &rx_buf[6], plen);
        }

        if (rx_len > frame_len) {
            memmove(&rx_buf[0], &rx_buf[frame_len], (size_t)(rx_len - frame_len));
        }
        rx_len = (uint16_t)(rx_len - frame_len);
    }
}

int main(void)
{
    bool blink = false;
    uint8_t i;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();

    leds_init();

    // Proven bring-up pattern before any UART activity.
    for (i = 0; i < 6U; i++) {
        led_ok_set((i & 1U) != 0U);
        if ((i & 1U) != 0U) GPIO_ResetBits(LED_PORT, LED_RX_PIN);
        else GPIO_SetBits(LED_PORT, LED_RX_PIN);
        delay_ms(150U);
    }
    GPIO_ResetBits(LED_PORT, (uint16_t)(LED_OK_PIN | LED_RX_PIN));

    bus_uart_init(BUS_UART_BAUD);

    while (1) {
        bus_poll_protocol();
        blink = !blink;
        led_ok_set(blink);
        delay_ms(500U);
    }
}
