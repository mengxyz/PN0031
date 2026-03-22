#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ch32v00X.h>

#define PN0031_STX 0xA0
#define PN0031_ETX 0x0D
#define PN0031_ADDR 0x00
#define PN0031_FT_CMD 0x02
#define PN0031_FC_READ_CARD 0x20

#define PN0031_UART_BAUD 115200U
#define UID_MAX_LEN 16
#define FRAME_MAX_DATA 96

#define LED_PORT GPIOC
#define LED_HB_PIN GPIO_Pin_6
#define LED_TAG_PIN GPIO_Pin_7

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
    gpio.GPIO_Pin = (uint16_t)(LED_HB_PIN | LED_TAG_PIN);
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LED_PORT, &gpio);
    GPIO_ResetBits(LED_PORT, (uint16_t)(LED_HB_PIN | LED_TAG_PIN));
}

static void pn_uart_init(uint32_t baud)
{
    GPIO_InitTypeDef gpio = {0};
    USART_InitTypeDef uart = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD | RCC_PB2Periph_AFIO | RCC_PB2Periph_USART2, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap3_USART2, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &gpio);

    USART_StructInit(&uart);
    uart.USART_BaudRate = baud;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &uart);
    USART_Cmd(USART2, ENABLE);
}

static void pn_uart_flush_rx(void)
{
    while ((USART2->STATR & USART_FLAG_RXNE) != 0) {
        (void)USART2->DATAR;
    }
}

static bool pn_uart_read_byte_timeout(uint8_t *out, uint32_t timeout_loops)
{
    while (timeout_loops-- != 0U) {
        if ((USART2->STATR & USART_FLAG_RXNE) != 0) {
            *out = (uint8_t)(USART2->DATAR & 0xFFU);
            return true;
        }
    }
    return false;
}

static void pn_uart_write(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++) {
        while ((USART2->STATR & USART_FLAG_TXE) == 0) { }
        USART2->DATAR = buf[i];
    }
    while ((USART2->STATR & USART_FLAG_TC) == 0) { }
}

static uint8_t xor_checksum(const uint8_t *data, uint16_t len)
{
    uint8_t x = 0;
    uint16_t i;
    for (i = 0; i < len; i++) x ^= data[i];
    return x;
}

static bool pn_send_cmd(uint8_t fc, const uint8_t *data, uint16_t len)
{
    uint8_t frame[8 + FRAME_MAX_DATA];
    if (len > FRAME_MAX_DATA) return false;

    frame[0] = PN0031_STX;
    frame[1] = PN0031_ADDR;
    frame[2] = PN0031_FT_CMD;
    frame[3] = fc;
    frame[4] = (uint8_t)(len & 0xFFU);
    frame[5] = (uint8_t)((len >> 8) & 0xFFU);
    memcpy(&frame[6], data, len);
    frame[6 + len] = xor_checksum(frame, (uint16_t)(6 + len));
    frame[7 + len] = PN0031_ETX;
    pn_uart_write(frame, (uint16_t)(8 + len));
    return true;
}

static bool pn_read_frame(uint8_t expect_fc, uint8_t *data_out, uint16_t *len_out, uint32_t timeout_loops)
{
    uint8_t b;
    uint8_t hdr[5];
    uint16_t i;
    uint16_t data_len;
    uint8_t bcc_rx;
    uint8_t etx_rx;
    uint8_t chk[6 + FRAME_MAX_DATA];

    do {
        if (!pn_uart_read_byte_timeout(&b, timeout_loops)) return false;
    } while (b != PN0031_STX);

    for (i = 0; i < 5; i++) {
        if (!pn_uart_read_byte_timeout(&hdr[i], timeout_loops)) return false;
    }

    data_len = (uint16_t)hdr[3] | ((uint16_t)hdr[4] << 8);
    if (data_len > FRAME_MAX_DATA) return false;

    for (i = 0; i < data_len; i++) {
        if (!pn_uart_read_byte_timeout(&data_out[i], timeout_loops)) return false;
    }
    if (!pn_uart_read_byte_timeout(&bcc_rx, timeout_loops)) return false;
    if (!pn_uart_read_byte_timeout(&etx_rx, timeout_loops)) return false;
    if (etx_rx != PN0031_ETX) return false;

    chk[0] = PN0031_STX;
    chk[1] = hdr[0];
    chk[2] = hdr[1];
    chk[3] = hdr[2];
    chk[4] = hdr[3];
    chk[5] = hdr[4];
    memcpy(&chk[6], data_out, data_len);
    if (xor_checksum(chk, (uint16_t)(6 + data_len)) != bcc_rx) return false;
    if (hdr[1] != PN0031_FT_CMD || hdr[2] != expect_fc) return false;

    *len_out = data_len;
    return true;
}

static bool pn_read_card(uint8_t antenna, uint8_t *status, uint8_t *uid, uint8_t *uid_len)
{
    uint8_t req[1];
    uint8_t resp[FRAME_MAX_DATA];
    uint16_t len = 0;

    req[0] = antenna;

    pn_uart_flush_rx();
    if (!pn_send_cmd(PN0031_FC_READ_CARD, req, 1)) return false;
    if (!pn_read_frame(PN0031_FC_READ_CARD, resp, &len, 400000U)) return false;
    if (len < 1U) return false;

    *status = resp[0];
    if (*status != 0x00U) return true;
    if (len < 3U) return false;

    *uid_len = resp[2];
    if (*uid_len > UID_MAX_LEN) *uid_len = UID_MAX_LEN;
    if ((uint16_t)(3U + *uid_len) > len) return false;
    memcpy(uid, &resp[3], *uid_len);
    return true;
}

int main(void)
{
    uint8_t antenna = 1U;
    uint8_t status = 0xFFU;
    uint8_t uid[UID_MAX_LEN];
    uint8_t uid_len = 0U;
    uint8_t last_uid[UID_MAX_LEN];
    uint8_t last_uid_len = 0U;
    bool hb = false;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    leds_init();
    pn_uart_init(PN0031_UART_BAUD);

    while (1) {
        hb = !hb;
        if (hb) GPIO_SetBits(LED_PORT, LED_HB_PIN);
        else GPIO_ResetBits(LED_PORT, LED_HB_PIN);

        if (pn_read_card(antenna, &status, uid, &uid_len) && status == 0x00U && uid_len > 0U) {
            if (uid_len != last_uid_len || memcmp(uid, last_uid, uid_len) != 0) {
                memcpy(last_uid, uid, uid_len);
                last_uid_len = uid_len;
                if (GPIO_ReadOutputDataBit(LED_PORT, LED_TAG_PIN) == Bit_SET) {
                    GPIO_ResetBits(LED_PORT, LED_TAG_PIN);
                } else {
                    GPIO_SetBits(LED_PORT, LED_TAG_PIN);
                }
            }
        }

        delay_ms(150U);
    }
}
