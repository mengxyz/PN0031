#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ch32v00X.h>

#define BL_VERSION_MAJOR 0x01
#define BL_VERSION_MINOR 0x00

#define RS485_DE_PORT GPIOD
#define RS485_DE_PIN  GPIO_Pin_4
#define LED_ACT_PORT  GPIOC
#define LED_ACT_PIN   GPIO_Pin_7
#define LED_ERR_PORT  GPIOC
#define LED_ERR_PIN   GPIO_Pin_6

#define APP_BASE_ADDR   0x08002000UL
#define FLASH_END_ADDR  0x0800F800UL
#define FLASH_PAGE_SIZE 256UL
#define APP_MAX_SIZE    (FLASH_END_ADDR - APP_BASE_ADDR)

#define UART_BAUD 115200UL

#define SOF0 0x55
#define SOF1 0xAA

#define CMD_PING   0x01
#define CMD_INFO   0x02
#define CMD_ERASE  0x10
#define CMD_WRITE  0x11
#define CMD_CRC32  0x12
#define CMD_RUN    0x20
#define CMD_RESET  0x21

#define ST_OK          0x00
#define ST_BAD_CMD     0x01
#define ST_BAD_ARG     0x02
#define ST_CRC_ERR     0x03
#define ST_FLASH_ERR   0x04
#define ST_RANGE_ERR   0x05

#define RX_MAX_PAYLOAD 272
#define TX_MAX_PAYLOAD 280

#define BOOT_WAIT_MS   1500U

#define BOOT_FLAG_ADDR0 ((volatile uint32_t *)0x20000000UL)
#define BOOT_FLAG_ADDR1 ((volatile uint32_t *)0x20000004UL)
#define BOOT_FLAG_MAGIC 0x424F4F54UL
#define BOOT_FLAG_MAGIC_INV 0xBDB0B0ABUL

static volatile uint32_t g_tick_ms = 0;

void SysTick_Handler(void)
{
    g_tick_ms++;
}

static uint32_t now_ms(void)
{
    return g_tick_ms;
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = now_ms();
    while ((now_ms() - start) < ms) { }
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
            if (crc & 0x8000U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static uint32_t crc32_ieee(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;
    for (i = 0; i < len; i++) {
        uint8_t b = data[i];
        uint8_t j;
        crc ^= (uint32_t)b;
        for (j = 0; j < 8; j++) {
            if (crc & 1U) {
                crc = (crc >> 1) ^ 0xEDB88320UL;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

static void rs485_set_tx(bool en)
{
    if (en) {
        GPIO_SetBits(RS485_DE_PORT, RS485_DE_PIN);
    } else {
        GPIO_ResetBits(RS485_DE_PORT, RS485_DE_PIN);
    }
}

static void leds_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC, ENABLE);

    gpio.GPIO_Pin = LED_ACT_PIN;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LED_ACT_PORT, &gpio);

    gpio.GPIO_Pin = LED_ERR_PIN;
    GPIO_Init(LED_ERR_PORT, &gpio);

    GPIO_ResetBits(LED_ACT_PORT, LED_ACT_PIN);
    GPIO_ResetBits(LED_ERR_PORT, LED_ERR_PIN);
}

static void led_act_set(bool on)
{
    if (on) {
        GPIO_SetBits(LED_ACT_PORT, LED_ACT_PIN);
    } else {
        GPIO_ResetBits(LED_ACT_PORT, LED_ACT_PIN);
    }
}

static void led_act_toggle(void)
{
    if (GPIO_ReadOutputDataBit(LED_ACT_PORT, LED_ACT_PIN) == Bit_SET) {
        GPIO_ResetBits(LED_ACT_PORT, LED_ACT_PIN);
    } else {
        GPIO_SetBits(LED_ACT_PORT, LED_ACT_PIN);
    }
}

static void led_err_set(bool on)
{
    if (on) {
        GPIO_SetBits(LED_ERR_PORT, LED_ERR_PIN);
    } else {
        GPIO_ResetBits(LED_ERR_PORT, LED_ERR_PIN);
    }
}

static void usart1_init_rs485(uint32_t baud)
{
    GPIO_InitTypeDef gpio = {0};
    USART_InitTypeDef uart = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA | RCC_PB2Periph_GPIOD | RCC_PB2Periph_AFIO | RCC_PB2Periph_USART1, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap1_USART1, ENABLE);

    gpio.GPIO_Pin = RS485_DE_PIN;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(RS485_DE_PORT, &gpio);
    rs485_set_tx(false);

    // RS485 bus on USART1 remap1 mapping TX=PD6 RX=PD5.
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
    uart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &uart);
    USART_Cmd(USART1, ENABLE);
}

static bool uart1_read_byte_timeout(uint8_t *out, uint32_t timeout_ms)
{
    uint32_t start = now_ms();
    while ((now_ms() - start) < timeout_ms) {
        if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET) {
            *out = (uint8_t)(USART_ReceiveData(USART1) & 0xFFU);
            return true;
        }
    }
    return false;
}

static void uart1_write_bytes(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    rs485_set_tx(true);
    delay_ms(1);
    for (i = 0; i < len; i++) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) { }
        USART_SendData(USART1, buf[i]);
    }
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) { }
    rs485_set_tx(false);
}

static void uart1_flush_rx(void)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET) {
        (void)USART_ReceiveData(USART1);
    }
}

static void put_u16le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static void put_u32le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

static uint16_t get_u16le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t get_u32le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void send_resp(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint16_t len)
{
    uint8_t frame[TX_MAX_PAYLOAD + 8];
    uint16_t crc;

    frame[0] = SOF0;
    frame[1] = SOF1;
    frame[2] = (uint8_t)(cmd | 0x80U);
    frame[3] = seq;
    put_u16le(&frame[4], len);
    if (len > 0) {
        memcpy(&frame[6], payload, len);
    }
    crc = crc16_ccitt(&frame[2], (uint16_t)(4 + len));
    put_u16le(&frame[6 + len], crc);

    uart1_write_bytes(frame, (uint16_t)(8 + len));
}

static bool recv_frame(uint8_t *cmd, uint8_t *seq, uint8_t *payload, uint16_t *len, uint32_t timeout_ms)
{
    uint8_t b;
    uint8_t hdr[4];
    uint16_t plen;
    uint16_t crc_rx;
    uint16_t crc_calc;
    uint16_t i;

    while (1) {
        if (!uart1_read_byte_timeout(&b, timeout_ms)) {
            return false;
        }
        if (b == SOF0) {
            break;
        }
    }

    if (!uart1_read_byte_timeout(&b, timeout_ms)) return false;
    if (b != SOF1) return false;

    for (i = 0; i < 4; i++) {
        if (!uart1_read_byte_timeout(&hdr[i], timeout_ms)) return false;
    }

    *cmd = hdr[0];
    *seq = hdr[1];
    plen = get_u16le(&hdr[2]);
    if (plen > RX_MAX_PAYLOAD) {
        return false;
    }

    for (i = 0; i < plen; i++) {
        if (!uart1_read_byte_timeout(&payload[i], timeout_ms)) return false;
    }

    if (!uart1_read_byte_timeout(&b, timeout_ms)) return false;
    crc_rx = b;
    if (!uart1_read_byte_timeout(&b, timeout_ms)) return false;
    crc_rx |= (uint16_t)b << 8;

    {
        uint8_t crc_buf[4 + RX_MAX_PAYLOAD];
        memcpy(crc_buf, hdr, 4);
        if (plen > 0) memcpy(&crc_buf[4], payload, plen);
        crc_calc = crc16_ccitt(crc_buf, (uint16_t)(4 + plen));
    }

    if (crc_calc != crc_rx) {
        return false;
    }

    *len = plen;
    return true;
}

static bool is_app_valid(void)
{
    uint32_t w0 = *(volatile uint32_t *)(APP_BASE_ADDR + 0U);
    uint32_t w1 = *(volatile uint32_t *)(APP_BASE_ADDR + 4U);

    if (w0 == 0xFFFFFFFFUL || w0 == 0x00000000UL) return false;
    if (w1 == 0xFFFFFFFFUL) return false;
    return true;
}

static void force_bootloader_next_reset(void)
{
    *BOOT_FLAG_ADDR0 = BOOT_FLAG_MAGIC;
    *BOOT_FLAG_ADDR1 = BOOT_FLAG_MAGIC_INV;
}

static void reset_to_app(void)
{
    USART_Cmd(USART1, DISABLE);
    rs485_set_tx(false);
    delay_ms(5);
    NVIC_SystemReset();
    while (1) { }
}

static uint8_t handle_cmd(uint8_t cmd, const uint8_t *in, uint16_t in_len, uint8_t *out, uint16_t *out_len)
{
    uint8_t status = ST_OK;
    *out_len = 0;

    switch (cmd) {
        case CMD_PING:
            out[0] = ST_OK;
            out[1] = BL_VERSION_MAJOR;
            out[2] = BL_VERSION_MINOR;
            out[3] = is_app_valid() ? 1U : 0U;
            *out_len = 4;
            break;

        case CMD_INFO:
            out[0] = ST_OK;
            put_u32le(&out[1], APP_BASE_ADDR);
            put_u32le(&out[5], APP_MAX_SIZE);
            put_u16le(&out[9], (uint16_t)FLASH_PAGE_SIZE);
            *out_len = 11;
            break;

        case CMD_ERASE: {
            uint32_t addr;
            uint32_t len;
            FLASH_Status fs;

            if (in_len != 8) {
                status = ST_BAD_ARG;
                break;
            }
            addr = get_u32le(&in[0]);
            len = get_u32le(&in[4]);

            if (addr < APP_BASE_ADDR || (addr + len) > FLASH_END_ADDR || len == 0U) {
                status = ST_RANGE_ERR;
                break;
            }
            if ((addr & (FLASH_PAGE_SIZE - 1U)) != 0U || (len & (FLASH_PAGE_SIZE - 1U)) != 0U) {
                status = ST_BAD_ARG;
                break;
            }

            fs = FLASH_ROM_ERASE(addr, len);
            if (fs != FLASH_COMPLETE) {
                status = ST_FLASH_ERR;
            }

            out[0] = status;
            *out_len = 1;
            break;
        }

        case CMD_WRITE: {
            uint32_t addr;
            uint16_t len;
            uint16_t i;
            uint32_t wbuf[64];
            FLASH_Status fs;

            if (in_len < 6) {
                status = ST_BAD_ARG;
                break;
            }
            addr = get_u32le(&in[0]);
            len = get_u16le(&in[4]);

            if ((uint16_t)(6 + len) != in_len) {
                status = ST_BAD_ARG;
                break;
            }
            if (len == 0U || len > 256U || (len & (FLASH_PAGE_SIZE - 1U)) != 0U) {
                status = ST_BAD_ARG;
                break;
            }
            if (addr < APP_BASE_ADDR || (addr + len) > FLASH_END_ADDR || (addr & (FLASH_PAGE_SIZE - 1U)) != 0U) {
                status = ST_RANGE_ERR;
                break;
            }

            for (i = 0; i < (uint16_t)(len / 4U); i++) {
                const uint8_t *p = &in[6 + i * 4U];
                wbuf[i] = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
            }

            fs = FLASH_ROM_WRITE(addr, wbuf, len);
            if (fs != FLASH_COMPLETE) {
                status = ST_FLASH_ERR;
            }

            out[0] = status;
            *out_len = 1;
            break;
        }

        case CMD_CRC32: {
            uint32_t addr;
            uint32_t len;
            uint32_t crc;

            if (in_len != 8) {
                status = ST_BAD_ARG;
                break;
            }
            addr = get_u32le(&in[0]);
            len = get_u32le(&in[4]);

            if (addr < APP_BASE_ADDR || (addr + len) > FLASH_END_ADDR) {
                status = ST_RANGE_ERR;
                break;
            }

            crc = crc32_ieee((const uint8_t *)addr, len);
            out[0] = ST_OK;
            put_u32le(&out[1], crc);
            *out_len = 5;
            break;
        }

        case CMD_RUN:
            out[0] = is_app_valid() ? ST_OK : ST_RANGE_ERR;
            *out_len = 1;
            break;
        case CMD_RESET:
            out[0] = ST_OK;
            *out_len = 1;
            break;

        default:
            status = ST_BAD_CMD;
            out[0] = status;
            *out_len = 1;
            break;
    }

    return status;
}

int main(void)
{
    uint32_t deadline;
    uint32_t last_led_blink = 0;
    uint32_t err_led_until = 0;
    uint8_t cmd, seq;
    uint16_t in_len;
    uint8_t in_buf[RX_MAX_PAYLOAD];
    uint8_t out_buf[TX_MAX_PAYLOAD];
    uint16_t out_len;
    uint8_t st;
    bool stay_in_boot = false;

    SystemCoreClockUpdate();

    SysTick->SR = 0;
    SysTick->CNT = 0;
    SysTick->CMP = (SystemCoreClock / 1000U) - 1U;
    SysTick->CTLR = 0x0FU;

    leds_init();
    usart1_init_rs485(UART_BAUD);
    uart1_flush_rx();

    if (!is_app_valid()) {
        led_err_set(true);
    }

    deadline = now_ms() + BOOT_WAIT_MS;

    while (1) {
        if ((now_ms() - last_led_blink) >= 200U) {
            last_led_blink = now_ms();
            // Blink while waiting for command/update activity.
            if (!is_app_valid() || stay_in_boot || ((int32_t)(now_ms() - deadline) < 0)) {
                led_act_toggle();
            } else {
                led_act_set(false);
            }
        }

        if (err_led_until != 0U && (int32_t)(now_ms() - err_led_until) >= 0) {
            err_led_until = 0U;
            if (is_app_valid()) {
                led_err_set(false);
            }
        }

        if (!stay_in_boot && is_app_valid() && ((int32_t)(now_ms() - deadline) >= 0)) {
            led_act_set(false);
            reset_to_app();
        }

        if (!recv_frame(&cmd, &seq, in_buf, &in_len, 50U)) {
            continue;
        }

        stay_in_boot = true;
        led_act_toggle();
        st = handle_cmd(cmd, in_buf, in_len, out_buf, &out_len);
        send_resp(cmd, seq, out_buf, out_len);

        if (st != ST_OK) {
            led_err_set(true);
            err_led_until = now_ms() + 500U;
        }

        if (cmd == CMD_RUN && st == ST_OK) {
            led_act_set(false);
            reset_to_app();
        }
        if (cmd == CMD_RESET && st == ST_OK) {
            force_bootloader_next_reset();
            delay_ms(5);
            NVIC_SystemReset();
        }
    }
}
