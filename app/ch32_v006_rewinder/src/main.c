#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <ch32v00X.h>
#include "ch32v00X_flash.h"
#include "debug.h"
#include "lib/pcf8574_sw_i2c.h"

#define PN0031_STX 0xA0
#define PN0031_ETX 0x0D
#define PN0031_ADDR 0x00
#define PN0031_FT_CMD 0x02
#define PN0031_FC_READ_CARD 0x20
#define PN0031_FC_READ_BLOCK 0x22

#define PN0031_UART_BAUD 115200U
#define BUS_UART_BAUD 115200U

#define RS485_DE_PORT GPIOD
#define RS485_DE_PIN  GPIO_Pin_4
#define READY_LED_PORT GPIOC
#define READY_LED_PIN  GPIO_Pin_6

#define PCF8574_INT_PORT GPIOC
#define PCF8574_INT_PIN  GPIO_Pin_4
#define PCF8574_I2C_PORT GPIOC
#define PCF8574_SDA_PIN  GPIO_Pin_1
#define PCF8574_SCL_PIN  GPIO_Pin_2
#define PCF8574_DEV_COUNT 2U
#define PCF8574_0_ADDR_7B 0x21U
#define PCF8574_1_ADDR_7B 0x20U
#define PCF8574_DEV_MOTOR 0U
#define PCF8574_DEV_INPUT 1U

#define APP_VERSION_MAJOR 0x01
#define APP_VERSION_MINOR 0x00
#define APP_VERSION_PATCH 0x00

#define SOF0 0x55
#define SOF1 0xAA

#define CMD_PING        0x01
#define CMD_GET_VERSION 0x30
#define CMD_READ_UID    0x32
#define CMD_ENTER_BOOT  0x22
#define CMD_MOTOR_CTRL  0x40

#define ST_OK      0x00
#define ST_BAD_CMD 0x01
#define ST_BAD_ARG 0x02
#define ST_IO_ERR  0x04
#define ST_NO_TAG  0x05
#define ST_NOT_READY 0x06

#define MOTOR_OP_STOP    0x00
#define MOTOR_OP_FORWARD 0x01
#define MOTOR_OP_REVERSE 0x02
#define MOTOR_DIR_STOP     0
#define MOTOR_DIR_FORWARD  1
#define MOTOR_DIR_REVERSE -1

#if ENABLE_SDI_LOG
#define LOGF(...) printf(__VA_ARGS__)
#else
#define LOGF(...) do { } while (0)
#endif

#define UID_MAX_LEN 16
#define FRAME_MAX_DATA 96
#define TAG_ANTENNA_COUNT 4
#define BUS_RX_MAX_PAYLOAD 32

typedef struct {
    char bambubus_filament_id[8];
    uint8_t color_R;
    uint8_t color_G;
    uint8_t color_B;
    uint8_t color_A;
    int16_t temperature_min;
    int16_t temperature_max;
    char name[20];
    uint64_t xhub_unique_id;

    uint8_t dryer_power;
    int8_t dryer_temperature;
    uint16_t dryer_time_left;

    bool online;
    uint8_t motion; // 0 = idle
    uint8_t seal_status;
    int8_t compartment_temperature;
    uint8_t compartment_humidity;
    float meters;
    float meters_virtual_count;

    uint8_t antenna;
    uint8_t status;
    uint8_t uid_len;
    uint8_t uid[UID_MAX_LEN];
    bool parsed_ok;

    char variant[9];
    char detailed_type[17];
    char production_time[17];
} FilamentInfo;

static volatile uint32_t g_tick = 0;
static volatile uint8_t g_pcf8574_irq_pending = 0;
static Pcf8574Bus g_pcf8574_bus = {PCF8574_I2C_PORT, PCF8574_SDA_PIN, PCF8574_SCL_PIN, 20U};
static Pcf8574Device g_pcf8574_devs[PCF8574_DEV_COUNT];
static uint8_t g_motor_port_shadow = 0xFFU;
static int8_t g_motor_dir[4] = {MOTOR_DIR_STOP, MOTOR_DIR_STOP, MOTOR_DIR_STOP, MOTOR_DIR_STOP};
FilamentInfo filaments[TAG_ANTENNA_COUNT];

typedef enum {
    BUS_CMD_NONE = 0,
    BUS_CMD_READ_FILAMENT = 1,
    BUS_CMD_REWINDER = 2
} BusCmd;

typedef struct {
    volatile uint8_t pending_read_filament;
    volatile uint8_t pending_rewinder;
} BusState;

static BusState g_bus = {1, 0}; // boot with filament read request

static void bus_request_read_filament(void);
static void bus_request_rewinder(void);
static bool pn_read_card(uint8_t antenna, uint8_t *status, uint8_t *uid, uint8_t *uid_len);

typedef struct {
    uint8_t a_bit;
    uint8_t b_bit;
} Tc118sPair;

// Motor output mapping on PCF8574 #0.
static const Tc118sPair k_tc118s_pairs[4] = {
    {6U, 7U}, // CH1: P6=ch1A, P7=ch1B
    {5U, 4U}, // CH2: P5=ch2A, P4=ch2B
    {0U, 1U}, // CH3: P0=ch3A, P1=ch3B
    {3U, 2U}  // CH4: P3=ch4A, P2=ch4B
};

static uint8_t switch_bit_for_channel(uint8_t channel_1to4) {
    return (uint8_t)(channel_1to4 + 3U); // CH1..4 => P4..P7
}

static bool switch_is_online(uint8_t channel_1to4) {
    const Pcf8574Device *dev = &g_pcf8574_devs[PCF8574_DEV_INPUT];
    uint8_t bit;
    uint8_t mask;

    if (channel_1to4 < 1U || channel_1to4 > 4U) return false;
    bit = switch_bit_for_channel(channel_1to4);
    mask = (uint8_t)(1U << bit);

    // Active-low switch wiring: 0 = filament online/present, 1 = offline.
    return (dev->last_state & mask) == 0U;
}

static void tc118s_flush_outputs(void) {
    (void)pcf8574_write_port(&g_pcf8574_bus, &g_pcf8574_devs[PCF8574_DEV_MOTOR], g_motor_port_shadow);
}

static void tc118s_set_ab(uint8_t channel_1to4, bool a_high, bool b_high) {
    const Tc118sPair *p;
    uint8_t mask_a;
    uint8_t mask_b;

    if (channel_1to4 < 1U || channel_1to4 > 4U) return;
    p = &k_tc118s_pairs[channel_1to4 - 1U];
    mask_a = (uint8_t)(1U << p->a_bit);
    mask_b = (uint8_t)(1U << p->b_bit);

    if (a_high) g_motor_port_shadow |= mask_a; else g_motor_port_shadow &= (uint8_t)~mask_a;
    if (b_high) g_motor_port_shadow |= mask_b; else g_motor_port_shadow &= (uint8_t)~mask_b;
    tc118s_flush_outputs();
}

static void tc118s_set_dir(uint8_t channel_1to4, int8_t dir) {
    if (channel_1to4 < 1U || channel_1to4 > 4U) return;
    if (dir > 0) {
        tc118s_set_ab(channel_1to4, true, false);
        g_motor_dir[channel_1to4 - 1U] = MOTOR_DIR_FORWARD;
    } else if (dir < 0) {
        tc118s_set_ab(channel_1to4, false, true);
        g_motor_dir[channel_1to4 - 1U] = MOTOR_DIR_REVERSE;
    } else {
        tc118s_set_ab(channel_1to4, false, false);
        g_motor_dir[channel_1to4 - 1U] = MOTOR_DIR_STOP;
    }
    LOGF("motor ch=%u dir=%d\r\n", channel_1to4, dir);
}

// --- minimal SHA256/HMAC/HKDF ---
typedef struct {
    uint32_t state[8];
    uint64_t bitlen;
    uint8_t data[64];
    uint32_t datalen;
} sha256_ctx_t;

static const uint32_t k_sha[64] = {
    0x428a2f98U,0x71374491U,0xb5c0fbcfU,0xe9b5dba5U,0x3956c25bU,0x59f111f1U,0x923f82a4U,0xab1c5ed5U,
    0xd807aa98U,0x12835b01U,0x243185beU,0x550c7dc3U,0x72be5d74U,0x80deb1feU,0x9bdc06a7U,0xc19bf174U,
    0xe49b69c1U,0xefbe4786U,0x0fc19dc6U,0x240ca1ccU,0x2de92c6fU,0x4a7484aaU,0x5cb0a9dcU,0x76f988daU,
    0x983e5152U,0xa831c66dU,0xb00327c8U,0xbf597fc7U,0xc6e00bf3U,0xd5a79147U,0x06ca6351U,0x14292967U,
    0x27b70a85U,0x2e1b2138U,0x4d2c6dfcU,0x53380d13U,0x650a7354U,0x766a0abbU,0x81c2c92eU,0x92722c85U,
    0xa2bfe8a1U,0xa81a664bU,0xc24b8b70U,0xc76c51a3U,0xd192e819U,0xd6990624U,0xf40e3585U,0x106aa070U,
    0x19a4c116U,0x1e376c08U,0x2748774cU,0x34b0bcb5U,0x391c0cb3U,0x4ed8aa4aU,0x5b9cca4fU,0x682e6ff3U,
    0x748f82eeU,0x78a5636fU,0x84c87814U,0x8cc70208U,0x90befffaU,0xa4506cebU,0xbef9a3f7U,0xc67178f2U
};

static uint32_t rotr(uint32_t x, uint32_t n) { return (x >> n) | (x << (32U - n)); }
static uint32_t ch(uint32_t x, uint32_t y, uint32_t z) { return (x & y) ^ (~x & z); }
static uint32_t maj(uint32_t x, uint32_t y, uint32_t z) { return (x & y) ^ (x & z) ^ (y & z); }
static uint32_t e0(uint32_t x) { return rotr(x,2) ^ rotr(x,13) ^ rotr(x,22); }
static uint32_t e1(uint32_t x) { return rotr(x,6) ^ rotr(x,11) ^ rotr(x,25); }
static uint32_t s0(uint32_t x) { return rotr(x,7) ^ rotr(x,18) ^ (x >> 3); }
static uint32_t s1(uint32_t x) { return rotr(x,17) ^ rotr(x,19) ^ (x >> 10); }

static void sha256_transform(sha256_ctx_t *ctx, const uint8_t data[64]) {
    uint32_t m[64];
    uint32_t a,b,c,d,e,f,g,h,t1,t2;
    uint32_t i;

    for (i = 0; i < 16; i++) {
        m[i] = ((uint32_t)data[i * 4] << 24) | ((uint32_t)data[i * 4 + 1] << 16) |
               ((uint32_t)data[i * 4 + 2] << 8) | ((uint32_t)data[i * 4 + 3]);
    }
    for (i = 16; i < 64; i++) {
        m[i] = s1(m[i - 2]) + m[i - 7] + s0(m[i - 15]) + m[i - 16];
    }

    a = ctx->state[0]; b = ctx->state[1]; c = ctx->state[2]; d = ctx->state[3];
    e = ctx->state[4]; f = ctx->state[5]; g = ctx->state[6]; h = ctx->state[7];

    for (i = 0; i < 64; i++) {
        t1 = h + e1(e) + ch(e,f,g) + k_sha[i] + m[i];
        t2 = e0(a) + maj(a,b,c);
        h = g; g = f; f = e; e = d + t1;
        d = c; c = b; b = a; a = t1 + t2;
    }

    ctx->state[0] += a; ctx->state[1] += b; ctx->state[2] += c; ctx->state[3] += d;
    ctx->state[4] += e; ctx->state[5] += f; ctx->state[6] += g; ctx->state[7] += h;
}

static void sha256_init(sha256_ctx_t *ctx) {
    ctx->datalen = 0;
    ctx->bitlen = 0;
    ctx->state[0] = 0x6a09e667U; ctx->state[1] = 0xbb67ae85U; ctx->state[2] = 0x3c6ef372U; ctx->state[3] = 0xa54ff53aU;
    ctx->state[4] = 0x510e527fU; ctx->state[5] = 0x9b05688cU; ctx->state[6] = 0x1f83d9abU; ctx->state[7] = 0x5be0cd19U;
}

static void sha256_update(sha256_ctx_t *ctx, const uint8_t *data, uint32_t len) {
    uint32_t i;
    for (i = 0; i < len; i++) {
        ctx->data[ctx->datalen++] = data[i];
        if (ctx->datalen == 64) {
            sha256_transform(ctx, ctx->data);
            ctx->bitlen += 512;
            ctx->datalen = 0;
        }
    }
}

static void sha256_final(sha256_ctx_t *ctx, uint8_t hash[32]) {
    uint32_t i;
    uint32_t dl = ctx->datalen;

    ctx->data[dl++] = 0x80U;
    if (dl > 56) {
        while (dl < 64) ctx->data[dl++] = 0x00U;
        sha256_transform(ctx, ctx->data);
        dl = 0;
    }
    while (dl < 56) ctx->data[dl++] = 0x00U;

    ctx->bitlen += (uint64_t)ctx->datalen * 8ULL;
    for (i = 0; i < 8; i++) {
        ctx->data[63 - i] = (uint8_t)(ctx->bitlen >> (8U * i));
    }
    sha256_transform(ctx, ctx->data);

    for (i = 0; i < 4; i++) {
        hash[i]      = (uint8_t)(ctx->state[0] >> (24U - i * 8U));
        hash[i + 4]  = (uint8_t)(ctx->state[1] >> (24U - i * 8U));
        hash[i + 8]  = (uint8_t)(ctx->state[2] >> (24U - i * 8U));
        hash[i + 12] = (uint8_t)(ctx->state[3] >> (24U - i * 8U));
        hash[i + 16] = (uint8_t)(ctx->state[4] >> (24U - i * 8U));
        hash[i + 20] = (uint8_t)(ctx->state[5] >> (24U - i * 8U));
        hash[i + 24] = (uint8_t)(ctx->state[6] >> (24U - i * 8U));
        hash[i + 28] = (uint8_t)(ctx->state[7] >> (24U - i * 8U));
    }
}

static void hmac_sha256(const uint8_t *key, uint32_t key_len, const uint8_t *msg, uint32_t msg_len, uint8_t out[32]) {
    uint8_t k0[64];
    uint8_t kipad[64];
    uint8_t kopad[64];
    uint8_t tmp[32];
    uint32_t i;
    sha256_ctx_t c;

    memset(k0, 0, sizeof(k0));
    if (key_len > 64U) {
        sha256_init(&c);
        sha256_update(&c, key, key_len);
        sha256_final(&c, tmp);
        memcpy(k0, tmp, 32U);
    } else {
        memcpy(k0, key, key_len);
    }

    for (i = 0; i < 64U; i++) {
        kipad[i] = (uint8_t)(k0[i] ^ 0x36U);
        kopad[i] = (uint8_t)(k0[i] ^ 0x5CU);
    }

    sha256_init(&c);
    sha256_update(&c, kipad, 64U);
    sha256_update(&c, msg, msg_len);
    sha256_final(&c, tmp);

    sha256_init(&c);
    sha256_update(&c, kopad, 64U);
    sha256_update(&c, tmp, 32U);
    sha256_final(&c, out);
}

static void hkdf_expand_sha256(const uint8_t prk[32], const uint8_t *info, uint8_t info_len, uint8_t *okm, uint16_t okm_len) {
    uint8_t t[32];
    uint8_t ctr = 1U;
    uint16_t out_pos = 0;
    uint8_t prev_len = 0;
    uint8_t msg[64];

    memset(t, 0, sizeof(t));
    while (out_pos < okm_len) {
        uint8_t msg_len = 0;
        if (prev_len > 0) {
            memcpy(msg, t, prev_len);
            msg_len = prev_len;
        }
        memcpy(msg + msg_len, info, info_len);
        msg_len += info_len;
        msg[msg_len++] = ctr;

        hmac_sha256(prk, 32U, msg, msg_len, t);
        prev_len = 32U;

        {
            uint16_t copy_len = (uint16_t)((okm_len - out_pos) < 32U ? (okm_len - out_pos) : 32U);
            memcpy(okm + out_pos, t, copy_len);
            out_pos += copy_len;
        }
        ctr++;
    }
}

static void derive_sector_keys(const uint8_t *uid, uint8_t uid_len, uint8_t keys_a[16][6], uint8_t keys_b[16][6]) {
    static const uint8_t salt[16] = {
        0x9a,0x75,0x9c,0xf2,0xc4,0xf7,0xca,0xff,0x22,0x2c,0xb9,0x76,0x9b,0x41,0xbc,0x96
    };
    static const uint8_t info_a[7] = {'R','F','I','D','-','A',0x00};
    static const uint8_t info_b[7] = {'R','F','I','D','-','B',0x00};
    uint8_t prk[32];
    uint8_t okm_a[96];
    uint8_t okm_b[96];
    uint8_t i;

    hmac_sha256(salt, sizeof(salt), uid, uid_len, prk);
    hkdf_expand_sha256(prk, info_a, sizeof(info_a), okm_a, sizeof(okm_a));
    hkdf_expand_sha256(prk, info_b, sizeof(info_b), okm_b, sizeof(okm_b));

    for (i = 0; i < 16U; i++) {
        memcpy(keys_a[i], &okm_a[i * 6U], 6U);
        memcpy(keys_b[i], &okm_b[i * 6U], 6U);
    }
}

// --- platform ---
void SysTick_Handler(void) { g_tick++; }
void EXTI7_0_IRQHandler(void) {
    if ((EXTI->INTFR & EXTI_Line4) != 0U) {
        EXTI->INTFR = EXTI_Line4; // write 1 to clear pending
        g_pcf8574_irq_pending = 1;
    }
}

static void delay_ms(uint32_t ms) {
    uint32_t start = g_tick;
    while ((g_tick - start) < ms) { }
}

static void usart1_init_full_duplex(uint32_t baud) {
    GPIO_InitTypeDef gpio = {0};
    USART_InitTypeDef uart = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD | RCC_PB2Periph_AFIO | RCC_PB2Periph_USART2, ENABLE);

    // PN0031 on USART2 remap3: TX=PD2, RX=PD3.
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

static void usart2_init_full_duplex(uint32_t baud) {
    GPIO_InitTypeDef gpio = {0};
    USART_InitTypeDef uart = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD | RCC_PB2Periph_AFIO | RCC_PB2Periph_USART1, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap1_USART1, ENABLE);

    gpio.GPIO_Pin = RS485_DE_PIN;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(RS485_DE_PORT, &gpio);
    GPIO_ResetBits(RS485_DE_PORT, RS485_DE_PIN); // RX mode (DE=0)

    // Bus UART on USART1 remap1: TX=PD6, RX=PD5.
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

static void pcf8574_int_init(void) {
    GPIO_InitTypeDef gpio = {0};
    NVIC_InitTypeDef nvic = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC | RCC_PB2Periph_AFIO, ENABLE);

    gpio.GPIO_Pin = PCF8574_INT_PIN;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_IPU; // PCF8574 INT is active-low open-drain
    GPIO_Init(PCF8574_INT_PORT, &gpio);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);
    EXTI->INTENR |= EXTI_Line4;     // interrupt enable
    EXTI->EVENR &= ~EXTI_Line4;     // event disable
    EXTI->RTENR &= ~EXTI_Line4;     // rising edge disable
    EXTI->FTENR |= EXTI_Line4;      // falling edge enable
    EXTI->INTFR = EXTI_Line4;       // clear stale pending

    nvic.NVIC_IRQChannel = EXTI7_0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

static void ready_led_init(void) {
    GPIO_InitTypeDef gpio = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC, ENABLE);
    gpio.GPIO_Pin = READY_LED_PIN;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(READY_LED_PORT, &gpio);
    GPIO_ResetBits(READY_LED_PORT, READY_LED_PIN);
}

static void ready_led_set(bool on) {
    if (on) {
        GPIO_SetBits(READY_LED_PORT, READY_LED_PIN);
    } else {
        GPIO_ResetBits(READY_LED_PORT, READY_LED_PIN);
    }
}

static void pcf8574_process_irq(void) {
    Pcf8574Device *dev = &g_pcf8574_devs[PCF8574_DEV_INPUT];
    uint8_t v;
    uint8_t ch;

    if (!pcf8574_refresh(&g_pcf8574_bus, dev)) {
        LOGF("pcf irq refresh fail\r\n");
        return;
    }
    v = dev->last_state;
    LOGF("pcf irq state=0x%02X changed=0x%02X\r\n", v, dev->changed_mask);

    // PCF8574 #1 inputs: P4..P7 = SW1..SW4 (active-low filament-present switches).
    // If a channel is rewinding and switch reports offline, stop immediately.
    for (ch = 1U; ch <= 4U; ch++) {
        uint8_t bit = switch_bit_for_channel(ch);
        uint8_t mask = (uint8_t)(1U << bit);
        if ((dev->changed_mask & mask) != 0U && (v & mask) != 0U) {
            if (g_motor_dir[ch - 1U] == MOTOR_DIR_REVERSE) {
                LOGF("switch offline stop ch=%u\r\n", ch);
                tc118s_set_dir(ch, MOTOR_DIR_STOP);
            }
        }
    }
}

static void usart1_write(const uint8_t *buf, uint16_t len) {
    uint16_t i;
    for (i = 0; i < len; i++) {
        while ((USART2->STATR & USART_FLAG_TXE) == 0) { }
        USART2->DATAR = buf[i];
    }
    while ((USART2->STATR & USART_FLAG_TC) == 0) { }
}

static bool usart1_read_byte_timeout(uint8_t *out, uint32_t timeout_ms) {
    uint32_t start = g_tick;
    while ((g_tick - start) < timeout_ms) {
        if ((USART2->STATR & USART_FLAG_RXNE) != 0) {
            *out = (uint8_t)(USART2->DATAR & 0xFFU);
            return true;
        }
    }
    return false;
}

static void usart1_flush_rx(void) {
    while ((USART2->STATR & USART_FLAG_RXNE) != 0) {
        (void)USART2->DATAR;
    }
}

static bool bus_uart_read_byte(uint8_t *out) {
    if ((USART1->STATR & USART_FLAG_RXNE) != 0) {
        *out = (uint8_t)(USART1->DATAR & 0xFFU);
        return true;
    }
    return false;
}

static void bus_uart_write(const uint8_t *buf, uint16_t len) {
    uint16_t i;
    volatile uint16_t t;
    GPIO_SetBits(RS485_DE_PORT, RS485_DE_PIN);
    for (t = 0; t < 200U; t++) { } // short turn-around delay
    for (i = 0; i < len; i++) {
        while ((USART1->STATR & USART_FLAG_TXE) == 0) { }
        USART1->DATAR = buf[i];
    }
    while ((USART1->STATR & USART_FLAG_TC) == 0) { }
    GPIO_ResetBits(RS485_DE_PORT, RS485_DE_PIN);
}

static uint16_t bus_get_u16le(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void bus_put_u16le(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len) {
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

static void bus_send_resp(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint16_t len) {
    uint8_t frame[6 + BUS_RX_MAX_PAYLOAD + 2];
    uint16_t crc;

    if (len > BUS_RX_MAX_PAYLOAD) return;

    frame[0] = SOF0;
    frame[1] = SOF1;
    frame[2] = (uint8_t)(cmd | 0x80U);
    frame[3] = seq;
    bus_put_u16le(&frame[4], len);
    if (len > 0U) {
        memcpy(&frame[6], payload, len);
    }
    crc = crc16_ccitt(&frame[2], (uint16_t)(4 + len));
    bus_put_u16le(&frame[6 + len], crc);
    bus_uart_write(frame, (uint16_t)(8 + len));
}

static void bus_handle_cmd(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint16_t len) {
    uint8_t out[BUS_RX_MAX_PAYLOAD];
    uint16_t out_len = 0;
    bool enter_boot = false;

    LOGF("bus cmd=0x%02X seq=%u len=%u\r\n", cmd, seq, len);

    if (cmd == CMD_GET_VERSION || cmd == CMD_PING) {
        out[0] = ST_OK;
        out[1] = APP_VERSION_MAJOR;
        out[2] = APP_VERSION_MINOR;
        out[3] = APP_VERSION_PATCH;
        out_len = 4;
    } else if (cmd == CMD_READ_UID) {
        uint8_t ant;
        uint8_t status = 0xFFU;
        uint8_t uid[UID_MAX_LEN];
        uint8_t uid_len = 0U;

        if (len != 1U) {
            out[0] = ST_BAD_ARG;
            out_len = 1;
        } else {
            ant = payload[0];
            if (ant < 1U || ant > 4U) {
                out[0] = ST_BAD_ARG;
                out_len = 1;
            } else if (!pn_read_card(ant, &status, uid, &uid_len)) {
                out[0] = ST_IO_ERR;
                out_len = 1;
            } else if (status != 0x00U || uid_len == 0U) {
                out[0] = ST_NO_TAG;
                out[1] = status;
                out_len = 2;
            } else {
                out[0] = ST_OK;
                out[1] = ant;
                out[2] = status;
                out[3] = uid_len;
                memcpy(&out[4], uid, uid_len);
                out_len = (uint16_t)(4U + uid_len);
            }
        }
    } else if (cmd == CMD_ENTER_BOOT) {
        if (len != 0U) {
            out[0] = ST_BAD_ARG;
            out_len = 1;
        } else {
            FLASH_Unlock_Fast();
            FLASH_ErasePage_Fast((0x08004000UL - 4UL) & 0xFFFFFF00UL);
            FLASH->CTLR |= ((uint32_t)0x00008000);
            FLASH->CTLR |= ((uint32_t)0x00000080);
            out[0] = ST_OK;
            out_len = 1;
            enter_boot = true;
        }
    } else if (cmd == CMD_MOTOR_CTRL) {
        uint8_t ch;
        uint8_t op;
        if (len != 2U) {
            out[0] = ST_BAD_ARG;
            out_len = 1;
        } else {
            ch = payload[0];
            op = payload[1];
            if (ch < 1U || ch > 4U) {
                out[0] = ST_BAD_ARG;
                out_len = 1;
            } else if (op == MOTOR_OP_STOP) {
                tc118s_set_dir(ch, MOTOR_DIR_STOP);
                out[0] = ST_OK;
                out_len = 1;
            } else if (op == MOTOR_OP_FORWARD) {
                if (!pcf8574_refresh(&g_pcf8574_bus, &g_pcf8574_devs[PCF8574_DEV_INPUT])) {
                    LOGF("motor ch=%u op=fwd input refresh fail\r\n", ch);
                    out[0] = ST_NOT_READY;
                    out_len = 1;
                } else if (!switch_is_online(ch)) {
                    LOGF("motor ch=%u op=fwd rejected offline\r\n", ch);
                    out[0] = ST_NOT_READY;
                    out_len = 1;
                } else {
                    tc118s_set_dir(ch, MOTOR_DIR_FORWARD);
                    out[0] = ST_OK;
                    out_len = 1;
                }
            } else if (op == MOTOR_OP_REVERSE) {
                if (!pcf8574_refresh(&g_pcf8574_bus, &g_pcf8574_devs[PCF8574_DEV_INPUT])) {
                    LOGF("motor ch=%u op=rev input refresh fail\r\n", ch);
                    out[0] = ST_NOT_READY;
                    out_len = 1;
                } else if (!switch_is_online(ch)) {
                    LOGF("motor ch=%u op=rev rejected offline\r\n", ch);
                    out[0] = ST_NOT_READY;
                    out_len = 1;
                } else {
                    tc118s_set_dir(ch, MOTOR_DIR_REVERSE);
                    out[0] = ST_OK;
                    out_len = 1;
                }
            } else {
                out[0] = ST_BAD_ARG;
                out_len = 1;
            }
        }
    } else {
        out[0] = ST_BAD_CMD;
        out_len = 1;
    }

    bus_send_resp(cmd, seq, out, out_len);

    if (enter_boot) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) { }
        RCC_ClearFlag();
        SystemReset_StartMode(Start_Mode_BOOT);
        NVIC_SystemReset();
        while (1) { }
    }
}

static void bus_poll_protocol(void) {
    static uint8_t rx_buf[6 + BUS_RX_MAX_PAYLOAD + 2];
    static uint16_t rx_len = 0;
    uint8_t b;

    while (bus_uart_read_byte(&b)) {
        if (rx_len >= sizeof(rx_buf)) {
            rx_len = 0;
        }
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

        plen = bus_get_u16le(&rx_buf[4]);
        if (plen > BUS_RX_MAX_PAYLOAD) {
            memmove(&rx_buf[0], &rx_buf[1], (size_t)(rx_len - 1U));
            rx_len--;
            continue;
        }

        frame_len = (uint16_t)(8U + plen);
        if (rx_len < frame_len) return;

        crc_rx = bus_get_u16le(&rx_buf[6U + plen]);
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

// --- PN0031 ---
static uint8_t xor_checksum(const uint8_t *data, uint16_t len) {
    uint8_t x = 0;
    uint16_t i;
    for (i = 0; i < len; i++) x ^= data[i];
    return x;
}

static bool pn_send_cmd(uint8_t fc, const uint8_t *data, uint16_t len) {
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
    usart1_write(frame, (uint16_t)(8 + len));
    return true;
}

static bool pn_read_frame(uint8_t expect_fc, uint8_t *data_out, uint16_t *len_out, uint32_t timeout_ms) {
    uint8_t b;
    uint8_t hdr[5];
    uint16_t i;
    uint16_t data_len;
    uint8_t bcc_rx;
    uint8_t etx_rx;
    uint8_t chk[6 + FRAME_MAX_DATA];

    do {
        if (!usart1_read_byte_timeout(&b, timeout_ms)) return false;
    } while (b != PN0031_STX);

    for (i = 0; i < 5; i++) {
        if (!usart1_read_byte_timeout(&hdr[i], timeout_ms)) return false;
    }
    data_len = (uint16_t)hdr[3] | ((uint16_t)hdr[4] << 8);
    if (data_len > FRAME_MAX_DATA) return false;

    for (i = 0; i < data_len; i++) {
        if (!usart1_read_byte_timeout(&data_out[i], timeout_ms)) return false;
    }
    if (!usart1_read_byte_timeout(&bcc_rx, timeout_ms)) return false;
    if (!usart1_read_byte_timeout(&etx_rx, timeout_ms)) return false;
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

static bool pn_read_card(uint8_t antenna, uint8_t *status, uint8_t *uid, uint8_t *uid_len) {
    uint8_t req[1];
    uint8_t resp[FRAME_MAX_DATA];
    uint16_t len = 0;
    req[0] = antenna;

    usart1_flush_rx();
    if (!pn_send_cmd(PN0031_FC_READ_CARD, req, 1)) return false;
    if (!pn_read_frame(PN0031_FC_READ_CARD, resp, &len, 100)) return false;
    if (len < 1) return false;

    *status = resp[0];
    if (*status != 0x00) return true;
    if (len < 3) return false;

    *uid_len = resp[2];
    if (*uid_len > UID_MAX_LEN) *uid_len = UID_MAX_LEN;
    if ((uint16_t)(3 + *uid_len) > len) return false;
    memcpy(uid, &resp[3], *uid_len);
    return true;
}

static bool pn_read_block(uint8_t antenna, uint8_t block, uint8_t key_type, const uint8_t key6[6], uint8_t out16[16]) {
    uint8_t req[10];
    uint8_t resp[FRAME_MAX_DATA];
    uint16_t len = 0;

    req[0] = antenna;
    req[1] = block;
    req[2] = 0x00;       // key position transport
    req[3] = key_type;   // 0 KEYA, 1 KEYB
    memcpy(&req[4], key6, 6);

    usart1_flush_rx();
    if (!pn_send_cmd(PN0031_FC_READ_BLOCK, req, 10)) return false;
    if (!pn_read_frame(PN0031_FC_READ_BLOCK, resp, &len, 120)) return false;
    if (len < 1) return false;
    if (resp[0] != 0x00) return false;
    if (len != 19) return false;
    memcpy(out16, &resp[3], 16);
    return true;
}

// --- parse helpers ---
static uint16_t u16le(const uint8_t *d) { return (uint16_t)d[0] | ((uint16_t)d[1] << 8); }
static float f32le(const uint8_t *d) {
    union { uint32_t u; float f; } v;
    v.u = (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
    return v.f;
}
static uint64_t u64be(const uint8_t *d) {
    uint64_t v = 0;
    uint8_t i;
    for (i = 0; i < 8; i++) {
        v = (v << 8) | d[i];
    }
    return v;
}

static void copy_ascii_trim(char *dst, uint8_t dst_sz, const uint8_t *src, uint8_t n) {
    uint8_t i;
    uint8_t out = 0;
    for (i = 0; i < n && out + 1 < dst_sz; i++) {
        uint8_t c = src[i];
        if (c == 0) break;
        if (c >= 32 && c <= 126) {
            dst[out++] = (char)c;
        }
    }
    dst[out] = 0;
}

static void filament_init_defaults(FilamentInfo *f) {
    memset(f, 0, sizeof(*f));
    memcpy(f->bambubus_filament_id, "GFG00", 6);
    f->color_R = 0xFF;
    f->color_G = 0xFF;
    f->color_B = 0xFF;
    f->color_A = 0xFF;
    f->temperature_min = 220;
    f->temperature_max = 240;
    memcpy(f->name, "PETG", 5);
    f->xhub_unique_id = 0;
    f->dryer_power = 0;
    f->dryer_temperature = 0;
    f->dryer_time_left = 0;
    f->online = true;
    f->motion = 0;
    f->seal_status = 0;
    f->compartment_temperature = 0;
    f->compartment_humidity = 0;
    f->meters = 1.0f;
    f->meters_virtual_count = 0.0f;
}

static void parse_filament_blocks(FilamentInfo *f, const uint8_t blocks[64][16]) {
    int16_t nmin;
    int16_t nmax;

    copy_ascii_trim(f->variant, sizeof(f->variant), &blocks[1][0], 8);
    copy_ascii_trim(f->bambubus_filament_id, sizeof(f->bambubus_filament_id), &blocks[1][8], 8);
    copy_ascii_trim(f->name, sizeof(f->name), &blocks[2][0], 16);
    copy_ascii_trim(f->detailed_type, sizeof(f->detailed_type), &blocks[4][0], 16);

    f->color_R = blocks[5][0];
    f->color_G = blocks[5][1];
    f->color_B = blocks[5][2];
    f->color_A = blocks[5][3];
    (void)f32le(&blocks[5][8]); // diameter available if needed

    f->dryer_temperature = (int8_t)u16le(&blocks[6][0]);
    f->dryer_time_left = u16le(&blocks[6][2]);
    nmin = (int16_t)u16le(&blocks[6][8]);
    nmax = (int16_t)u16le(&blocks[6][10]);
    if (nmin > nmax) {
        int16_t t = nmin;
        nmin = nmax;
        nmax = t;
    }
    f->temperature_min = nmin;
    f->temperature_max = nmax;
    f->xhub_unique_id = u64be(&blocks[9][0]);
    copy_ascii_trim(f->production_time, sizeof(f->production_time), &blocks[12][0], 16);
    f->parsed_ok = true;
}

static void poll_filament(uint8_t antenna, FilamentInfo *f) {
    uint8_t status = 0xFF;
    uint8_t uid[UID_MAX_LEN];
    uint8_t uid_len = 0;
    uint8_t keys_a[16][6];
    uint8_t keys_b[16][6];
    static const uint8_t needed_blocks[] = {1,2,4,5,6,9,12,16};
    uint8_t raw[64][16];
    uint8_t i;

    filament_init_defaults(f);
    f->antenna = antenna;

    if (!pn_read_card(antenna, &status, uid, &uid_len)) {
        f->status = 0xFE; // comm fail
        f->online = false;
        return;
    }
    f->status = status;
    if (status != 0x00 || uid_len == 0U) {
        f->online = false;
        return;
    }

    f->online = true;
    f->uid_len = uid_len;
    memcpy(f->uid, uid, uid_len);
    memset(raw, 0, sizeof(raw));

    derive_sector_keys(uid, uid_len, keys_a, keys_b);

    for (i = 0; i < sizeof(needed_blocks); i++) {
        uint8_t b = needed_blocks[i];
        uint8_t sec = (uint8_t)(b / 4U);

        if (!pn_read_block(antenna, b, 0, keys_a[sec], raw[b])) {
            if (!pn_read_block(antenna, b, 1, keys_b[sec], raw[b])) {
                f->parsed_ok = false;
                return;
            }
        }
        delay_ms(3);
    }

    parse_filament_blocks(f, raw);
}

static void bus_request_read_filament(void) { g_bus.pending_read_filament = 1; }
static void bus_request_rewinder(void) { g_bus.pending_rewinder = 1; }

static BusCmd bus_next_cmd(void) {
    // Filament read always has higher priority than rewinder actions.
    if (g_bus.pending_read_filament) return BUS_CMD_READ_FILAMENT;
    if (g_bus.pending_rewinder) return BUS_CMD_REWINDER;
    return BUS_CMD_NONE;
}

static void run_read_filament_task(void) {
    uint8_t ant;
    LOGF("filament read start\r\n");
    for (ant = 1; ant <= TAG_ANTENNA_COUNT; ant++) {
        bus_poll_protocol();
        poll_filament(ant, &filaments[ant - 1]);
        delay_ms(10);
    }
    g_bus.pending_read_filament = 0;
    LOGF("filament read done\r\n");
}

static void run_rewinder_task(void) {
    // Safe default: stop all mapped TC118S channels.
    tc118s_set_dir(1, MOTOR_DIR_STOP);
    tc118s_set_dir(2, MOTOR_DIR_STOP);
    tc118s_set_dir(3, MOTOR_DIR_STOP);
    tc118s_set_dir(4, MOTOR_DIR_STOP);
    g_bus.pending_rewinder = 0;
}

int main(void) {
    uint32_t last_periodic_read = 0;
    uint32_t last_log_ping = 0;
    uint32_t last_ready_blink = 0;
    bool pcf_motor_ok;
    bool pcf_input_write_ok;
    bool pcf_motor_refresh_ok;
    bool pcf_input_refresh_ok;
    bool pcf_ready;

    SystemCoreClockUpdate();

    SysTick->SR = 0;
    SysTick->CNT = 0;
    SysTick->CMP = (SystemCoreClock / 1000U) - 1U;
    SysTick->CTLR = 0xF;

    // SDI_Printf_Enable();
    LOGF("rewinder boot v=%u.%u.%u\r\n", APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_VERSION_PATCH);

    ready_led_init();
    usart1_init_full_duplex(PN0031_UART_BAUD);
    usart2_init_full_duplex(BUS_UART_BAUD);
    pcf8574_bus_init(&g_pcf8574_bus);
    pcf8574_device_init(&g_pcf8574_devs[0], PCF8574_0_ADDR_7B);
    pcf8574_device_init(&g_pcf8574_devs[1], PCF8574_1_ADDR_7B);
    pcf8574_int_init();
    g_motor_port_shadow = 0x00U;
    pcf_motor_ok = pcf8574_write_port(&g_pcf8574_bus, &g_pcf8574_devs[PCF8574_DEV_MOTOR], g_motor_port_shadow);
    pcf_input_write_ok = pcf8574_write_port(&g_pcf8574_bus, &g_pcf8574_devs[1], 0xFFU);
    pcf_motor_refresh_ok = pcf8574_refresh(&g_pcf8574_bus, &g_pcf8574_devs[0]);
    pcf_input_refresh_ok = pcf8574_refresh(&g_pcf8574_bus, &g_pcf8574_devs[1]);
    delay_ms(20);
    pcf_ready = pcf_motor_ok && pcf_input_write_ok && pcf_motor_refresh_ok && pcf_input_refresh_ok;
    ready_led_set(pcf_ready);
    LOGF("init done motor=0x%02X in0=0x%02X in1=0x%02X\r\n",
         g_motor_port_shadow,
         g_pcf8574_devs[0].last_state,
         g_pcf8574_devs[1].last_state);

    while (1) {
        bus_poll_protocol();
        if (g_pcf8574_irq_pending != 0U) {
            g_pcf8574_irq_pending = 0;
            pcf8574_process_irq();
        }

        BusCmd cmd = bus_next_cmd();
        switch (cmd) {
            case BUS_CMD_READ_FILAMENT:
                run_read_filament_task();
                break;
            case BUS_CMD_REWINDER:
                run_rewinder_task();
                break;
            default:
                break;
        }

        // Keep filament data fresh even when no explicit command is queued.
        if ((g_tick - last_periodic_read) > 1000U) {
            last_periodic_read = g_tick;
            bus_request_read_filament();
        }

        if ((g_tick - last_log_ping) > 1000U) {
            last_log_ping = g_tick;
            LOGF("ping tick=%lu\r\n", (unsigned long)g_tick);
        }

        if (!pcf_ready && (g_tick - last_ready_blink) > 300U) {
            last_ready_blink = g_tick;
            if (GPIO_ReadOutputDataBit(READY_LED_PORT, READY_LED_PIN) == Bit_SET) {
                GPIO_ResetBits(READY_LED_PORT, READY_LED_PIN);
            } else {
                GPIO_SetBits(READY_LED_PORT, READY_LED_PIN);
            }
        }

        // Example hook: when you enqueue rewinder work, filament read still wins first.
        // bus_request_rewinder();
    }
}
