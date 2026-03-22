#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "ch32v20x.h"

#define PN0031_STX 0xA0
#define PN0031_ETX 0x0D
#define PN0031_ADDR 0x00
#define PN0031_FT_CMD 0x02
#define PN0031_FC_READ_CARD 0x20
#define PN0031_FC_READ_BLOCK 0x22

#define PN0031_UART_BAUD 115200U
#define BUS_UART_BAUD 115200U

#define UID_MAX_LEN 16
#define FRAME_MAX_DATA 96
#define TAG_ANTENNA_COUNT 4

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

static void delay_ms(uint32_t ms) {
    uint32_t start = g_tick;
    while ((g_tick - start) < ms) { }
}

static void usart1_init_full_duplex(uint32_t baud) {
    GPIO_InitTypeDef gpio = {0};
    USART_InitTypeDef uart = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);

    // CH32V203F6P6 (TSSOP20): use USART1 remap to PB6/PB7.
    // Default PA9/PA10 are not bonded on this package.
    GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Speed = GPIO_Speed_10MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &gpio);

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

static void usart2_init_full_duplex(uint32_t baud) {
    GPIO_InitTypeDef gpio = {0};
    USART_InitTypeDef uart = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // USART2 TX = PA2, RX = PA3.
    gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Speed = GPIO_Speed_10MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

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

static void usart1_write(const uint8_t *buf, uint16_t len) {
    uint16_t i;
    for (i = 0; i < len; i++) {
        while ((USART1->STATR & USART_FLAG_TXE) == 0) { }
        USART1->DATAR = buf[i];
    }
    while ((USART1->STATR & USART_FLAG_TC) == 0) { }
}

static bool usart1_read_byte_timeout(uint8_t *out, uint32_t timeout_ms) {
    uint32_t start = g_tick;
    while ((g_tick - start) < timeout_ms) {
        if ((USART1->STATR & USART_FLAG_RXNE) != 0) {
            *out = (uint8_t)(USART1->DATAR & 0xFFU);
            return true;
        }
    }
    return false;
}

static void usart1_flush_rx(void) {
    while ((USART1->STATR & USART_FLAG_RXNE) != 0) {
        (void)USART1->DATAR;
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
    for (ant = 1; ant <= TAG_ANTENNA_COUNT; ant++) {
        poll_filament(ant, &filaments[ant - 1]);
        delay_ms(10);
    }
    g_bus.pending_read_filament = 0;
}

static void run_rewinder_task(void) {
    // TODO: add rewinder command state-machine here.
    g_bus.pending_rewinder = 0;
}

int main(void) {
    uint32_t last_periodic_read = 0;

    SystemCoreClockUpdate();

    SysTick->SR = 0;
    SysTick->CNT = 0;
    SysTick->CMP = (SystemCoreClock / 1000U) - 1U;
    SysTick->CTLR = 0xF;

    usart1_init_full_duplex(PN0031_UART_BAUD);
    usart2_init_full_duplex(BUS_UART_BAUD);
    delay_ms(20);

    while (1) {
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

        // Example hook: when you enqueue rewinder work, filament read still wins first.
        // bus_request_rewinder();
        delay_ms(5);
    }
}
