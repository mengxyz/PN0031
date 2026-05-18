#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <ch32v00X.h>
#include <ch32v00X_dma.h>
#include <ch32v00X_spi.h>
#include <ch32v00X_tim.h>
#include "ch32v00X_flash.h"
#include "lib/pcf8574_sw_i2c.h"

#define AHBPCENR HBPCENR
#define APB2PCENR PB2PCENR
#define RCC_AHBPeriph_DMA1 RCC_HBPeriph_DMA1
#define RCC_APB2Periph_GPIOC RCC_PB2Periph_GPIOC
#define RCC_APB2Periph_SPI1 RCC_PB2Periph_SPI1
#define GPIO_Speed_10MHz GPIO_Speed_30MHz
#define GPIO_CNF_OUT_PP_AF 0x08
#define CTLR1_SPE_Set ((uint16_t)0x0040)

#define WS_LED_COUNT 5
#define WS2812DMA_IMPLEMENTATION
#define DMALEDS 8
#define WSGRB
#include "../../../docs/ws2812b/ws2812b_dma_spi_led_driver.h"

#define BUS_UART_BAUD 460800U
#define PN0031_UART_BAUD 115200U
#define PWM_PC3_FREQ_HZ 5000U
#define PWM_PC3_DUTY_PCT 0U
#define PWM_PC7_FREQ_HZ 5000U
#define PWM_PC7_DUTY_PCT 0U
#define PWM_PC5_FREQ_HZ 5000U
#define PWM_PC5_DUTY_PCT 0U
#define PWM_PC0_FREQ_HZ 5000U
#define PWM_PC0_DUTY_PCT 0U
#define USE_PC6_WS2812 1
#define USE_PCF8574 1
#define USE_PN0031 1
#define USE_PWM_OUTPUTS 1

#define RS485_DE_PORT GPIOD
#define RS485_DE_PIN  GPIO_Pin_4

#define PCF8574_INT_PORT GPIOC
#define PCF8574_INT_PIN  GPIO_Pin_4
#define PCF8574_I2C_PORT GPIOC
#define PCF8574_SDA_PIN  GPIO_Pin_1
#define PCF8574_SCL_PIN  GPIO_Pin_2
#define PCF8574_ADDR_7B 0x20U

#define LED_STAT_PORT GPIOD
#define LED_STAT1_PIN GPIO_Pin_0
#define LED_STAT2_PIN GPIO_Pin_1

#define MCU_LED_BOOT  0x000080U
#define MCU_LED_READY 0x008000U
#define MCU_LED_ERROR 0x000080U
// #define MOTOR_LED_IDLE 0x808080U
#define MOTOR_LED_IDLE 0x0D0D0D
#define MOTOR_LED_ACTIVE 0x008000U

#define DEFAULT_MOTOR_DUTY_PCT 100U

#define SOF0 0x55
#define SOF1 0xAA

#define CMD_PING        0x01
#define CMD_DISCOVER    0x10
#define CMD_CONFIRM     0x11
#define CMD_HEARTBEAT   0x12
#define CMD_SET_ADDRESS 0x13
#define CMD_ENTER_BOOT  0x22
#define CMD_GET_VERSION 0x30
#define CMD_READ_UID    0x32
#define CMD_READ_FILAMENT 0x33
#define CMD_GET_SWITCH_STATUS 0x41
#define CMD_MOTOR_CTRL  0x40

#define ST_OK       0x00
#define ST_BAD_CMD  0x01
#define ST_BAD_ARG  0x02
#define ST_IO_ERR   0x04
#define ST_NO_TAG   0x05
#define ST_NOT_READY 0x06

#define APP_VERSION_MAJOR 0x01
#define APP_VERSION_MINOR 0x03
#define APP_VERSION_PATCH 0x00

#ifndef BOOT_TEST_STAGE_LIMIT
#define BOOT_TEST_STAGE_LIMIT 0U
#endif

#ifndef RUNTIME_TEST_STAGE_LIMIT
#define RUNTIME_TEST_STAGE_LIMIT 0U
#endif

#ifndef BUSPOLL_TEST_LEVEL
#define BUSPOLL_TEST_LEVEL 0U
#endif

#ifndef DEVICE_ADDR
#define DEVICE_ADDR 0x01U
#endif

#ifndef BUS_DEVICE_ADDR
#define BUS_DEVICE_ADDR DEVICE_ADDR
#endif
#define BUS_BROADCAST_ADDR 0xFFU

#define BUS_RX_MAX_PAYLOAD 96
#define BUS_DEVICE_KIND_REWINDER 0x01U
#define BUS_HEARTBEAT_TIMEOUT_MS 3000U

#define CONFIG_FLASH_ADDR 0x0800F800UL
#define CONFIG_FLASH_PAGE_SIZE 256U
#define CONFIG_MAGIC 0x31474643UL /* CFG1 */
#define CONFIG_VERSION 0x01U

#define PN0031_STX 0xA0
#define PN0031_ETX 0x0D
#define PN0031_ADDR 0x00
#define PN0031_FT_CMD 0x02
#define PN0031_FC_READ_CARD 0x20
#define PN0031_FC_READ_BLOCK 0x22
#define UID_MAX_LEN 16
#define FRAME_MAX_DATA 96

#define MOTOR_DIR_STOP     0
#define MOTOR_DIR_FORWARD  1
#define MOTOR_DIR_REVERSE -1

typedef struct {
    uint8_t antenna;
    uint8_t status;
    uint8_t uid_len;
    uint8_t uid[UID_MAX_LEN];
    uint8_t color_rgba[4];
    int16_t drying_temp_c;
    uint16_t drying_time_h;
    int16_t nozzle_min_c;
    int16_t nozzle_max_c;
    char variant[9];
    char material_id[9];
    char filament_type[17];
    char detailed_type[17];
    char production_time[17];
    bool parsed_ok;
} FilamentDecoded;

static volatile uint32_t g_tick = 0U;
static uint16_t g_led_rx_ticks = 0U;
static volatile uint8_t g_pcf8574_irq_pending = 0U;
static bool g_ws2812_started = false;
static bool g_bus_master_confirmed = false;
static uint32_t g_bus_last_seen_tick = 0U;
static uint8_t g_bus_device_addr = BUS_DEVICE_ADDR;
static volatile uint32_t g_ws2812_colors[WS_LED_COUNT] = {
    MOTOR_LED_IDLE, MOTOR_LED_IDLE, MOTOR_LED_IDLE, MOTOR_LED_IDLE, MCU_LED_BOOT
};
static Pcf8574Bus g_pcf8574_bus = {PCF8574_I2C_PORT, PCF8574_SDA_PIN, PCF8574_SCL_PIN, 20U};
static Pcf8574Device g_pcf8574_input_dev;
static int8_t g_motor_dir[4] = {MOTOR_DIR_STOP, MOTOR_DIR_STOP, MOTOR_DIR_STOP, MOTOR_DIR_STOP};
static uint8_t g_motor_duty_pct[4] = {
    DEFAULT_MOTOR_DUTY_PCT,
    DEFAULT_MOTOR_DUTY_PCT,
    DEFAULT_MOTOR_DUTY_PCT,
    DEFAULT_MOTOR_DUTY_PCT
};
static uint16_t g_pwm_tim1_arr = 0U;
static uint16_t g_pwm_tim2_arr = 0U;

static bool pwm_set_duty_index(uint8_t index, uint8_t duty_pct);
static void bus_get_device_sn(uint8_t out[12]);
static void bus_touch_master(bool confirmed);
static void bus_housekeeping(void);
static void update_registration_status(void);

typedef struct {
    uint32_t magic;
    uint8_t version;
    uint8_t device_kind;
    uint8_t device_addr;
    uint8_t reserved0;
    uint16_t crc16;
    uint16_t reserved1;
} DeviceConfigRecord;

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

static void sha256_transform(sha256_ctx_t *ctx, const uint8_t data[64])
{
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

static void sha256_init(sha256_ctx_t *ctx)
{
    ctx->datalen = 0;
    ctx->bitlen = 0;
    ctx->state[0] = 0x6a09e667U; ctx->state[1] = 0xbb67ae85U; ctx->state[2] = 0x3c6ef372U; ctx->state[3] = 0xa54ff53aU;
    ctx->state[4] = 0x510e527fU; ctx->state[5] = 0x9b05688cU; ctx->state[6] = 0x1f83d9abU; ctx->state[7] = 0x5be0cd19U;
}

static void sha256_update(sha256_ctx_t *ctx, const uint8_t *data, uint32_t len)
{
    uint32_t i;
    for (i = 0; i < len; i++) {
        ctx->data[ctx->datalen++] = data[i];
        if (ctx->datalen == 64U) {
            sha256_transform(ctx, ctx->data);
            ctx->bitlen += 512U;
            ctx->datalen = 0;
        }
    }
}

static void sha256_final(sha256_ctx_t *ctx, uint8_t hash[32])
{
    uint32_t i;
    uint32_t dl = ctx->datalen;

    ctx->data[dl++] = 0x80U;
    if (dl > 56U) {
        while (dl < 64U) ctx->data[dl++] = 0x00U;
        sha256_transform(ctx, ctx->data);
        dl = 0U;
    }
    while (dl < 56U) ctx->data[dl++] = 0x00U;

    ctx->bitlen += (uint64_t)ctx->datalen * 8ULL;
    for (i = 0; i < 8U; i++) {
        ctx->data[63U - i] = (uint8_t)(ctx->bitlen >> (8U * i));
    }
    sha256_transform(ctx, ctx->data);

    for (i = 0; i < 4U; i++) {
        hash[i]      = (uint8_t)(ctx->state[0] >> (24U - i * 8U));
        hash[i + 4U]  = (uint8_t)(ctx->state[1] >> (24U - i * 8U));
        hash[i + 8U]  = (uint8_t)(ctx->state[2] >> (24U - i * 8U));
        hash[i + 12U] = (uint8_t)(ctx->state[3] >> (24U - i * 8U));
        hash[i + 16U] = (uint8_t)(ctx->state[4] >> (24U - i * 8U));
        hash[i + 20U] = (uint8_t)(ctx->state[5] >> (24U - i * 8U));
        hash[i + 24U] = (uint8_t)(ctx->state[6] >> (24U - i * 8U));
        hash[i + 28U] = (uint8_t)(ctx->state[7] >> (24U - i * 8U));
    }
}

static void hmac_sha256(const uint8_t *key, uint32_t key_len, const uint8_t *msg, uint32_t msg_len, uint8_t out[32])
{
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

static void hkdf_expand_sha256(const uint8_t prk[32], const uint8_t *info, uint8_t info_len, uint8_t *okm, uint16_t okm_len)
{
    uint8_t t[32];
    uint8_t ctr = 1U;
    uint16_t out_pos = 0U;
    uint8_t prev_len = 0U;
    uint8_t msg[64];

    memset(t, 0, sizeof(t));
    while (out_pos < okm_len) {
        uint8_t msg_len = 0U;
        if (prev_len > 0U) {
            memcpy(msg, t, prev_len);
            msg_len = prev_len;
        }
        memcpy(msg + msg_len, info, info_len);
        msg_len += info_len;
        msg[msg_len++] = ctr;

        hmac_sha256(prk, 32U, msg, msg_len, t);
        prev_len = 32U;

        {
            uint16_t remaining = (uint16_t)(okm_len - out_pos);
            uint16_t copy_len = (remaining < 32U) ? remaining : 32U;
            memcpy(okm + out_pos, t, copy_len);
            out_pos += copy_len;
        }
        ctr++;
    }
}

static void derive_sector_keys(const uint8_t *uid, uint8_t uid_len, uint8_t keys_a[16][6], uint8_t keys_b[16][6])
{
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

static void sanitize_boot_handoff_state(void)
{
    uint8_t i;
    DMA_Channel_TypeDef *dma_channels[] = {
        DMA1_Channel1,
        DMA1_Channel2,
        DMA1_Channel3,
        DMA1_Channel4,
        DMA1_Channel5,
        DMA1_Channel6,
        DMA1_Channel7,
    };

    __disable_irq();

    SysTick->CTLR = 0U;
    SysTick->SR = 0U;
    SysTick->CNT = 0U;
    SysTick->CMP = 0U;

    for (i = 0U; i < 8U; i++) {
        NVIC->IRER[i] = 0xFFFFFFFFUL;
        NVIC->IPRR[i] = 0xFFFFFFFFUL;
    }

    EXTI->INTENR = 0U;
    EXTI->EVENR = 0U;
    EXTI->RTENR = 0U;
    EXTI->FTENR = 0U;
    EXTI->SWIEVR = 0U;
    EXTI->INTFR = 0xFFFFFFFFUL;
    GPIO_AFIODeInit();

    USART_Cmd(USART1, DISABLE);
    USART_Cmd(USART2, DISABLE);
    USART_DeInit(USART1);
    USART_DeInit(USART2);

    TIM_Cmd(TIM1, DISABLE);
    TIM_Cmd(TIM2, DISABLE);
    TIM_DeInit(TIM1);
    TIM_DeInit(TIM2);

    SPI_Cmd(SPI1, DISABLE);
    SPI_I2S_DeInit(SPI1);

    DMA1->INTFCR = 0xFFFFFFFFUL;
    for (i = 0U; i < (uint8_t)(sizeof(dma_channels) / sizeof(dma_channels[0])); i++) {
        dma_channels[i]->CFGR = 0U;
        dma_channels[i]->CNTR = 0U;
        dma_channels[i]->PADDR = 0U;
        dma_channels[i]->MADDR = 0U;
    }

    g_pcf8574_irq_pending = 0U;
    g_led_rx_ticks = 0U;

    __enable_irq();
}

void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void SysTick_Handler(void)
{
    g_tick++;
    if (g_led_rx_ticks > 0U) {
        g_led_rx_ticks--;
        if (g_led_rx_ticks == 0U) {
            GPIO_ResetBits(LED_STAT_PORT, LED_STAT2_PIN);
        }
    }
    SysTick->SR = 0;
}

void EXTI7_0_IRQHandler(void)
{
    if ((EXTI->INTFR & EXTI_Line4) != 0U) {
        EXTI->INTFR = EXTI_Line4;
        g_pcf8574_irq_pending = 1U;
    }
}

static void leds_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD, ENABLE);
    gpio.GPIO_Pin = LED_STAT1_PIN | LED_STAT2_PIN;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LED_STAT_PORT, &gpio);
    GPIO_ResetBits(LED_STAT_PORT, LED_STAT1_PIN | LED_STAT2_PIN);
}

static void led_stat1_set(bool on)
{
    if (on) GPIO_SetBits(LED_STAT_PORT, LED_STAT1_PIN);
    else GPIO_ResetBits(LED_STAT_PORT, LED_STAT1_PIN);
}

static void led_stat2_pulse(void)
{
    GPIO_SetBits(LED_STAT_PORT, LED_STAT2_PIN);
    g_led_rx_ticks = 50U;
}

static void led_ok_set(bool on)
{
    led_stat1_set(on);
}

uint32_t WS2812BLEDCallback(int ledno)
{
    if (ledno < WS_LED_COUNT) {
        return g_ws2812_colors[ledno];
    }
    return 0x000000U;
}

static void ws2812_start_blocking(void)
{
    uint32_t systick_ctlr = SysTick->CTLR;

    SysTick->CTLR = 0U;
    while (WS2812BLEDInUse) { }
    WS2812BDMAStart(WS_LED_COUNT);
    while (WS2812BLEDInUse) { }
    SysTick->CTLR = systick_ctlr;
    g_ws2812_started = true;
}

static uint32_t motor_state_color(uint8_t channel_1to4)
{
    if (channel_1to4 < 1U || channel_1to4 > 4U) {
        return MOTOR_LED_IDLE;
    }
    return (g_motor_dir[channel_1to4 - 1U] == MOTOR_DIR_STOP) ? MOTOR_LED_IDLE : MOTOR_LED_ACTIVE;
}

static void ws2812_refresh_status(void)
{
#if USE_PC6_WS2812
    uint32_t next_colors[WS_LED_COUNT];
    uint8_t i;

    for (i = 0U; i < 4U; i++) {
        next_colors[i] = motor_state_color((uint8_t)(i + 1U));
    }
    next_colors[4] = g_ws2812_colors[4];

    if (!g_ws2812_started || memcmp((const void *)g_ws2812_colors, next_colors, sizeof(next_colors)) != 0) {
        memcpy((void *)g_ws2812_colors, next_colors, sizeof(next_colors));
        ws2812_start_blocking();
    }
#else
    (void)motor_state_color;
#endif
}

static void ws2812_set_mcu_color(uint32_t color)
{
#if USE_PC6_WS2812
    if (g_ws2812_colors[4] != color) {
        g_ws2812_colors[4] = color;
        ws2812_start_blocking();
    }
#else
    (void)color;
#endif
}

static void update_registration_status(void)
{
    ws2812_set_mcu_color(g_bus_master_confirmed ? MCU_LED_READY : MCU_LED_ERROR);
}

static void boot_test_hold(uint8_t stage)
{
#if BOOT_TEST_STAGE_LIMIT > 0U
    uint8_t i;

    if (stage != (uint8_t)BOOT_TEST_STAGE_LIMIT) {
        return;
    }

    while (1) {
        for (i = 0U; i < stage; i++) {
            GPIO_SetBits(LED_STAT_PORT, LED_STAT1_PIN | LED_STAT2_PIN);
            delay_ms(120U);
            GPIO_ResetBits(LED_STAT_PORT, LED_STAT1_PIN | LED_STAT2_PIN);
            delay_ms(180U);
        }
        delay_ms(900U);
    }
#else
    (void)stage;
#endif
}

static void runtime_test_loop(uint8_t stage)
{
#if RUNTIME_TEST_STAGE_LIMIT > 0U
    static uint32_t last_toggle = 0U;
    static bool led_on = false;

    if (stage > (uint8_t)RUNTIME_TEST_STAGE_LIMIT) {
        return;
    }

    if ((uint32_t)(g_tick - last_toggle) >= 250U) {
        last_toggle = g_tick;
        led_on = !led_on;
        if (led_on) GPIO_SetBits(LED_STAT_PORT, LED_STAT1_PIN | LED_STAT2_PIN);
        else GPIO_ResetBits(LED_STAT_PORT, LED_STAT1_PIN | LED_STAT2_PIN);
    }
#else
    (void)stage;
#endif
}

static uint8_t switch_bit_for_channel(uint8_t channel_1to4)
{
    switch (channel_1to4) {
        case 1U: return 7U; /* SW1 */
        case 2U: return 5U; /* SW2 */
        case 3U: return 1U; /* SW3 */
        case 4U: return 3U; /* SW4 */
        default: return 0xFFU;
    }
}

static bool switch_is_online(uint8_t channel_1to4)
{
#if !USE_PCF8574
    (void)channel_1to4;
    return false;
#else
    const Pcf8574Device *dev = &g_pcf8574_input_dev;
    uint8_t bit;
    uint8_t mask;

    if (channel_1to4 < 1U || channel_1to4 > 4U) return false;
    bit = switch_bit_for_channel(channel_1to4);
    if (bit > 7U) return false;
    mask = (uint8_t)(1U << bit);
    return (dev->last_state & mask) == 0U;
#endif
}

static uint8_t switch_online_mask(void)
{
    uint8_t mask = 0U;
    uint8_t ch;
    for (ch = 1U; ch <= 4U; ch++) {
        if (switch_is_online(ch)) {
            mask |= (uint8_t)(1U << (ch - 1U));
        }
    }
    return mask;
}

static uint8_t pwm_index_for_channel(uint8_t channel_1to4)
{
    switch (channel_1to4) {
        case 1U: return 1U; /* PC3 */
        case 2U: return 0U; /* PC0 */
        case 3U: return 2U; /* PC5 */
        case 4U: return 3U; /* PC7 */
        default: return 0xFFU;
    }
}

static void motor_apply_channel(uint8_t channel_1to4)
{
    uint8_t pwm_index;
    uint8_t duty;

    if (channel_1to4 < 1U || channel_1to4 > 4U) return;
    pwm_index = pwm_index_for_channel(channel_1to4);
    if (pwm_index == 0xFFU) return;
    duty = (g_motor_dir[channel_1to4 - 1U] == MOTOR_DIR_STOP) ? 0U : g_motor_duty_pct[channel_1to4 - 1U];
    (void)pwm_set_duty_index(pwm_index, duty);
    ws2812_refresh_status();
}

static void tc118s_set_dir(uint8_t channel_1to4, int8_t dir)
{
    if (channel_1to4 < 1U || channel_1to4 > 4U) return;
    g_motor_dir[channel_1to4 - 1U] = (dir == MOTOR_DIR_STOP) ? MOTOR_DIR_STOP : MOTOR_DIR_FORWARD;
    motor_apply_channel(channel_1to4);
}

static void pcf8574_int_init(void)
{
#if USE_PCF8574
    GPIO_InitTypeDef gpio = {0};
    NVIC_InitTypeDef nvic = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC | RCC_PB2Periph_AFIO, ENABLE);

    gpio.GPIO_Pin = PCF8574_INT_PIN;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(PCF8574_INT_PORT, &gpio);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);
    EXTI->INTENR |= EXTI_Line4;
    EXTI->EVENR &= (uint32_t)~EXTI_Line4;
    EXTI->RTENR &= (uint32_t)~EXTI_Line4;
    EXTI->FTENR |= EXTI_Line4;
    EXTI->INTFR = EXTI_Line4;

    nvic.NVIC_IRQChannel = EXTI7_0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
#endif
}

static void pcf8574_process_irq(void)
{
#if USE_PCF8574
    Pcf8574Device *dev = &g_pcf8574_input_dev;
    uint8_t v;
    uint8_t ch;

    if (!pcf8574_refresh(&g_pcf8574_bus, dev)) return;
    v = dev->last_state;

    for (ch = 1U; ch <= 4U; ch++) {
        uint8_t bit = switch_bit_for_channel(ch);
        uint8_t mask = (uint8_t)(1U << bit);
        if ((dev->changed_mask & mask) != 0U && (v & mask) != 0U) {
            if (g_motor_dir[ch - 1U] != MOTOR_DIR_STOP) {
                tc118s_set_dir(ch, MOTOR_DIR_STOP);
            }
        }
    }
#endif
}

static void pwm_pc3_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OCInitTypeDef oc = {0};
    TIM_TimeBaseInitTypeDef tb = {0};
    uint16_t psc = 48U - 1U;
    uint16_t arr = (uint16_t)((48000000UL / 48UL) / PWM_PC3_FREQ_HZ);
    uint16_t ccr;

    if (arr == 0U) arr = 1U;
    ccr = (uint16_t)((arr * PWM_PC3_DUTY_PCT) / 100U);
    g_pwm_tim2_arr = arr;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC | RCC_PB2Periph_AFIO, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_TIM2, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap4_TIM2, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &gpio);

    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Prescaler = psc;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    tb.TIM_Period = (uint16_t)(arr - 1U);
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &tb);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = ccr;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC3Init(TIM2, &oc);

    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

static void pwm_pc7_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OCInitTypeDef oc = {0};
    TIM_TimeBaseInitTypeDef tb = {0};
    uint16_t psc = 48U - 1U;
    uint16_t arr = (uint16_t)((48000000UL / 48UL) / PWM_PC7_FREQ_HZ);
    uint16_t ccr;

    if (arr == 0U) arr = 1U;
    ccr = (uint16_t)((arr * PWM_PC7_DUTY_PCT) / 100U);
    g_pwm_tim1_arr = arr;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC | RCC_PB2Periph_AFIO | RCC_PB2Periph_TIM1, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap7_TIM1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_7;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &gpio);

    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Prescaler = psc;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    tb.TIM_Period = (uint16_t)(arr - 1U);
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(TIM1, &tb);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = ccr;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init(TIM1, &oc);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

static void pwm_pc5_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OCInitTypeDef oc = {0};
    TIM_TimeBaseInitTypeDef tb = {0};
    uint16_t psc = 48U - 1U;
    uint16_t arr = (uint16_t)((48000000UL / 48UL) / PWM_PC5_FREQ_HZ);
    uint16_t ccr;

    if (arr == 0U) arr = 1U;
    ccr = (uint16_t)((arr * PWM_PC5_DUTY_PCT) / 100U);
    g_pwm_tim1_arr = arr;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC | RCC_PB2Periph_AFIO | RCC_PB2Periph_TIM1, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap7_TIM1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_5;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &gpio);

    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Prescaler = psc;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    tb.TIM_Period = (uint16_t)(arr - 1U);
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(TIM1, &tb);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = ccr;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM1, &oc);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

static void pwm_pc0_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OCInitTypeDef oc = {0};
    TIM_TimeBaseInitTypeDef tb = {0};
    uint16_t psc = 48U - 1U;
    uint16_t arr = (uint16_t)((48000000UL / 48UL) / PWM_PC0_FREQ_HZ);
    uint16_t ccr;

    if (arr == 0U) arr = 1U;
    ccr = (uint16_t)((arr * PWM_PC0_DUTY_PCT) / 100U);
    g_pwm_tim2_arr = arr;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC | RCC_PB2Periph_AFIO, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_TIM2, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap4_TIM2, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &gpio);

    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Prescaler = psc;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    tb.TIM_Period = (uint16_t)(arr - 1U);
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &tb);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = ccr;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &oc);

    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

static uint16_t pwm_ccr_from_pct(uint16_t arr, uint8_t duty_pct)
{
    if (duty_pct >= 100U) return arr;
    return (uint16_t)((arr * duty_pct) / 100U);
}

static bool pwm_set_duty_index(uint8_t index, uint8_t duty_pct)
{
#if !USE_PWM_OUTPUTS
    (void)index;
    (void)duty_pct;
    return false;
#else
    uint16_t ccr;

    if (duty_pct > 100U) return false;

    switch (index) {
        case 0U: /* PC0 */
            ccr = pwm_ccr_from_pct(g_pwm_tim2_arr, duty_pct);
            TIM_SetCompare1(TIM2, ccr);
            return true;
        case 1U: /* PC3 */
            ccr = pwm_ccr_from_pct(g_pwm_tim2_arr, duty_pct);
            TIM_SetCompare3(TIM2, ccr);
            return true;
        case 2U: /* PC5 */
            ccr = pwm_ccr_from_pct(g_pwm_tim1_arr, duty_pct);
            TIM_SetCompare2(TIM1, ccr);
            return true;
        case 3U: /* PC7 */
            ccr = pwm_ccr_from_pct(g_pwm_tim1_arr, duty_pct);
            TIM_SetCompare4(TIM1, ccr);
            return true;
        default:
            return false;
    }
#endif
}

static void bus_uart_init(uint32_t baud)
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
    gpio.GPIO_Mode = GPIO_Mode_IPU;
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

static bool bus_uart_read_byte(uint8_t *out)
{
    uint16_t stat = USART1->STATR;

    if ((stat & (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)) != 0U) {
        (void)USART1->DATAR;
        return false;
    }

    if ((stat & USART_FLAG_RXNE) != 0U) {
        *out = (uint8_t)(USART1->DATAR & 0xFFU);
        return true;
    }
    return false;
}

static bool bus_uart_wait_flag(uint16_t flag, bool want_set, uint32_t timeout_loops)
{
    while (timeout_loops-- != 0U) {
        bool is_set = (USART1->STATR & flag) != 0U;
        if (is_set == want_set) {
            return true;
        }
    }
    return false;
}

static bool bus_uart_write(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    volatile uint16_t t;

    GPIO_SetBits(RS485_DE_PORT, RS485_DE_PIN);
    for (t = 0; t < 200U; t++) { }
    for (i = 0; i < len; i++) {
        if (!bus_uart_wait_flag(USART_FLAG_TXE, true, 200000U)) {
            GPIO_ResetBits(RS485_DE_PORT, RS485_DE_PIN);
            return false;
        }
        USART1->DATAR = buf[i];
    }
    if (!bus_uart_wait_flag(USART_FLAG_TC, true, 200000U)) {
        GPIO_ResetBits(RS485_DE_PORT, RS485_DE_PIN);
        return false;
    }
    GPIO_ResetBits(RS485_DE_PORT, RS485_DE_PIN);
    return true;
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

static bool device_addr_is_valid(uint8_t addr)
{
    return (addr != 0U && addr != BUS_BROADCAST_ADDR);
}

static bool device_config_record_valid(const DeviceConfigRecord *cfg)
{
    uint16_t calc;

    if (cfg->magic != CONFIG_MAGIC) return false;
    if (cfg->version != CONFIG_VERSION) return false;
    if (cfg->device_kind != BUS_DEVICE_KIND_REWINDER) return false;
    if (!device_addr_is_valid(cfg->device_addr)) return false;

    calc = crc16_ccitt((const uint8_t *)cfg, (uint16_t)offsetof(DeviceConfigRecord, crc16));
    return calc == cfg->crc16;
}

static void device_config_load(void)
{
    const DeviceConfigRecord *cfg = (const DeviceConfigRecord *)CONFIG_FLASH_ADDR;

    if (device_config_record_valid(cfg)) {
        g_bus_device_addr = cfg->device_addr;
    } else {
        g_bus_device_addr = BUS_DEVICE_ADDR;
    }
}

static bool device_config_store_addr(uint8_t addr)
{
    uint8_t page[CONFIG_FLASH_PAGE_SIZE] __attribute__((aligned(4)));
    DeviceConfigRecord *cfg = (DeviceConfigRecord *)page;
    FLASH_Status fs;

    if (!device_addr_is_valid(addr)) return false;

    memset(page, 0xFF, sizeof(page));
    cfg->magic = CONFIG_MAGIC;
    cfg->version = CONFIG_VERSION;
    cfg->device_kind = BUS_DEVICE_KIND_REWINDER;
    cfg->device_addr = addr;
    cfg->reserved0 = 0xFFU;
    cfg->reserved1 = 0xFFFFU;
    cfg->crc16 = crc16_ccitt(page, (uint16_t)offsetof(DeviceConfigRecord, crc16));

    fs = FLASH_ROM_ERASE(CONFIG_FLASH_ADDR, CONFIG_FLASH_PAGE_SIZE);
    if (fs != FLASH_COMPLETE) return false;
    fs = FLASH_ROM_WRITE(CONFIG_FLASH_ADDR, (uint32_t *)page, CONFIG_FLASH_PAGE_SIZE);
    return (fs == FLASH_COMPLETE);
}

static uint8_t xor_checksum(const uint8_t *data, uint16_t len)
{
    uint8_t x = 0;
    uint16_t i;
    for (i = 0; i < len; i++) x ^= data[i];
    return x;
}

static void bus_get_device_sn(uint8_t out[12])
{
    volatile const uint8_t *uid = (volatile const uint8_t *)0x1FFFF7E8U;
    uint8_t i;
    for (i = 0U; i < 12U; i++) {
        out[i] = uid[i];
    }
}

static void bus_touch_master(bool confirmed)
{
    g_bus_last_seen_tick = g_tick;
    if (confirmed) {
        g_bus_master_confirmed = true;
        update_registration_status();
    }
}

static void bus_housekeeping(void)
{
    if (g_bus_master_confirmed) {
        uint32_t age = g_tick - g_bus_last_seen_tick;
        if (age > BUS_HEARTBEAT_TIMEOUT_MS) {
            g_bus_master_confirmed = false;
            update_registration_status();
        }
    }
}

static void bus_send_resp(uint8_t addr, uint8_t cmd, uint8_t seq, const uint8_t *payload, uint16_t len)
{
    uint8_t frame[7 + BUS_RX_MAX_PAYLOAD + 2];
    uint16_t crc;

    frame[0] = SOF0;
    frame[1] = SOF1;
    frame[2] = addr;
    frame[3] = (uint8_t)(cmd | 0x80U);
    frame[4] = seq;
    put_u16le(&frame[5], len);
    if (len > 0U) memcpy(&frame[7], payload, len);
    crc = crc16_ccitt(&frame[2], (uint16_t)(5U + len));
    put_u16le(&frame[7 + len], crc);
    (void)bus_uart_write(frame, (uint16_t)(9U + len));
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
#if !USE_PN0031
    (void)antenna;
    (void)status;
    (void)uid;
    (void)uid_len;
    return false;
#else
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
#endif
}

static bool pn_read_block(uint8_t antenna, uint8_t block, uint8_t key_type, const uint8_t key6[6], uint8_t out16[16])
{
#if !USE_PN0031
    (void)antenna;
    (void)block;
    (void)key_type;
    (void)key6;
    (void)out16;
    return false;
#else
    uint8_t req[10];
    uint8_t resp[FRAME_MAX_DATA];
    uint16_t len = 0;

    req[0] = antenna;
    req[1] = block;
    req[2] = 0x00U;
    req[3] = key_type;
    memcpy(&req[4], key6, 6);

    pn_uart_flush_rx();
    if (!pn_send_cmd(PN0031_FC_READ_BLOCK, req, 10)) return false;
    if (!pn_read_frame(PN0031_FC_READ_BLOCK, resp, &len, 400000U)) return false;
    if (len < 1U) return false;
    if (resp[0] != 0x00U) return false;
    if (len != 19U) return false;
    memcpy(out16, &resp[3], 16U);
    return true;
#endif
}

static uint16_t u16le(const uint8_t *d) { return (uint16_t)d[0] | ((uint16_t)d[1] << 8); }

static void copy_ascii_trim(char *dst, uint8_t dst_sz, const uint8_t *src, uint8_t n)
{
    uint8_t i;
    uint8_t out = 0U;
    for (i = 0U; i < n && out + 1U < dst_sz; i++) {
        uint8_t c = src[i];
        if (c == 0U) break;
        if (c >= 32U && c <= 126U) {
            dst[out++] = (char)c;
        }
    }
    dst[out] = 0;
}

static void filament_decoded_init(FilamentDecoded *f)
{
    memset(f, 0, sizeof(*f));
}

static void parse_filament_blocks(FilamentDecoded *f, const uint8_t blocks[64][16])
{
    int16_t nmin;
    int16_t nmax;

    copy_ascii_trim(f->variant, sizeof(f->variant), &blocks[1][0], 8);
    copy_ascii_trim(f->material_id, sizeof(f->material_id), &blocks[1][8], 8);
    copy_ascii_trim(f->filament_type, sizeof(f->filament_type), &blocks[2][0], 16);
    copy_ascii_trim(f->detailed_type, sizeof(f->detailed_type), &blocks[4][0], 16);

    f->color_rgba[0] = blocks[5][0];
    f->color_rgba[1] = blocks[5][1];
    f->color_rgba[2] = blocks[5][2];
    f->color_rgba[3] = blocks[5][3];

    f->drying_temp_c = (int16_t)u16le(&blocks[6][0]);
    f->drying_time_h = u16le(&blocks[6][2]);
    nmin = (int16_t)u16le(&blocks[6][8]);
    nmax = (int16_t)u16le(&blocks[6][10]);
    if (nmin > nmax) {
        int16_t t = nmin;
        nmin = nmax;
        nmax = t;
    }
    f->nozzle_min_c = nmin;
    f->nozzle_max_c = nmax;

    copy_ascii_trim(f->production_time, sizeof(f->production_time), &blocks[12][0], 16);
    f->parsed_ok = true;
}

static bool poll_filament_decoded(uint8_t antenna, FilamentDecoded *f)
{
    uint8_t status = 0xFFU;
    uint8_t uid[UID_MAX_LEN];
    uint8_t uid_len = 0U;
    uint8_t keys_a[16][6];
    uint8_t keys_b[16][6];
    static const uint8_t needed_blocks[] = {1,2,4,5,6,12};
    uint8_t raw[64][16];
    uint8_t i;

    filament_decoded_init(f);
    f->antenna = antenna;

    if (!pn_read_card(antenna, &status, uid, &uid_len)) {
        return false;
    }
    f->status = status;
    if (status != 0x00U || uid_len == 0U) {
        return true;
    }

    f->uid_len = uid_len;
    memcpy(f->uid, uid, uid_len);
    memset(raw, 0, sizeof(raw));

    derive_sector_keys(uid, uid_len, keys_a, keys_b);
    for (i = 0U; i < sizeof(needed_blocks); i++) {
        uint8_t b = needed_blocks[i];
        uint8_t sec = (uint8_t)(b / 4U);
        if (!pn_read_block(antenna, b, 0U, keys_a[sec], raw[b])) {
            if (!pn_read_block(antenna, b, 1U, keys_b[sec], raw[b])) {
                return false;
            }
        }
        delay_ms(3U);
    }

    parse_filament_blocks(f, raw);
    return true;
}

static void bus_handle_cmd(uint8_t addr, uint8_t cmd, uint8_t seq, const uint8_t *payload, uint16_t len)
{
    uint8_t out[BUS_RX_MAX_PAYLOAD];
    uint16_t out_len = 0;
    bool enter_boot = false;
    bool is_broadcast = (addr == BUS_BROADCAST_ADDR);

    led_stat2_pulse();

    if (cmd == CMD_PING || cmd == CMD_GET_VERSION) {
        out[0] = ST_OK;
        out[1] = APP_VERSION_MAJOR;
        out[2] = APP_VERSION_MINOR;
        out[3] = APP_VERSION_PATCH;
        out_len = 4;
    } else if (cmd == CMD_DISCOVER) {
        uint8_t sn[12];
        bus_get_device_sn(sn);
        out[0] = ST_OK;
        out[1] = BUS_DEVICE_KIND_REWINDER;
        out[2] = g_bus_device_addr;
        out[3] = APP_VERSION_MAJOR;
        out[4] = APP_VERSION_MINOR;
        out[5] = APP_VERSION_PATCH;
        out[6] = (uint8_t)(g_bus_master_confirmed ? 1U : 0U);
        put_u16le(&out[7], BUS_HEARTBEAT_TIMEOUT_MS);
        memcpy(&out[9], sn, 12U);
        out_len = 21;
    } else if (cmd == CMD_SET_ADDRESS) {
        uint8_t new_addr;
        if (len != 1U) {
            out[0] = ST_BAD_ARG;
            out_len = 1;
        } else if (is_broadcast) {
            out[0] = ST_BAD_ARG;
            out_len = 1;
        } else {
            new_addr = payload[0];
            if (!device_addr_is_valid(new_addr)) {
                out[0] = ST_BAD_ARG;
                out_len = 1;
            } else if (!device_config_store_addr(new_addr)) {
                out[0] = ST_IO_ERR;
                out_len = 1;
            } else {
                out[0] = ST_OK;
                out[1] = new_addr;
                out_len = 2;
            }
        }
    } else if (cmd == CMD_CONFIRM) {
        if (len != 0U) {
            out[0] = ST_BAD_ARG;
            out_len = 1;
        } else if (is_broadcast) {
            out[0] = ST_BAD_ARG;
            out_len = 1;
        } else {
            bus_touch_master(true);
            out[0] = ST_OK;
            out[1] = 1U;
            out_len = 2;
        }
    } else if (cmd == CMD_HEARTBEAT) {
        if (len != 0U) {
            out[0] = ST_BAD_ARG;
            out_len = 1;
        } else if (is_broadcast) {
            out[0] = ST_BAD_ARG;
            out_len = 1;
        } else {
            if (g_bus_master_confirmed) {
                bus_touch_master(true);
            }
            out[0] = ST_OK;
            out[1] = (uint8_t)(g_bus_master_confirmed ? 1U : 0U);
            put_u16le(&out[2], (uint16_t)(g_tick & 0xFFFFU));
            out_len = 4;
        }
    } else if (cmd == CMD_ENTER_BOOT) {
        if (len != 0U) {
            out[0] = ST_BAD_ARG;
            out_len = 1;
        } else if (is_broadcast) {
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
    } else if (cmd == CMD_READ_UID) {
#if !USE_PN0031
        out[0] = ST_NOT_READY;
        out_len = 1;
#else
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
                led_ok_set(true);
            }
        }
#endif
    } else if (cmd == CMD_READ_FILAMENT) {
#if !USE_PN0031
        out[0] = ST_NOT_READY;
        out_len = 1;
#else
        uint8_t ant;
        FilamentDecoded f;

        if (len != 1U) {
            out[0] = ST_BAD_ARG;
            out_len = 1;
        } else {
            ant = payload[0];
            if (ant < 1U || ant > 4U) {
                out[0] = ST_BAD_ARG;
                out_len = 1;
            } else if (!poll_filament_decoded(ant, &f)) {
                out[0] = ST_IO_ERR;
                out_len = 1;
            } else if (f.status != 0x00U || f.uid_len == 0U) {
                out[0] = ST_NO_TAG;
                out[1] = f.status;
                out_len = 2;
            } else if (!f.parsed_ok) {
                out[0] = ST_IO_ERR;
                out_len = 1;
            } else {
                uint8_t pos = 0U;
                memset(out, 0, sizeof(out));
                out[pos++] = ST_OK;
                out[pos++] = f.antenna;
                out[pos++] = f.status;
                out[pos++] = f.uid_len;
                memcpy(&out[pos], f.uid, f.uid_len);
                pos = (uint8_t)(pos + f.uid_len);
                memcpy(&out[pos], f.color_rgba, 4U);
                pos = (uint8_t)(pos + 4U);
                out[pos++] = (uint8_t)(f.drying_temp_c & 0xFF);
                out[pos++] = (uint8_t)((uint16_t)f.drying_temp_c >> 8);
                out[pos++] = (uint8_t)(f.drying_time_h & 0xFF);
                out[pos++] = (uint8_t)(f.drying_time_h >> 8);
                out[pos++] = (uint8_t)(f.nozzle_min_c & 0xFF);
                out[pos++] = (uint8_t)((uint16_t)f.nozzle_min_c >> 8);
                out[pos++] = (uint8_t)(f.nozzle_max_c & 0xFF);
                out[pos++] = (uint8_t)((uint16_t)f.nozzle_max_c >> 8);
                memcpy(&out[pos], f.variant, 8U); pos = (uint8_t)(pos + 8U);
                memcpy(&out[pos], f.material_id, 8U); pos = (uint8_t)(pos + 8U);
                memcpy(&out[pos], f.filament_type, 16U); pos = (uint8_t)(pos + 16U);
                memcpy(&out[pos], f.detailed_type, 16U); pos = (uint8_t)(pos + 16U);
                memcpy(&out[pos], f.production_time, 16U); pos = (uint8_t)(pos + 16U);
                out_len = pos;
                led_ok_set(true);
            }
        }
#endif
    } else if (cmd == CMD_GET_SWITCH_STATUS) {
        if (len != 0U) {
            out[0] = ST_BAD_ARG;
            out_len = 1;
#if USE_PCF8574
        } else if (!pcf8574_refresh(&g_pcf8574_bus, &g_pcf8574_input_dev)) {
            out[0] = ST_NOT_READY;
            out_len = 1;
        } else {
            out[0] = ST_OK;
            out[1] = g_pcf8574_input_dev.last_state;
            out[2] = switch_online_mask();
            out_len = 3;
        }
#else
        } else {
            out[0] = ST_NOT_READY;
            out_len = 1;
        }
#endif
    } else if (cmd == CMD_MOTOR_CTRL) {
        uint8_t ch;
        uint8_t speed;
        if (len != 2U) {
            out[0] = ST_BAD_ARG;
            out_len = 1;
        } else {
            ch = payload[0];
            speed = payload[1];
            if (ch < 1U || ch > 4U || speed > 100U) {
                out[0] = ST_BAD_ARG;
                out_len = 1;
            } else if (speed == 0U) {
                tc118s_set_dir(ch, MOTOR_DIR_STOP);
                out[0] = ST_OK;
                out[1] = ch;
                out[2] = 0U;
                out_len = 3;
            } else {
#if USE_PCF8574
                if (!pcf8574_refresh(&g_pcf8574_bus, &g_pcf8574_input_dev)) {
                    out[0] = ST_NOT_READY;
                    out_len = 1;
                } else if (!switch_is_online(ch)) {
                    out[0] = ST_NOT_READY;
                    out_len = 1;
                } else {
                    g_motor_duty_pct[ch - 1U] = speed;
                    tc118s_set_dir(ch, MOTOR_DIR_FORWARD);
                    out[0] = ST_OK;
                    out[1] = ch;
                    out[2] = speed;
                    out_len = 3;
                }
#else
                g_motor_duty_pct[ch - 1U] = speed;
                tc118s_set_dir(ch, MOTOR_DIR_FORWARD);
                out[0] = ST_OK;
                out[1] = ch;
                out[2] = speed;
                out_len = 3;
#endif
            }
        }
    } else {
        out[0] = ST_BAD_CMD;
        out_len = 1;
    }

    if (!is_broadcast) {
        bus_send_resp(addr, cmd, seq, out, out_len);
    }

    if (cmd == CMD_SET_ADDRESS && out_len >= 2U && out[0] == ST_OK) {
        g_bus_device_addr = out[1];
        g_bus_master_confirmed = false;
        update_registration_status();
    }

    if (enter_boot) {
        while ((USART1->STATR & USART_FLAG_TC) == 0U) { }
        RCC_ClearFlag();
        SystemReset_StartMode(Start_Mode_BOOT);
        NVIC_SystemReset();
        while (1) { }
    }
}

static void bus_poll_protocol(void)
{
    static uint8_t rx_buf[7 + BUS_RX_MAX_PAYLOAD + 2];
    static uint16_t rx_len = 0;
    uint8_t drain_budget = 32U;
    uint8_t b;

    while (drain_budget-- != 0U && bus_uart_read_byte(&b)) {
        if (rx_len >= sizeof(rx_buf)) rx_len = 0;
        rx_buf[rx_len++] = b;
    }

#if BUSPOLL_TEST_LEVEL == 1U
    return;
#endif

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
        if (rx_len < 7U) return;

        plen = get_u16le(&rx_buf[5]);
        if (plen > BUS_RX_MAX_PAYLOAD) {
            memmove(&rx_buf[0], &rx_buf[1], (size_t)(rx_len - 1U));
            rx_len--;
            continue;
        }

        frame_len = (uint16_t)(9U + plen);
        if (rx_len < frame_len) return;

#if BUSPOLL_TEST_LEVEL == 2U
        if (rx_len > frame_len) {
            memmove(&rx_buf[0], &rx_buf[frame_len], (size_t)(rx_len - frame_len));
        }
        rx_len = (uint16_t)(rx_len - frame_len);
        continue;
#endif

        crc_rx = get_u16le(&rx_buf[7U + plen]);
        crc_calc = crc16_ccitt(&rx_buf[2], (uint16_t)(5U + plen));

#if BUSPOLL_TEST_LEVEL == 3U
        if (rx_len > frame_len) {
            memmove(&rx_buf[0], &rx_buf[frame_len], (size_t)(rx_len - frame_len));
        }
        rx_len = (uint16_t)(rx_len - frame_len);
        (void)crc_rx;
        (void)crc_calc;
        continue;
#endif

        if (crc_calc == crc_rx) {
            uint8_t addr = rx_buf[2];
            if (addr == g_bus_device_addr || addr == BUS_BROADCAST_ADDR) {
#if BUSPOLL_TEST_LEVEL == 4U
                if (rx_len > frame_len) {
                    memmove(&rx_buf[0], &rx_buf[frame_len], (size_t)(rx_len - frame_len));
                }
                rx_len = (uint16_t)(rx_len - frame_len);
                continue;
#endif
                bus_handle_cmd(addr, rx_buf[3], rx_buf[4], &rx_buf[7], plen);
            }
        }

        if (rx_len > frame_len) {
            memmove(&rx_buf[0], &rx_buf[frame_len], (size_t)(rx_len - frame_len));
        }
        rx_len = (uint16_t)(rx_len - frame_len);
    }
}

int main(void)
{
    uint8_t i;
    bool pcf_input_write_ok;
    bool pcf_input_refresh_ok;
    bool pcf_ready = false;

    SystemCoreClockUpdate();
    sanitize_boot_handoff_state();
    /* TEMP PROBE: bypass device_config_load to isolate 3d hang */
    /* device_config_load(); */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_EnableIRQ(SysTick_IRQn);
    SysTick->SR = 0;
    SysTick->CNT = 0;
    SysTick->CMP = (SystemCoreClock / 1000U) - 1U;
    SysTick->CTLR = 0x0FU;
    leds_init();
    ws2812_set_mcu_color(MCU_LED_BOOT);
    boot_test_hold(1U);
#if USE_PWM_OUTPUTS
    pwm_pc3_init();
    pwm_pc0_init();
    pwm_pc5_init();
    pwm_pc7_init();
#endif
    boot_test_hold(2U);
    for (i = 0; i < 6U; i++) {
        delay_ms(150U);
    }

    bus_uart_init(BUS_UART_BAUD);
    boot_test_hold(3U);
#if USE_PN0031
    pn_uart_init(PN0031_UART_BAUD);
#endif
    boot_test_hold(4U);
#if USE_PCF8574
    pcf8574_bus_init(&g_pcf8574_bus);
    pcf8574_device_init(&g_pcf8574_input_dev, PCF8574_ADDR_7B);
    pcf8574_int_init();
    pcf_input_write_ok = pcf8574_write_port(&g_pcf8574_bus, &g_pcf8574_input_dev, 0xFFU);
    pcf_input_refresh_ok = pcf8574_refresh(&g_pcf8574_bus, &g_pcf8574_input_dev);
    pcf_ready = pcf_input_write_ok && pcf_input_refresh_ok;
    (void)pcf_input_write_ok;
    (void)pcf_input_refresh_ok;
#endif
    boot_test_hold(5U);
#if USE_PC6_WS2812
    delay_ms(100U);
    WS2812BDMAInit();
    delay_ms(100U);
    update_registration_status();
    ws2812_refresh_status();
    delay_ms(100U);
#endif
    boot_test_hold(6U);
    led_stat1_set(pcf_ready || !USE_PCF8574);
    while (1) {
        runtime_test_loop(1U);
#if RUNTIME_TEST_STAGE_LIMIT == 1U
        continue;
#endif
        bus_housekeeping();
        runtime_test_loop(2U);
#if RUNTIME_TEST_STAGE_LIMIT == 2U
        continue;
#endif
        bus_poll_protocol();
        runtime_test_loop(3U);
#if RUNTIME_TEST_STAGE_LIMIT == 3U
        continue;
#endif
#if USE_PCF8574
        if (g_pcf8574_irq_pending != 0U) {
            g_pcf8574_irq_pending = 0U;
            pcf8574_process_irq();
        }
#endif
        runtime_test_loop(4U);
    }
}
