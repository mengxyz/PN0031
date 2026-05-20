/* main_heater.c
 *
 * Heater device for CH32V006.
 * Bus USART1 RS485 460800 8N1 PD5=TX PD6=RX PD4=DE
 * PC0 ARGB, PC4 boot/status LED, PC5 heartbeat LED, PC6 heater, PC7 fan PWM
 * PA1 fan status, PA2 power status, PC3 door, PD2/PD3 NTC ADC.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <ch32v00X.h>
#include "ch32v00X_tim.h"
#include "ch32v00X_dma.h"
#include "ch32v00X_spi.h"
#include "ch32v00X_flash.h"

#define AHBPCENR               HBPCENR
#define APB2PCENR              PB2PCENR
#define RCC_AHBPeriph_DMA1     RCC_HBPeriph_DMA1
#define RCC_APB2Periph_GPIOC   RCC_PB2Periph_GPIOC
#define RCC_APB2Periph_AFIO    RCC_PB2Periph_AFIO
#define RCC_APB2Periph_SPI1    RCC_PB2Periph_SPI1
#define GPIO_Speed_10MHz       GPIO_Speed_30MHz
#define GPIO_CNF_OUT_PP_AF     0x08
#define CTLR1_SPE_Set          ((uint16_t)0x0040)
#define WS_LED_COUNT           ARGB_LED_COUNT
#define WS2812_GPIO_PORT       GPIOC
#define WS2812_GPIO_PORT_CLOCK RCC_PB2Periph_GPIOC
#define WS2812_GPIO_PIN_INDEX  0
#define WS2812_SPI_REMAP       GPIO_PartialRemap3_SPI1
#define WS2812DMA_IMPLEMENTATION
#define DMALEDS                8
#define WSGRB
#include "lib/ws2812b_dma_spi_led_driver.h"

#define DEV_TYPE_HEATER   0x00U
#define DEVICE_TYPE       DEV_TYPE_HEATER
#define DEVICE_ADDR       0x02U
#define BROADCAST_ADDR    0xFFU

#define APP_VERSION_MAJOR 0x01U
#define APP_VERSION_MINOR 0x00U
#define APP_VERSION_PATCH 0x1DU
#define UID_WORD0         (*(volatile uint32_t *)0x1FFFF7E8U)
#define UID_WORD1         (*(volatile uint32_t *)0x1FFFF7ECU)

#define RS485_DE_PORT GPIOD
#define RS485_DE_PIN  GPIO_Pin_4
#define BOOT_LED_PORT GPIOC
#define BOOT_LED_PIN  GPIO_Pin_4
#define HB_LED_PORT   GPIOC
#define HB_LED_PIN    GPIO_Pin_5
#define HEATER_PORT   GPIOC
#define HEATER_PIN    GPIO_Pin_6
#define FAN_PORT      GPIOC
#define FAN_PIN       GPIO_Pin_7
#define DOOR_PORT     GPIOC
#define DOOR_PIN      GPIO_Pin_3
#define FAN_STAT_PORT GPIOA
#define FAN_STAT_PIN  GPIO_Pin_1
#define PWR_STAT_PORT GPIOA
#define PWR_STAT_PIN  GPIO_Pin_2
#define I2C_PORT     GPIOC
#define I2C_SDA_PIN  GPIO_Pin_1
#define I2C_SCL_PIN  GPIO_Pin_2
#define ARGB_LED_COUNT 1U

#define BUS_UART_BAUD 460800U
#define SOF0 0x55U
#define SOF1 0xAAU
#define BUS_RX_MAX_PAYLOAD 96U
#define BUS_UART_RX_RING 128U
#define BUS_UART_RX_MASK (BUS_UART_RX_RING - 1U)

#define CMD_DISCOVER      0x00U
#define CMD_PING          0x01U
#define CMD_ENTER_BOOT    0x22U
#define CMD_GET_VERSION   0x30U
#define CMD_HEATER_STATUS 0x50U
#define CMD_HEATER_START  0x51U
#define CMD_HEATER_STOP   0x52U
#define CMD_HEATER_CLEAR_ERROR 0x53U
#define CMD_HB            0x70U

#define ST_OK        0x00U
#define ST_BAD_CMD   0x01U
#define ST_BAD_ARG   0x02U
#define ST_IO_ERR    0x04U
#define ST_NOT_READY 0x06U

#define HOST_TIMEOUT_MS 5000U
#define CONTROL_MS 250U
#define STATUS_SAMPLE_MS 2000U
#define DISPLAY_UPDATE_MS 1000U
#define AIR_SAMPLE_MS 5000U
#define OLED_ADDR 0x3CU
#define OLED_W 128U
#define OLED_H 32U
#define OLED_PAGES 4U
#define HEATER_WINDOW_MS 10000U
#define FAN_PWM_PERIOD 2399U
#define FAN_DRY_PWM 2100U
#define FAN_COOL_PWM 1800U
#define SOFTSTART_MS 30000U
#define TEMP_UNAVAILABLE 0x7FFF
#define HUM_UNAVAILABLE 0xFFFFU

enum {
    DRY_IDLE = 0,
    DRY_DRYING = 1,
    DRY_COOLDOWN = 2,
    DRY_ERROR = 3
};

enum {
    ERR_FAN = 1U << 0,
    ERR_HEATER_OVERHEAT = 1U << 1,
    ERR_DOOR_OPEN = 1U << 2,
    ERR_PCB_OVERHEAT = 1U << 3,
    ERR_NTC = 1U << 4,
    ERR_MASTER_LOST = 1U << 5
};

enum {
    WARN_AIR_SENSOR = 1U << 0,
    WARN_HEATER_HOT = 1U << 1,
    WARN_FAN = 1U << 2,
    WARN_DOOR_OPEN = 1U << 3
};

static volatile uint32_t g_tick;
static uint8_t g_registered;
static uint32_t g_last_host_tick;
static uint32_t g_last_hb_blink_tick;
static uint8_t g_hb_led_on;
static uint32_t g_last_control_tick;
static uint32_t g_last_sample_tick;
static uint32_t g_last_air_sample_tick;
static uint32_t g_last_display_tick;
static uint32_t g_window_start_tick;
static uint32_t g_dry_start_tick;
static uint32_t g_dry_duration_ms;
static uint8_t g_state = DRY_IDLE;
static uint8_t g_target_temp_c;
static uint16_t g_time_left_min;
static uint16_t g_error_bits;
static uint16_t g_warn_bits;
static int16_t g_pcb_temp_c10 = TEMP_UNAVAILABLE;
static int16_t g_heater_temp_c10 = TEMP_UNAVAILABLE;
static int16_t g_air_temp_c10 = TEMP_UNAVAILABLE;
static uint16_t g_humidity_rh10 = HUM_UNAVAILABLE;
static uint8_t g_heater_output_pct;
static uint8_t g_fan_pwm_byte;
static volatile uint32_t g_argb[ARGB_LED_COUNT];
static uint8_t g_argb_fault;
static uint8_t g_air_sensor_type; /* 0=none, 1=SHT40, 2=AHT20 */
static uint8_t g_oled_ok;
static uint8_t g_oled[OLED_W * OLED_PAGES];
static uint8_t g_argb_update_pending;
static uint8_t g_display_dirty = 1U;
static uint8_t g_display_last_colon = 0xFFU;
static int16_t g_display_last_heater_temp_c10 = TEMP_UNAVAILABLE;
static uint16_t g_display_last_humidity_rh10 = HUM_UNAVAILABLE;
static uint32_t g_display_last_time_s = 0xFFFFFFFFUL;
static volatile uint8_t g_bus_rx_buf[BUS_UART_RX_RING];
static volatile uint8_t g_bus_rx_head;
static volatile uint8_t g_bus_rx_tail;

void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void SysTick_Handler(void) { g_tick++; SysTick->SR = 0; }

void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void)
{
    while ((USART1->STATR & USART_FLAG_RXNE) != 0U) {
        uint8_t next;
        uint8_t b = (uint8_t)(USART1->DATAR & 0xFFU);
        next = (uint8_t)((g_bus_rx_head + 1U) & BUS_UART_RX_MASK);
        if (next != g_bus_rx_tail) {
            g_bus_rx_buf[g_bus_rx_head] = b;
            g_bus_rx_head = next;
        }
    }
}

static void put_u16le(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static uint16_t get_u16le(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static void put_i16le(uint8_t *p, int16_t v) { put_u16le(p, (uint16_t)v); }

static void led_set(GPIO_TypeDef *port, uint16_t pin, bool on)
{
    if (on) GPIO_SetBits(port, pin);
    else GPIO_ResetBits(port, pin);
}

uint32_t WS2812BLEDCallback(int ledno)
{
    return (ledno == 0) ? g_argb[0] : 0U;
}

static void argb_flush(void)
{
    uint32_t ctlr = SysTick->CTLR;
    uint32_t guard;

    if (g_argb_fault) return;

    SysTick->CTLR = 0U;

    guard = 60000U;
    while (WS2812BLEDInUse && --guard) {}
    if (!guard) {
        SysTick->CTLR = ctlr;
        g_argb_fault = 1U;
        return;
    }

    WS2812BDMAStart(ARGB_LED_COUNT);

    guard = 60000U;
    while (WS2812BLEDInUse && --guard) {}
    if (!guard) {
        g_argb_fault = 1U;
    }

    SysTick->CTLR = ctlr;
}

static void argb_set(uint8_t r, uint8_t g, uint8_t b)
{
    g_argb[0] = ((uint32_t)b << 16) | ((uint32_t)g << 8) | r;
}

static void argb_status_update(void)
{
    if (g_registered) argb_set(0U, 12U, 0U);
    else argb_set(12U, 0U, 0U);
    argb_flush();
}

static void request_argb_update(void)
{
    g_argb_update_pending = 1U;
}

static void gpio_init_all(void)
{
    GPIO_InitTypeDef g = {0};
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA | RCC_PB2Periph_GPIOC | RCC_PB2Periph_GPIOD | RCC_PB2Periph_AFIO, ENABLE);

    g.GPIO_Speed = GPIO_Speed_30MHz;
    g.GPIO_Mode = GPIO_Mode_Out_PP;
    g.GPIO_Pin = BOOT_LED_PIN | HB_LED_PIN | HEATER_PIN;
    GPIO_Init(GPIOC, &g);
    led_set(BOOT_LED_PORT, BOOT_LED_PIN, false);
    led_set(HB_LED_PORT, HB_LED_PIN, false);
    led_set(HEATER_PORT, HEATER_PIN, false);

    g.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    g.GPIO_Pin = DOOR_PIN;
    GPIO_Init(DOOR_PORT, &g);
    g.GPIO_Pin = FAN_STAT_PIN | PWR_STAT_PIN;
    GPIO_Init(GPIOA, &g);

    g.GPIO_Mode = GPIO_Mode_AIN;
    g.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_Init(GPIOD, &g);

    g.GPIO_Mode = GPIO_Mode_Out_OD;
    g.GPIO_Pin = I2C_SDA_PIN | I2C_SCL_PIN;
    GPIO_Init(I2C_PORT, &g);
    GPIO_SetBits(I2C_PORT, I2C_SDA_PIN | I2C_SCL_PIN);
}

static void fan_pwm_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_TimeBaseInitTypeDef tb = {0};
    TIM_OCInitTypeDef oc = {0};
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC | RCC_PB2Periph_AFIO | RCC_PB2Periph_TIM1, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap7_TIM1, ENABLE);
    gpio.GPIO_Pin = FAN_PIN;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(FAN_PORT, &gpio);
    tb.TIM_Period = FAN_PWM_PERIOD;
    tb.TIM_Prescaler = 0;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &tb);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 0;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init(TIM1, &oc);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

static void fan_set_pwm(uint16_t ccr)
{
    if (ccr > FAN_PWM_PERIOD) ccr = FAN_PWM_PERIOD;
    TIM1->CH4CVR = ccr;
    g_fan_pwm_byte = (uint8_t)((uint32_t)ccr * 255U / FAN_PWM_PERIOD);
}

static void adc_init_simple(void)
{
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);
    ADC1->SAMPTR2 = ADC_SMP3 | ADC_SMP4;
    ADC1->RSQR1 = 0U;
    ADC1->CTLR2 = ADC_ADON | ADC_EXTTRIG;
}

static uint16_t adc_read(uint8_t ch)
{
    uint32_t t = 100000U;
    ADC1->RSQR3 = ch;
    ADC1->STATR = 0U;
    ADC1->CTLR2 |= ADC_SWSTART;
    while (((ADC1->STATR & ADC_EOC) == 0U) && --t) {}
    return (uint16_t)(ADC1->RDATAR & 0x0FFFU);
}

static int16_t ntc_raw_to_c10(uint16_t raw)
{
    int32_t c10;
    if (raw < 32U || raw > 4064U) return TEMP_UNAVAILABLE;
    c10 = 250 + (((int32_t)raw - 2048) * 5) / 7;
    if (c10 < -400) c10 = -400;
    if (c10 > 1500) c10 = 1500;
    return (int16_t)c10;
}

static uint8_t door_open(void) { return (GPIO_ReadInputDataBit(DOOR_PORT, DOOR_PIN) == 0U) ? 1U : 0U; }
static uint8_t fan_ok(void) { return (GPIO_ReadInputDataBit(FAN_STAT_PORT, FAN_STAT_PIN) != 0U) ? 1U : 0U; }
static uint8_t power_ok(void) { return (GPIO_ReadInputDataBit(PWR_STAT_PORT, PWR_STAT_PIN) != 0U) ? 1U : 0U; }

static void i2c_delay(void) { for (volatile uint16_t d = 0U; d < 60U; d++) {} }
static void busy_delay_ms(uint16_t ms) { while (ms--) for (volatile uint32_t d = 0U; d < 12000U; d++) {} }
static void i2c_sda(bool high) { if (high) GPIO_SetBits(I2C_PORT, I2C_SDA_PIN); else GPIO_ResetBits(I2C_PORT, I2C_SDA_PIN); }
static void i2c_scl(bool high) { if (high) GPIO_SetBits(I2C_PORT, I2C_SCL_PIN); else GPIO_ResetBits(I2C_PORT, I2C_SCL_PIN); }
static uint8_t i2c_sda_read(void) { return GPIO_ReadInputDataBit(I2C_PORT, I2C_SDA_PIN) ? 1U : 0U; }

static void i2c_start(void)
{
    i2c_sda(true); i2c_scl(true); i2c_delay();
    i2c_sda(false); i2c_delay(); i2c_scl(false); i2c_delay();
}

static void i2c_stop(void)
{
    i2c_sda(false); i2c_delay(); i2c_scl(true); i2c_delay(); i2c_sda(true); i2c_delay();
}

static bool i2c_write_byte(uint8_t b)
{
    uint8_t i, ack;
    for (i = 0U; i < 8U; i++) {
        i2c_sda((b & 0x80U) != 0U); i2c_delay(); i2c_scl(true); i2c_delay(); i2c_scl(false); b <<= 1;
    }
    i2c_sda(true); i2c_delay(); i2c_scl(true); i2c_delay(); ack = (uint8_t)!i2c_sda_read(); i2c_scl(false); i2c_delay();
    return ack != 0U;
}

static uint8_t i2c_read_byte(bool ack)
{
    uint8_t i, b = 0U;
    i2c_sda(true);
    for (i = 0U; i < 8U; i++) {
        b <<= 1; i2c_scl(true); i2c_delay(); if (i2c_sda_read()) b |= 1U; i2c_scl(false); i2c_delay();
    }
    i2c_sda(!ack); i2c_delay(); i2c_scl(true); i2c_delay(); i2c_scl(false); i2c_sda(true);
    return b;
}

static bool i2c_probe(uint8_t addr)
{
    bool ok;
    i2c_start();
    ok = i2c_write_byte((uint8_t)(addr << 1));
    i2c_stop();
    return ok;
}

static bool oled_cmd(uint8_t cmd)
{
    i2c_start();
    if (!i2c_write_byte(OLED_ADDR << 1) || !i2c_write_byte(0x00U) || !i2c_write_byte(cmd)) { i2c_stop(); return false; }
    i2c_stop();
    return true;
}

static bool oled_data_chunk(const uint8_t *data, uint8_t len)
{
    uint8_t i;
    i2c_start();
    if (!i2c_write_byte(OLED_ADDR << 1) || !i2c_write_byte(0x40U)) { i2c_stop(); return false; }
    for (i = 0U; i < len; i++) {
        if (!i2c_write_byte(data[i])) { i2c_stop(); return false; }
    }
    i2c_stop();
    return true;
}

static void oled_clear_buf(void)
{
    memset(g_oled, 0, sizeof(g_oled));
}

static uint8_t font5(char c, uint8_t col)
{
    static const uint8_t digits[10][5] = {
        {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
        {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
        {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
        {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
        {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}
    };
    uint8_t g[5] = {0,0,0,0,0};
    if (c >= '0' && c <= '9') return digits[c - '0'][col];
    switch (c) {
    case 'A': { uint8_t t[5]={0x7E,0x11,0x11,0x11,0x7E}; memcpy(g,t,5); break; }
    case 'B': { uint8_t t[5]={0x7F,0x49,0x49,0x49,0x36}; memcpy(g,t,5); break; }
    case 'C': { uint8_t t[5]={0x3E,0x41,0x41,0x41,0x22}; memcpy(g,t,5); break; }
    case 'D': { uint8_t t[5]={0x7F,0x41,0x41,0x22,0x1C}; memcpy(g,t,5); break; }
    case 'E': { uint8_t t[5]={0x7F,0x49,0x49,0x49,0x41}; memcpy(g,t,5); break; }
    case 'F': { uint8_t t[5]={0x7F,0x09,0x09,0x09,0x01}; memcpy(g,t,5); break; }
    case 'G': { uint8_t t[5]={0x3E,0x41,0x49,0x49,0x7A}; memcpy(g,t,5); break; }
    case 'H': { uint8_t t[5]={0x7F,0x08,0x08,0x08,0x7F}; memcpy(g,t,5); break; }
    case 'U': { uint8_t t[5]={0x3F,0x40,0x40,0x40,0x3F}; memcpy(g,t,5); break; }
    case 'I': { uint8_t t[5]={0x00,0x41,0x7F,0x41,0x00}; memcpy(g,t,5); break; }
    case 'L': { uint8_t t[5]={0x7F,0x40,0x40,0x40,0x40}; memcpy(g,t,5); break; }
    case 'M': { uint8_t t[5]={0x7F,0x02,0x0C,0x02,0x7F}; memcpy(g,t,5); break; }
    case 'N': { uint8_t t[5]={0x7F,0x04,0x08,0x10,0x7F}; memcpy(g,t,5); break; }
    case 'O': { uint8_t t[5]={0x3E,0x41,0x41,0x41,0x3E}; memcpy(g,t,5); break; }
    case 'P': { uint8_t t[5]={0x7F,0x09,0x09,0x09,0x06}; memcpy(g,t,5); break; }
    case 'R': { uint8_t t[5]={0x7F,0x09,0x19,0x29,0x46}; memcpy(g,t,5); break; }
    case 'S': { uint8_t t[5]={0x46,0x49,0x49,0x49,0x31}; memcpy(g,t,5); break; }
    case 'T': { uint8_t t[5]={0x01,0x01,0x7F,0x01,0x01}; memcpy(g,t,5); break; }
    case 'Y': { uint8_t t[5]={0x07,0x08,0x70,0x08,0x07}; memcpy(g,t,5); break; }
    case '.': { uint8_t t[5]={0x00,0x60,0x60,0x00,0x00}; memcpy(g,t,5); break; }
    case ':': { uint8_t t[5]={0x00,0x36,0x36,0x00,0x00}; memcpy(g,t,5); break; }
    case '-': { uint8_t t[5]={0x08,0x08,0x08,0x08,0x08}; memcpy(g,t,5); break; }
    case '%': { uint8_t t[5]={0x62,0x64,0x08,0x13,0x23}; memcpy(g,t,5); break; }
    case '|': { uint8_t t[5]={0x00,0x00,0x7F,0x00,0x00}; memcpy(g,t,5); break; }
    case ' ': default: break;
    }
    return g[col];
}

static void oled_pixel(uint8_t x, uint8_t y, bool on)
{
    uint16_t idx;
    uint8_t mask;
    if (x >= OLED_W || y >= OLED_H) return;
    idx = (uint16_t)((uint16_t)(y >> 3U) * OLED_W + x);
    mask = (uint8_t)(1U << (y & 7U));
    if (on) g_oled[idx] |= mask;
    else g_oled[idx] &= (uint8_t)~mask;
}

static void oled_draw_char_xy(uint8_t x, uint8_t y, char c)
{
    uint8_t col, row, bits;
    if (x > (OLED_W - 6U) || y > (OLED_H - 7U)) return;
    for (col = 0U; col < 5U; col++) {
        bits = font5(c, col);
        for (row = 0U; row < 7U; row++) {
            oled_pixel((uint8_t)(x + col), (uint8_t)(y + row), (bits & (1U << row)) != 0U);
        }
    }
}

static void oled_draw_text_xy(uint8_t x, uint8_t y, const char *s)
{
    while (*s && x <= (OLED_W - 6U)) {
        oled_draw_char_xy(x, y, *s++);
        x = (uint8_t)(x + 6U);
    }
}

static void oled_draw_char_bold_xy(uint8_t x, uint8_t y, char c)
{
    uint8_t col, row, bits;
    if (x > (OLED_W - 7U) || y > (OLED_H - 8U)) return;
    for (col = 0U; col < 5U; col++) {
        bits = font5(c, col);
        for (row = 0U; row < 7U; row++) {
            if ((bits & (1U << row)) == 0U) continue;
            oled_pixel((uint8_t)(x + col), (uint8_t)(y + row), true);
            oled_pixel((uint8_t)(x + col + 1U), (uint8_t)(y + row), true);
            oled_pixel((uint8_t)(x + col), (uint8_t)(y + row + 1U), true);
        }
    }
}

static void oled_draw_digit_compact_scaled(uint8_t x, uint8_t y, uint8_t d, uint8_t scale)
{
    static const uint8_t glyph[10][3] = {
        {0x1F,0x11,0x1F}, {0x00,0x1F,0x00}, {0x1D,0x15,0x17}, {0x15,0x15,0x1F},
        {0x07,0x04,0x1F}, {0x17,0x15,0x1D}, {0x1F,0x15,0x1D}, {0x01,0x01,0x1F},
        {0x1F,0x15,0x1F}, {0x17,0x15,0x1F}
    };
    uint8_t col, row, xx, yy, bits;
    d %= 10U;
    if (scale < 1U) scale = 1U;
    for (col = 0U; col < 3U; col++) {
        bits = glyph[d][col];
        for (row = 0U; row < 5U; row++) {
            if ((bits & (1U << row)) == 0U) continue;
            for (xx = 0U; xx < scale; xx++) {
                for (yy = 0U; yy < scale; yy++) {
                    oled_pixel((uint8_t)(x + (col * scale) + xx), (uint8_t)(y + (row * scale) + yy), true);
                }
            }
        }
    }
}

static void oled_draw_digit_compact2(uint8_t x, uint8_t y, uint8_t d)
{
    oled_draw_digit_compact_scaled(x, y, d, 2U);
}

static void oled_draw_colon_compact2(uint8_t x, uint8_t y)
{
    oled_pixel(x, (uint8_t)(y + 2U), true);
    oled_pixel(x, (uint8_t)(y + 3U), true);
    oled_pixel((uint8_t)(x + 1U), (uint8_t)(y + 2U), true);
    oled_pixel((uint8_t)(x + 1U), (uint8_t)(y + 3U), true);
    oled_pixel(x, (uint8_t)(y + 7U), true);
    oled_pixel(x, (uint8_t)(y + 8U), true);
    oled_pixel((uint8_t)(x + 1U), (uint8_t)(y + 7U), true);
    oled_pixel((uint8_t)(x + 1U), (uint8_t)(y + 8U), true);
}

static uint8_t oled_draw_u16_compact_scaled(uint8_t x, uint8_t y, uint16_t v, uint8_t scale)
{
    char tmp[5];
    uint8_t n = 0U;
    uint8_t step = (uint8_t)((3U * scale) + 1U);
    if (v == 0U) {
        oled_draw_digit_compact_scaled(x, y, 0U, scale);
        return (uint8_t)(x + step);
    }
    while (v && n < sizeof(tmp)) { tmp[n++] = (char)(v % 10U); v = (uint16_t)(v / 10U); }
    while (n) {
        oled_draw_digit_compact_scaled(x, y, (uint8_t)tmp[--n], scale);
        x = (uint8_t)(x + step);
    }
    return x;
}

static uint8_t compact_u16_width_scaled(uint16_t v, uint8_t scale)
{
    uint8_t digits = 1U;
    uint8_t step = (uint8_t)((3U * scale) + 1U);
    while (v >= 10U) {
        digits++;
        v = (uint16_t)(v / 10U);
    }
    return (uint8_t)(digits * step);
}

static uint32_t display_time_left_s(void)
{
    if (g_state != DRY_DRYING && g_state != DRY_COOLDOWN) return 0U;
    if ((g_tick - g_dry_start_tick) >= g_dry_duration_ms) return 0U;
    return (g_dry_duration_ms - (g_tick - g_dry_start_tick) + 999UL) / 1000UL;
}

static uint8_t center_x(uint8_t left, uint8_t right, uint8_t w)
{
    uint8_t col_w = (uint8_t)(right - left + 1U);
    if (w >= col_w) return left;
    return (uint8_t)(left + ((col_w - w) / 2U));
}

static void display_draw_c10_value(uint8_t left, uint8_t right, uint8_t y, int16_t v, char unit)
{
    int16_t rounded;
    bool neg = false;
    uint8_t x, tx, w;
    uint8_t scale = 2U;
    if (v == TEMP_UNAVAILABLE) {
        x = center_x(left, right, 6U);
        oled_draw_char_bold_xy(x, (uint8_t)(y + 2U), '-');
        return;
    }
    if (v < 0) {
        neg = true;
        v = (int16_t)-v;
    }
    rounded = (int16_t)((v + 5) / 10);
    w = (uint8_t)(compact_u16_width_scaled((uint16_t)rounded, scale) + 7U + (neg ? 6U : 0U));
    x = center_x(left, right, w);
    if (neg) {
        oled_draw_text_xy(x, (uint8_t)(y + 5U), "-");
        x = (uint8_t)(x + 6U);
    }
    tx = oled_draw_u16_compact_scaled(x, y, (uint16_t)rounded, scale);
    oled_draw_char_bold_xy((uint8_t)(tx + 1U), (uint8_t)(y + 2U), unit);
}

static void display_draw_rh_value(uint8_t left, uint8_t right, uint8_t y, uint16_t v)
{
    uint8_t x, tx, w;
    uint16_t rounded;
    uint8_t scale = 2U;
    if (v == HUM_UNAVAILABLE) {
        x = center_x(left, right, 6U);
        oled_draw_char_bold_xy(x, (uint8_t)(y + 2U), '-');
        return;
    }
    rounded = (uint16_t)((v + 5U) / 10U);
    w = (uint8_t)(compact_u16_width_scaled(rounded, scale) + 7U);
    x = center_x(left, right, w);
    tx = oled_draw_u16_compact_scaled(x, y, rounded, scale);
    oled_draw_char_bold_xy((uint8_t)(tx + 1U), (uint8_t)(y + 2U), '%');
}

static void display_draw_time(uint8_t left, uint8_t right, uint8_t y, uint32_t seconds)
{
    uint8_t x;
    uint16_t h = (uint16_t)(seconds / 3600UL);
    uint16_t m = (uint16_t)((seconds / 60UL) % 60UL);
    uint16_t s = (uint16_t)(seconds % 60UL);
    bool colon_on = (((g_tick / 1000UL) & 1UL) != 0UL);
    if (h > 99U) h = 99U;
    x = center_x(left, right, 48U);
    oled_draw_digit_compact2(x, y, (uint8_t)(h / 10U)); x = (uint8_t)(x + 7U);
    oled_draw_digit_compact2(x, y, (uint8_t)(h % 10U)); x = (uint8_t)(x + 7U);
    if (colon_on) oled_draw_colon_compact2(x, y);
    x = (uint8_t)(x + 3U);
    oled_draw_digit_compact2(x, y, (uint8_t)(m / 10U)); x = (uint8_t)(x + 7U);
    oled_draw_digit_compact2(x, y, (uint8_t)(m % 10U)); x = (uint8_t)(x + 7U);
    if (colon_on) oled_draw_colon_compact2(x, y);
    x = (uint8_t)(x + 3U);
    oled_draw_digit_compact2(x, y, (uint8_t)(s / 10U)); x = (uint8_t)(x + 7U);
    oled_draw_digit_compact2(x, y, (uint8_t)(s % 10U));
}

static bool oled_flush(void)
{
    uint16_t off;
    if (!oled_cmd(0x21U) || !oled_cmd(0U) || !oled_cmd(OLED_W - 1U)) return false;
    if (!oled_cmd(0x22U) || !oled_cmd(0U) || !oled_cmd(OLED_PAGES - 1U)) return false;
    for (off = 0U; off < sizeof(g_oled); off = (uint16_t)(off + 16U)) {
        if (!oled_data_chunk(&g_oled[off], 16U)) return false;
    }
    return true;
}

static bool oled_init(void)
{
    static const uint8_t init[] = {
        0xAE,       /* display off */
        0xD5,0x80,  /* clock */
        0xA8,0x1F,  /* multiplex 1/32 */
        0xD3,0x00,  /* offset */
        0x40,       /* start line */
        0x8D,0x14,  /* charge pump */
        0x20,0x00,  /* horizontal addressing */
        0xA1,       /* segment remap */
        0xC8,       /* COM scan dec */
        0xDA,0x02,  /* COM pins for 128x32 */
        0x81,0x8F,  /* contrast */
        0xD9,0xF1,  /* precharge */
        0xDB,0x40,  /* vcom */
        0xA4,       /* resume RAM */
        0xA6,       /* normal */
        0x2E,       /* scroll off */
        0xAF        /* display on */
    };
    uint8_t i;
    busy_delay_ms(50U);
    if (!i2c_probe(OLED_ADDR)) return false;
    for (i = 0U; i < sizeof(init); i++) {
        if (!oled_cmd(init[i])) return false;
    }
    oled_clear_buf();
    return oled_flush();
}

static void display_task(bool force)
{
    uint8_t colon;
    uint32_t time_s;
    if (!g_oled_ok) return;
    colon = (uint8_t)((g_tick / 1000UL) & 1UL);
    time_s = display_time_left_s();
    if (g_heater_temp_c10 != g_display_last_heater_temp_c10 ||
        g_humidity_rh10 != g_display_last_humidity_rh10 ||
        time_s != g_display_last_time_s ||
        colon != g_display_last_colon) {
        g_display_dirty = 1U;
    }
    if (!force && !g_display_dirty) return;
    if (!force && (g_tick - g_last_display_tick) < DISPLAY_UPDATE_MS) return;
    g_last_display_tick = g_tick;
    g_display_dirty = 0U;
    g_display_last_colon = colon;
    g_display_last_heater_temp_c10 = g_heater_temp_c10;
    g_display_last_humidity_rh10 = g_humidity_rh10;
    g_display_last_time_s = time_s;
    oled_clear_buf();

    oled_draw_text_xy(8U, 0U, "TEM");
    oled_draw_text_xy(45U, 0U, "HUMI");
    oled_draw_text_xy(90U, 0U, "TIME");

    display_draw_c10_value(1U, 38U, 15U, g_heater_temp_c10, 'C');
    display_draw_rh_value(40U, 74U, 15U, g_humidity_rh10);
    display_draw_time(76U, 127U, 15U, time_s);

    if (!oled_flush()) g_oled_ok = 0U;
}

static void air_sensor_detect(void)
{
    if (i2c_probe(0x44U)) g_air_sensor_type = 1U;
    else if (i2c_probe(0x38U)) g_air_sensor_type = 2U;
    else g_air_sensor_type = 0U;
}

static bool sht40_read(int16_t *temp_c10, uint16_t *rh10)
{
    uint8_t d[6], i;
    uint16_t raw_t, raw_h;
    i2c_start();
    if (!i2c_write_byte(0x44U << 1) || !i2c_write_byte(0xFD)) { i2c_stop(); return false; }
    i2c_stop();
    for (volatile uint32_t w = 0U; w < 120000U; w++) {}
    i2c_start();
    if (!i2c_write_byte((uint8_t)((0x44U << 1) | 1U))) { i2c_stop(); return false; }
    for (i = 0U; i < 6U; i++) d[i] = i2c_read_byte(i < 5U);
    i2c_stop();
    raw_t = ((uint16_t)d[0] << 8) | d[1];
    raw_h = ((uint16_t)d[3] << 8) | d[4];
    *temp_c10 = (int16_t)((1750L * raw_t) / 65535L - 450L);
    *rh10 = (uint16_t)((1250UL * raw_h) / 65535UL);
    if (*rh10 > 1000U) *rh10 = 1000U;
    return true;
}

static bool aht20_read(int16_t *temp_c10, uint16_t *rh10)
{
    uint8_t d[6], i;
    uint32_t raw_h, raw_t;
    i2c_start();
    if (!i2c_write_byte(0x38U << 1) || !i2c_write_byte(0xAC) || !i2c_write_byte(0x33) || !i2c_write_byte(0x00)) { i2c_stop(); return false; }
    i2c_stop();
    for (volatile uint32_t w = 0U; w < 900000U; w++) {}
    i2c_start();
    if (!i2c_write_byte((uint8_t)((0x38U << 1) | 1U))) { i2c_stop(); return false; }
    for (i = 0U; i < 6U; i++) d[i] = i2c_read_byte(i < 5U);
    i2c_stop();
    if ((d[0] & 0x80U) != 0U) return false;
    raw_h = ((uint32_t)d[1] << 12) | ((uint32_t)d[2] << 4) | ((uint32_t)d[3] >> 4);
    raw_t = (((uint32_t)d[3] & 0x0FU) << 16) | ((uint32_t)d[4] << 8) | d[5];
    *rh10 = (uint16_t)((raw_h * 1000UL) >> 20);
    *temp_c10 = (int16_t)(((int32_t)((raw_t * 2000UL) >> 20)) - 500);
    return true;
}

static void sample_task(bool force)
{
    if (!force && (g_tick - g_last_sample_tick) < STATUS_SAMPLE_MS) return;
    g_last_sample_tick = g_tick;
    g_pcb_temp_c10 = ntc_raw_to_c10(adc_read(3U));
    g_heater_temp_c10 = ntc_raw_to_c10(adc_read(4U));
    if (!force && (g_tick - g_last_air_sample_tick) < AIR_SAMPLE_MS) return;
    g_last_air_sample_tick = g_tick;
    if ((g_air_sensor_type == 1U && sht40_read(&g_air_temp_c10, &g_humidity_rh10)) ||
        (g_air_sensor_type == 2U && aht20_read(&g_air_temp_c10, &g_humidity_rh10))) {
        g_warn_bits &= (uint16_t)~WARN_AIR_SENSOR;
    } else {
        g_warn_bits |= WARN_AIR_SENSOR;
        g_air_temp_c10 = TEMP_UNAVAILABLE;
        g_humidity_rh10 = HUM_UNAVAILABLE;
        air_sensor_detect();
    }
}

static void heater_set(bool on) { led_set(HEATER_PORT, HEATER_PIN, on); }

static void dry_stop(uint16_t err)
{
    heater_set(false);
    g_heater_output_pct = 0U;
    if (err) g_error_bits |= err;
    if (err) g_state = DRY_ERROR;
    else g_state = DRY_COOLDOWN;
    fan_set_pwm(FAN_COOL_PWM);
    request_argb_update();
}

static void bus_mark_master_seen(void)
{
    uint8_t changed = 0U;
    g_last_host_tick = g_tick;
    if ((g_error_bits & ERR_MASTER_LOST) != 0U) {
        g_error_bits &= (uint16_t)~ERR_MASTER_LOST;
        if (g_state == DRY_ERROR && g_error_bits == 0U) g_state = DRY_IDLE;
        changed = 1U;
    }
    if (!g_registered) {
        g_registered = 1U;
        g_last_hb_blink_tick = g_tick;
        g_hb_led_on = 1U;
        led_set(HB_LED_PORT, HB_LED_PIN, true);
        changed = 1U;
    }
    if (changed) request_argb_update();
}

static uint16_t crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU, i, j;
    for (i = 0U; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (j = 0U; j < 8U; j++) crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
    return crc;
}

static void bus_uart_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    USART_InitTypeDef uart = {0};
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD | RCC_PB2Periph_AFIO | RCC_PB2Periph_USART1, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap1_USART1, DISABLE);
    gpio.GPIO_Pin = RS485_DE_PIN;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(RS485_DE_PORT, &gpio);
    GPIO_ResetBits(RS485_DE_PORT, RS485_DE_PIN);
    gpio.GPIO_Pin = GPIO_Pin_5;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &gpio);
    USART_StructInit(&uart);
    uart.USART_BaudRate = BUS_UART_BAUD;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &uart);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
    NVIC_EnableIRQ(USART1_IRQn);
    GPIO_PinRemapConfig(GPIO_PartialRemap1_USART1, DISABLE);
}

static void bus_uart_write(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    GPIO_SetBits(RS485_DE_PORT, RS485_DE_PIN);
    for (volatile uint16_t t = 0U; t < 200U; t++) {}
    for (i = 0U; i < len; i++) {
        while ((USART1->STATR & USART_FLAG_TXE) == 0U) {}
        USART1->DATAR = buf[i];
    }
    while ((USART1->STATR & USART_FLAG_TC) == 0U) {}
    GPIO_ResetBits(RS485_DE_PORT, RS485_DE_PIN);
}

static bool bus_uart_read_byte(uint8_t *out)
{
    if (g_bus_rx_tail == g_bus_rx_head) return false;
    *out = g_bus_rx_buf[g_bus_rx_tail];
    g_bus_rx_tail = (uint8_t)((g_bus_rx_tail + 1U) & BUS_UART_RX_MASK);
    return true;
}

static void bus_send(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint16_t plen)
{
    uint8_t frame[9 + BUS_RX_MAX_PAYLOAD];
    uint16_t crc;
    frame[0] = SOF0; frame[1] = SOF1; frame[2] = DEVICE_ADDR; frame[3] = (uint8_t)(cmd | 0x80U); frame[4] = seq;
    put_u16le(&frame[5], plen);
    if (plen) memcpy(&frame[7], payload, plen);
    crc = crc16(&frame[2], (uint16_t)(5U + plen));
    put_u16le(&frame[7 + plen], crc);
    bus_uart_write(frame, (uint16_t)(9U + plen));
}

static void fill_identity(uint8_t *out, uint16_t *out_len)
{
    uint32_t uid0 = UID_WORD0, uid1 = UID_WORD1;
    out[0]=ST_OK; out[1]=DEVICE_TYPE; out[2]=DEVICE_ADDR;
    out[3]=APP_VERSION_MAJOR; out[4]=APP_VERSION_MINOR; out[5]=APP_VERSION_PATCH;
    out[6]=(uint8_t)uid0; out[7]=(uint8_t)(uid0>>8); out[8]=(uint8_t)(uid0>>16); out[9]=(uint8_t)(uid0>>24);
    out[10]=(uint8_t)uid1; out[11]=(uint8_t)(uid1>>8); out[12]=(uint8_t)(uid1>>16); out[13]=(uint8_t)(uid1>>24);
    *out_len = 14U;
}

static void fill_status(uint8_t *out, uint16_t *out_len)
{
    uint8_t p = 0U;
    out[p++] = ST_OK;
    out[p++] = g_state;
    put_u16le(&out[p], g_error_bits); p += 2U;
    put_u16le(&out[p], g_warn_bits); p += 2U;
    put_i16le(&out[p], g_pcb_temp_c10); p += 2U;
    put_i16le(&out[p], g_heater_temp_c10); p += 2U;
    put_i16le(&out[p], g_air_temp_c10); p += 2U;
    put_u16le(&out[p], g_humidity_rh10); p += 2U;
    out[p++] = door_open();
    out[p++] = fan_ok();
    out[p++] = power_ok();
    out[p++] = g_target_temp_c;
    put_u16le(&out[p], g_time_left_min); p += 2U;
    out[p++] = g_heater_output_pct;
    out[p++] = g_fan_pwm_byte;
    *out_len = p;
}

static void bus_handle_cmd(uint8_t dest, uint8_t cmd, uint8_t seq, const uint8_t *payload, uint16_t plen)
{
    uint8_t out[BUS_RX_MAX_PAYLOAD];
    uint16_t out_len = 0U;
    bool enter_boot = false;
    if (dest == BROADCAST_ADDR) for (volatile uint32_t d = 0U; d < (uint32_t)DEVICE_ADDR * 96000UL; d++) {}
    switch (cmd) {
    case CMD_DISCOVER:
    case CMD_PING:
    case CMD_GET_VERSION:
        bus_mark_master_seen();
        fill_identity(out, &out_len);
        break;
    case CMD_HEATER_STATUS:
        fill_status(out, &out_len);
        break;
    case CMD_HEATER_START: {
        uint16_t mins;
        if (plen != 3U) { out[0]=ST_BAD_ARG; out_len=1U; break; }
        mins = get_u16le(&payload[1]);
        if (payload[0] < 40U || payload[0] > 60U || mins < 1U || mins > 1440U) { out[0]=ST_BAD_ARG; out_len=1U; break; }
        bus_mark_master_seen();
        g_error_bits = 0U; g_warn_bits = 0U; g_target_temp_c = payload[0]; g_time_left_min = mins;
        g_dry_duration_ms = (uint32_t)mins * 60000UL;
        g_dry_start_tick = g_tick; g_window_start_tick = g_tick; g_state = DRY_DRYING;
        fan_set_pwm(FAN_DRY_PWM); request_argb_update(); out[0]=ST_OK; out_len=1U; break;
    }
    case CMD_HEATER_STOP:
        if (plen != 0U) { out[0]=ST_BAD_ARG; out_len=1U; break; }
        dry_stop(0U); out[0]=ST_OK; out_len=1U; break;
    case CMD_HEATER_CLEAR_ERROR:
        if (plen != 0U) { out[0]=ST_BAD_ARG; out_len=1U; break; }
        g_error_bits = 0U; g_warn_bits = 0U; if (g_state == DRY_ERROR) g_state = DRY_IDLE; request_argb_update();
        out[0]=ST_OK; out_len=1U; break;
    case CMD_ENTER_BOOT:
        if (plen != 0U) { out[0]=ST_BAD_ARG; out_len=1U; break; }
        heater_set(false); fan_set_pwm(0U); argb_set(0U,0U,0U); argb_flush();
        FLASH_Unlock_Fast();
        FLASH_ErasePage_Fast((0x08000000UL + 62UL*1024UL - 4UL) & 0xFFFFFF00UL);
        FLASH->CTLR |= (uint32_t)0x00008000UL;
        FLASH->CTLR |= (uint32_t)0x00000080UL;
        out[0]=ST_OK; out_len=1U; enter_boot = true; break;
    case CMD_HB:
        if (g_registered) {
            g_last_host_tick = g_tick;
        }
        out[0]=ST_OK; out[1]=DEVICE_TYPE; out[2]=DEVICE_ADDR; out_len=3U; break;
    default:
        out[0]=ST_BAD_CMD; out_len=1U; break;
    }
    bus_send(cmd, seq, out, out_len);
    if (enter_boot) {
        while ((USART1->STATR & USART_FLAG_TC) == 0U) {}
        RCC_ClearFlag();
        SystemReset_StartMode(Start_Mode_BOOT);
        NVIC_SystemReset();
        while (1) {}
    }
}

static void bus_poll(void)
{
    static uint8_t rx[9 + BUS_RX_MAX_PAYLOAD];
    static uint16_t rx_len;
    uint8_t b;
    while (bus_uart_read_byte(&b)) {
        if (rx_len >= sizeof(rx)) rx_len = 0U;
        rx[rx_len++] = b;
    }
    while (rx_len >= 2U) {
        uint16_t plen, frame_len, crc_rx, crc_calc;
        if (rx[0] != SOF0 || rx[1] != SOF1) { memmove(rx, rx+1, (size_t)(rx_len-1U)); rx_len--; continue; }
        if (rx_len < 9U) return;
        plen = get_u16le(&rx[5]);
        if (plen > BUS_RX_MAX_PAYLOAD) { memmove(rx, rx+1, (size_t)(rx_len-1U)); rx_len--; continue; }
        frame_len = (uint16_t)(9U + plen);
        if (rx_len < frame_len) return;
        crc_rx = get_u16le(&rx[7 + plen]);
        crc_calc = crc16(&rx[2], (uint16_t)(5U + plen));
        if (crc_rx == crc_calc) {
            uint8_t dest = rx[2], cmd = rx[3], seq = rx[4];
            if (dest == DEVICE_ADDR || (dest == BROADCAST_ADDR && cmd == CMD_DISCOVER)) {
                bus_handle_cmd(dest, cmd, seq, &rx[7], plen);
            }
        }
        if (rx_len > frame_len) memmove(rx, rx+frame_len, (size_t)(rx_len-frame_len));
        rx_len = (uint16_t)(rx_len - frame_len);
    }
}

static void bus_housekeeping(void)
{
    if (g_registered && (g_tick - g_last_host_tick) >= HOST_TIMEOUT_MS) {
        g_registered = 0U;
        g_hb_led_on = 0U;
        led_set(HB_LED_PORT, HB_LED_PIN, false);
        dry_stop(ERR_MASTER_LOST);
    }
}

static void argb_update_task(void)
{
    if (!g_argb_update_pending) return;
    g_argb_update_pending = 0U;
    argb_status_update();
}

static void hb_led_task(void)
{
    if (!g_registered) return;
    if ((g_tick - g_last_hb_blink_tick) >= 1000U) {
        g_last_hb_blink_tick = g_tick;
        g_hb_led_on = (uint8_t)!g_hb_led_on;
        led_set(HB_LED_PORT, HB_LED_PIN, g_hb_led_on != 0U);
    }
}

static void control_task(void)
{
    int16_t target_c10;
    int16_t err;
    uint8_t max_pct;

    sample_task(false);
    if ((g_tick - g_last_control_tick) < CONTROL_MS) return;
    g_last_control_tick = g_tick;
    if (g_state != DRY_DRYING && g_state != DRY_COOLDOWN) return;

    if (g_heater_temp_c10 == TEMP_UNAVAILABLE || g_pcb_temp_c10 == TEMP_UNAVAILABLE) { dry_stop(ERR_NTC); return; }
    if (door_open()) {
        g_warn_bits |= WARN_DOOR_OPEN;
        g_heater_output_pct = 0U;
        heater_set(false);
        fan_set_pwm(FAN_COOL_PWM);
        return;
    }
    g_warn_bits &= (uint16_t)~WARN_DOOR_OPEN;
    if (g_pcb_temp_c10 >= 800) { dry_stop(ERR_PCB_OVERHEAT); return; }
    if (g_heater_temp_c10 >= 1100) { dry_stop(ERR_HEATER_OVERHEAT); return; }
    if ((g_state == DRY_DRYING) && !fan_ok()) g_warn_bits |= WARN_FAN;
    if (!power_ok()) g_warn_bits |= WARN_FAN;
    if (g_heater_temp_c10 >= 950) { g_warn_bits |= WARN_HEATER_HOT; fan_set_pwm(FAN_DRY_PWM); }

    if (g_state == DRY_COOLDOWN) {
        heater_set(false);
        if (g_heater_temp_c10 < 450) { fan_set_pwm(0U); g_state = g_error_bits ? DRY_ERROR : DRY_IDLE; }
        return;
    }

    if ((g_tick - g_dry_start_tick) >= g_dry_duration_ms) { dry_stop(0U); return; }
    g_time_left_min = (uint16_t)((g_dry_duration_ms - (g_tick - g_dry_start_tick) + 59999UL) / 60000UL);
    target_c10 = (int16_t)g_target_temp_c * 10;
    err = (int16_t)(target_c10 - g_heater_temp_c10);
    if (err <= 0) g_heater_output_pct = 0U;
    else if (err >= 300) g_heater_output_pct = 100U;
    else g_heater_output_pct = (uint8_t)(err / 3);
    max_pct = ((g_tick - g_dry_start_tick) < SOFTSTART_MS) ? 35U : 100U;
    if (g_heater_output_pct > max_pct) g_heater_output_pct = max_pct;
    if ((g_tick - g_window_start_tick) >= HEATER_WINDOW_MS) g_window_start_tick = g_tick;
    heater_set(((g_tick - g_window_start_tick) < ((uint32_t)g_heater_output_pct * HEATER_WINDOW_MS / 100U)) && g_state == DRY_DRYING);
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    SysTick->CTLR = 0;
    SysTick->SR = 0;
    SysTick->CMP = (SystemCoreClock / 1000U) - 1U;
    SysTick->CNT = 0;
    SysTick->CTLR = 0x0F;
    NVIC_EnableIRQ(SysTicK_IRQn);

    gpio_init_all();
    fan_pwm_init();
    WS2812BDMAInit();
    argb_status_update();
    bus_uart_init();
    adc_init_simple();
    air_sensor_detect();
    sample_task(true);
    g_oled_ok = oled_init() ? 1U : 0U;
    display_task(true);

    while (1) {
        bus_poll();
        bus_housekeeping();
        bus_poll();
        control_task();
        bus_poll();
        hb_led_task();
        argb_update_task();
        bus_poll();
        display_task(false);
        bus_poll();
    }
}
