/* main_rewinder.c
 *
 * Bus    USART1 RS485 460800 8N1 PD6=TX PD5=RX PD4=DE
 * PN0031 USART2       115200 8N1 PD2=TX PD3=RX  (GPIO_PartialRemap3_USART2)
 * Motors TIM2_CH1(PC0) TIM2_CH3(PC3) TIM1_CH2(PC5) TIM1_CH4(PC7) — SI2318A MOSFET, one-way PWM
 * PCF8574 SW-I2C PC1=SDA PC2=SCL PC4=INT  addr 0x20
 *   P7=SW1  P6=SWI1  P5=SW2  P4=SWI2  P3=SW4  P2=SWI4  P1=SW3  P0=SWI3
 * ARGB    WS2812 x5  PC6   LEDs 0-3 = motors, LED 4 = system
 * LED     PD0
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <ch32v00X.h>
#include "ch32v00X_tim.h"
#include "ch32v00X_dma.h"
#include "ch32v00X_spi.h"
#include "lib/pcf8574_sw_i2c.h"
#include "ch32v00X_flash.h"

/* WS2812B SPI+DMA driver — CH32V006 register name shims */
#define AHBPCENR               HBPCENR
#define APB2PCENR              PB2PCENR
#define RCC_AHBPeriph_DMA1     RCC_HBPeriph_DMA1
#define RCC_APB2Periph_GPIOC   RCC_PB2Periph_GPIOC
#define RCC_APB2Periph_SPI1    RCC_PB2Periph_SPI1
#define GPIO_Speed_10MHz       GPIO_Speed_30MHz
#define GPIO_CNF_OUT_PP_AF     0x08
#define CTLR1_SPE_Set          ((uint16_t)0x0040)
#define WS_LED_COUNT           ARGB_LED_COUNT
#define WS2812DMA_IMPLEMENTATION
#define DMALEDS                8
#define WSGRB
#include "lib/ws2812b_dma_spi_led_driver.h"

/* ------------------------------------------------------------------ */
/* Device identity                                                      */
/* ------------------------------------------------------------------ */
#define DEV_TYPE_HEATER   0x00U
#define DEV_TYPE_REWINDER 0x01U
#define DEVICE_TYPE       DEV_TYPE_REWINDER
#define DEVICE_ADDR       0x01U   /* unique per device, 0x01-0xFE */
#define BROADCAST_ADDR    0xFFU

#define APP_VERSION_MAJOR 0x01U
#define APP_VERSION_MINOR 0x01U
#define APP_VERSION_PATCH 0x01U

#define UID_WORD0         (*(volatile uint32_t *)0x1FFFF7E8U)  /* CH32V006 ESIG_UNIID1 */
#define UID_WORD1         (*(volatile uint32_t *)0x1FFFF7ECU)  /* CH32V006 ESIG_UNIID2 */

/* ------------------------------------------------------------------ */
/* Pin definitions                                                      */
/* ------------------------------------------------------------------ */
#define RS485_DE_PORT    GPIOD
#define RS485_DE_PIN     GPIO_Pin_4

#define LED_PORT         GPIOD
#define LED_PIN          GPIO_Pin_0

#define ARGB_PORT        GPIOC
#define ARGB_PIN         GPIO_Pin_6
#define ARGB_LED_COUNT   5U

#define PCF8574_I2C_PORT GPIOC
#define PCF8574_SDA_PIN  GPIO_Pin_1
#define PCF8574_SCL_PIN  GPIO_Pin_2
#define PCF8574_INT_PORT GPIOC
#define PCF8574_INT_PIN  GPIO_Pin_4
#define PCF8574_ADDR     0x20U

/* SW and SWI bit positions per channel (PCF8574 port, active-low) */
#define SW_BIT_CH(n)  ((n)==1 ? 7 : (n)==2 ? 5 : (n)==3 ? 1 : 3)
#define SWI_BIT_CH(n) ((n)==1 ? 6 : (n)==2 ? 4 : (n)==3 ? 0 : 2)

/* ------------------------------------------------------------------ */
/* Protocol constants                                                   */
/* ------------------------------------------------------------------ */
#define BUS_UART_BAUD    460800U
#define PN0031_UART_BAUD 115200U

#define SOF0  0x55U
#define SOF1  0xAAU

#define CMD_DISCOVER         0x00U
#define CMD_PING             0x01U
#define CMD_ENTER_BOOT       0x22U
#define CMD_GET_VERSION      0x30U
#define CMD_READ_UID         0x32U
#define CMD_READ_FILAMENT    0x33U
#define CMD_MOTOR_CTRL       0x40U
#define CMD_GET_SWITCH_STATUS 0x41U
#define CMD_HB               0x70U   /* heartbeat, device-initiated */

#define ST_OK        0x00U
#define ST_BAD_CMD   0x01U
#define ST_BAD_ARG   0x02U
#define ST_IO_ERR    0x04U
#define ST_NO_TAG    0x05U
#define ST_NOT_READY 0x06U

#define BUS_RX_MAX_PAYLOAD 96U

/* PN0031 protocol */
#define PN0031_STX         0xA0U
#define PN0031_ETX         0x0DU
#define PN0031_ADDR        0x00U
#define PN0031_FT_CMD      0x02U
#define PN0031_FC_READ_CARD  0x20U
#define PN0031_FC_READ_BLOCK 0x22U
#define UID_MAX_LEN        16U
#define FRAME_MAX_DATA     96U

/* PWM */
#define PWM_PERIOD  2399U   /* ARR for 10 kHz @ 24 MHz, PSC=0 */
#define MOTOR_START_DUTY 200U
#define MOTOR_RAMP_STEP  40U
#define MOTOR_RAMP_MS    10U

/* ------------------------------------------------------------------ */
/* Types                                                                */
/* ------------------------------------------------------------------ */
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

/* ------------------------------------------------------------------ */
/* Global state                                                         */
/* ------------------------------------------------------------------ */
static volatile uint32_t g_tick = 0U;
static volatile uint8_t  g_pcf_irq = 0U;
static volatile uint8_t  g_registered = 0U;
static volatile uint32_t g_last_host_tick = 0U;
#define HOST_TIMEOUT_MS  5000U
static Pcf8574Bus        g_pcf_bus = {PCF8574_I2C_PORT, PCF8574_SDA_PIN, PCF8574_SCL_PIN, 20U};
static Pcf8574Device     g_pcf_dev;
static uint8_t           g_motor_speed[4] = {0};   /* speed per channel 1-4 stored at index 0-3 */
static uint16_t          g_motor_ccr[4] = {0};
static uint16_t          g_motor_target_ccr[4] = {0};
static volatile uint32_t g_argb[ARGB_LED_COUNT];    /* packed 0x00RRGGBB, sent as GRB via WSGRB */
static uint16_t          g_led_rx_ticks = 0U;
static uint32_t          g_last_hb_tick = 0U;
static uint32_t          g_last_motor_ramp_tick = 0U;

/* ------------------------------------------------------------------ */
/* Delay / SysTick                                                      */
/* ------------------------------------------------------------------ */
void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void SysTick_Handler(void)
{
    g_tick++;
    SysTick->SR = 0;  /* must clear — if left set, SysTick re-asserts immediately and starves DMA IRQ */
}

static void delay_ms(uint32_t ms)
{
    /* busy-loop — independent of SysTick so it works even when SysTick is disabled during DMA */
    while (ms-- != 0U) {
        for (volatile uint32_t d = 0; d < 12000U; d++) {}  /* ~1ms @ 48MHz, 4 cycles/iter */
    }
}

/* ------------------------------------------------------------------ */
/* LED (PD0)                                                            */
/* ------------------------------------------------------------------ */
static void led_init(void)
{
    GPIO_InitTypeDef g = {0};
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD, ENABLE);
    g.GPIO_Pin   = LED_PIN;
    g.GPIO_Speed = GPIO_Speed_30MHz;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(LED_PORT, &g);
    GPIO_ResetBits(LED_PORT, LED_PIN);
}

static void led_set(bool on)
{
    if (on) GPIO_SetBits(LED_PORT, LED_PIN);
    else    GPIO_ResetBits(LED_PORT, LED_PIN);
}

/* ------------------------------------------------------------------ */
/* ARGB — WS2812B via SPI+DMA on PC6 (SPI1 MOSI)                      */
/* WS2812BDMAInit() configures PC6 as AF push-pull and starts DMA.     */
/* ------------------------------------------------------------------ */

uint32_t WS2812BLEDCallback(int ledno)
{
    if (ledno < (int)ARGB_LED_COUNT) return g_argb[ledno];
    return 0U;
}

static void argb_flush(void)
{
    uint32_t ctlr = SysTick->CTLR;
    SysTick->CTLR = 0U;
    while (WS2812BLEDInUse) { }
    WS2812BDMAStart(ARGB_LED_COUNT);
    while (WS2812BLEDInUse) { }
    SysTick->CTLR = ctlr;
}

static void argb_set(uint8_t idx, uint8_t r, uint8_t g, uint8_t b)
{
    if (idx >= ARGB_LED_COUNT) return;
    /* WSGRB driver maps bits[23:16]->B, bits[15:8]->G, bits[7:0]->R on wire */
    g_argb[idx] = ((uint32_t)b << 16) | ((uint32_t)g << 8) | r;
}

static void argb_update_sys(void)
{
    /* WSGRB packing: (b<<16)|(g<<8)|r */
    g_argb[4] = g_registered ? 0x00000C00U   /* green */
                             : 0x0000000CU;  /* red   */
}

static void argb_update_motors(void)
{
    uint8_t ch;
    for (ch = 1U; ch <= 4U; ch++) {
        if (g_motor_speed[ch - 1U] > 0U) {
            argb_set(ch - 1U, 0U, 127U, 0U);   /* green 50% = active */
        } else {
            argb_set(ch - 1U, 12U, 12U, 12U);  /* white 5% = idle */
        }
    }
}

/* ------------------------------------------------------------------ */
/* PCF8574 & switches                                                   */
/* ------------------------------------------------------------------ */
void EXTI7_0_IRQHandler(void)
{
    if ((EXTI->INTFR & EXTI_Line4) != 0U) {
        EXTI->INTFR = EXTI_Line4;
        g_pcf_irq = 1U;
    }
}

static void pcf_int_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    NVIC_InitTypeDef nvic = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC | RCC_PB2Periph_AFIO, ENABLE);
    gpio.GPIO_Pin   = PCF8574_INT_PIN;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(PCF8574_INT_PORT, &gpio);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);
    EXTI->INTENR |= EXTI_Line4;
    EXTI->FTENR  |= EXTI_Line4;
    EXTI->INTFR   = EXTI_Line4;
    nvic.NVIC_IRQChannel                   = EXTI7_0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);
}

static bool sw_is_triggered(uint8_t ch_1to4)
{
    uint8_t bit  = (uint8_t)SW_BIT_CH(ch_1to4);
    uint8_t mask = (uint8_t)(1U << bit);
    return (g_pcf_dev.last_state & mask) == 0U; /* active-low */
}

static uint8_t sw_online_mask(void)
{
    uint8_t m = 0U, ch;
    for (ch = 1U; ch <= 4U; ch++) {
        if (sw_is_triggered(ch)) m |= (uint8_t)(1U << (ch - 1U));
    }
    return m;
}

/* ------------------------------------------------------------------ */
/* PWM — TIM2 (CH1 PC0, CH3 PC3, remap4) + TIM1 (CH2 PC5, CH4 PC7, remap7) */
/* ------------------------------------------------------------------ */
static void pwm_init(void)
{
    GPIO_InitTypeDef        gpio = {0};
    TIM_TimeBaseInitTypeDef tb   = {0};
    TIM_OCInitTypeDef       oc   = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC | RCC_PB2Periph_AFIO, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_TIM2, ENABLE);
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_TIM1, ENABLE);

    GPIO_PinRemapConfig(GPIO_PartialRemap4_TIM2, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap7_TIM1, ENABLE);

    /* PC0, PC3, PC5, PC7 = AF_PP */
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_7;
    GPIO_Init(GPIOC, &gpio);

    /* TIM2: CH1 (PC0) and CH3 (PC3) at 10 kHz */
    tb.TIM_Period        = PWM_PERIOD;
    tb.TIM_Prescaler     = 0;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tb);

    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = 0;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &oc);
    TIM_OC3Init(TIM2, &oc);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    /* TIM1: CH2 (PC5) and CH4 (PC7) at 10 kHz */
    TIM_TimeBaseInit(TIM1, &tb);
    TIM_OC2Init(TIM1, &oc);
    TIM_OC4Init(TIM1, &oc);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);  /* MOE bit required for TIM1 */
    TIM_Cmd(TIM1, ENABLE);
}

static void motor_apply_ccr(uint8_t ch_1to4, uint16_t ccr)
{
    if (ch_1to4 < 1U || ch_1to4 > 4U) return;
    if (ccr > PWM_PERIOD) ccr = PWM_PERIOD;
    switch (ch_1to4) {
    case 1: TIM2->CH1CVR = ccr; break;
    case 2: TIM2->CH3CVR = ccr; break;
    case 3: TIM1->CH2CVR = ccr; break;
    case 4: TIM1->CH4CVR = ccr; break;
    }
}

static uint16_t motor_speed_to_ccr(uint8_t speed)
{
    uint32_t ccr;
    if (speed == 0U) return 0U;
    ccr = (uint32_t)speed * (PWM_PERIOD + 1U) / 255U;
    if (ccr < MOTOR_START_DUTY) ccr = MOTOR_START_DUTY;
    if (ccr > PWM_PERIOD) ccr = PWM_PERIOD;
    return (uint16_t)ccr;
}

static void motor_set_speed(uint8_t ch_1to4, uint8_t speed)
{
    uint8_t idx;
    uint16_t target;
    if (ch_1to4 < 1U || ch_1to4 > 4U) return;
    idx = (uint8_t)(ch_1to4 - 1U);
    target = motor_speed_to_ccr(speed);
    g_motor_speed[idx] = speed;
    g_motor_target_ccr[idx] = target;
    if (target == 0U) {
        g_motor_ccr[idx] = 0U;
        motor_apply_ccr(ch_1to4, 0U);
    } else if (g_motor_ccr[idx] == 0U) {
        g_motor_ccr[idx] = MOTOR_START_DUTY;
        motor_apply_ccr(ch_1to4, g_motor_ccr[idx]);
    }
    argb_update_motors();
}

static void motor_ramp_task(void)
{
    uint8_t ch;
    if ((g_tick - g_last_motor_ramp_tick) < MOTOR_RAMP_MS) return;
    g_last_motor_ramp_tick = g_tick;

    for (ch = 1U; ch <= 4U; ch++) {
        uint8_t idx = (uint8_t)(ch - 1U);
        uint16_t cur = g_motor_ccr[idx];
        uint16_t target = g_motor_target_ccr[idx];
        if (cur == target) continue;
        if (cur < target) {
            uint16_t next = (uint16_t)(cur + MOTOR_RAMP_STEP);
            cur = (next > target || next < cur) ? target : next;
        } else {
            cur = (uint16_t)((cur - target) > MOTOR_RAMP_STEP ? (cur - MOTOR_RAMP_STEP) : target);
        }
        g_motor_ccr[idx] = cur;
        motor_apply_ccr(ch, cur);
    }
}

/* auto-stop motors when SW switch triggers (called on PCF IRQ) */
static void motor_sw_check(void)
{
    uint8_t ch;
    if (!pcf8574_refresh(&g_pcf_bus, &g_pcf_dev)) return;
    for (ch = 1U; ch <= 4U; ch++) {
        uint8_t bit  = (uint8_t)SW_BIT_CH(ch);
        uint8_t mask = (uint8_t)(1U << bit);
        if ((g_pcf_dev.changed_mask & mask) != 0U &&
            (g_pcf_dev.last_state & mask) == 0U) {
            /* SW went low (triggered) → stop motor */
            motor_set_speed(ch, 0U);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Bus UART — USART1 RS485                                              */
/* ------------------------------------------------------------------ */
static void bus_uart_init(void)
{
    GPIO_InitTypeDef  gpio = {0};
    USART_InitTypeDef uart = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD | RCC_PB2Periph_AFIO | RCC_PB2Periph_USART1, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap1_USART1, ENABLE);

    gpio.GPIO_Pin   = RS485_DE_PIN;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(RS485_DE_PORT, &gpio);
    GPIO_ResetBits(RS485_DE_PORT, RS485_DE_PIN);

    gpio.GPIO_Pin   = GPIO_Pin_6;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &gpio);

    gpio.GPIO_Pin   = GPIO_Pin_5;
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &gpio);

    USART_StructInit(&uart);
    uart.USART_BaudRate            = BUS_UART_BAUD;
    uart.USART_WordLength          = USART_WordLength_8b;
    uart.USART_StopBits            = USART_StopBits_1;
    uart.USART_Parity              = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &uart);
    USART_Cmd(USART1, ENABLE);
}

static void bus_uart_write(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    GPIO_SetBits(RS485_DE_PORT, RS485_DE_PIN);
    for (volatile uint16_t t = 0; t < 200U; t++) {}
    for (i = 0; i < len; i++) {
        while ((USART1->STATR & USART_FLAG_TXE) == 0) {}
        USART1->DATAR = buf[i];
    }
    while ((USART1->STATR & USART_FLAG_TC) == 0) {}
    GPIO_ResetBits(RS485_DE_PORT, RS485_DE_PIN);
}

static bool bus_uart_read_byte(uint8_t *out)
{
    if ((USART1->STATR & USART_FLAG_RXNE) != 0) {
        *out = (uint8_t)(USART1->DATAR & 0xFFU);
        return true;
    }
    return false;
}

/* ------------------------------------------------------------------ */
/* PN0031 UART — USART2                                                 */
/* NOTE: GPIO_PartialRemap3_USART2 maps TX=PD2, RX=PD3.               */
/* Migration doc labels PD2=RX_2, PD3=TX_2 from PN0031 perspective.   */
/* ------------------------------------------------------------------ */
static void pn_uart_init(void)
{
    GPIO_InitTypeDef  gpio = {0};
    USART_InitTypeDef uart = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD | RCC_PB2Periph_AFIO | RCC_PB2Periph_USART2, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap3_USART2, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_2;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &gpio);

    gpio.GPIO_Pin   = GPIO_Pin_3;
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &gpio);

    USART_StructInit(&uart);
    uart.USART_BaudRate            = PN0031_UART_BAUD;
    uart.USART_WordLength          = USART_WordLength_8b;
    uart.USART_StopBits            = USART_StopBits_1;
    uart.USART_Parity              = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &uart);
    USART_Cmd(USART2, ENABLE);
}

static bool pn_read_byte(uint8_t *out, uint32_t timeout_loops)
{
    while (timeout_loops-- != 0U) {
        if ((USART2->STATR & USART_FLAG_RXNE) != 0) {
            *out = (uint8_t)(USART2->DATAR & 0xFFU);
            return true;
        }
    }
    return false;
}

static void pn_flush_rx(void)
{
    while ((USART2->STATR & USART_FLAG_RXNE) != 0) { (void)USART2->DATAR; }
}

static void pn_write(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++) {
        while ((USART2->STATR & USART_FLAG_TXE) == 0) {}
        USART2->DATAR = buf[i];
    }
    while ((USART2->STATR & USART_FLAG_TC) == 0) {}
}

/* ------------------------------------------------------------------ */
/* PN0031 protocol helpers                                              */
/* ------------------------------------------------------------------ */
static uint8_t xor_checksum(const uint8_t *d, uint16_t len)
{
    uint8_t x = 0; uint16_t i;
    for (i = 0; i < len; i++) x ^= d[i];
    return x;
}

static bool pn_send_cmd(uint8_t fc, const uint8_t *data, uint16_t len)
{
    uint8_t frame[8 + FRAME_MAX_DATA];
    if (len > FRAME_MAX_DATA) return false;
    frame[0] = PN0031_STX; frame[1] = PN0031_ADDR;
    frame[2] = PN0031_FT_CMD; frame[3] = fc;
    frame[4] = (uint8_t)(len & 0xFFU); frame[5] = (uint8_t)((len >> 8) & 0xFFU);
    memcpy(&frame[6], data, len);
    frame[6 + len] = xor_checksum(frame, (uint16_t)(6 + len));
    frame[7 + len] = PN0031_ETX;
    pn_write(frame, (uint16_t)(8 + len));
    return true;
}

static bool pn_read_frame(uint8_t expect_fc, uint8_t *data_out, uint16_t *len_out, uint32_t tmo)
{
    uint8_t b, hdr[5], bcc, etx;
    uint16_t i, data_len;
    uint8_t chk[6 + FRAME_MAX_DATA];

    do { if (!pn_read_byte(&b, tmo)) return false; } while (b != PN0031_STX);
    for (i = 0; i < 5U; i++) { if (!pn_read_byte(&hdr[i], tmo)) return false; }
    data_len = (uint16_t)hdr[3] | ((uint16_t)hdr[4] << 8);
    if (data_len > FRAME_MAX_DATA) return false;
    for (i = 0; i < data_len; i++) { if (!pn_read_byte(&data_out[i], tmo)) return false; }
    if (!pn_read_byte(&bcc, tmo) || !pn_read_byte(&etx, tmo)) return false;
    if (etx != PN0031_ETX) return false;
    chk[0] = PN0031_STX; chk[1] = hdr[0]; chk[2] = hdr[1];
    chk[3] = hdr[2]; chk[4] = hdr[3]; chk[5] = hdr[4];
    memcpy(&chk[6], data_out, data_len);
    if (xor_checksum(chk, (uint16_t)(6 + data_len)) != bcc) return false;
    if (hdr[1] != PN0031_FT_CMD || hdr[2] != expect_fc) return false;
    *len_out = data_len;
    return true;
}

static bool pn_read_card(uint8_t ant, uint8_t *status, uint8_t *uid, uint8_t *uid_len)
{
    uint8_t req[1], resp[FRAME_MAX_DATA];
    uint16_t len = 0;
    req[0] = ant;
    pn_flush_rx();
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

static bool pn_read_block(uint8_t ant, uint8_t block, uint8_t key_type,
                          const uint8_t key6[6], uint8_t out16[16])
{
    uint8_t req[10], resp[FRAME_MAX_DATA];
    uint16_t len = 0;
    req[0] = ant; req[1] = block; req[2] = 0x00U; req[3] = key_type;
    memcpy(&req[4], key6, 6);
    pn_flush_rx();
    if (!pn_send_cmd(PN0031_FC_READ_BLOCK, req, 10)) return false;
    if (!pn_read_frame(PN0031_FC_READ_BLOCK, resp, &len, 400000U)) return false;
    if (len < 1U || resp[0] != 0x00U || len != 19U) return false;
    memcpy(out16, &resp[3], 16U);
    return true;
}

/* ------------------------------------------------------------------ */
/* SHA-256 / HMAC / HKDF for sector key derivation                     */
/* ------------------------------------------------------------------ */
typedef struct { uint32_t state[8]; uint64_t bitlen; uint8_t data[64]; uint32_t datalen; } sha256_ctx_t;
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
static void sha256_transform(sha256_ctx_t *ctx, const uint8_t data[64]) {
    uint32_t m[64], a,b,c,d,e,f,g,h,t1,t2,i;
    for (i=0;i<16U;i++) m[i]=((uint32_t)data[i*4]<<24)|((uint32_t)data[i*4+1]<<16)|((uint32_t)data[i*4+2]<<8)|data[i*4+3];
    for (i=16U;i<64U;i++) { uint32_t s0=(rotr(m[i-15],7)^rotr(m[i-15],18)^(m[i-15]>>3)); uint32_t s1=(rotr(m[i-2],17)^rotr(m[i-2],19)^(m[i-2]>>10)); m[i]=m[i-16]+s0+m[i-7]+s1; }
    a=ctx->state[0];b=ctx->state[1];c=ctx->state[2];d=ctx->state[3];e=ctx->state[4];f=ctx->state[5];g=ctx->state[6];h=ctx->state[7];
    for (i=0;i<64U;i++) { t1=h+(rotr(e,6)^rotr(e,11)^rotr(e,25))+((e&f)^(~e&g))+k_sha[i]+m[i]; t2=(rotr(a,2)^rotr(a,13)^rotr(a,22))+((a&b)^(a&c)^(b&c)); h=g;g=f;f=e;e=d+t1;d=c;c=b;b=a;a=t1+t2; }
    ctx->state[0]+=a;ctx->state[1]+=b;ctx->state[2]+=c;ctx->state[3]+=d;ctx->state[4]+=e;ctx->state[5]+=f;ctx->state[6]+=g;ctx->state[7]+=h;
}
static void sha256_init(sha256_ctx_t *ctx) {
    ctx->datalen=0;ctx->bitlen=0;
    ctx->state[0]=0x6a09e667U;ctx->state[1]=0xbb67ae85U;ctx->state[2]=0x3c6ef372U;ctx->state[3]=0xa54ff53aU;
    ctx->state[4]=0x510e527fU;ctx->state[5]=0x9b05688cU;ctx->state[6]=0x1f83d9abU;ctx->state[7]=0x5be0cd19U;
}
static void sha256_update(sha256_ctx_t *ctx, const uint8_t *data, uint32_t len) {
    uint32_t i;
    for (i=0;i<len;i++) { ctx->data[ctx->datalen++]=data[i]; if (ctx->datalen==64U) { sha256_transform(ctx,ctx->data); ctx->bitlen+=512U; ctx->datalen=0; } }
}
static void sha256_final(sha256_ctx_t *ctx, uint8_t hash[32]) {
    uint32_t i,dl=ctx->datalen;
    ctx->data[dl++]=0x80U; if (dl>56U) { while(dl<64U) ctx->data[dl++]=0; sha256_transform(ctx,ctx->data); dl=0; }
    while(dl<56U) ctx->data[dl++]=0;
    ctx->bitlen+=(uint64_t)ctx->datalen*8ULL;
    for (i=0;i<8U;i++) ctx->data[63U-i]=(uint8_t)(ctx->bitlen>>(8U*i));
    sha256_transform(ctx,ctx->data);
    for (i=0;i<4U;i++) { hash[i]=(uint8_t)(ctx->state[0]>>(24U-i*8U)); hash[i+4]=(uint8_t)(ctx->state[1]>>(24U-i*8U)); hash[i+8]=(uint8_t)(ctx->state[2]>>(24U-i*8U)); hash[i+12]=(uint8_t)(ctx->state[3]>>(24U-i*8U)); hash[i+16]=(uint8_t)(ctx->state[4]>>(24U-i*8U)); hash[i+20]=(uint8_t)(ctx->state[5]>>(24U-i*8U)); hash[i+24]=(uint8_t)(ctx->state[6]>>(24U-i*8U)); hash[i+28]=(uint8_t)(ctx->state[7]>>(24U-i*8U)); }
}
static void hmac_sha256(const uint8_t *key, uint32_t kl, const uint8_t *msg, uint32_t ml, uint8_t out[32]) {
    uint8_t k0[64],kipad[64],kopad[64],tmp[32]; uint32_t i; sha256_ctx_t c;
    memset(k0,0,64);
    if (kl>64U) { sha256_init(&c); sha256_update(&c,key,kl); sha256_final(&c,tmp); memcpy(k0,tmp,32); } else memcpy(k0,key,kl);
    for (i=0;i<64U;i++) { kipad[i]=(uint8_t)(k0[i]^0x36U); kopad[i]=(uint8_t)(k0[i]^0x5CU); }
    sha256_init(&c); sha256_update(&c,kipad,64); sha256_update(&c,msg,ml); sha256_final(&c,tmp);
    sha256_init(&c); sha256_update(&c,kopad,64); sha256_update(&c,tmp,32); sha256_final(&c,out);
}
static void hkdf_expand(const uint8_t prk[32], const uint8_t *info, uint8_t il, uint8_t *okm, uint16_t ol) {
    uint8_t t[32],msg[64],ctr=1U,prev=0U; uint16_t pos=0U;
    memset(t,0,32);
    while (pos<ol) { uint8_t ml=0; if (prev>0) { memcpy(msg,t,prev); ml=prev; } memcpy(msg+ml,info,il); ml=(uint8_t)(ml+il); msg[ml++]=ctr; hmac_sha256(prk,32,msg,ml,t); prev=32;
        uint16_t cp=(uint16_t)((ol-pos)<32U?(ol-pos):32U); memcpy(okm+pos,t,cp); pos=(uint16_t)(pos+cp); ctr++; }
}
static void derive_sector_keys(const uint8_t *uid, uint8_t ul, uint8_t ka[16][6], uint8_t kb[16][6]) {
    static const uint8_t salt[16]={0x9a,0x75,0x9c,0xf2,0xc4,0xf7,0xca,0xff,0x22,0x2c,0xb9,0x76,0x9b,0x41,0xbc,0x96};
    static const uint8_t ia[7]={'R','F','I','D','-','A',0}; static const uint8_t ib[7]={'R','F','I','D','-','B',0};
    uint8_t prk[32],oa[96],ob[96]; uint8_t i;
    hmac_sha256(salt,16,uid,ul,prk); hkdf_expand(prk,ia,7,oa,96); hkdf_expand(prk,ib,7,ob,96);
    for (i=0;i<16U;i++) { memcpy(ka[i],&oa[i*6U],6); memcpy(kb[i],&ob[i*6U],6); }
}

/* ------------------------------------------------------------------ */
/* Filament decode                                                       */
/* ------------------------------------------------------------------ */
static bool poll_filament(uint8_t ant, FilamentDecoded *f)
{
    uint8_t status=0xFFU, uid[UID_MAX_LEN], uid_len=0;
    uint8_t ka[16][6], kb[16][6], raw[64][16];
    static const uint8_t nb[] = {1,2,4,5,6,12};
    uint8_t i;

    memset(f, 0, sizeof(*f)); f->antenna = ant;
    if (!pn_read_card(ant, &status, uid, &uid_len)) return false;
    f->status = status;
    if (status != 0x00U || uid_len == 0U) return true;
    f->uid_len = uid_len; memcpy(f->uid, uid, uid_len);
    derive_sector_keys(uid, uid_len, ka, kb);
    memset(raw, 0, sizeof(raw));
    for (i=0; i<(uint8_t)sizeof(nb); i++) {
        uint8_t blk=nb[i], sec=(uint8_t)(blk/4U);
        if (!pn_read_block(ant,blk,0,ka[sec],raw[blk]))
            if (!pn_read_block(ant,blk,1,kb[sec],raw[blk])) return false;
        delay_ms(3);
    }
    /* parse */
    {
        int16_t nmin=(int16_t)((uint16_t)raw[6][8]|((uint16_t)raw[6][9]<<8));
        int16_t nmax=(int16_t)((uint16_t)raw[6][10]|((uint16_t)raw[6][11]<<8));
        if (nmin>nmax) { int16_t t=nmin; nmin=nmax; nmax=t; }
        f->nozzle_min_c=nmin; f->nozzle_max_c=nmax;
        f->drying_temp_c=(int16_t)((uint16_t)raw[6][0]|((uint16_t)raw[6][1]<<8));
        f->drying_time_h=(uint16_t)raw[6][2]|((uint16_t)raw[6][3]<<8);
        f->color_rgba[0]=raw[5][0]; f->color_rgba[1]=raw[5][1]; f->color_rgba[2]=raw[5][2]; f->color_rgba[3]=raw[5][3];
        /* ascii strings */
        uint8_t k, out, src;
        #define COPY_ASCII(dst, dsz, src_ptr, n) do { out=0; for(k=0;k<(n)&&out+1<(dsz);k++){src=(src_ptr)[k];if(!src)break;if(src>=32&&src<=126)(dst)[out++]=(char)src;} (dst)[out]=0; } while(0)
        COPY_ASCII(f->variant,      9,  &raw[1][0],  8);
        COPY_ASCII(f->material_id,  9,  &raw[1][8],  8);
        COPY_ASCII(f->filament_type,17, &raw[2][0], 16);
        COPY_ASCII(f->detailed_type,17, &raw[4][0], 16);
        COPY_ASCII(f->production_time,17,&raw[12][0],16);
        #undef COPY_ASCII
        f->parsed_ok = true;
    }
    return true;
}

/* ------------------------------------------------------------------ */
/* Bus protocol                                                          */
/* Frame: 55 AA DEST CMD SEQ PLEN_L PLEN_H [PAYLOAD] CRC16_L CRC16_H  */
/* Response same structure with DEV_ADDR instead of DEST, CMD|0x80.    */
/* CRC16-CCITT (init 0xFFFF, poly 0x1021) over [DEST..PAYLOAD].        */
/* Heartbeat CMD_HB is device-initiated, no request frame.             */
/* ------------------------------------------------------------------ */
static uint16_t crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU, i, j;
    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (j = 0; j < 8U; j++)
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
    return crc;
}

static void put_u16le(uint8_t *p, uint16_t v) { p[0]=(uint8_t)(v&0xFF); p[1]=(uint8_t)(v>>8); }
static uint16_t get_u16le(const uint8_t *p) { return (uint16_t)p[0]|((uint16_t)p[1]<<8); }

static void bus_send(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint16_t plen)
{
    uint8_t frame[8 + BUS_RX_MAX_PAYLOAD];
    uint16_t crc;
    frame[0] = SOF0; frame[1] = SOF1;
    frame[2] = DEVICE_ADDR;
    frame[3] = (uint8_t)(cmd | 0x80U);
    frame[4] = seq;
    put_u16le(&frame[5], plen);
    if (plen > 0U) memcpy(&frame[7], payload, plen);
    crc = crc16(&frame[2], (uint16_t)(5U + plen));
    put_u16le(&frame[7 + plen], crc);
    bus_uart_write(frame, (uint16_t)(9U + plen));
}

static void bus_send_hb(void)
{
    /* minimal alive ping: [type, addr] */
    uint8_t pl[2] = { DEVICE_TYPE, DEVICE_ADDR };
    uint8_t frame[9 + 2];
    uint16_t crc;
    frame[0]=SOF0; frame[1]=SOF1;
    frame[2]=DEVICE_ADDR; frame[3]=CMD_HB; frame[4]=0x00U;
    put_u16le(&frame[5], 2U);
    memcpy(&frame[7], pl, 2U);
    crc = crc16(&frame[2], (uint16_t)(5U + 2U));
    put_u16le(&frame[9], crc);
    bus_uart_write(frame, 11U);
}

static void bus_handle_cmd(uint8_t dest, uint8_t cmd, uint8_t seq,
                           const uint8_t *payload, uint16_t plen)
{
    uint8_t out[BUS_RX_MAX_PAYLOAD];
    uint16_t out_len = 0U;
    bool enter_boot = false;
    bool is_broadcast = (dest == BROADCAST_ADDR);

    /* stagger broadcast responses to avoid RS485 collision */
    if (is_broadcast) {
        for (volatile uint32_t d = 0; d < (uint32_t)DEVICE_ADDR * 96000UL; d++) {}
    }

    GPIO_SetBits(LED_PORT, LED_PIN);
    g_led_rx_ticks = 20U;
    g_last_host_tick = g_tick;

    switch (cmd) {
    case CMD_DISCOVER:
    case CMD_PING:
    case CMD_GET_VERSION: {
        g_registered = 1U;
        uint32_t uid0 = UID_WORD0, uid1 = UID_WORD1;
        out[0]=ST_OK; out[1]=DEVICE_TYPE; out[2]=DEVICE_ADDR;
        out[3]=APP_VERSION_MAJOR; out[4]=APP_VERSION_MINOR; out[5]=APP_VERSION_PATCH;
        out[6]=(uint8_t)(uid0); out[7]=(uint8_t)(uid0>>8);
        out[8]=(uint8_t)(uid0>>16); out[9]=(uint8_t)(uid0>>24);
        out[10]=(uint8_t)(uid1); out[11]=(uint8_t)(uid1>>8);
        out[12]=(uint8_t)(uid1>>16); out[13]=(uint8_t)(uid1>>24);
        out_len = 14U;
        break;
    }
    case CMD_ENTER_BOOT:
        if (plen != 0U) { out[0]=ST_BAD_ARG; out_len=1U; break; }
        FLASH_Unlock_Fast();
        FLASH_ErasePage_Fast((0x08000000UL + 62UL*1024UL - 4UL) & 0xFFFFFF00UL); /* IAP_CAL_ADDR page = 0x0800F700 */
        FLASH->CTLR |= (uint32_t)0x00008000UL;
        FLASH->CTLR |= (uint32_t)0x00000080UL;
        out[0]=ST_OK; out_len=1U; enter_boot=true; break;

    case CMD_READ_UID: {
        if (plen != 1U || payload[0] < 1U || payload[0] > 4U) { out[0]=ST_BAD_ARG; out_len=1U; break; }
        uint8_t ant=payload[0], status=0xFF, uid[UID_MAX_LEN], uid_len=0;
        if (!pn_read_card(ant,&status,uid,&uid_len)) { out[0]=ST_IO_ERR; out_len=1U; break; }
        if (status!=0||uid_len==0) { out[0]=ST_NO_TAG; out[1]=status; out_len=2U; break; }
        out[0]=ST_OK; out[1]=ant; out[2]=status; out[3]=uid_len;
        memcpy(&out[4],uid,uid_len); out_len=(uint16_t)(4U+uid_len); break;
    }
    case CMD_READ_FILAMENT: {
        if (plen != 1U || payload[0] < 1U || payload[0] > 4U) { out[0]=ST_BAD_ARG; out_len=1U; break; }
        FilamentDecoded f;
        if (!poll_filament(payload[0],&f)) { out[0]=ST_IO_ERR; out_len=1U; break; }
        if (f.status!=0||f.uid_len==0) { out[0]=ST_NO_TAG; out[1]=f.status; out_len=2U; break; }
        if (!f.parsed_ok) { out[0]=ST_IO_ERR; out_len=1U; break; }
        uint8_t p=0;
        memset(out,0,sizeof(out));
        out[p++]=ST_OK; out[p++]=f.antenna; out[p++]=f.status; out[p++]=f.uid_len;
        memcpy(&out[p],f.uid,f.uid_len); p=(uint8_t)(p+f.uid_len);
        memcpy(&out[p],f.color_rgba,4); p=(uint8_t)(p+4);
        out[p++]=(uint8_t)((uint16_t)f.drying_temp_c); out[p++]=(uint8_t)((uint16_t)f.drying_temp_c>>8);
        out[p++]=(uint8_t)(f.drying_time_h); out[p++]=(uint8_t)(f.drying_time_h>>8);
        out[p++]=(uint8_t)((uint16_t)f.nozzle_min_c); out[p++]=(uint8_t)((uint16_t)f.nozzle_min_c>>8);
        out[p++]=(uint8_t)((uint16_t)f.nozzle_max_c); out[p++]=(uint8_t)((uint16_t)f.nozzle_max_c>>8);
        memcpy(&out[p],f.variant,8); p=(uint8_t)(p+8);
        memcpy(&out[p],f.material_id,8); p=(uint8_t)(p+8);
        memcpy(&out[p],f.filament_type,16); p=(uint8_t)(p+16);
        memcpy(&out[p],f.detailed_type,16); p=(uint8_t)(p+16);
        memcpy(&out[p],f.production_time,16); p=(uint8_t)(p+16);
        out_len=p; break;
    }
    case CMD_GET_SWITCH_STATUS:
        if (plen != 0U) { out[0]=ST_BAD_ARG; out_len=1U; break; }
        if (!pcf8574_refresh(&g_pcf_bus, &g_pcf_dev)) { out[0]=ST_NOT_READY; out_len=1U; break; }
        out[0]=ST_OK; out[1]=g_pcf_dev.last_state; out[2]=sw_online_mask(); out_len=3U; break;

    case CMD_MOTOR_CTRL:
        /* payload: [ch(1), speed(1)] — speed 0=stop, 1-255=PWM */
        if (plen != 2U || payload[0]<1U || payload[0]>4U) { out[0]=ST_BAD_ARG; out_len=1U; break; }
        motor_set_speed(payload[0], payload[1]);
        out[0]=ST_OK; out_len=1U; break;

    case CMD_HB:
        /* master keepalive — timeout already reset at top of this function */
        out[0]=ST_OK; out_len=1U; break;

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

static void bus_housekeeping(void)
{
    if (g_registered && (g_tick - g_last_host_tick) >= HOST_TIMEOUT_MS) {
        g_registered = 0U;
    }
}

static void bus_poll(void)
{
    static uint8_t rx[9 + BUS_RX_MAX_PAYLOAD];
    static uint16_t rx_len = 0U;
    uint8_t b;

    while (bus_uart_read_byte(&b)) {
        if (rx_len >= sizeof(rx)) rx_len = 0U;
        rx[rx_len++] = b;
    }

    while (rx_len >= 2U) {
        uint16_t plen, frame_len, crc_rx, crc_calc;

        if (rx[0] != SOF0 || rx[1] != SOF1) {
            memmove(rx, rx+1, (size_t)(rx_len-1U)); rx_len--; continue;
        }
        if (rx_len < 9U) return;   /* min frame: SOF(2)+DEST+CMD+SEQ+PLEN(2)+CRC(2) */

        plen = get_u16le(&rx[5]);
        if (plen > BUS_RX_MAX_PAYLOAD) {
            memmove(rx, rx+1, (size_t)(rx_len-1U)); rx_len--; continue;
        }
        frame_len = (uint16_t)(9U + plen);
        if (rx_len < frame_len) return;

        crc_rx   = get_u16le(&rx[7 + plen]);
        crc_calc = crc16(&rx[2], (uint16_t)(5U + plen));
        if (crc_calc == crc_rx) {
            uint8_t dest = rx[2], cmd = rx[3], seq = rx[4];
            if (dest == DEVICE_ADDR || dest == BROADCAST_ADDR) {
                bus_handle_cmd(dest, cmd, seq, &rx[7], plen);
            }
        }

        if (rx_len > frame_len)
            memmove(rx, rx+frame_len, (size_t)(rx_len-frame_len));
        rx_len = (uint16_t)(rx_len - frame_len);
    }
}

/* ------------------------------------------------------------------ */
/* main                                                                 */
/* ------------------------------------------------------------------ */
int main(void)
{
    uint8_t i;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();

    SysTick->SR   = 0;
    SysTick->CNT  = 0;
    SysTick->CMP  = (SystemCoreClock / 1000U) - 1U;
    SysTick->CTLR = 0x0FU;
    NVIC_EnableIRQ(SysTicK_IRQn);

    led_init();
    WS2812BDMAInit();

    /* startup blink — busy-loop so WS2812B DMA init can't interfere */
    for (i = 0; i < 6U; i++) {
        led_set((i & 1U) != 0U);
        for (volatile uint32_t d = 0; d < 3600000U; d++) {}
    }
    led_set(true);

    /* init ARGB — motors white idle, system LED reflects registration state */
    for (i = 0; i < ARGB_LED_COUNT - 1U; i++) argb_set(i, 12U, 12U, 12U);
    argb_update_sys();  /* red = unregistered at startup */

    bus_uart_init();
    pn_uart_init();
    pwm_init();

    pcf8574_bus_init(&g_pcf_bus);
    pcf8574_device_init(&g_pcf_dev, PCF8574_ADDR);
    (void)pcf8574_write_port(&g_pcf_bus, &g_pcf_dev, 0xFFU); /* all inputs */
    (void)pcf8574_refresh(&g_pcf_bus, &g_pcf_dev);
    pcf_int_init();

    g_last_hb_tick = g_tick;

    while (1) {
        bus_housekeeping();
        bus_poll();
        motor_ramp_task();

        if (g_pcf_irq) {
            g_pcf_irq = 0U;
            motor_sw_check();
            argb_update_motors();
            argb_flush();
        }

        /* heartbeat every 1 s */
        if ((g_tick - g_last_hb_tick) >= 1000U) {
            g_last_hb_tick = g_tick;
            bus_send_hb();
            led_set((g_tick / 1000U) & 1U);
            argb_update_motors();
            argb_update_sys();
            argb_flush();
        }

        if (g_led_rx_ticks > 0U) {
            g_led_rx_ticks--;
            if (g_led_rx_ticks == 0U) led_set(true);
        }
    }
}
