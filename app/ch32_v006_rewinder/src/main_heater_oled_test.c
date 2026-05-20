/* Heater OLED SSD1306 128x32 test.
 *
 * PC1 = SDA, PC2 = SCL, OLED address 0x3C.
 * PC4 blinks each time a new test page is drawn.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ch32v00X.h>

#define I2C_PORT    GPIOC
#define I2C_SDA_PIN GPIO_Pin_1
#define I2C_SCL_PIN GPIO_Pin_2
#define LED_PORT    GPIOC
#define LED_PIN     GPIO_Pin_4
#define OLED_ADDR   0x3CU
#define OLED_W      128U
#define OLED_PAGES  4U

static uint8_t fb[OLED_W * OLED_PAGES];

static void delay_cycles(volatile uint32_t cycles) { while (cycles-- != 0U) {} }
static void delay_ms(uint32_t ms) { while (ms-- != 0U) delay_cycles(12000U); }

static void led_set(bool on)
{
    if (on) GPIO_SetBits(LED_PORT, LED_PIN);
    else GPIO_ResetBits(LED_PORT, LED_PIN);
}

static void gpio_init_all(void)
{
    GPIO_InitTypeDef g = {0};
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC, ENABLE);

    g.GPIO_Pin = LED_PIN;
    g.GPIO_Speed = GPIO_Speed_30MHz;
    g.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LED_PORT, &g);
    led_set(false);

    g.GPIO_Pin = I2C_SDA_PIN | I2C_SCL_PIN;
    g.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(I2C_PORT, &g);
    GPIO_SetBits(I2C_PORT, I2C_SDA_PIN | I2C_SCL_PIN);
}

static void i2c_delay(void) { for (volatile uint16_t d = 0U; d < 80U; d++) {} }
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
        i2c_sda((b & 0x80U) != 0U);
        i2c_delay();
        i2c_scl(true);
        i2c_delay();
        i2c_scl(false);
        b <<= 1;
    }
    i2c_sda(true);
    i2c_delay();
    i2c_scl(true);
    i2c_delay();
    ack = (uint8_t)!i2c_sda_read();
    i2c_scl(false);
    i2c_delay();
    return ack != 0U;
}

static bool oled_cmd(uint8_t cmd)
{
    i2c_start();
    if (!i2c_write_byte(OLED_ADDR << 1) || !i2c_write_byte(0x00U) || !i2c_write_byte(cmd)) {
        i2c_stop();
        return false;
    }
    i2c_stop();
    return true;
}

static bool oled_data(const uint8_t *data, uint8_t len)
{
    uint8_t i;
    i2c_start();
    if (!i2c_write_byte(OLED_ADDR << 1) || !i2c_write_byte(0x40U)) {
        i2c_stop();
        return false;
    }
    for (i = 0U; i < len; i++) {
        if (!i2c_write_byte(data[i])) {
            i2c_stop();
            return false;
        }
    }
    i2c_stop();
    return true;
}

static bool oled_flush(void)
{
    uint8_t page, off;
    for (page = 0U; page < OLED_PAGES; page++) {
        if (!oled_cmd((uint8_t)(0xB0U + page)) || !oled_cmd(0x00U) || !oled_cmd(0x10U)) return false;
        for (off = 0U; off < OLED_W; off = (uint8_t)(off + 16U)) {
            if (!oled_data(&fb[(page * OLED_W) + off], 16U)) return false;
        }
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
        0x81,0xCF,  /* contrast */
        0xD9,0xF1,  /* precharge */
        0xDB,0x40,  /* vcom */
        0xA4,       /* resume RAM */
        0xA6,       /* normal */
        0x2E,       /* scroll off */
        0xAF        /* display on */
    };
    uint8_t i;
    for (i = 0U; i < sizeof(init); i++) {
        if (!oled_cmd(init[i])) return false;
    }
    memset(fb, 0, sizeof(fb));
    return oled_flush();
}

static void pixel(uint8_t x, uint8_t y, bool on)
{
    uint16_t idx;
    if (x >= OLED_W || y >= 32U) return;
    idx = (uint16_t)((y >> 3U) * OLED_W + x);
    if (on) fb[idx] |= (uint8_t)(1U << (y & 7U));
    else fb[idx] &= (uint8_t)~(1U << (y & 7U));
}

static void rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool on)
{
    uint8_t xx, yy;
    for (yy = y; yy < (uint8_t)(y + h); yy++) {
        for (xx = x; xx < (uint8_t)(x + w); xx++) {
            pixel(xx, yy, on);
        }
    }
}

static void draw_digit_big(uint8_t x, uint8_t y, uint8_t d)
{
    static const uint8_t segs[10] = {
        0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F
    };
    uint8_t s = segs[d % 10U];
    if (s & 0x01U) rect(x + 2U, y,      12U, 3U, true);
    if (s & 0x02U) rect(x + 14U, y+2U,  3U, 10U, true);
    if (s & 0x04U) rect(x + 14U, y+14U, 3U, 10U, true);
    if (s & 0x08U) rect(x + 2U, y+24U, 12U, 3U, true);
    if (s & 0x10U) rect(x,      y+14U,  3U, 10U, true);
    if (s & 0x20U) rect(x,      y+2U,   3U, 10U, true);
    if (s & 0x40U) rect(x + 2U, y+12U, 12U, 3U, true);
}

static void pattern(uint8_t n)
{
    uint16_t i;
    memset(fb, 0, sizeof(fb));
    if (n == 0U) {
        memset(fb, 0xFF, sizeof(fb));
    } else if (n == 1U) {
        for (i = 0U; i < sizeof(fb); i++) fb[i] = (i & 1U) ? 0xAAU : 0x55U;
    } else if (n == 2U) {
        for (i = 0U; i < sizeof(fb); i++) fb[i] = ((i % OLED_W) & 8U) ? 0xFFU : 0x00U;
    } else if (n == 3U) {
        for (i = 0U; i < OLED_W; i++) {
            fb[i] = 0xFFU;
            fb[OLED_W + i] = 0x00U;
            fb[(2U * OLED_W) + i] = 0xFFU;
            fb[(3U * OLED_W) + i] = 0x00U;
        }
    } else {
        draw_digit_big(6U, 2U, 1U);
        draw_digit_big(30U, 2U, 2U);
        draw_digit_big(54U, 2U, 3U);
        draw_digit_big(78U, 2U, 4U);
    }
}

int main(void)
{
    uint8_t n = 0U;
    SystemCoreClockUpdate();
    gpio_init_all();

    while (!oled_init()) {
        led_set(true);
        delay_ms(80U);
        led_set(false);
        delay_ms(420U);
    }

    while (1) {
        pattern(n);
        oled_flush();
        led_set(true);
        delay_ms(80U);
        led_set(false);
        delay_ms(1400U);
        n = (uint8_t)((n + 1U) % 5U);
    }
}
