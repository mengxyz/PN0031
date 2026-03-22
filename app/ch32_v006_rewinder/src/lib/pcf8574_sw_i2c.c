#include "pcf8574_sw_i2c.h"

static uint32_t pcf8574_gpio_clock(Pcf8574Bus *bus)
{
    if (bus->port == GPIOA) return RCC_PB2Periph_GPIOA;
    if (bus->port == GPIOB) return RCC_PB2Periph_GPIOB;
    if (bus->port == GPIOC) return RCC_PB2Periph_GPIOC;
    if (bus->port == GPIOD) return RCC_PB2Periph_GPIOD;
    return 0U;
}

static void i2c_delay(Pcf8574Bus *bus)
{
    volatile uint16_t t;
    for (t = 0; t < bus->delay_cycles; t++) { }
}

static void sda_high(Pcf8574Bus *bus) { GPIO_SetBits(bus->port, bus->sda_pin); }
static void sda_low(Pcf8574Bus *bus)  { GPIO_ResetBits(bus->port, bus->sda_pin); }
static void scl_high(Pcf8574Bus *bus) { GPIO_SetBits(bus->port, bus->scl_pin); }
static void scl_low(Pcf8574Bus *bus)  { GPIO_ResetBits(bus->port, bus->scl_pin); }
static uint8_t sda_read(Pcf8574Bus *bus)
{
    return (GPIO_ReadInputDataBit(bus->port, bus->sda_pin) == Bit_SET) ? 1U : 0U;
}

static void i2c_start(Pcf8574Bus *bus)
{
    sda_high(bus);
    scl_high(bus);
    i2c_delay(bus);
    sda_low(bus);
    i2c_delay(bus);
    scl_low(bus);
}

static void i2c_stop(Pcf8574Bus *bus)
{
    sda_low(bus);
    i2c_delay(bus);
    scl_high(bus);
    i2c_delay(bus);
    sda_high(bus);
    i2c_delay(bus);
}

static bool i2c_write_byte(Pcf8574Bus *bus, uint8_t b)
{
    uint8_t i;
    for (i = 0; i < 8U; i++) {
        if ((b & 0x80U) != 0U) sda_high(bus); else sda_low(bus);
        i2c_delay(bus);
        scl_high(bus);
        i2c_delay(bus);
        scl_low(bus);
        b <<= 1;
    }

    sda_high(bus);
    i2c_delay(bus);
    scl_high(bus);
    i2c_delay(bus);
    {
        bool ack = (sda_read(bus) == 0U);
        scl_low(bus);
        return ack;
    }
}

static uint8_t i2c_read_byte(Pcf8574Bus *bus, bool ack)
{
    uint8_t i;
    uint8_t b = 0U;

    sda_high(bus);
    for (i = 0; i < 8U; i++) {
        b <<= 1;
        scl_high(bus);
        i2c_delay(bus);
        if (sda_read(bus) != 0U) b |= 1U;
        scl_low(bus);
        i2c_delay(bus);
    }

    if (ack) sda_low(bus); else sda_high(bus);
    i2c_delay(bus);
    scl_high(bus);
    i2c_delay(bus);
    scl_low(bus);
    sda_high(bus);
    return b;
}

void pcf8574_bus_init(Pcf8574Bus *bus)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t clk = pcf8574_gpio_clock(bus);

    if (bus->delay_cycles == 0U) {
        bus->delay_cycles = 20U;
    }

    if (clk != 0U) {
        RCC_PB2PeriphClockCmd(clk, ENABLE);
    }

    gpio.GPIO_Pin = (uint16_t)(bus->sda_pin | bus->scl_pin);
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(bus->port, &gpio);

    sda_high(bus);
    scl_high(bus);
}

void pcf8574_device_init(Pcf8574Device *dev, uint8_t addr7)
{
    dev->addr7 = (uint8_t)(addr7 & 0x7FU);
    dev->last_state = 0xFFU;
    dev->changed_mask = 0x00U;
    dev->online = false;
}

bool pcf8574_write_port(Pcf8574Bus *bus, Pcf8574Device *dev, uint8_t value)
{
    uint8_t addr_w = (uint8_t)(dev->addr7 << 1);
    i2c_start(bus);
    if (!i2c_write_byte(bus, addr_w)) {
        i2c_stop(bus);
        dev->online = false;
        return false;
    }
    if (!i2c_write_byte(bus, value)) {
        i2c_stop(bus);
        dev->online = false;
        return false;
    }
    i2c_stop(bus);
    dev->online = true;
    dev->changed_mask = (uint8_t)(dev->last_state ^ value);
    dev->last_state = value;
    return true;
}

bool pcf8574_read_port(Pcf8574Bus *bus, Pcf8574Device *dev, uint8_t *value)
{
    uint8_t addr_r = (uint8_t)((dev->addr7 << 1) | 1U);
    i2c_start(bus);
    if (!i2c_write_byte(bus, addr_r)) {
        i2c_stop(bus);
        dev->online = false;
        return false;
    }
    *value = i2c_read_byte(bus, false);
    i2c_stop(bus);
    dev->online = true;
    return true;
}

bool pcf8574_refresh(Pcf8574Bus *bus, Pcf8574Device *dev)
{
    uint8_t v;
    if (!pcf8574_read_port(bus, dev, &v)) {
        return false;
    }
    dev->changed_mask = (uint8_t)(dev->last_state ^ v);
    dev->last_state = v;
    return true;
}

