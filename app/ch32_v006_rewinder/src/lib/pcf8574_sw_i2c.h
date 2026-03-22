#ifndef PCF8574_SW_I2C_H
#define PCF8574_SW_I2C_H

#include <stdbool.h>
#include <stdint.h>
#include <ch32v00X.h>

typedef struct {
    GPIO_TypeDef *port;
    uint16_t sda_pin;
    uint16_t scl_pin;
    uint16_t delay_cycles;
} Pcf8574Bus;

typedef struct {
    uint8_t addr7;
    uint8_t last_state;
    uint8_t changed_mask;
    bool online;
} Pcf8574Device;

void pcf8574_bus_init(Pcf8574Bus *bus);
void pcf8574_device_init(Pcf8574Device *dev, uint8_t addr7);

bool pcf8574_write_port(Pcf8574Bus *bus, Pcf8574Device *dev, uint8_t value);
bool pcf8574_read_port(Pcf8574Bus *bus, Pcf8574Device *dev, uint8_t *value);
bool pcf8574_refresh(Pcf8574Bus *bus, Pcf8574Device *dev);

#endif

