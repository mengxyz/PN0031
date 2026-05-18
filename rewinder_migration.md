# pins
- PD7 RTS (** do not use it hardware reset pin)
- PA1 XTAL
- PA2 XTAL
- PD0 LED STAT 1
- PD1 LED STAT 2 (not used)
- PC0 MC_PWM_2
- PC1 SDA
- PC2 SCL
- PC3 MC_PWM_1
- PC4 PCF8574 INTERUPT
- PC5 MC_PWM_3
- PC6 ARGB_OUT 5 LEDS
- PC7 MC_PWM_4
- PD2 RX_2
- PD3 TX_2
- PD4 DIR
- PD5 RX
- PD6 TX

# pwm channel map and used
- PC0 -> TIM2_CH1_4
  used as PWM
- PC3 -> TIM2_CH3_4
  used as PWM
- PC5 -> TIM1_CH2_7
  used as PWM
- PC7 -> TIM1_CH4_7
  used as PWM

# PCF8574
- ADDR ALL -> GND
- P0 SWI3
- P1 SW3
- P2 SWI4
- P3 SW4
- P4 SWI2
- P5 SW2
- P6 SWI1
- P7 SW1

# Bus
- use TX, RX and DIR
- follow current code and change mc to one way just on off or on white speed
- new need device type, address it will have another devices on bus ex. heater
- need diccovery exchange device data like version, sn to master handle online offline state like hearbeat after discovery
- it multi type and multi address in daysy chanin it plan maybe in prod just sigle type of device just implement for futue device type it like bootloader too

# PN0031 
- use TX2, RX2

# MC
- now mc use one way drive by using mosfet
- pls select best pwm for SI2318A mosfet
- status white as idle and when mc active use green
- maybe new command forward with speed 

# Led statsus 
- follow current code just move to new pin

# ARGB
- use 0-3 for MC status
- use 4 for mcu status or bus stat pls select it upto you
- pls use white color as 5% brightness
- and another solid color at 50%

# fw
- app use user area limit 64K

# code
- dir app/ch32_v006_rewinder
- env platformio.ini

# tools
- update tools to match current protocal tools/ch32v006_rs485_app.py