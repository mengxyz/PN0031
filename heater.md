# pins
- PD7 RTS (** do not use it hardware reset pin)
- PA1 FAN TACH INPUT, pull-up, falling-edge pulse count
- PA2 POWER STATUS  DIGITAL INPUT
- PD0 NOT USE
- PD1 SWIO (not used)
- PC0 ARGB OUT 1 LEDS
- PC1 SDA
- PC2 SCL
- PC3 DOOR SENSOR DIGITAL INPUT
- PC4 LED 1 / bootloader status LED
- PC5 LED 2 / heartbeat status LED like rewinder
- PC6 HEATER CTL ON/OFF PID LOGIC
- PC7 FAN CTL ON/OFF LOGIC SIMPLE, PWM MOSFET NOT TEST PREFER PWM
- PD2 PCB TEMP NTC 10K 3590
- PD3 HEATER TEMP NTC 10K 3590
- PD4 RS485 DE/DIR, active high
- PD5 TX BUS, USART1 default mapping
- PD6 RX BUS, USART1 default mapping

# NTC WIRE
```text
GND --- 10K NTC --|-- 10K --- VCC
              |
             ADC
```

# BUS 
- BUS follow rewinder but different devicve kind
- Tested heater PCB UART mapping is USART1 default, not remap1:
  - PD5 = TX
  - PD6 = RX
  - PD4 = RS485 DE/DIR active high
  - baud = 460800
- Device type is `0x00` heater.
- Default device address is `0x01`.

# I2C
- SHT 40/ AHT 20

# Control
- poll data
- data report pcb temp, heater temp, air temp/humi, door status, dryer status, fan status, power status, error status
- about dryer status, is drying, dry temp, time left
- if lost connection stop dry
- start dry temp, time min 40, nax 60C

# ARGB 
- use same bus status like rewinder

# LED 
- PC4 stay when enter bootloader
- PC5 use as HB like rewinder
- Device must register first by discover/ping/version/start before PC5 heartbeat blinking.
- Addressed heartbeat alone must not register device; if device is not registered ARGB stays red and PC5 stays off.
- When registered/online ARGB is green and PC5 toggles like rewinder.
- When master timeout/lost connection ARGB returns red and PC5 turns off.

# bootloader
- boot loader use same code base but new device kine and led use PC4
- pls split new env

# firmeware
- split new env

# about heater
- heater is 2 kind AC Triacs and and DC MOSFET 
- need soft preheat to prevent inrush current

# about fan
- pls design when use it

# about heater temp
- pls if reach 110C plsy stop dry if still in pid after reduce pid and turn on fan if temp still not reduce pls stop dry and remark error bit on poll data

# about error 
- just like simple fan not run, heater over heat, door open when dry, pcb over heat
- just report when poll not all error stop dry some just warn pls design

# dryer 
- fully dry logic pid, fan is control by device not master
- master just start and stop it

# Display
- OLED SSD1306 128x32
- I2C address `0x3C`
- Shares I2C bus with SHT40/AHT20:
  - PC1 = SDA
  - PC2 = SCL
- pls design output
- on AMS-HT display. TEMP | RH | TIME 
- Current firmware output:
  - row 1: heater TEMP
  - row 2: RH
  - row 3: TIME left in minutes
  - row 4: READY / DRY / COOL / ERROR / OFFLINE
```text
          +--------+--------+-----------------------+
          | TEMP   |  HUMI  |         TIME          |
          +--------+--------+-----------------------+
          | 10c       10%        00:00:00           |
          +-----------------------------------------+
          | EXTRA STATUS                            |
          +-----------------------------------------+
```
