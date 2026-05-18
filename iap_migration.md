# iap migration
- now after user enter bootloader 
- it can update by send command via bus
- code in app/ch32_v006_bootloader env rs485_iap

# issue 
```text
if i daisy chain device on bus i dont know where device i looking for
i want to add devive type (heater, rewinder, etc just place holder start at 0x00) and device address i dont known is possible to do it or not let sugest me if not possible just use only device type is enought 
```
- some command if daisy chain when bootloader mode i need some master can scan devices
- if master scan devices some devices in app mode need silent 
- !!! important bootloader iap is limited by 3328-byte pls compact the code base you can remove or cut some not use full command or code

# pin change 
- remove another led used and use PD0 as status you can manage led to show user what you want

# tools
- update current tools match current code /tools/ch32v006_rs485_iap.py