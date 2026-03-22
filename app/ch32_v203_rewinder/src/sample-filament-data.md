```c
struct _filament {
    char bambubus_filament_id[8] = "GFG00";
    uint8_t color_R = 0xFF;
    uint8_t color_G = 0xFF;
    uint8_t color_B = 0xFF;
    uint8_t color_A = 0xFF;
    int16_t temperature_min = 220;
    int16_t temperature_max = 240;
    char name[20] = "PETG";
    uint64_t xhub_unique_id = 0;

    uint8_t dryer_power = 0;
    int8_t dryer_temperature = 0;
    uint16_t dryer_time_left = 0;

    bool online = true;
    _filament_motion motion = _filament_motion::idle;
    uint8_t seal_status = 0;
    int8_t compartment_temperature = 0;
    uint8_t compartment_humidity = 0;
    float meters = 1;
    float meters_virtual_count = 0;

    void init() {
        xhub_unique_id = 0;
        bambubus_filament_id[0] = 'G';
        bambubus_filament_id[1] = 'F';
        bambubus_filament_id[2] = 'G';
        bambubus_filament_id[3] = '0';
        bambubus_filament_id[4] = '0';
        bambubus_filament_id[5] = '\0';
        color_R = 0xFF;
        color_G = 0xFF;
        color_B = 0xFF;
        color_A = 0xFF;
        temperature_min = 220;
        temperature_max = 240;
        name[0] = 'P';
        name[1] = 'E';
        name[2] = 'T';
        name[3] = 'G';
        name[4] = '\0';
        meters = 1;
        meters_virtual_count = 0;
        online = true;
        motion = _filament_motion::idle;
        compartment_temperature = 0;
        compartment_humidity = 0;
    }
} __attribute__((packed));
```