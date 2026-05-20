#include "pin_config.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/rmt_common.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr const char *TAG = "main";

constexpr i2c_port_t kI2cPort = I2C_NUM_0;
constexpr uint32_t kI2cSpeedHz = 400000;
constexpr uint8_t kSsd1306Addr = 0x3C;
constexpr uint8_t kIna219Addr = 0x40;

constexpr int kOledWidth = 128;
constexpr int kOledHeight = 32;
constexpr int kOledPages = kOledHeight / 8;
uint8_t g_fb[kOledWidth * kOledPages];

constexpr float kShuntOhms = 0.1f;
constexpr float kCurrentLsbA = 0.0001f;
constexpr float kPowerLsbW = kCurrentLsbA * 20.0f;
constexpr uint16_t kIna219Cal = 4096;
constexpr uint16_t kIna219Config = 0x3FFF;
constexpr float kDisplayMaxA = 3.0f;
constexpr adc_channel_t kNtcAdcChannel = ADC_CHANNEL_0;
constexpr float kNtcSupplyMv = 3500.0f;
constexpr float kNtcFixedOhms = 9620.0f;
constexpr float kNtcNominalOhms = 10000.0f;
constexpr float kNtcBeta = 3590.0f;
constexpr float kNtcNominalK = 25.0f + 273.15f;
constexpr bool kNtcToGround = false;

constexpr uint32_t kRgbRmtResolutionHz = 10000000;
constexpr uint16_t kWs0h = 4;
constexpr uint16_t kWs0l = 9;
constexpr uint16_t kWs1h = 8;
constexpr uint16_t kWs1l = 6;
rmt_channel_handle_t g_rgb_channel = nullptr;
rmt_encoder_handle_t g_rgb_encoder = nullptr;
uint8_t g_rgb_data[RGB_NUM * 3];
bool g_rgb_ready = false;
adc_oneshot_unit_handle_t g_adc_handle = nullptr;
adc_cali_handle_t g_adc_cali_handle = nullptr;
bool g_adc_cali_ok = false;
bool g_oled_ok = false;

enum InaReg : uint8_t {
    INA_CONFIG = 0x00,
    INA_SHUNT_VOLTAGE = 0x01,
    INA_BUS_VOLTAGE = 0x02,
    INA_POWER = 0x03,
    INA_CURRENT = 0x04,
    INA_CALIBRATION = 0x05,
};

struct Measurement {
    float bus_v = 0.0f;
    float current_a = 0.0f;
    float power_w = 0.0f;
    float temp_c = 0.0f;
    float ntc_ohms = 0.0f;
    int ntc_raw = 0;
    int ntc_mv = 0;
    bool ina_ok = false;
    bool ntc_ok = false;
};

struct Rgb {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

const uint8_t kFont5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // space
    {0x00, 0x00, 0x5f, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7f, 0x14, 0x7f, 0x14}, // #
    {0x24, 0x2a, 0x7f, 0x2a, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1c, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1c, 0x00}, // )
    {0x14, 0x08, 0x3e, 0x08, 0x14}, // *
    {0x08, 0x08, 0x3e, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3e, 0x51, 0x49, 0x45, 0x3e}, // 0
    {0x00, 0x42, 0x7f, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4b, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7f, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3c, 0x4a, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1e}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3e}, // @
    {0x7e, 0x11, 0x11, 0x11, 0x7e}, // A
    {0x7f, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3e, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7f, 0x41, 0x41, 0x22, 0x1c}, // D
    {0x7f, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7f, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3e, 0x41, 0x49, 0x49, 0x7a}, // G
    {0x7f, 0x08, 0x08, 0x08, 0x7f}, // H
    {0x00, 0x41, 0x7f, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3f, 0x01}, // J
    {0x7f, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7f, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7f, 0x02, 0x0c, 0x02, 0x7f}, // M
    {0x7f, 0x04, 0x08, 0x10, 0x7f}, // N
    {0x3e, 0x41, 0x41, 0x41, 0x3e}, // O
    {0x7f, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3e, 0x41, 0x51, 0x21, 0x5e}, // Q
    {0x7f, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7f, 0x01, 0x01}, // T
    {0x3f, 0x40, 0x40, 0x40, 0x3f}, // U
    {0x1f, 0x20, 0x40, 0x20, 0x1f}, // V
    {0x3f, 0x40, 0x38, 0x40, 0x3f}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
};
const uint8_t kGlyphLowerM[5] = {0x7c, 0x04, 0x78, 0x04, 0x78};

const uint8_t *glyph_for(char &ch) {
    if (ch == 'm') {
        return kGlyphLowerM;
    }
    if (ch < ' ' || ch > 'Z') {
        ch = ' ';
    }
    return kFont5x7[ch - ' '];
}

esp_err_t i2c_write(uint8_t addr, const uint8_t *data, size_t len) {
    return i2c_master_write_to_device(kI2cPort, addr, data, len,
                                      pdMS_TO_TICKS(50));
}

esp_err_t i2c_write_reg16(uint8_t addr, uint8_t reg, uint16_t value) {
    uint8_t data[] = {reg, static_cast<uint8_t>(value >> 8),
                      static_cast<uint8_t>(value & 0xff)};
    return i2c_write(addr, data, sizeof(data));
}

esp_err_t i2c_read_reg16(uint8_t addr, uint8_t reg, uint16_t &value) {
    uint8_t data[2] = {};
    esp_err_t err = i2c_master_write_read_device(kI2cPort, addr, &reg, 1, data,
                                                 sizeof(data), pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        return err;
    }
    value = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    return ESP_OK;
}

void i2c_init() {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = static_cast<gpio_num_t>(SDA_PIN);
    conf.scl_io_num = static_cast<gpio_num_t>(SCL_PIN);
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = kI2cSpeedHz;
    ESP_ERROR_CHECK(i2c_param_config(kI2cPort, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(kI2cPort, conf.mode, 0, 0, 0));
}

void dir_pins_low() {
    constexpr uint64_t pins =
        (1ULL << BUS_DIR_PIN) | (1ULL << RS485_DIR2_PIN);
    gpio_config_t io = {};
    io.pin_bit_mask = pins;
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(static_cast<gpio_num_t>(BUS_DIR_PIN), 0);
    gpio_set_level(static_cast<gpio_num_t>(RS485_DIR2_PIN), 0);
}

void ntc_init() {
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &g_adc_handle));

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(g_adc_handle, kNtcAdcChannel,
                                   &channel_config));

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = kNtcAdcChannel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    esp_err_t err =
        adc_cali_create_scheme_curve_fitting(&cali_config, &g_adc_cali_handle);
    g_adc_cali_ok = err == ESP_OK;
    if (!g_adc_cali_ok) {
        ESP_LOGW(TAG, "ADC calibration unavailable: %s", esp_err_to_name(err));
    }
}

bool ntc_read(float &temp_c, float &ntc_ohms_out, int &raw_out, int &mv_out) {
    if (g_adc_handle == nullptr) {
        return false;
    }

    int raw_sum = 0;
    constexpr int samples = 16;
    for (int i = 0; i < samples; ++i) {
        int raw = 0;
        if (adc_oneshot_read(g_adc_handle, kNtcAdcChannel, &raw) != ESP_OK) {
            return false;
        }
        raw_sum += raw;
    }

    const float raw_avg = static_cast<float>(raw_sum) / samples;
    raw_out = static_cast<int>(raw_avg + 0.5f);
    if (raw_avg < 1.0f || raw_avg > 4094.0f) {
        return false;
    }

    if (g_adc_cali_ok &&
        adc_cali_raw_to_voltage(g_adc_cali_handle, raw_out, &mv_out) ==
            ESP_OK) {
        const float adc_mv = static_cast<float>(mv_out);
        if (adc_mv <= 0.0f || adc_mv >= kNtcSupplyMv) {
            return false;
        }
        ntc_ohms_out = kNtcToGround
                           ? kNtcFixedOhms * adc_mv / (kNtcSupplyMv - adc_mv)
                           : kNtcFixedOhms * (kNtcSupplyMv - adc_mv) / adc_mv;
    } else {
        mv_out = static_cast<int>(raw_avg * kNtcSupplyMv / 4095.0f + 0.5f);
        ntc_ohms_out = kNtcToGround
                           ? kNtcFixedOhms * raw_avg / (4095.0f - raw_avg)
                           : kNtcFixedOhms * (4095.0f - raw_avg) / raw_avg;
    }
    const float inv_k = (1.0f / kNtcNominalK) +
                        (logf(ntc_ohms_out / kNtcNominalOhms) / kNtcBeta);
    temp_c = (1.0f / inv_k) - 273.15f;
    return isfinite(temp_c);
}

uint8_t wave8(uint8_t x) {
    return x < 128 ? x * 2 : (255 - x) * 2;
}

Rgb color_wheel(uint8_t pos) {
    pos = 255 - pos;
    if (pos < 85) {
        return {static_cast<uint8_t>(255 - pos * 3), 0,
                static_cast<uint8_t>(pos * 3)};
    }
    if (pos < 170) {
        pos -= 85;
        return {0, static_cast<uint8_t>(pos * 3),
                static_cast<uint8_t>(255 - pos * 3)};
    }
    pos -= 170;
    return {static_cast<uint8_t>(pos * 3),
            static_cast<uint8_t>(255 - pos * 3), 0};
}

uint8_t scale8(uint8_t value, uint8_t scale) {
    return static_cast<uint8_t>((static_cast<uint16_t>(value) * scale) / 255);
}

void rgb_show(const Rgb *colors) {
    if (!g_rgb_ready) {
        return;
    }

    for (int i = 0; i < RGB_NUM; ++i) {
        g_rgb_data[i * 3 + 0] = colors[i].g;
        g_rgb_data[i * 3 + 1] = colors[i].r;
        g_rgb_data[i * 3 + 2] = colors[i].b;
    }

    rmt_transmit_config_t tx_config = {};
    esp_err_t err = rmt_transmit(g_rgb_channel, g_rgb_encoder, g_rgb_data,
                                 sizeof(g_rgb_data), &tx_config);
    if (err == ESP_OK) {
        err = rmt_tx_wait_all_done(g_rgb_channel, pdMS_TO_TICKS(100));
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ARGB refresh skipped: %s", esp_err_to_name(err));
        rmt_disable(g_rgb_channel);
        rmt_enable(g_rgb_channel);
        rmt_encoder_reset(g_rgb_encoder);
    }
}

void rgb_task(void *arg) {
    (void)arg;
    Rgb colors[RGB_NUM] = {};
    uint8_t phase = 0;

    while (true) {
        for (int i = 0; i < RGB_NUM; ++i) {
            Rgb c = color_wheel(phase + i * 54);
            uint8_t glow = static_cast<uint8_t>(32 + wave8(phase + i * 35) / 5);
            colors[i] = {scale8(c.r, glow), scale8(c.g, glow),
                         scale8(c.b, glow)};
        }
        rgb_show(colors);
        phase += 3;
        vTaskDelay(pdMS_TO_TICKS(35));
    }
}

void rgb_init() {
    rmt_tx_channel_config_t tx_config = {};
    tx_config.gpio_num = static_cast<gpio_num_t>(RGB_PIN);
    tx_config.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_config.resolution_hz = kRgbRmtResolutionHz;
    tx_config.mem_block_symbols = 64;
    tx_config.trans_queue_depth = 2;
    esp_err_t err = rmt_new_tx_channel(&tx_config, &g_rgb_channel);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ARGB disabled: %s", esp_err_to_name(err));
        return;
    }

    rmt_bytes_encoder_config_t encoder_config = {
        .bit0 = {
            .duration0 = kWs0h,
            .level0 = 1,
            .duration1 = kWs0l,
            .level1 = 0,
        },
        .bit1 = {
            .duration0 = kWs1h,
            .level0 = 1,
            .duration1 = kWs1l,
            .level1 = 0,
        },
        .flags = {
            .msb_first = 1,
        },
    };
    err = rmt_new_bytes_encoder(&encoder_config, &g_rgb_encoder);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ARGB encoder disabled: %s", esp_err_to_name(err));
        return;
    }
    err = rmt_enable(g_rgb_channel);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ARGB enable failed: %s", esp_err_to_name(err));
        return;
    }

    g_rgb_ready = true;
    xTaskCreate(rgb_task, "rgb_task", 2048, nullptr, 3, nullptr);
}

esp_err_t oled_cmd(uint8_t cmd) {
    uint8_t data[] = {0x00, cmd};
    return i2c_write(kSsd1306Addr, data, sizeof(data));
}

esp_err_t oled_data(const uint8_t *data, size_t len) {
    uint8_t chunk[17] = {0x40};
    while (len > 0) {
        size_t n = len > 16 ? 16 : len;
        memcpy(&chunk[1], data, n);
        esp_err_t err = i2c_write(kSsd1306Addr, chunk, n + 1);
        if (err != ESP_OK) {
            return err;
        }
        data += n;
        len -= n;
    }
    return ESP_OK;
}

bool oled_init() {
    const uint8_t cmds[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x1F, 0xD3, 0x00, 0x40, 0x8D, 0x14,
        0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x02, 0x81, 0x8F, 0xD9, 0xF1,
        0xDB, 0x40, 0xA4, 0xA6, 0x2E, 0xAF,
    };
    vTaskDelay(pdMS_TO_TICKS(50));
    for (uint8_t cmd : cmds) {
        esp_err_t err = oled_cmd(cmd);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "OLED disabled: %s", esp_err_to_name(err));
            return false;
        }
    }
    return true;
}

void fb_clear() {
    memset(g_fb, 0, sizeof(g_fb));
}

void fb_pixel(int x, int y, bool on) {
    if (x < 0 || x >= kOledWidth || y < 0 || y >= kOledHeight) {
        return;
    }
    uint8_t &b = g_fb[(y / 8) * kOledWidth + x];
    uint8_t mask = 1U << (y & 7);
    if (on) {
        b |= mask;
    } else {
        b &= ~mask;
    }
}

void fb_hline(int x, int y, int w, bool on = true) {
    for (int i = 0; i < w; ++i) {
        fb_pixel(x + i, y, on);
    }
}

void fb_vline(int x, int y, int h, bool on = true) {
    for (int i = 0; i < h; ++i) {
        fb_pixel(x, y + i, on);
    }
}

void fb_circle_outline(int cx, int cy, int r) {
    int x = r;
    int y = 0;
    int err = 0;

    while (x >= y) {
        fb_pixel(cx + x, cy + y, true);
        fb_pixel(cx + y, cy + x, true);
        fb_pixel(cx - y, cy + x, true);
        fb_pixel(cx - x, cy + y, true);
        fb_pixel(cx - x, cy - y, true);
        fb_pixel(cx - y, cy - x, true);
        fb_pixel(cx + y, cy - x, true);
        fb_pixel(cx + x, cy - y, true);

        y++;
        if (err <= 0) {
            err += 2 * y + 1;
        } else {
            x--;
            err -= 2 * x + 1;
        }
    }
}

void fb_circle_fill(int cx, int cy, int r) {
    for (int y = -r; y <= r; ++y) {
        for (int x = -r; x <= r; ++x) {
            if (x * x + y * y <= r * r) {
                fb_pixel(cx + x, cy + y, true);
            }
        }
    }
}

void fb_char(int x, int y, char ch, bool invert = false) {
    const uint8_t *glyph = glyph_for(ch);
    for (int col = 0; col < 5; ++col) {
        for (int row = 0; row < 7; ++row) {
            bool on = glyph[col] & (1U << row);
            fb_pixel(x + col, y + row, invert ? !on : on);
        }
    }
    for (int row = 0; row < 7; ++row) {
        fb_pixel(x + 5, y + row, invert);
    }
}

void fb_text(int x, int y, const char *text, bool invert = false) {
    while (*text && x < kOledWidth - 5) {
        fb_char(x, y, *text++, invert);
        x += 6;
    }
}

int fb_text_width(const char *text, int scale = 1) {
    return static_cast<int>(strlen(text)) * 6 * scale;
}

void fb_char_scaled(int x, int y, char ch, int scale, bool invert = false) {
    const uint8_t *glyph = glyph_for(ch);
    for (int col = 0; col < 5; ++col) {
        for (int row = 0; row < 7; ++row) {
            bool on = glyph[col] & (1U << row);
            for (int xx = 0; xx < scale; ++xx) {
                for (int yy = 0; yy < scale; ++yy) {
                    fb_pixel(x + col * scale + xx, y + row * scale + yy,
                             invert ? !on : on);
                }
            }
        }
    }
}

void fb_text_scaled(int x, int y, const char *text, int scale,
                    bool invert = false) {
    while (*text && x < kOledWidth - 5 * scale) {
        fb_char_scaled(x, y, *text++, scale, invert);
        x += 6 * scale;
    }
}

void fb_text_right(int x_right, int y, const char *text, bool invert = false) {
    fb_text(x_right - fb_text_width(text), y, text, invert);
}

void fb_flush() {
    if (!g_oled_ok) {
        return;
    }

    esp_err_t err = oled_cmd(0x21);
    err |= oled_cmd(0);
    err |= oled_cmd(kOledWidth - 1);
    err |= oled_cmd(0x22);
    err |= oled_cmd(0);
    err |= oled_cmd(kOledPages - 1);
    if (err == ESP_OK) {
        err = oled_data(g_fb, sizeof(g_fb));
    }
    if (err != ESP_OK) {
        g_oled_ok = false;
        ESP_LOGW(TAG, "OLED disabled after write error: %s",
                 esp_err_to_name(err));
    }
}

bool ina219_init() {
    uint16_t id = 0;
    if (i2c_read_reg16(kIna219Addr, INA_CONFIG, id) != ESP_OK) {
        return false;
    }
    ESP_ERROR_CHECK(i2c_write_reg16(kIna219Addr, INA_CONFIG, kIna219Config));
    ESP_ERROR_CHECK(i2c_write_reg16(kIna219Addr, INA_CALIBRATION, kIna219Cal));
    return true;
}

Measurement ina219_read() {
    Measurement m;
    uint16_t bus_raw = 0;
    uint16_t current_raw = 0;
    uint16_t power_raw = 0;

    esp_err_t err = i2c_write_reg16(kIna219Addr, INA_CALIBRATION, kIna219Cal);
    err |= i2c_read_reg16(kIna219Addr, INA_BUS_VOLTAGE, bus_raw);
    err |= i2c_read_reg16(kIna219Addr, INA_CURRENT, current_raw);
    err |= i2c_read_reg16(kIna219Addr, INA_POWER, power_raw);
    if (err != ESP_OK) {
        return m;
    }

    m.bus_v = static_cast<float>((bus_raw >> 3) & 0x1fff) * 0.004f;
    m.current_a = static_cast<int16_t>(current_raw) * kCurrentLsbA;
    m.power_w = power_raw * kPowerLsbW;
    m.ina_ok = true;
    m.ntc_ok = ntc_read(m.temp_c, m.ntc_ohms, m.ntc_raw, m.ntc_mv);
    return m;
}

void draw_dashboard(const Measurement &m) {
    char line[24];
    fb_clear();

    snprintf(line, sizeof(line), "%5.2FV", m.bus_v);
    fb_text(0, 0, line);
    fb_vline(44, 0, 8);
    if (m.ntc_ok) {
        snprintf(line, sizeof(line), "%2.0FC", m.temp_c);
    } else {
        snprintf(line, sizeof(line), "--C");
    }
    fb_text(50, 0, line);
    if (m.ina_ok) {
        const bool beat =
            ((xTaskGetTickCount() * portTICK_PERIOD_MS) / 500) % 2 == 0;
        fb_circle_fill(123, 3, beat ? 3 : 2);
    } else {
        fb_circle_outline(123, 3, 3);
    }
    fb_hline(0, 8, 128);

    const float abs_current = fabsf(m.current_a);
    const char *current_unit = "A";
    if (abs_current < 0.05f) {
        snprintf(line, sizeof(line), "%4.1F", m.current_a * 1000.0f);
        current_unit = "mA";
    } else {
        snprintf(line, sizeof(line), "%5.2F", m.current_a);
    }
    fb_text_scaled(0, 13, line, 2);
    fb_text(fb_text_width(line, 2) + 2, 20, current_unit);

    fb_vline(86, 9, 18);
    fb_text(92, 10, "PWR");
    snprintf(line, sizeof(line), "%4.0FW", m.power_w);
    fb_text_right(127, 20, line);

    int bar = static_cast<int>((abs_current / kDisplayMaxA) * 126.0f);
    if (bar > 126) {
        bar = 126;
    }
    fb_hline(0, 30, 128);
    fb_hline(0, 31, 128);
    fb_hline(1, 30, bar);

    fb_flush();
}
} // namespace

extern "C" void app_main() {
    dir_pins_low();
    ntc_init();
    rgb_init();
    i2c_init();
    g_oled_ok = oled_init();
    bool ina_ok = ina219_init();

    ESP_LOGI(TAG, "INA219 + SSD1306 monitor");
    ESP_LOGI(TAG, "DIR low: GPIO%d GPIO%d, ARGB GPIO%d x%d", BUS_DIR_PIN,
             RS485_DIR2_PIN, RGB_PIN, RGB_NUM);
    ESP_LOGI(TAG, "SDA=%d SCL=%d shunt=%.4f ohm cal=%u", SDA_PIN, SCL_PIN,
             kShuntOhms, kIna219Cal);
    ESP_LOGI(TAG,
             "NTC GPIO%d ADC1_CH%d beta=%.0f fixed=%.0fohm cal=%d",
             NTC1_PIN, static_cast<int>(kNtcAdcChannel), kNtcBeta,
             kNtcFixedOhms, g_adc_cali_ok);
    ESP_LOGI(TAG, "OLED %s", g_oled_ok ? "ready" : "not installed");

    while (true) {
        Measurement m = ina219_read();
        if (!ina_ok) {
            ina_ok = ina219_init();
        }
        m.ina_ok = m.ina_ok && ina_ok;
        draw_dashboard(m);
        ESP_LOGI(TAG,
                 "INA: ok:%d V:%.3f I:%.3fA P:%.2fW T:%.1fC ntc:%d raw:%d mv:%d R:%.0fohm",
                 m.ina_ok, m.bus_v, m.current_a, m.power_w, m.temp_c,
                 m.ntc_ok, m.ntc_raw, m.ntc_mv, m.ntc_ohms);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
