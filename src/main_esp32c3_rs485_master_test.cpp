#include <Arduino.h>

static const uint32_t BUS_BAUD = 115200;

// Adjust to your ESP32-C3 wiring.
static const int BUS_RX_PIN = 7;
static const int BUS_TX_PIN = 6;
static const int BUS_DIR_PIN = 5;
static const bool BUS_DIR_TX_ENABLE_HIGH = true;

static const int LED_PIN = LED_BUILTIN;

static const uint8_t SOF0 = 0x55;
static const uint8_t SOF1 = 0xAA;
static const size_t MAX_PAYLOAD = 96;
static const size_t MAX_FRAME = 8 + MAX_PAYLOAD;

HardwareSerial BusSerial(1);

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFFU;
  for (uint16_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t j = 0; j < 8; ++j) {
      if ((crc & 0x8000U) != 0U) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

static uint16_t get_u16le(const uint8_t *p) {
  return static_cast<uint16_t>(p[0]) |
         (static_cast<uint16_t>(p[1]) << 8);
}

static void bus_set_rx_only() {
  if (BUS_DIR_PIN < 0) {
    return;
  }
  digitalWrite(BUS_DIR_PIN, BUS_DIR_TX_ENABLE_HIGH ? LOW : HIGH);
}

static void dump_hex(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    if (data[i] < 0x10U) {
      Serial.print('0');
    }
    Serial.print(data[i], HEX);
  }
}

static void poll_raw_bytes() {
  while (BusSerial.available() > 0) {
    int v = BusSerial.read();
    if (v < 0) {
      break;
    }
    Serial.print("rx 0x");
    if (v < 0x10) {
      Serial.print('0');
    }
    Serial.println(v, HEX);
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

static void poll_frames() {
  static uint8_t buf[MAX_FRAME];
  static size_t len = 0;

  while (BusSerial.available() > 0) {
    int v = BusSerial.read();
    if (v < 0) {
      break;
    }
    if (len >= sizeof(buf)) {
      len = 0;
    }
    buf[len++] = static_cast<uint8_t>(v);
  }

  while (len >= 2U) {
    if (buf[0] != SOF0 || buf[1] != SOF1) {
      memmove(&buf[0], &buf[1], len - 1U);
      len--;
      continue;
    }

    if (len < 6U) {
      return;
    }

    uint16_t payloadLen = get_u16le(&buf[4]);
    if (payloadLen > MAX_PAYLOAD) {
      Serial.println("frame bad len");
      memmove(&buf[0], &buf[1], len - 1U);
      len--;
      continue;
    }

    uint16_t frameLen = static_cast<uint16_t>(8U + payloadLen);
    if (len < frameLen) {
      return;
    }

    uint16_t crcRx = get_u16le(&buf[6U + payloadLen]);
    uint16_t crcCalc = crc16_ccitt(&buf[2], static_cast<uint16_t>(4U + payloadLen));

    Serial.print("frame cmd=0x");
    if (buf[2] < 0x10U) {
      Serial.print('0');
    }
    Serial.print(buf[2], HEX);
    Serial.print(" seq=");
    Serial.print(buf[3]);
    Serial.print(" len=");
    Serial.print(payloadLen);
    Serial.print(" crc=");
    Serial.print(crcRx == crcCalc ? "ok" : "bad");
    Serial.print(" payload=");
    dump_hex(&buf[6], payloadLen);
    Serial.println();

    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    if (len > frameLen) {
      memmove(&buf[0], &buf[frameLen], len - frameLen);
    }
    len -= frameLen;
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  delay(300);

  if (BUS_DIR_PIN >= 0) {
    pinMode(BUS_DIR_PIN, OUTPUT);
    bus_set_rx_only();
  }

  BusSerial.begin(BUS_BAUD, SERIAL_8N1, BUS_RX_PIN, BUS_TX_PIN);

  Serial.println("esp32c3 rs485 readonly monitor");
  Serial.println("mode=frame parser + raw fallback disabled tx");
}

void loop() {
  poll_frames();
}
