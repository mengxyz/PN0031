#include <Arduino.h>
#include <PN0031.h>

static const uint32_t BUS_BAUD = 115200;
static const uint32_t PN_BAUD = 115200;

// Adjust these pins to your board wiring.
static const int BUS_RX_PIN = 6;
static const int BUS_TX_PIN = 7;
static const int BUS_DIR_PIN = 5;
static const bool BUS_DIR_TX_ENABLE_HIGH = true;

static const int PN_RX_PIN = 20;
static const int PN_TX_PIN = 21;
static const int PN_DIR_PIN = -1;
static const bool PN_DIR_TX_ENABLE_HIGH = true;

static const int LED_PIN = LED_BUILTIN;

static const uint8_t SOF0 = 0x55;
static const uint8_t SOF1 = 0xAA;

static const uint8_t CMD_PING = 0x01;
static const uint8_t CMD_GET_VERSION = 0x30;
static const uint8_t CMD_READ_UID = 0x32;
static const uint8_t CMD_READ_ALL_UIDS = 0x33;

static const uint8_t ST_OK = 0x00;
static const uint8_t ST_BAD_CMD = 0x01;
static const uint8_t ST_BAD_ARG = 0x02;
static const uint8_t ST_IO_ERR = 0x04;
static const uint8_t ST_NO_TAG = 0x05;

static const uint8_t APP_VERSION_MAJOR = 1;
static const uint8_t APP_VERSION_MINOR = 0;
static const uint8_t APP_VERSION_PATCH = 0;

static const size_t BUS_MAX_PAYLOAD = 96;
static const size_t BUS_FRAME_MAX = 8 + BUS_MAX_PAYLOAD;

HardwareSerial BusSerial(0);
HardwareSerial PnSerial(1);
PN0031 reader;

struct BusParser {
  uint8_t buf[BUS_FRAME_MAX];
  size_t len;
};

static BusParser g_bus = {};

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

static void put_u16le(uint8_t *p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFFU);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFFU);
}

static void set_bus_tx(bool enable) {
  if (BUS_DIR_PIN < 0) {
    return;
  }
  digitalWrite(BUS_DIR_PIN, enable == BUS_DIR_TX_ENABLE_HIGH ? HIGH : LOW);
}

static void bus_write_frame(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint16_t len) {
  uint8_t frame[BUS_FRAME_MAX];
  uint16_t crc;

  frame[0] = SOF0;
  frame[1] = SOF1;
  frame[2] = static_cast<uint8_t>(cmd | 0x80U);
  frame[3] = seq;
  put_u16le(&frame[4], len);
  if (len > 0U) {
    memcpy(&frame[6], payload, len);
  }
  crc = crc16_ccitt(&frame[2], static_cast<uint16_t>(4U + len));
  put_u16le(&frame[6 + len], crc);

  set_bus_tx(true);
  delayMicroseconds(200);
  BusSerial.write(frame, 8U + len);
  BusSerial.flush();
  delayMicroseconds(200);
  set_bus_tx(false);
}

static void send_status_only(uint8_t cmd, uint8_t seq, uint8_t status) {
  uint8_t payload[1] = {status};
  bus_write_frame(cmd, seq, payload, 1);
}

static void handle_read_uid(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint16_t len) {
  if (len != 1U || payload[0] < 1U || payload[0] > 4U) {
    send_status_only(cmd, seq, ST_BAD_ARG);
    return;
  }

  PN0031::CardInfo card;
  uint8_t out[1 + 1 + 1 + 1 + PN0031::UID_MAX_LEN];

  if (!reader.readCard(payload[0], card, 150)) {
    if (reader.lastError() == PN0031::STATUS_ERROR) {
      out[0] = ST_NO_TAG;
      out[1] = reader.lastStatus();
      bus_write_frame(cmd, seq, out, 2);
    } else {
      send_status_only(cmd, seq, ST_IO_ERR);
    }
    return;
  }

  out[0] = ST_OK;
  out[1] = card.antenna;
  out[2] = 0x00;
  out[3] = card.uidLen;
  memcpy(&out[4], card.uid, card.uidLen);
  bus_write_frame(cmd, seq, out, static_cast<uint16_t>(4U + card.uidLen));
}

static void handle_read_all_uids(uint8_t cmd, uint8_t seq) {
  PN0031::CardInfo cards[4];
  size_t cardCount = 0;
  uint8_t out[1 + 1 + 4 * (1 + 1 + PN0031::UID_MAX_LEN)];
  uint16_t outLen = 0;

  if (!reader.readAllCards(cards, 4, cardCount, 200)) {
    if (reader.lastError() == PN0031::STATUS_ERROR) {
      out[0] = ST_NO_TAG;
      out[1] = reader.lastStatus();
      bus_write_frame(cmd, seq, out, 2);
    } else {
      send_status_only(cmd, seq, ST_IO_ERR);
    }
    return;
  }

  out[outLen++] = ST_OK;
  out[outLen++] = static_cast<uint8_t>(cardCount);
  for (size_t i = 0; i < cardCount; ++i) {
    out[outLen++] = cards[i].antenna;
    out[outLen++] = cards[i].uidLen;
    memcpy(&out[outLen], cards[i].uid, cards[i].uidLen);
    outLen = static_cast<uint16_t>(outLen + cards[i].uidLen);
  }
  bus_write_frame(cmd, seq, out, outLen);
}

static void handle_bus_cmd(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint16_t len) {
  uint8_t out[8];

  digitalWrite(LED_PIN, !digitalRead(LED_PIN));

  if (cmd == CMD_PING || cmd == CMD_GET_VERSION) {
    out[0] = ST_OK;
    out[1] = APP_VERSION_MAJOR;
    out[2] = APP_VERSION_MINOR;
    out[3] = APP_VERSION_PATCH;
    bus_write_frame(cmd, seq, out, 4);
    return;
  }

  if (cmd == CMD_READ_UID) {
    handle_read_uid(cmd, seq, payload, len);
    return;
  }

  if (cmd == CMD_READ_ALL_UIDS) {
    if (len != 0U) {
      send_status_only(cmd, seq, ST_BAD_ARG);
    } else {
      handle_read_all_uids(cmd, seq);
    }
    return;
  }

  send_status_only(cmd, seq, ST_BAD_CMD);
}

static void poll_bus() {
  while (BusSerial.available() > 0) {
    int v = BusSerial.read();
    if (v < 0) {
      break;
    }
    if (g_bus.len >= sizeof(g_bus.buf)) {
      g_bus.len = 0;
    }
    g_bus.buf[g_bus.len++] = static_cast<uint8_t>(v);
  }

  while (g_bus.len >= 2U) {
    if (g_bus.buf[0] != SOF0 || g_bus.buf[1] != SOF1) {
      memmove(&g_bus.buf[0], &g_bus.buf[1], g_bus.len - 1U);
      g_bus.len--;
      continue;
    }

    if (g_bus.len < 6U) {
      return;
    }

    uint16_t payloadLen = get_u16le(&g_bus.buf[4]);
    if (payloadLen > BUS_MAX_PAYLOAD) {
      memmove(&g_bus.buf[0], &g_bus.buf[1], g_bus.len - 1U);
      g_bus.len--;
      continue;
    }

    uint16_t frameLen = static_cast<uint16_t>(8U + payloadLen);
    if (g_bus.len < frameLen) {
      return;
    }

    uint16_t crcRx = get_u16le(&g_bus.buf[6U + payloadLen]);
    uint16_t crcCalc = crc16_ccitt(&g_bus.buf[2], static_cast<uint16_t>(4U + payloadLen));
    if (crcRx == crcCalc) {
      handle_bus_cmd(g_bus.buf[2], g_bus.buf[3], &g_bus.buf[6], payloadLen);
    }

    if (g_bus.len > frameLen) {
      memmove(&g_bus.buf[0], &g_bus.buf[frameLen], g_bus.len - frameLen);
    }
    g_bus.len -= frameLen;
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  delay(200);

  if (BUS_DIR_PIN >= 0) {
    pinMode(BUS_DIR_PIN, OUTPUT);
    set_bus_tx(false);
  }
  BusSerial.begin(BUS_BAUD, SERIAL_8N1, BUS_RX_PIN, BUS_TX_PIN);

  reader.begin(PnSerial, PN_BAUD, PN_RX_PIN, PN_TX_PIN);
  if (PN_DIR_PIN >= 0) {
    reader.setRs485DirectionPin(PN_DIR_PIN, PN_DIR_TX_ENABLE_HIGH, 120);
  }

  Serial.println("esp32c3 proxy ready");
}

void loop() {
  poll_bus();
}
