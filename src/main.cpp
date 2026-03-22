#include <Arduino.h>
#include <PN0031.h>

// ESP32-C6 UART pins connected to RS485 transceiver.
static const int PN0031_RX_PIN = 23;
static const int PN0031_TX_PIN = 22;
static const int PN0031_DIR_PIN = 21;

PN0031 reader;

void printUid(const PN0031::CardInfo &card) {
  Serial.print("Antenna ");
  Serial.print(card.antenna);
  Serial.print(" UID: ");
  for (uint8_t i = 0; i < card.uidLen; ++i) {
    if (card.uid[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(card.uid[i], HEX);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  reader.begin(Serial1, 115200, PN0031_RX_PIN, PN0031_TX_PIN);
  reader.setRs485DirectionPin(PN0031_DIR_PIN, true, 120);

  Serial.println("PN0031 over UART-RS485 started");
}

void loop() {
  PN0031::CardInfo card;
  if (reader.readCard(1, card, 150)) {
    printUid(card);
  } else if (reader.lastError() == PN0031::STATUS_ERROR) {
    // Module returned non-zero status.
  } else {
    Serial.print("Read error=");
    Serial.print(static_cast<int>(reader.lastError()));
    Serial.print(" status=0x");
    Serial.println(reader.lastStatus(), HEX);
  }

  delay(300);
}
