#include <Arduino.h>
#include <PN0031.h>

static const int PN0031_RX_PIN = 17;
static const int PN0031_TX_PIN = 16;

PN0031 reader;

void setup() {
  Serial.begin(115200);
  delay(500);

  reader.begin(Serial1, 115200, PN0031_RX_PIN, PN0031_TX_PIN);
  Serial.println("PN0031 example started");
}

void loop() {
  PN0031::CardInfo card;
  if (reader.readCard(1, card, 120)) {
    Serial.print("UID: ");
    for (uint8_t i = 0; i < card.uidLen; ++i) {
      if (card.uid[i] < 0x10) {
        Serial.print('0');
      }
      Serial.print(card.uid[i], HEX);
    }
    Serial.println();
  }

  delay(300);
}
