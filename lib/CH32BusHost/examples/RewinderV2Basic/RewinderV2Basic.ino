#include <Arduino.h>
#include <CH32BusHost.h>

static const int BUS_RX_PIN = 44;
static const int BUS_TX_PIN = 43;
static const int BUS_DIR_PIN = -1;

CH32BusHost host;

static void printHex(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    if (data[i] < 0x10U) Serial.print('0');
    Serial.print(data[i], HEX);
  }
}

static void printDevice(const CH32BusHost::DeviceInfo &info) {
  Serial.print("status=0x");
  if (info.status < 0x10U) Serial.print('0');
  Serial.print(info.status, HEX);
  Serial.print(" type=0x");
  if (info.deviceType < 0x10U) Serial.print('0');
  Serial.print(info.deviceType, HEX);
  Serial.print(" addr=0x");
  if (info.deviceAddr < 0x10U) Serial.print('0');
  Serial.print(info.deviceAddr, HEX);
  Serial.print(" version=");
  Serial.print(info.versionMajor);
  Serial.print('.');
  Serial.print(info.versionMinor);
  Serial.print('.');
  Serial.print(info.versionPatch);
  Serial.print(" uid=");
  printHex(info.uid, sizeof(info.uid));
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial1.begin(460800, SERIAL_8N1, BUS_RX_PIN, BUS_TX_PIN);
  host.begin(Serial1, BUS_RX_PIN, BUS_TX_PIN, 460800, 460800, BUS_DIR_PIN);
  host.setDeviceAddress(0x01);

  CH32BusHost::DeviceInfo devices[4];
  size_t count = 0;
  if (host.discover(devices, 4, count)) {
    Serial.print("discover count=");
    Serial.println((unsigned long)count);
    for (size_t i = 0; i < count; ++i) printDevice(devices[i]);
  } else {
    Serial.print("discover failed: ");
    Serial.println(host.lastError());
  }

  CH32BusHost::DeviceInfo info;
  if (host.ping(info)) {
    Serial.print("ping ");
    printDevice(info);
  } else {
    Serial.print("ping failed: ");
    Serial.println(host.lastError());
  }

  uint8_t status = 0xFF;
  if (host.motorCtrl(1, 128, &status)) {
    Serial.print("motor 1 speed 128 status=0x");
    Serial.println(status, HEX);
    delay(1000);
    host.motorCtrl(1, 0, &status);
    Serial.print("motor 1 stop status=0x");
    Serial.println(status, HEX);
  } else {
    Serial.print("motor failed: ");
    Serial.println(host.lastError());
  }
}

void loop() {
}
