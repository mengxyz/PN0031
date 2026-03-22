#include "PN0031.h"

PN0031::PN0031()
    : _serial(nullptr),
      _address(DEFAULT_ADDRESS),
      _dirPin(-1),
      _txEnableHigh(true),
      _guardTimeUs(100),
      _lastError(OK),
      _lastStatus(0) {}

bool PN0031::begin(HardwareSerial &serial,
                   uint32_t baud,
                   int8_t rxPin,
                   int8_t txPin,
                   uint8_t address) {
#if defined(ARDUINO_ARCH_ESP32)
  if (rxPin >= 0 && txPin >= 0) {
    serial.begin(baud, SERIAL_8N1, rxPin, txPin);
  } else {
    serial.begin(baud);
  }
#else
  (void)rxPin;
  (void)txPin;
  serial.begin(baud);
#endif

  attach(serial, address);
  return true;
}

void PN0031::attach(Stream &serial, uint8_t address) {
  _serial = &serial;
  _address = address;
  setRxMode();
  clearError();
}

void PN0031::setRs485DirectionPin(int8_t dirPin,
                                  bool txEnableHigh,
                                  uint16_t guardTimeUs) {
  _dirPin = dirPin;
  _txEnableHigh = txEnableHigh;
  _guardTimeUs = guardTimeUs;
  pinMode(_dirPin, OUTPUT);
  setRxMode();
}

void PN0031::disableRs485DirectionPin() { _dirPin = -1; }

PN0031::Error PN0031::lastError() const { return _lastError; }

uint8_t PN0031::lastStatus() const { return _lastStatus; }

void PN0031::clearError() {
  _lastError = OK;
  _lastStatus = 0;
}

void PN0031::setError(Error err, uint8_t status) {
  _lastError = err;
  _lastStatus = status;
}

uint8_t PN0031::computeXor(const uint8_t *data, size_t len) {
  uint8_t bcc = 0;
  for (size_t i = 0; i < len; ++i) {
    bcc ^= data[i];
  }
  return bcc;
}

bool PN0031::readByte(uint8_t &value, uint32_t deadlineMs) {
  while (millis() < deadlineMs) {
    int c = _serial->read();
    if (c >= 0) {
      value = static_cast<uint8_t>(c);
      return true;
    }
    delay(1);
  }
  return false;
}

bool PN0031::sendCommand(uint8_t ft,
                         uint8_t fc,
                         const uint8_t *data,
                         uint16_t len) {
  if (_serial == nullptr) {
    setError(NO_STREAM);
    return false;
  }

  if (len > FRAME_MAX_DATA_LEN) {
    setError(BUFFER_TOO_SMALL);
    return false;
  }

  uint8_t frame[6 + FRAME_MAX_DATA_LEN + 2];
  frame[0] = STX;
  frame[1] = _address;
  frame[2] = ft;
  frame[3] = fc;
  frame[4] = static_cast<uint8_t>(len & 0xFF);
  frame[5] = static_cast<uint8_t>((len >> 8) & 0xFF);

  for (uint16_t i = 0; i < len; ++i) {
    frame[6 + i] = data[i];
  }

  const size_t payloadLen = 6 + len;
  frame[payloadLen] = computeXor(frame, payloadLen);
  frame[payloadLen + 1] = ETX;

  setTxMode();
  if (_guardTimeUs > 0) {
    delayMicroseconds(_guardTimeUs);
  }
  _serial->write(frame, payloadLen + 2);
  _serial->flush();
  if (_guardTimeUs > 0) {
    delayMicroseconds(_guardTimeUs);
  }
  setRxMode();
  return true;
}

bool PN0031::readFrame(Frame &frame, uint32_t timeoutMs) {
  if (_serial == nullptr) {
    setError(NO_STREAM);
    return false;
  }

  const uint32_t deadline = millis() + timeoutMs;
  uint8_t byte = 0;

  do {
    if (!readByte(byte, deadline)) {
      setError(TIMEOUT);
      return false;
    }
  } while (byte != STX);

  uint8_t header[5];
  for (size_t i = 0; i < sizeof(header); ++i) {
    if (!readByte(header[i], deadline)) {
      setError(TIMEOUT);
      return false;
    }
  }

  frame.address = header[0];
  frame.ft = header[1];
  frame.fc = header[2];
  frame.len = static_cast<uint16_t>(header[3]) |
              (static_cast<uint16_t>(header[4]) << 8);

  if (frame.len > FRAME_MAX_DATA_LEN) {
    setError(BUFFER_TOO_SMALL);
    return false;
  }

  for (uint16_t i = 0; i < frame.len; ++i) {
    if (!readByte(frame.data[i], deadline)) {
      setError(TIMEOUT);
      return false;
    }
  }

  uint8_t rxBcc = 0;
  uint8_t rxEtx = 0;
  if (!readByte(rxBcc, deadline) || !readByte(rxEtx, deadline)) {
    setError(TIMEOUT);
    return false;
  }

  if (rxEtx != ETX) {
    setError(FRAME_ERROR);
    return false;
  }

  uint8_t check[6 + FRAME_MAX_DATA_LEN];
  check[0] = STX;
  check[1] = frame.address;
  check[2] = frame.ft;
  check[3] = frame.fc;
  check[4] = static_cast<uint8_t>(frame.len & 0xFF);
  check[5] = static_cast<uint8_t>((frame.len >> 8) & 0xFF);
  for (uint16_t i = 0; i < frame.len; ++i) {
    check[6 + i] = frame.data[i];
  }

  const uint8_t expectedBcc = computeXor(check, 6 + frame.len);
  if (expectedBcc != rxBcc) {
    setError(BAD_CHECKSUM);
    return false;
  }

  return true;
}

bool PN0031::transact(uint8_t ft,
                      uint8_t fc,
                      const uint8_t *request,
                      uint16_t requestLen,
                      Frame &response,
                      uint32_t timeoutMs) {
  clearError();
  setRxMode();

  while (_serial != nullptr && _serial->available() > 0) {
    (void)_serial->read();
  }

  if (!sendCommand(ft, fc, request, requestLen)) {
    return false;
  }

  if (!readFrame(response, timeoutMs)) {
    return false;
  }

  if (response.ft != ft || response.fc != fc) {
    setError(RESPONSE_MISMATCH);
    return false;
  }

  if (response.len < 1) {
    setError(FRAME_ERROR);
    return false;
  }

  _lastStatus = response.data[0];
  if (_lastStatus != 0x00) {
    setError(STATUS_ERROR, _lastStatus);
    return false;
  }

  clearError();
  return true;
}

bool PN0031::setAddress(uint8_t newAddress, uint8_t timeoutMs) {
  Frame frame;
  uint8_t request[1] = {newAddress};
  if (!transact(SET_ADDRESS_FT, FC_SET_ADDRESS, request, sizeof(request), frame,
                timeoutMs)) {
    return false;
  }

  _address = newAddress;
  return true;
}

bool PN0031::readCard(uint8_t antenna, CardInfo &card, uint32_t timeoutMs) {
  Frame frame;
  uint8_t request[1] = {antenna};

  if (!transact(COMMAND_FT, FC_READ_SPECIFIC_ANTENNA, request,
                sizeof(request), frame, timeoutMs)) {
    return false;
  }

  if (frame.len < 3) {
    setError(FRAME_ERROR);
    return false;
  }

  card.antenna = frame.data[1];
  card.uidLen = frame.data[2];
  if (card.uidLen > UID_MAX_LEN) {
    setError(BUFFER_TOO_SMALL);
    return false;
  }

  if (frame.len != static_cast<uint16_t>(3 + card.uidLen)) {
    setError(FRAME_ERROR);
    return false;
  }

  for (uint8_t i = 0; i < card.uidLen; ++i) {
    card.uid[i] = frame.data[3 + i];
  }

  clearError();
  return true;
}

bool PN0031::readAllCards(CardInfo *cards,
                          size_t maxCards,
                          size_t &cardCount,
                          uint32_t timeoutMs) {
  cardCount = 0;
  if (cards == nullptr || maxCards == 0) {
    setError(INVALID_ARG);
    return false;
  }

  Frame frame;
  if (!transact(COMMAND_FT, FC_READ_ALL_ANTENNAS, nullptr, 0, frame,
                timeoutMs)) {
    return false;
  }

  size_t idx = 1;
  while (idx < frame.len) {
    if ((idx + 2) > frame.len) {
      setError(FRAME_ERROR);
      return false;
    }

    if (cardCount >= maxCards) {
      setError(BUFFER_TOO_SMALL);
      return false;
    }

    CardInfo &c = cards[cardCount];
    c.antenna = frame.data[idx++];
    c.uidLen = frame.data[idx++];

    if (c.uidLen > UID_MAX_LEN || (idx + c.uidLen) > frame.len) {
      setError(FRAME_ERROR);
      return false;
    }

    for (uint8_t i = 0; i < c.uidLen; ++i) {
      c.uid[i] = frame.data[idx++];
    }

    cardCount++;
  }

  clearError();
  return true;
}

bool PN0031::readBlock(uint8_t antenna,
                       uint8_t block,
                       uint8_t keyPos,
                       uint8_t keyType,
                       const uint8_t key[6],
                       uint8_t outData[16],
                       uint32_t timeoutMs) {
  if (key == nullptr || outData == nullptr) {
    setError(INVALID_ARG);
    return false;
  }

  uint8_t request[10];
  request[0] = antenna;
  request[1] = block;
  request[2] = keyPos;
  request[3] = keyType;
  for (uint8_t i = 0; i < 6; ++i) {
    request[4 + i] = key[i];
  }

  Frame frame;
  if (!transact(COMMAND_FT, FC_READ_BLOCK, request, sizeof(request), frame,
                timeoutMs)) {
    return false;
  }

  if (frame.len != (1 + 1 + 1 + 16)) {
    setError(FRAME_ERROR);
    return false;
  }

  for (uint8_t i = 0; i < 16; ++i) {
    outData[i] = frame.data[3 + i];
  }

  clearError();
  return true;
}

bool PN0031::writeBlock(uint8_t antenna,
                        uint8_t block,
                        uint8_t keyPos,
                        uint8_t keyType,
                        const uint8_t key[6],
                        const uint8_t inData[16],
                        uint32_t timeoutMs) {
  if (key == nullptr || inData == nullptr) {
    setError(INVALID_ARG);
    return false;
  }

  uint8_t request[26];
  request[0] = antenna;
  request[1] = block;
  request[2] = keyPos;
  request[3] = keyType;

  for (uint8_t i = 0; i < 6; ++i) {
    request[4 + i] = key[i];
  }

  for (uint8_t i = 0; i < 16; ++i) {
    request[10 + i] = inData[i];
  }

  Frame frame;
  if (!transact(COMMAND_FT, FC_WRITE_BLOCK, request, sizeof(request), frame,
                timeoutMs)) {
    return false;
  }

  if (frame.len != 3) {
    setError(FRAME_ERROR);
    return false;
  }

  if (frame.data[1] != antenna || frame.data[2] != block) {
    setError(RESPONSE_MISMATCH);
    return false;
  }

  clearError();
  return true;
}

void PN0031::setTxMode() {
  if (_dirPin >= 0) {
    digitalWrite(_dirPin, _txEnableHigh ? HIGH : LOW);
  }
}

void PN0031::setRxMode() {
  if (_dirPin >= 0) {
    digitalWrite(_dirPin, _txEnableHigh ? LOW : HIGH);
  }
}
