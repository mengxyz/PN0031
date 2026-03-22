#ifndef PN0031_H
#define PN0031_H

#include <Arduino.h>

class PN0031 {
public:
  static const uint8_t DEFAULT_ADDRESS = 0x00;
  static const uint8_t COMMAND_FT = 0x02;
  static const uint8_t SET_ADDRESS_FT = 0x00;

  static const uint8_t FC_SET_ADDRESS = 0x00;
  static const uint8_t FC_READ_SPECIFIC_ANTENNA = 0x20;
  static const uint8_t FC_READ_ALL_ANTENNAS = 0x21;
  static const uint8_t FC_READ_BLOCK = 0x22;
  static const uint8_t FC_WRITE_BLOCK = 0x23;

  static const size_t UID_MAX_LEN = 16;
  static const size_t FRAME_MAX_DATA_LEN = 128;

  enum Error {
    OK = 0,
    NO_STREAM,
    TIMEOUT,
    FRAME_ERROR,
    BAD_CHECKSUM,
    RESPONSE_MISMATCH,
    BUFFER_TOO_SMALL,
    STATUS_ERROR,
    INVALID_ARG
  };

  struct CardInfo {
    uint8_t antenna;
    uint8_t uidLen;
    uint8_t uid[UID_MAX_LEN];
  };

  PN0031();

  bool begin(HardwareSerial &serial,
             uint32_t baud = 115200,
             int8_t rxPin = -1,
             int8_t txPin = -1,
             uint8_t address = DEFAULT_ADDRESS);

  void attach(Stream &serial, uint8_t address = DEFAULT_ADDRESS);
  void setRs485DirectionPin(int8_t dirPin,
                            bool txEnableHigh = true,
                            uint16_t guardTimeUs = 100);
  void disableRs485DirectionPin();

  Error lastError() const;
  uint8_t lastStatus() const;

  bool setAddress(uint8_t newAddress, uint8_t timeoutMs = 100);
  bool readCard(uint8_t antenna, CardInfo &card, uint32_t timeoutMs = 100);
  bool readAllCards(CardInfo *cards,
                    size_t maxCards,
                    size_t &cardCount,
                    uint32_t timeoutMs = 100);

  bool readBlock(uint8_t antenna,
                 uint8_t block,
                 uint8_t keyPos,
                 uint8_t keyType,
                 const uint8_t key[6],
                 uint8_t outData[16],
                 uint32_t timeoutMs = 120);

  bool writeBlock(uint8_t antenna,
                  uint8_t block,
                  uint8_t keyPos,
                  uint8_t keyType,
                  const uint8_t key[6],
                  const uint8_t inData[16],
                  uint32_t timeoutMs = 120);

private:
  struct Frame {
    uint8_t address;
    uint8_t ft;
    uint8_t fc;
    uint16_t len;
    uint8_t data[FRAME_MAX_DATA_LEN];
  };

  static const uint8_t STX = 0xA0;
  static const uint8_t ETX = 0x0D;

  Stream *_serial;
  uint8_t _address;
  int8_t _dirPin;
  bool _txEnableHigh;
  uint16_t _guardTimeUs;
  Error _lastError;
  uint8_t _lastStatus;

  void clearError();
  void setError(Error err, uint8_t status = 0xFF);

  bool sendCommand(uint8_t ft,
                   uint8_t fc,
                   const uint8_t *data,
                   uint16_t len);

  bool readFrame(Frame &frame, uint32_t timeoutMs);
  bool transact(uint8_t ft,
                uint8_t fc,
                const uint8_t *request,
                uint16_t requestLen,
                Frame &response,
                uint32_t timeoutMs);

  bool readByte(uint8_t &value, uint32_t deadlineMs);
  void setTxMode();
  void setRxMode();

  static uint8_t computeXor(const uint8_t *data, size_t len);
};

#endif
