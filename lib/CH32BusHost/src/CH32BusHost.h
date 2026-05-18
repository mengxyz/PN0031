#ifndef CH32_BUS_HOST_H
#define CH32_BUS_HOST_H

#include <Arduino.h>
#include <FS.h>

class CH32BusHost {
public:
  static const uint8_t ST_OK = 0x00;
  static const uint8_t ST_BAD_CMD = 0x01;
  static const uint8_t ST_BAD_ARG = 0x02;
  static const uint8_t ST_IO_ERR = 0x04;
  static const uint8_t ST_NO_TAG = 0x05;
  static const uint8_t ST_NOT_READY = 0x06;

  struct UidInfo {
    uint8_t antenna;
    uint8_t pnStatus;
    uint8_t uidLen;
    uint8_t uid[16];
  };

  struct SwitchStatus {
    uint8_t rawState;
    uint8_t onlineMask;
  };

  struct FilamentRaw {
    uint8_t len;
    uint8_t data[96];
  };

  struct DeviceInfo {
    uint8_t status;
    uint8_t deviceType;
    uint8_t deviceAddr;
    uint8_t versionMajor;
    uint8_t versionMinor;
    uint8_t versionPatch;
    uint8_t uid[8];
  };

  struct HeartbeatInfo {
    uint8_t deviceAddr;
    uint8_t deviceType;
    uint8_t reportedAddr;
  };

  CH32BusHost();

  typedef size_t (*FsStatFn)();

  bool begin(HardwareSerial &serial,
             int8_t rxPin,
             int8_t txPin,
             uint32_t busBaud = 460800,
             uint32_t iapBaud = 115200,
             int8_t dirPin = -1,
             bool dirTxHigh = true);

  void attachFilesystem(fs::FS &fs,
                        const char *fwPath = "/ch32_fw.bin",
                        const char *tmpPath = "/ch32_fw.tmp");
  void attachFilesystemStats(FsStatFn totalBytesFn, FsStatFn usedBytesFn);

  bool ping(uint8_t &major, uint8_t &minor, uint8_t &patch, uint8_t *status = nullptr);
  bool ping(DeviceInfo &out);
  bool getVersion(uint8_t &major, uint8_t &minor, uint8_t &patch, uint8_t *status = nullptr);
  bool getVersion(DeviceInfo &out);
  bool discover(DeviceInfo *out, size_t maxDevices, size_t &count, uint32_t collectMs = 2000);
  bool enterBoot(uint8_t *status = nullptr);
  bool readUid(uint8_t antenna, UidInfo &info, uint8_t *status = nullptr);
  bool readFilament(uint8_t antenna, FilamentRaw &out, uint8_t *status = nullptr);
  bool getSwitchStatus(SwitchStatus &out, uint8_t *status = nullptr);
  bool motorCtrl(uint8_t channel, uint8_t speed, uint8_t *status = nullptr);
  bool sendHeartbeat(uint8_t *status = nullptr);
  bool readHeartbeat(HeartbeatInfo &out, uint32_t timeoutMs = 1000);

  bool firmwareInfo(size_t &sizeBytes);
  bool firmwareRead(size_t offset, uint8_t *out, size_t maxLen, size_t &readLen);
  bool filesystemStatus(size_t &totalBytes, size_t &usedBytes, size_t &freeBytes);
  bool firmwareDelete();
  bool firmwareBeginWrite(size_t expectedBytes);
  bool firmwareAppend(const uint8_t *data, size_t len);
  bool firmwareEndWrite();
  size_t firmwareExpectedBytes() const;
  size_t firmwareWrittenBytes() const;
  bool firmwareReceiving() const;

  void setIapDevAddr(uint8_t addr);
  void setDeviceAddress(uint8_t addr);

  bool iapProbe();
  bool flashStoredFirmware(Stream *log = nullptr);

  const char *lastError() const;

private:
  static const uint8_t SOF0 = 0x55;
  static const uint8_t SOF1 = 0xAA;
  static const uint8_t CMD_DISCOVER = 0x00;
  static const uint8_t CMD_PING = 0x01;
  static const uint8_t CMD_ENTER_BOOT = 0x22;
  static const uint8_t CMD_GET_VERSION = 0x30;
  static const uint8_t CMD_READ_UID = 0x32;
  static const uint8_t CMD_READ_FILAMENT = 0x33;
  static const uint8_t CMD_MOTOR_CTRL = 0x40;
  static const uint8_t CMD_GET_SWITCH_STATUS = 0x41;
  static const uint8_t CMD_HB = 0x70;
  static const uint8_t BROADCAST_ADDR = 0xFF;

  static const uint8_t IAP_SYNC1 = 0xAA;
  static const uint8_t IAP_SYNC2 = 0x55;
  static const uint8_t CMD_IAP_SCAN = 0x01;
  static const uint8_t CMD_IAP_PROM = 0x80;
  static const uint8_t CMD_IAP_ERASE = 0x81;
  static const uint8_t CMD_IAP_VERIFY = 0x82;
  static const uint8_t CMD_IAP_END = 0x83;
  static const uint8_t CMD_IAP_CRC = 0x85;
  static const uint8_t IAP_ERR_SUCCESS = 0x00;
  static const size_t IAP_CHUNK_SIZE = 128;

  static const size_t MAX_PAYLOAD = 96;
  static const size_t MAX_FRAME = 9 + MAX_PAYLOAD;

  struct BusFrame {
    uint8_t addr;
    uint8_t cmd;
    uint8_t seq;
    uint8_t payload[MAX_PAYLOAD];
    uint16_t payloadLen;
  };

  HardwareSerial *_serial;
  fs::FS *_fs;
  FsStatFn _fsTotalBytesFn;
  FsStatFn _fsUsedBytesFn;
  File _tmpFile;
  const char *_fwPath;
  const char *_tmpPath;
  uint32_t _busBaud;
  uint32_t _iapBaud;
  int8_t _rxPin;
  int8_t _txPin;
  int8_t _dirPin;
  bool _dirTxHigh;
  uint8_t _devAddr;
  uint8_t _iapDevAddr;
  bool _iapDevAddrOverridden;
  uint8_t _seq;
  uint8_t _busRx[MAX_FRAME];
  size_t _busRxLen;
  size_t _fwExpected;
  size_t _fwWritten;
  bool _fwReceiving;
  const char *_lastError;

  void setError(const char *msg);
  void clearError();
  void setTxMode(bool en);
  void setBaud(uint32_t baud);
  void flushRx();

  static uint16_t crc16Ccitt(const uint8_t *data, uint16_t len);
  static uint16_t getU16Le(const uint8_t *p);
  static void putU16Le(uint8_t *p, uint16_t v);
  static uint16_t iapChecksum16(uint8_t dest, uint8_t cmd, uint8_t length, const uint8_t *data);
  static bool parseDeviceInfoPayload(const uint8_t *payload, uint16_t payloadLen, DeviceInfo &out);

  bool busRequest(uint8_t cmd,
                  const uint8_t *payload,
                  uint16_t payloadLen,
                  uint8_t *respCmd,
                  uint8_t *respSeq,
                  uint8_t *respPayload,
                  uint16_t *respLen,
                  uint32_t timeoutMs);
  bool busRequestTo(uint8_t dest,
                    uint8_t cmd,
                    const uint8_t *payload,
                    uint16_t payloadLen,
                    uint8_t *respCmd,
                    uint8_t *respSeq,
                    uint8_t *respPayload,
                    uint16_t *respLen,
                    uint32_t timeoutMs);
  bool busReadFrame(BusFrame &out, uint32_t timeoutMs);
  void busSendFrame(uint8_t dest, uint8_t cmd, uint8_t seq, const uint8_t *payload, uint16_t payloadLen);

  void iapBuildFrame(uint8_t *out, size_t *outLen, uint8_t dest, uint8_t cmd, const uint8_t *data, uint8_t len);
  bool iapReadAck(uint8_t *err, uint32_t timeoutMs);
  bool iapSendAndAck(uint8_t cmd, const uint8_t *data, uint8_t len, uint32_t timeoutMs, uint8_t *err);
  bool iapErase();
  bool iapProgramChunk(const uint8_t *chunk, uint8_t len);
  bool iapVerifyCrc(size_t fwSize, uint16_t expectedCrc);
  bool iapEnd();
  bool ensureIapReady();
};

#endif
