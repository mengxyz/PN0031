#include "CH32BusHost.h"

#include <string.h>

CH32BusHost::CH32BusHost()
    : _serial(nullptr), _fs(nullptr), _fwPath("/ch32_fw.bin"), _tmpPath("/ch32_fw.tmp"),
      _fsTotalBytesFn(nullptr), _fsUsedBytesFn(nullptr),
      _busBaud(460800), _iapBaud(115200), _rxPin(-1), _txPin(-1), _dirPin(-1),
      _dirTxHigh(true), _seq(0), _fwExpected(0), _fwWritten(0), _fwReceiving(false),
      _lastError("") {}

bool CH32BusHost::begin(HardwareSerial &serial,
                        int8_t rxPin,
                        int8_t txPin,
                        uint32_t busBaud,
                        uint32_t iapBaud,
                        int8_t dirPin,
                        bool dirTxHigh) {
  _serial = &serial;
  _rxPin = rxPin;
  _txPin = txPin;
  _busBaud = busBaud;
  _iapBaud = iapBaud;
  _dirPin = dirPin;
  _dirTxHigh = dirTxHigh;
  if (_dirPin >= 0) {
    pinMode(_dirPin, OUTPUT);
    digitalWrite(_dirPin, _dirTxHigh ? LOW : HIGH);
  }
  setBaud(_busBaud);
  clearError();
  return true;
}

void CH32BusHost::attachFilesystem(fs::FS &fs, const char *fwPath, const char *tmpPath) {
  _fs = &fs;
  _fwPath = fwPath;
  _tmpPath = tmpPath;
}

void CH32BusHost::attachFilesystemStats(FsStatFn totalBytesFn, FsStatFn usedBytesFn) {
  _fsTotalBytesFn = totalBytesFn;
  _fsUsedBytesFn = usedBytesFn;
}

void CH32BusHost::setError(const char *msg) { _lastError = msg; }
void CH32BusHost::clearError() { _lastError = ""; }
const char *CH32BusHost::lastError() const { return _lastError; }
size_t CH32BusHost::firmwareExpectedBytes() const { return _fwExpected; }
size_t CH32BusHost::firmwareWrittenBytes() const { return _fwWritten; }
bool CH32BusHost::firmwareReceiving() const { return _fwReceiving; }

void CH32BusHost::setTxMode(bool en) {
  if (_dirPin < 0) return;
  digitalWrite(_dirPin, en ? (_dirTxHigh ? HIGH : LOW) : (_dirTxHigh ? LOW : HIGH));
}

void CH32BusHost::setBaud(uint32_t baud) {
  if (_serial == nullptr) return;
  _serial->flush();
  _serial->end();
  _serial->begin(baud, SERIAL_8N1, _rxPin, _txPin);
  delay(10);
}

void CH32BusHost::flushRx() {
  if (_serial == nullptr) return;
  while (_serial->available() > 0) {
    _serial->read();
  }
}

uint16_t CH32BusHost::crc16Ccitt(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFFU;
  for (uint16_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t j = 0; j < 8; ++j) {
      crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U) : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

uint16_t CH32BusHost::getU16Le(const uint8_t *p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

void CH32BusHost::putU16Le(uint8_t *p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFFU);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFFU);
}

bool CH32BusHost::busRequest(uint8_t cmd,
                             const uint8_t *payload,
                             uint16_t payloadLen,
                             uint8_t *respCmd,
                             uint8_t *respSeq,
                             uint8_t *respPayload,
                             uint16_t *respLen,
                             uint32_t timeoutMs) {
  if (_serial == nullptr) {
    setError("no serial");
    return false;
  }
  uint8_t frame[MAX_FRAME];
  uint32_t start = millis();
  _seq++;
  frame[0] = SOF0;
  frame[1] = SOF1;
  frame[2] = cmd;
  frame[3] = _seq;
  putU16Le(&frame[4], payloadLen);
  if (payloadLen > 0U) memcpy(&frame[6], payload, payloadLen);
  uint16_t crc = crc16Ccitt(&frame[2], static_cast<uint16_t>(4U + payloadLen));
  putU16Le(&frame[6 + payloadLen], crc);

  setBaud(_busBaud);
  flushRx();
  setTxMode(true);
  _serial->write(frame, 8U + payloadLen);
  _serial->flush();
  setTxMode(false);

  static uint8_t rx[MAX_FRAME];
  size_t rxLen = 0;
  while ((millis() - start) < timeoutMs) {
    while (_serial->available() > 0) {
      int v = _serial->read();
      if (v < 0) break;
      if (rxLen < sizeof(rx)) rx[rxLen++] = static_cast<uint8_t>(v);
      else rxLen = 0;
    }
    while (rxLen >= 2U) {
      if (rx[0] != SOF0 || rx[1] != SOF1) {
        memmove(&rx[0], &rx[1], rxLen - 1U);
        rxLen--;
        continue;
      }
      if (rxLen < 6U) break;
      uint16_t ln = getU16Le(&rx[4]);
      if (ln > MAX_PAYLOAD) {
        memmove(&rx[0], &rx[1], rxLen - 1U);
        rxLen--;
        continue;
      }
      uint16_t frameLen = static_cast<uint16_t>(8U + ln);
      if (rxLen < frameLen) break;
      uint16_t crcRx = getU16Le(&rx[6 + ln]);
      uint16_t crcCalc = crc16Ccitt(&rx[2], static_cast<uint16_t>(4U + ln));
      if (crcRx == crcCalc) {
        *respCmd = rx[2];
        *respSeq = rx[3];
        *respLen = ln;
        if (ln > 0U) memcpy(respPayload, &rx[6], ln);
        clearError();
        return true;
      }
      if (rxLen > frameLen) memmove(&rx[0], &rx[frameLen], rxLen - frameLen);
      rxLen -= frameLen;
    }
    delay(1);
  }
  setError("bus timeout");
  return false;
}

bool CH32BusHost::ping(uint8_t &major, uint8_t &minor, uint8_t &patch, uint8_t *status) {
  uint8_t rcCmd = 0, rcSeq = 0, pl[MAX_PAYLOAD];
  uint16_t plLen = 0;
  if (!busRequest(CMD_PING, nullptr, 0, &rcCmd, &rcSeq, pl, &plLen, 1000)) return false;
  if (plLen < 4U) { setError("bad ping payload"); return false; }
  if (status) *status = pl[0];
  major = pl[1]; minor = pl[2]; patch = pl[3];
  return true;
}

bool CH32BusHost::getVersion(uint8_t &major, uint8_t &minor, uint8_t &patch, uint8_t *status) {
  uint8_t rcCmd = 0, rcSeq = 0, pl[MAX_PAYLOAD];
  uint16_t plLen = 0;
  if (!busRequest(CMD_GET_VERSION, nullptr, 0, &rcCmd, &rcSeq, pl, &plLen, 1000)) return false;
  if (plLen < 4U) { setError("bad version payload"); return false; }
  if (status) *status = pl[0];
  major = pl[1]; minor = pl[2]; patch = pl[3];
  return true;
}

bool CH32BusHost::enterBoot(uint8_t *status) {
  uint8_t rcCmd = 0, rcSeq = 0, pl[MAX_PAYLOAD];
  uint16_t plLen = 0;
  if (!busRequest(CMD_ENTER_BOOT, nullptr, 0, &rcCmd, &rcSeq, pl, &plLen, 1500)) return false;
  if (plLen < 1U) { setError("bad enter-boot payload"); return false; }
  if (status) *status = pl[0];
  return true;
}

bool CH32BusHost::readUid(uint8_t antenna, UidInfo &info, uint8_t *status) {
  uint8_t req[1] = {antenna};
  uint8_t rcCmd = 0, rcSeq = 0, pl[MAX_PAYLOAD];
  uint16_t plLen = 0;
  if (!busRequest(CMD_READ_UID, req, 1, &rcCmd, &rcSeq, pl, &plLen, 1500)) return false;
  if (plLen < 1U) { setError("bad uid payload"); return false; }
  if (status) *status = pl[0];
  if (pl[0] != ST_OK) return true;
  if (plLen < 4U || (4U + pl[3]) > plLen || pl[3] > sizeof(info.uid)) { setError("short uid payload"); return false; }
  info.antenna = pl[1];
  info.pnStatus = pl[2];
  info.uidLen = pl[3];
  memcpy(info.uid, &pl[4], info.uidLen);
  return true;
}

bool CH32BusHost::readFilament(uint8_t antenna, FilamentRaw &out, uint8_t *status) {
  uint8_t req[1] = {antenna};
  uint8_t rcCmd = 0, rcSeq = 0, pl[MAX_PAYLOAD];
  uint16_t plLen = 0;
  if (!busRequest(CMD_READ_FILAMENT, req, 1, &rcCmd, &rcSeq, pl, &plLen, 4000)) return false;
  if (plLen < 1U) { setError("bad filament payload"); return false; }
  if (status) *status = pl[0];
  if (plLen > sizeof(out.data)) { setError("filament payload too large"); return false; }
  out.len = static_cast<uint8_t>(plLen);
  memcpy(out.data, pl, plLen);
  return true;
}

bool CH32BusHost::getSwitchStatus(SwitchStatus &out, uint8_t *status) {
  uint8_t rcCmd = 0, rcSeq = 0, pl[MAX_PAYLOAD];
  uint16_t plLen = 0;
  if (!busRequest(CMD_GET_SWITCH_STATUS, nullptr, 0, &rcCmd, &rcSeq, pl, &plLen, 1000)) return false;
  if (plLen < 1U) { setError("bad switch payload"); return false; }
  if (status) *status = pl[0];
  if (pl[0] != ST_OK) return true;
  if (plLen < 3U) { setError("short switch payload"); return false; }
  out.rawState = pl[1];
  out.onlineMask = pl[2];
  return true;
}

bool CH32BusHost::motorCtrl(uint8_t channel, uint8_t action, uint8_t *status) {
  uint8_t req[2] = {channel, action};
  uint8_t rcCmd = 0, rcSeq = 0, pl[MAX_PAYLOAD];
  uint16_t plLen = 0;
  if (!busRequest(CMD_MOTOR_CTRL, req, 2, &rcCmd, &rcSeq, pl, &plLen, 1000)) return false;
  if (plLen < 1U) { setError("bad motor payload"); return false; }
  if (status) *status = pl[0];
  return true;
}

bool CH32BusHost::firmwareInfo(size_t &sizeBytes) {
  if (_fs == nullptr) { setError("no fs"); return false; }
  if (!_fs->exists(_fwPath)) { setError("no firmware"); return false; }
  File f = _fs->open(_fwPath, "r");
  if (!f) { setError("open failed"); return false; }
  sizeBytes = static_cast<size_t>(f.size());
  f.close();
  return true;
}

bool CH32BusHost::filesystemStatus(size_t &totalBytes, size_t &usedBytes, size_t &freeBytes) {
  if (_fs == nullptr) { setError("no fs"); return false; }
  if (_fsTotalBytesFn == nullptr || _fsUsedBytesFn == nullptr) {
    setError("fs stats unsupported");
    return false;
  }
  totalBytes = _fsTotalBytesFn();
  usedBytes = _fsUsedBytesFn();
  freeBytes = totalBytes >= usedBytes ? (totalBytes - usedBytes) : 0U;
  return true;
}

bool CH32BusHost::firmwareDelete() {
  if (_fs == nullptr) { setError("no fs"); return false; }
  if (_fwReceiving) {
    _tmpFile.close();
    _fwReceiving = false;
  }
  _fs->remove(_tmpPath);
  return (!_fs->exists(_fwPath) || _fs->remove(_fwPath));
}

bool CH32BusHost::firmwareBeginWrite(size_t expectedBytes) {
  if (_fs == nullptr) { setError("no fs"); return false; }
  if (_fwReceiving) {
    _tmpFile.close();
    _fwReceiving = false;
  }
  _fs->remove(_tmpPath);
  _tmpFile = _fs->open(_tmpPath, "w");
  if (!_tmpFile) { setError("open tmp failed"); return false; }
  _fwExpected = expectedBytes;
  _fwWritten = 0;
  _fwReceiving = true;
  return true;
}

bool CH32BusHost::firmwareAppend(const uint8_t *data, size_t len) {
  if (!_fwReceiving) { setError("no begin"); return false; }
  if ((_fwWritten + len) > _fwExpected) { setError("too large"); return false; }
  size_t wr = _tmpFile.write(data, len);
  if (wr != len) { setError("write failed"); return false; }
  _fwWritten += wr;
  return true;
}

bool CH32BusHost::firmwareEndWrite() {
  if (!_fwReceiving) { setError("no begin"); return false; }
  _tmpFile.flush();
  _tmpFile.close();
  _fwReceiving = false;
  if (_fwWritten != _fwExpected) {
    _fs->remove(_tmpPath);
    setError("size mismatch");
    return false;
  }
  if (_fs->exists(_fwPath)) _fs->remove(_fwPath);
  if (!_fs->rename(_tmpPath, _fwPath)) {
    setError("rename failed");
    return false;
  }
  return true;
}

uint16_t CH32BusHost::iapChecksum16(uint8_t cmd, uint8_t length, const uint8_t *extra4, const uint8_t *data) {
  uint16_t total = static_cast<uint16_t>(cmd + length);
  if (extra4 != nullptr) for (uint8_t i = 0; i < 4U; ++i) total = static_cast<uint16_t>(total + extra4[i]);
  if (data != nullptr) for (uint8_t i = 0; i < length; ++i) total = static_cast<uint16_t>(total + data[i]);
  return total;
}

void CH32BusHost::iapBuildFrame(uint8_t *out, size_t *outLen, uint8_t cmd, const uint8_t *data, uint8_t len, const uint8_t *extra4) {
  size_t pos = 0;
  out[pos++] = IAP_SYNC1;
  out[pos++] = IAP_SYNC2;
  out[pos++] = cmd;
  out[pos++] = len;
  if (extra4 != nullptr) { memcpy(&out[pos], extra4, 4U); pos += 4U; }
  if (len > 0U && data != nullptr) { memcpy(&out[pos], data, len); pos += len; }
  uint16_t sum = iapChecksum16(cmd, len, extra4, data);
  out[pos++] = static_cast<uint8_t>(sum & 0xFFU);
  out[pos++] = static_cast<uint8_t>((sum >> 8) & 0xFFU);
  out[pos++] = IAP_SYNC2;
  out[pos++] = IAP_SYNC1;
  *outLen = pos;
}

bool CH32BusHost::iapReadAck(uint8_t *err, uint32_t timeoutMs) {
  uint32_t start = millis();
  while ((millis() - start) < timeoutMs) {
    if (_serial->available() <= 0) { delay(1); continue; }
    int b0 = _serial->read();
    if (b0 != IAP_SYNC1) continue;
    uint8_t rest[5];
    size_t got = _serial->readBytes(rest, sizeof(rest));
    if (got != sizeof(rest)) continue;
    if (rest[0] != IAP_SYNC2 || rest[1] != 0x00U || rest[3] != IAP_SYNC2 || rest[4] != IAP_SYNC1) continue;
    *err = rest[2];
    return true;
  }
  setError("iap ack timeout");
  return false;
}

bool CH32BusHost::iapSendAndAck(uint8_t cmd, const uint8_t *data, uint8_t len, const uint8_t *extra4, uint32_t timeoutMs, uint8_t *err) {
  uint8_t frame[80];
  size_t frameLen = 0;
  iapBuildFrame(frame, &frameLen, cmd, data, len, extra4);
  flushRx();
  setTxMode(true);
  _serial->write(frame, frameLen);
  _serial->flush();
  setTxMode(false);
  return iapReadAck(err, timeoutMs);
}

bool CH32BusHost::iapProbe() {
  setBaud(_iapBaud);
  uint8_t err = 0xFF;
  if (!iapSendAndAck(CMD_JUMP_IAP, nullptr, 0, nullptr, 1500, &err)) return false;
  if (err != IAP_ERR_SUCCESS) { setError("iap probe failed"); return false; }
  return true;
}

bool CH32BusHost::iapErase() {
  uint8_t extra4[4] = {0,0,0,0};
  uint8_t err = 0xFF;
  if (!iapSendAndAck(CMD_IAP_ERASE, nullptr, 0, extra4, 15000, &err)) return false;
  if (err != IAP_ERR_SUCCESS) { setError("iap erase failed"); return false; }
  return true;
}

bool CH32BusHost::iapProgramChunk(const uint8_t *chunk, uint8_t len) {
  uint8_t err = 0xFF;
  if (!iapSendAndAck(CMD_IAP_PROM, chunk, len, nullptr, 1500, &err)) return false;
  if (err != IAP_ERR_SUCCESS) { setError("iap program failed"); return false; }
  return true;
}

bool CH32BusHost::iapVerifyChunk(const uint8_t *chunk, uint8_t len) {
  uint8_t extra4[4] = {0,0,0,0};
  uint8_t err = 0xFF;
  if (!iapSendAndAck(CMD_IAP_VERIFY, chunk, len, extra4, 1500, &err)) return false;
  if (err != IAP_ERR_SUCCESS) { setError("iap verify failed"); return false; }
  return true;
}

bool CH32BusHost::iapEnd() {
  uint8_t err = 0xFF;
  if (!iapSendAndAck(CMD_IAP_END, nullptr, 0, nullptr, 1500, &err)) return true;
  return (err == IAP_ERR_SUCCESS || err == 0x02U);
}

bool CH32BusHost::ensureIapReady() {
  if (iapProbe()) return true;
  uint8_t status = 0xFF;
  if (!enterBoot(&status)) return false;
  delay(400);
  return iapProbe();
}

bool CH32BusHost::flashStoredFirmware(Stream *log) {
  if (_fs == nullptr) { setError("no fs"); return false; }
  if (!_fs->exists(_fwPath)) { setError("no firmware"); return false; }
  File f = _fs->open(_fwPath, "r");
  if (!f) { setError("open failed"); return false; }
  if (!ensureIapReady()) { f.close(); return false; }
  if (log) {
    log->print("FLASH erase size=");
    log->println(static_cast<unsigned long>(f.size()));
  }
  if (!iapErase()) { f.close(); return false; }

  uint8_t page[256];
  uint8_t verifyChunk[64];
  size_t total = static_cast<size_t>(f.size());
  size_t done = 0;
  while (done < total) {
    memset(page, 0xFF, sizeof(page));
    size_t pageLen = f.read(page, sizeof(page));
    if (pageLen == 0) { f.close(); setError("read failed"); return false; }
    for (size_t off = 0; off < sizeof(page); off += 64U) {
      if (!iapProgramChunk(&page[off], 64U)) { f.close(); return false; }
      if (log) {
        log->print("FLASH program ");
        log->print(static_cast<unsigned long>(min(done + off + 64U, total)));
        log->print("/");
        log->println(static_cast<unsigned long>(total));
      }
    }
    for (size_t off = 0; off < pageLen; off += 64U) {
      size_t partLen = min(static_cast<size_t>(64U), pageLen - off);
      memcpy(verifyChunk, &page[off], partLen);
      if (!iapVerifyChunk(verifyChunk, static_cast<uint8_t>(partLen))) { f.close(); return false; }
      if (log) {
        log->print("FLASH verify ");
        log->print(static_cast<unsigned long>(min(done + off + partLen, total)));
        log->print("/");
        log->println(static_cast<unsigned long>(total));
      }
    }
    done += pageLen;
  }
  f.close();
  if (log) log->println("FLASH end");
  iapEnd();
  delay(400);
  setBaud(_busBaud);
  if (log) log->println("FLASH done");
  return true;
}
