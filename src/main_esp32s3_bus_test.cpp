#include <Arduino.h>
#include <LittleFS.h>
#include <CH32BusHost.h>

static const uint32_t USB_BAUD = 115200;
static const uint32_t BUS_BAUD = 460800;
static const uint32_t IAP_BAUD = 460800;
static const int BUS_RX_PIN = 44;
static const int BUS_TX_PIN = 43;
static const int BUS_DIR_PIN = -1;
static const int LED_PIN = LED_BUILTIN;

static CH32BusHost g_host;

static size_t littlefs_total_bytes() { return LittleFS.totalBytes(); }
static size_t littlefs_used_bytes() { return LittleFS.usedBytes(); }

static const char *status_name(uint8_t st) {
  switch (st) {
    case CH32BusHost::ST_OK: return "OK";
    case CH32BusHost::ST_BAD_CMD: return "BAD_CMD";
    case CH32BusHost::ST_BAD_ARG: return "BAD_ARG";
    case CH32BusHost::ST_IO_ERR: return "IO_ERR";
    case CH32BusHost::ST_NO_TAG: return "NO_TAG";
    case CH32BusHost::ST_NOT_READY: return "NOT_READY";
    default: return "UNKNOWN";
  }
}

static void dump_hex(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    if (data[i] < 0x10U) Serial.print('0');
    Serial.print(data[i], HEX);
  }
}

static void print_device_info(const CH32BusHost::DeviceInfo &info) {
  Serial.print("status=");
  Serial.print(status_name(info.status));
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
  dump_hex(info.uid, sizeof(info.uid));
  Serial.println();
}

static int hex_nibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
  if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
  return -1;
}

static bool decode_hex_string(const String &hex, uint8_t *out, size_t max_out, size_t *out_len) {
  if ((hex.length() & 1U) != 0U) return false;
  size_t n = hex.length() / 2U;
  if (n > max_out) return false;
  for (size_t i = 0; i < n; ++i) {
    int hi = hex_nibble(hex[2U * i]);
    int lo = hex_nibble(hex[2U * i + 1U]);
    if (hi < 0 || lo < 0) return false;
    out[i] = static_cast<uint8_t>((hi << 4) | lo);
  }
  *out_len = n;
  return true;
}

static void print_help() {
  Serial.println("Commands:");
  Serial.println("  a        show target address");
  Serial.println("  a 0x02   set target address");
  Serial.println("  p        ping");
  Serial.println("  v        version");
  Serial.println("  i        device info");
  Serial.println("  d        discover");
  Serial.println("  h        heartbeat listen");
  Serial.println("  k        host heartbeat keepalive");
  Serial.println("  u1..u4   read uid");
  Serial.println("  f1..f4   read filament");
  Serial.println("  s        get switch status");
  Serial.println("  m1 128   motor channel 1 speed 128");
  Serial.println("  m1 0     motor channel 1 stop");
  Serial.println("  b        enter boot");
  Serial.println("  fwi      firmware info");
  Serial.println("  fwr O N  read stored firmware chunk");
  Serial.println("  fws      filesystem status");
  Serial.println("  fwx      delete stored firmware");
  Serial.println("  fwp      IAP probe");
  Serial.println("  fwf      flash stored firmware");
  Serial.println("  fwb N    begin USB firmware upload (size bytes)");
  Serial.println("  fwc HEX  append firmware chunk");
  Serial.println("  fwe      finish USB firmware upload");
}

static void cmd_device_info() {
  CH32BusHost::DeviceInfo info;
  if (!g_host.getVersion(info)) {
    Serial.println(g_host.lastError());
    return;
  }
  print_device_info(info);
}

static void cmd_discover() {
  CH32BusHost::DeviceInfo devices[8];
  size_t count = 0;
  if (!g_host.discover(devices, 8, count)) {
    Serial.println(g_host.lastError());
    return;
  }
  Serial.print("devices=");
  Serial.println((unsigned long)count);
  for (size_t i = 0; i < count; ++i) {
    Serial.print("  ");
    print_device_info(devices[i]);
  }
  if (count == 1U) {
    Serial.print("target addr=0x");
    if (devices[0].deviceAddr < 0x10U) Serial.print('0');
    Serial.println(devices[0].deviceAddr, HEX);
  }
}

static void cmd_addr(const String &args) {
  if (args.length() == 0U) {
    uint8_t addr = g_host.deviceAddress();
    Serial.print("target addr=0x");
    if (addr < 0x10U) Serial.print('0');
    Serial.println(addr, HEX);
    return;
  }
  unsigned long addr = strtoul(args.c_str(), nullptr, 0);
  if (addr > 0xFEUL) {
    Serial.println("ERR addr_range");
    return;
  }
  g_host.setDeviceAddress((uint8_t)addr);
  Serial.print("target addr=0x");
  if (addr < 0x10UL) Serial.print('0');
  Serial.println(addr, HEX);
}

static void cmd_heartbeat() {
  CH32BusHost::HeartbeatInfo hb;
  if (!g_host.readHeartbeat(hb, 5000)) {
    Serial.println(g_host.lastError());
    return;
  }
  Serial.print("heartbeat addr=0x");
  if (hb.deviceAddr < 0x10U) Serial.print('0');
  Serial.print(hb.deviceAddr, HEX);
  Serial.print(" type=0x");
  if (hb.deviceType < 0x10U) Serial.print('0');
  Serial.print(hb.deviceType, HEX);
  Serial.print(" reported_addr=0x");
  if (hb.reportedAddr < 0x10U) Serial.print('0');
  Serial.println(hb.reportedAddr, HEX);
}

static void cmd_keepalive() {
  uint8_t st;
  if (!g_host.sendHeartbeat(&st)) {
    Serial.print("NO_CONNECTION lost host heartbeat ack: ");
    Serial.println(g_host.lastError());
    return;
  }
  Serial.print("heartbeat_ack status=");
  Serial.println(status_name(st));
}

static void cmd_ping() {
  uint8_t maj, min_, patch, st;
  if (!g_host.ping(maj, min_, patch, &st)) {
    Serial.println(g_host.lastError());
    return;
  }
  Serial.print("status=");
  Serial.print(status_name(st));
  Serial.print(" version=");
  Serial.print(maj);
  Serial.print('.');
  Serial.print(min_);
  Serial.print('.');
  Serial.println(patch);
}

static void cmd_version() {
  uint8_t maj, min_, patch, st;
  if (!g_host.getVersion(maj, min_, patch, &st)) {
    Serial.println(g_host.lastError());
    return;
  }
  Serial.print("status=");
  Serial.print(status_name(st));
  Serial.print(" version=");
  Serial.print(maj);
  Serial.print('.');
  Serial.print(min_);
  Serial.print('.');
  Serial.println(patch);
}

static void cmd_uid(uint8_t ant) {
  CH32BusHost::UidInfo info;
  uint8_t st;
  if (!g_host.readUid(ant, info, &st)) {
    Serial.println(g_host.lastError());
    return;
  }
  if (st != CH32BusHost::ST_OK) {
    Serial.print("status=");
    Serial.println(status_name(st));
    return;
  }
  Serial.print("uid=");
  dump_hex(info.uid, info.uidLen);
  Serial.println();
}

static void cmd_filament(uint8_t ant) {
  CH32BusHost::FilamentRaw out;
  uint8_t st;
  if (!g_host.readFilament(ant, out, &st)) {
    Serial.println(g_host.lastError());
    return;
  }
  if (st != CH32BusHost::ST_OK) {
    Serial.print("status=");
    Serial.println(status_name(st));
    return;
  }
  Serial.print("filament payload=");
  dump_hex(out.data, out.len);
  Serial.println();
}

static void cmd_switch() {
  CH32BusHost::SwitchStatus sw;
  uint8_t st;
  if (!g_host.getSwitchStatus(sw, &st)) {
    Serial.println(g_host.lastError());
    return;
  }
  if (st != CH32BusHost::ST_OK) {
    Serial.print("status=");
    Serial.println(status_name(st));
    return;
  }
  Serial.print("raw=0x");
  if (sw.rawState < 0x10U) Serial.print('0');
  Serial.print(sw.rawState, HEX);
  Serial.print(" online_mask=0x");
  Serial.println(sw.onlineMask, HEX);
}

static void cmd_motor(uint8_t ch, uint8_t speed) {
  uint8_t st;
  if (!g_host.motorCtrl(ch, speed, &st)) {
    Serial.println(g_host.lastError());
    return;
  }
  Serial.print("status=");
  Serial.print(status_name(st));
  Serial.print(" channel=");
  Serial.print(ch);
  Serial.print(" speed=");
  Serial.println(speed);
}

static void cmd_enter_boot() {
  uint8_t st;
  if (!g_host.enterBoot(&st)) {
    Serial.println(g_host.lastError());
    return;
  }
  Serial.print("status=");
  Serial.println(status_name(st));
}

static void cmd_fw_info() {
  size_t size = 0;
  if (!g_host.firmwareInfo(size)) {
    Serial.print("ERR ");
    Serial.println(g_host.lastError());
    return;
  }
  Serial.print("OK size=");
  Serial.println((unsigned long)size);
}

static void cmd_fw_read(const String &args) {
  int sep = args.indexOf(' ');
  if (sep <= 0) {
    Serial.println("ERR bad_args");
    return;
  }
  size_t offset = (size_t)strtoul(args.substring(0, sep).c_str(), nullptr, 0);
  size_t len = (size_t)strtoul(args.substring(sep + 1).c_str(), nullptr, 0);
  if (len > 128U) {
    Serial.println("ERR len_range");
    return;
  }
  uint8_t buf[128];
  size_t got = 0;
  if (!g_host.firmwareRead(offset, buf, len, got)) {
    Serial.print("ERR ");
    Serial.println(g_host.lastError());
    return;
  }
  Serial.print("OK data ");
  dump_hex(buf, got);
  Serial.println();
}

static void cmd_fs_status() {
  size_t total, used, free_bytes;
  if (!g_host.filesystemStatus(total, used, free_bytes)) {
    Serial.print("ERR ");
    Serial.println(g_host.lastError());
    return;
  }
  Serial.print("OK total=");
  Serial.print((unsigned long)total);
  Serial.print(" used=");
  Serial.print((unsigned long)used);
  Serial.print(" free=");
  Serial.println((unsigned long)free_bytes);
}

static void cmd_fw_delete() {
  if (!g_host.firmwareDelete()) {
    Serial.print("ERR ");
    Serial.println(g_host.lastError());
    return;
  }
  Serial.println("OK deleted");
}

static void cmd_fw_begin(size_t size) {
  if (!g_host.firmwareBeginWrite(size)) {
    Serial.print("ERR ");
    Serial.println(g_host.lastError());
    return;
  }
  Serial.print("OK begin ");
  Serial.println((unsigned long)size);
}

static void cmd_fw_chunk(const String &hex) {
  uint8_t buf[128];
  size_t len = 0;
  if (!decode_hex_string(hex, buf, sizeof(buf), &len)) {
    Serial.println("ERR bad_hex");
    return;
  }
  if (!g_host.firmwareAppend(buf, len)) {
    Serial.print("ERR ");
    Serial.println(g_host.lastError());
    return;
  }
  Serial.print("OK chunk ");
  Serial.println((unsigned long)g_host.firmwareWrittenBytes());
}

static void cmd_fw_end() {
  if (!g_host.firmwareEndWrite()) {
    Serial.print("ERR ");
    Serial.println(g_host.lastError());
    return;
  }
  Serial.print("OK end ");
  Serial.println((unsigned long)g_host.firmwareWrittenBytes());
}

static void cmd_iap_probe() {
  if (!g_host.iapProbe()) {
    Serial.print("ERR ");
    Serial.println(g_host.lastError());
    return;
  }
  Serial.println("OK iap_probe");
}

static void cmd_flash() {
  if (!g_host.flashStoredFirmware(&Serial)) {
    Serial.print("ERR ");
    Serial.println(g_host.lastError());
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(USB_BAUD);
  delay(300);
  LittleFS.begin(true);
  g_host.begin(Serial1, BUS_RX_PIN, BUS_TX_PIN, BUS_BAUD, IAP_BAUD, BUS_DIR_PIN);
  g_host.setDeviceAddress(0x01);
  g_host.attachFilesystem(LittleFS);
  g_host.attachFilesystemStats(littlefs_total_bytes, littlefs_used_bytes);
  Serial.println("esp32s3 bus test");
  Serial.println("bus rx=44 tx=43 dir=auto");
  print_help();
}

void loop() {
  if (Serial.available() <= 0) {
    delay(2);
    return;
  }
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.isEmpty()) return;

  if (cmd == "p") cmd_ping();
  else if (cmd == "a") cmd_addr("");
  else if (cmd.startsWith("a ")) cmd_addr(cmd.substring(2));
  else if (cmd == "v") cmd_version();
  else if (cmd == "i") cmd_device_info();
  else if (cmd == "d" || cmd == "discover") cmd_discover();
  else if (cmd == "h" || cmd == "heartbeat listen") cmd_heartbeat();
  else if (cmd == "k") cmd_keepalive();
  else if (cmd == "s") cmd_switch();
  else if (cmd == "b") cmd_enter_boot();
  else if (cmd == "fwi") cmd_fw_info();
  else if (cmd.startsWith("fwr ")) cmd_fw_read(cmd.substring(4));
  else if (cmd == "fws") cmd_fs_status();
  else if (cmd == "fwx") cmd_fw_delete();
  else if (cmd == "fwp") cmd_iap_probe();
  else if (cmd == "fwf") cmd_flash();
  else if (cmd == "fwe") cmd_fw_end();
  else if (cmd.startsWith("fwb ")) cmd_fw_begin((size_t)cmd.substring(4).toInt());
  else if (cmd.startsWith("fwc ")) cmd_fw_chunk(cmd.substring(4));
  else if (cmd.length() == 2 && cmd[0] == 'u' && cmd[1] >= '1' && cmd[1] <= '4') cmd_uid((uint8_t)(cmd[1] - '0'));
  else if (cmd.length() == 2 && cmd[0] == 'f' && cmd[1] >= '1' && cmd[1] <= '4') cmd_filament((uint8_t)(cmd[1] - '0'));
  else if (cmd.length() >= 4 && cmd[0] == 'm' && cmd[1] >= '1' && cmd[1] <= '4' && cmd[2] == ' ') {
    int speed = cmd.substring(3).toInt();
    if (speed < 0 || speed > 255) {
      Serial.println("ERR speed_range");
    } else {
      cmd_motor((uint8_t)(cmd[1] - '0'), (uint8_t)speed);
    }
  } else {
    print_help();
  }
}
