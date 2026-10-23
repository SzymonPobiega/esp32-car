// OBD connector: talks to a BLE ELM327 dongle, polls transmission and boost readings
// from the Land Cruiser and forwards them over ESP-NOW to the ToyotaMonitor (via the C6 modem).
//
// Requests (see "Toyota Techstream PIDs.pdf"):
//   21DA @ 7E1  gear (A & 7) and torque converter lockup (bit 4 of A)   - experimental on VDJ200
//   21D9 @ 7E1  ATF pan temperature (E - 40) and converter outlet (G - 40)
//   010B @ 7DF  manifold absolute pressure, scaled by 014F byte D if supported
//   0133 @ 7DF  barometric pressure
//
// Target board: standard ESP32.

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <freertos/stream_buffer.h>
#include "ContractsInclude.hpp"

using namespace Contracts;

// -------------------- USER CONFIG --------------------
// Set to the dongle's BLE address (e.g. "aa:bb:cc:dd:ee:ff") to pin it. Empty = auto-detect
// by name or by a known ELM327 BLE service. Every device seen during the scan is logged.
static const char *DONGLE_MAC = "";

static const char *const DONGLE_NAME_HINTS[] = {
  "OBD", "ELM", "VLINK", "V-LINK", "VGATE", "VEEPEAK", "KONNWEI", "IOS-", "ANDROID-"
};

static const uint32_t ATF_POLL_MS = 2000;      // temperatures change slowly
static const uint32_t BARO_POLL_MS = 30000;
static const uint32_t SEND_INTERVAL_MS = 250;  // max ESP-NOW update rate while polling
static const uint32_t STATUS_INTERVAL_MS = 1000;  // ping rate while not polling
static const uint8_t  MAX_FAILURES = 3;        // consecutive failures before a value is cleared
// -----------------------------------------------------

// Serial services used by common ELM327 BLE dongles
static const NimBLEUUID KNOWN_SERVICES[] = {
  NimBLEUUID("fff0"),
  NimBLEUUID("ffe0"),
  NimBLEUUID("18f0"),
  NimBLEUUID("e7810a71-73ae-499d-8c15-faa9aef0c3f2"),
};

enum class State { Scanning, Initializing, Polling };

static State state = State::Scanning;
static NimBLEClient *client = nullptr;
static NimBLERemoteCharacteristic *chWrite = nullptr;
static StreamBufferHandle_t rxStream = nullptr;

static ObdPacket packet;
static uint32_t lastSendMs = 0;

static float mapScale = 1.0f;   // from 014F, 1.0 if not supported
static int16_t currentHeader = 0;

struct Poll {
  uint32_t lastMs;
  uint8_t failures;
};
static Poll pollGear = {};
static Poll pollAtf = {};
static Poll pollMap = {};
static Poll pollBaro = {};

// ---------- ESP-NOW ----------
void onEspNowSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
}

bool initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(Modem::ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  uint8_t staMac[6];
  esp_read_mac(staMac, ESP_MAC_WIFI_STA);
  Serial.printf("ESP-NOW (STA) MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                staMac[0], staMac[1], staMac[2], staMac[3], staMac[4], staMac[5]);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return false;
  }
  esp_now_register_send_cb(onEspNowSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, P4_4_1_MAC, 6);
  peerInfo.channel = 0;   // 0 = current Wi-Fi channel
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer!");
    return false;
  }
  return true;
}

void sendPacket() {
  packet.type = TYPE_OBD;
  esp_now_send(P4_4_1_MAC, (uint8_t *)&packet, sizeof(packet));
  lastSendMs = millis();
}

void clearValues() {
  packet.gear = OBD_NO_VALUE;
  packet.lockup = OBD_NO_VALUE;
  packet.atf_pan = OBD_NO_VALUE;
  packet.atf_outlet = OBD_NO_VALUE;
  packet.map_kpa = OBD_NO_VALUE;
  packet.baro_kpa = OBD_NO_VALUE;
  packet.boost = OBD_NO_VALUE;
}

// ---------- BLE ----------
class ClientCallbacks : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient *c, int reason) override {
    Serial.printf("Dongle disconnected (reason %d)\n", reason);
  }
};
static ClientCallbacks clientCallbacks;

static void onNotify(NimBLERemoteCharacteristic *, uint8_t *data, size_t len, bool) {
  xStreamBufferSend(rxStream, data, len, 0);
}

static bool nameMatches(const std::string &name) {
  String upper = String(name.c_str());
  upper.toUpperCase();
  for (const char *hint : DONGLE_NAME_HINTS) {
    if (upper.indexOf(hint) >= 0) return true;
  }
  return false;
}

static bool isDongle(const NimBLEAdvertisedDevice *dev) {
  if (DONGLE_MAC[0]) {
    return strcasecmp(dev->getAddress().toString().c_str(), DONGLE_MAC) == 0;
  }
  for (const NimBLEUUID &uuid : KNOWN_SERVICES) {
    if (dev->isAdvertisingService(uuid)) return true;
  }
  return dev->haveName() && nameMatches(dev->getName());
}

static bool isKnownService(const NimBLEUUID &uuid) {
  for (const NimBLEUUID &known : KNOWN_SERVICES) {
    if (uuid == known) return true;
  }
  return false;
}

// Finds a service with a notify/indicate and a write characteristic (the ELM serial port).
// Known ELM services are preferred over any other service that happens to match.
static bool subscribeSerialService() {
  NimBLERemoteCharacteristic *bestNotify = nullptr;
  NimBLERemoteCharacteristic *bestWrite = nullptr;
  bool bestKnown = false;

  for (NimBLERemoteService *svc : client->getServices(true)) {
    NimBLERemoteCharacteristic *notify = nullptr;
    NimBLERemoteCharacteristic *write = nullptr;
    for (NimBLERemoteCharacteristic *chr : svc->getCharacteristics(true)) {
      if (!notify && (chr->canNotify() || chr->canIndicate())) notify = chr;
      if (!write && (chr->canWrite() || chr->canWriteNoResponse())) write = chr;
    }
    if (!notify || !write) continue;

    bool known = isKnownService(svc->getUUID());
    Serial.printf("  serial candidate: service %s notify %s write %s%s\n",
                  svc->getUUID().toString().c_str(), notify->getUUID().toString().c_str(),
                  write->getUUID().toString().c_str(), known ? " (known ELM service)" : "");
    if (!bestNotify || (known && !bestKnown)) {
      bestNotify = notify;
      bestWrite = write;
      bestKnown = known;
    }
  }

  if (!bestNotify) {
    Serial.println("No serial service found on the dongle");
    return false;
  }

  chWrite = bestWrite;
  return bestNotify->subscribe(bestNotify->canNotify(), onNotify);
}

static bool findAndConnect() {
  Serial.println("Scanning for OBD dongle...");
  NimBLEScan *scan = NimBLEDevice::getScan();
  NimBLEScanResults results = scan->getResults(3000, false);

  const NimBLEAdvertisedDevice *found = nullptr;
  for (int i = 0; i < results.getCount(); i++) {
    const NimBLEAdvertisedDevice *dev = results.getDevice(i);
    if (dev->haveName()) {
      Serial.printf("  seen %s \"%s\" rssi=%d\n", dev->getAddress().toString().c_str(),
                    dev->getName().c_str(), dev->getRSSI());
    }
    if (!found && isDongle(dev)) found = dev;
  }
  if (!found) return false;

  Serial.printf("Connecting to %s \"%s\"\n", found->getAddress().toString().c_str(),
                found->getName().c_str());
  if (!client->connect(found)) {
    Serial.println("Connect failed");
    return false;
  }
  if (!subscribeSerialService()) {
    client->disconnect();
    return false;
  }
  Serial.println("Connected and subscribed");
  return true;
}

// ---------- ELM327 ----------
// Sends a command and collects the reply up to the '>' prompt.
static bool elmCommand(const char *cmd, String &reply, uint32_t timeoutMs = 1000) {
  reply = "";
  if (!client->isConnected()) return false;

  xStreamBufferReset(rxStream);
  String line = String(cmd) + "\r";
  if (!chWrite->writeValue((const uint8_t *)line.c_str(), line.length(), !chWrite->canWriteNoResponse())) {
    return false;
  }

  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    uint8_t buf[64];
    size_t n = xStreamBufferReceive(rxStream, buf, sizeof(buf), pdMS_TO_TICKS(20));
    for (size_t i = 0; i < n; i++) {
      if (buf[i] == '>') return true;
      if (buf[i] != 0) reply += (char)buf[i];
    }
    if (!client->isConnected()) return false;
  }
  Serial.printf("ELM timeout on %s\n", cmd);
  return false;
}

static int hexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// Extracts the data bytes that follow the positive response (e.g. 61 D9) in an ELM reply.
// Handles single frames and CAN auto-formatted multi-frame replies ("0: ..", "1: ..")
// with headers off. Returns the number of data bytes, or -1 if there is no positive response.
static int parseResponse(const String &reply, uint8_t service, uint8_t pid, uint8_t *out, int maxLen) {
  uint8_t bytes[128];
  int count = 0;

  int pos = 0;
  while (pos < (int)reply.length()) {
    int end = reply.indexOf('\r', pos);
    if (end < 0) end = reply.length();
    String l = reply.substring(pos, end);
    pos = end + 1;
    l.trim();
    if (l.length() == 0) continue;

    // Multi-frame byte count line, e.g. "00E"
    if (l.length() <= 3 && l.indexOf(' ') < 0) continue;

    // Multi-frame index prefix, e.g. "0: "
    int colon = l.indexOf(':');
    if (colon >= 0) l = l.substring(colon + 1);

    // Skip text lines such as "NO DATA", "CAN ERROR" or "SEARCHING..."
    bool isData = true;
    for (unsigned i = 0; i < l.length(); i++) {
      if (l[i] != ' ' && hexValue(l[i]) < 0) {
        isData = false;
        break;
      }
    }
    if (!isData) continue;

    int hi = -1;
    for (unsigned i = 0; i < l.length(); i++) {
      int v = hexValue(l[i]);
      if (v < 0) continue;
      if (hi < 0) {
        hi = v;
      } else {
        if (count < (int)sizeof(bytes)) bytes[count++] = (uint8_t)(hi << 4 | v);
        hi = -1;
      }
    }
  }

  for (int i = 0; i + 1 < count; i++) {
    if (bytes[i] == service && bytes[i + 1] == pid) {
      int n = 0;
      for (int j = i + 2; j < count && n < maxLen; j++) out[n++] = bytes[j];
      return n;
    }
  }
  return -1;
}

// Sends a request and returns its data bytes, or -1 on failure
static int request(int16_t header, const char *cmd, uint8_t *out, int maxLen, uint32_t timeoutMs = 1000) {
  String reply;
  if (header != currentHeader) {
    const char *setHeader = header == 0x7E1 ? "ATSH7E1" : "ATSH7DF";
    if (!elmCommand(setHeader, reply)) return -1;
    currentHeader = header;
  }

  if (!elmCommand(cmd, reply, timeoutMs)) return -1;

  uint8_t service = (uint8_t)(hexValue(cmd[0]) << 4 | hexValue(cmd[1])) + 0x40;
  uint8_t pid = (uint8_t)(hexValue(cmd[2]) << 4 | hexValue(cmd[3]));
  int n = parseResponse(reply, service, pid, out, maxLen);
  if (n < 0) {
    String compact = reply;
    compact.replace("\r", " | ");
    Serial.printf("%s -> %s\n", cmd, compact.c_str());
  }
  return n;
}

static bool initElm() {
  static const char *const INIT[] = {
    "ATE0",   // echo off
    "ATL0",   // no linefeeds
    "ATS1",   // spaces between bytes
    "ATH0",   // no headers
    "ATAT1",  // adaptive timing
    "ATSP6",  // ISO 15765-4 CAN 11-bit 500 kbps
    "ATCAF1", // CAN auto formatting (ISO-TP)
  };
  String reply;

  // ATZ resets the ELM, it takes up to a second
  if (!elmCommand("ATZ", reply, 3000)) return false;
  Serial.printf("ELM: %s\n", reply.c_str());

  for (const char *cmd : INIT) {
    if (!elmCommand(cmd, reply)) return false;
  }
  currentHeader = 0x7DF;
  return true;
}

// Reads MAP scaling (014F byte D); 1.0 when not supported
static void readMapScaling() {
  uint8_t d[8];
  int n = request(0x7DF, "014F", d, sizeof(d), 3000);
  mapScale = (n >= 4 && d[3] != 0) ? (10.0f * d[3] / 255.0f) : 1.0f;
  Serial.printf("MAP scale: %.3f\n", mapScale);
}

// Marks a poll result; returns true when the value should be cleared
static bool pollFailed(Poll &p) {
  if (p.failures < 255) p.failures++;
  return p.failures >= MAX_FAILURES;
}

static void updateBoost() {
  if (packet.map_kpa == OBD_NO_VALUE || packet.baro_kpa == OBD_NO_VALUE) {
    packet.boost = OBD_NO_VALUE;
  } else {
    packet.boost = packet.map_kpa - packet.baro_kpa;  // kPa -> centi-bar is 1:1
  }
}

static void pollOnce() {
  uint32_t now = millis();
  uint8_t d[16];

  // Gear and lockup every cycle
  int n = request(0x7E1, "21DA", d, sizeof(d));
  if (n >= 1) {
    pollGear.failures = 0;
    packet.gear = d[0] & 0x07;
    packet.lockup = (d[0] >> 4) & 0x01;
  } else if (pollFailed(pollGear)) {
    packet.gear = OBD_NO_VALUE;
    packet.lockup = OBD_NO_VALUE;
  }

  if (pollAtf.lastMs == 0 || now - pollAtf.lastMs >= ATF_POLL_MS) {
    pollAtf.lastMs = now;
    n = request(0x7E1, "21D9", d, sizeof(d));
    if (n >= 7) {
      pollAtf.failures = 0;
      packet.atf_pan = (int16_t)d[4] - 40;
      packet.atf_outlet = (int16_t)d[6] - 40;
    } else if (pollFailed(pollAtf)) {
      packet.atf_pan = OBD_NO_VALUE;
      packet.atf_outlet = OBD_NO_VALUE;
    }
  }

  n = request(0x7DF, "010B", d, sizeof(d));
  if (n >= 1) {
    pollMap.failures = 0;
    packet.map_kpa = (int16_t)lroundf(d[0] * mapScale);
  } else if (pollFailed(pollMap)) {
    packet.map_kpa = OBD_NO_VALUE;
  }

  if (pollBaro.lastMs == 0 || now - pollBaro.lastMs >= BARO_POLL_MS) {
    pollBaro.lastMs = now;
    n = request(0x7DF, "0133", d, sizeof(d));
    if (n >= 1) {
      pollBaro.failures = 0;
      packet.baro_kpa = d[0];
    } else if (pollFailed(pollBaro)) {
      packet.baro_kpa = OBD_NO_VALUE;
    }
  }

  updateBoost();

  bool anyOk = pollGear.failures == 0 || pollAtf.failures == 0 || pollMap.failures == 0;
  packet.status = anyOk ? OBD_STATUS_OK : OBD_STATUS_NO_ECU;
}

static void resetPolls() {
  pollGear = {};
  pollAtf = {};
  pollMap = {};
  pollBaro = {};
  clearValues();
}

// ---------- Arduino ----------
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("OBD connector: ELM327 BLE -> ESP-NOW");

  clearValues();
  packet.status = OBD_STATUS_NO_DONGLE;

  if (!initEspNow()) {
    delay(1000);
    ESP.restart();
  }

  rxStream = xStreamBufferCreate(1024, 1);

  NimBLEDevice::init("OBD-Connector");
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);

  client = NimBLEDevice::createClient();
  client->setClientCallbacks(&clientCallbacks, false);
  client->setConnectTimeout(5000);
}

void loop() {
  uint32_t now = millis();

  if (state != State::Scanning && !client->isConnected()) {
    Serial.println("Dongle lost, scanning again");
    state = State::Scanning;
  }

  switch (state) {
  case State::Scanning:
    packet.status = OBD_STATUS_NO_DONGLE;
    resetPolls();
    sendPacket();
    if (findAndConnect()) state = State::Initializing;
    break;

  case State::Initializing:
    packet.status = OBD_STATUS_NO_ECU;
    sendPacket();
    if (initElm()) {
      readMapScaling();
      state = State::Polling;
    } else {
      Serial.println("ELM init failed");
      client->disconnect();
      state = State::Scanning;
    }
    break;

  case State::Polling:
    pollOnce();
    if (now - lastSendMs >= SEND_INTERVAL_MS) {
      sendPacket();
      Serial.printf("status=%d gear=%d lockup=%d atf=%d/%d map=%d baro=%d boost=%d\n",
                    packet.status, packet.gear, packet.lockup, packet.atf_pan,
                    packet.atf_outlet, packet.map_kpa, packet.baro_kpa, packet.boost);
    }
    // Vehicle not answering (ignition off): poll slowly but keep pinging the monitor
    if (packet.status != OBD_STATUS_OK) {
      delay(STATUS_INTERVAL_MS);
      sendPacket();
    }
    break;
  }
}
