// OBD connector: talks to a BLE ELM327 dongle, polls transmission and boost readings
// from the Land Cruiser and forwards them over ESP-NOW to the ToyotaMonitor (via the C6 modem).
//
// Requests (see "Toyota Techstream PIDs.pdf"):
//   21DA @ 7E1  gear (A & 7) and torque converter lockup (bit 4 of A)   - experimental on VDJ200
//   21D9 @ 7E1  ATF pan temperature (E - 40) and converter outlet (G - 40)
//   010B @ 7DF  manifold absolute pressure, scaled by 014F byte D if supported
//   0133 @ 7DF  barometric pressure
//   2105 @ 7E0  coolant temperature (A - 40), every 3 s
//   2142 @ 7E0  starter battery voltage (A·B, mV), every 3 s
// The Right to Readout file's other engine blocks (2146, 2162, 216D, 2170, 2171, 2177, 2178)
// get no answer at all from this ECU, engine running or not (checked 2026-11-15).
// Fault codes (DtcPacket), when the vehicle starts answering and then every minute:
//   0101, 03, 07 @ 7E0 (engine) and 03, 07 @ 7E1 (gearbox); 1800FF00 or 13 @ 7B0 (ABS/VSC)
//
// Target board: standard ESP32.

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <esp_timer.h>
#include <freertos/stream_buffer.h>
#include <Preferences.h>
#include "ContractsInclude.hpp"
#include "../Shared/Maintenance.hpp"

using namespace Contracts;

// -------------------- USER CONFIG --------------------
// The dongle's BLE address is Vehicle::OBD_DONGLE_MAC (Shared/Vehicles/Toyota.hpp). Empty =
// auto-detect by name or by a known ELM327 BLE service. Every device seen during the scan is logged.
static const char *DONGLE_MAC = Vehicle::OBD_DONGLE_MAC;

static const char *const DONGLE_NAME_HINTS[] = {
  "OBD", "ELM", "VLINK", "V-LINK", "VGATE", "VEEPEAK", "KONNWEI", "IOS-", "ANDROID-"
};

static const uint32_t ATF_POLL_MS = 2000;      // temperatures change slowly
static const uint32_t BARO_POLL_MS = 30000;
static const uint32_t ENGINE_POLL_MS = 3000;   // coolant and starter battery
static const uint32_t SEND_INTERVAL_MS = 250;  // max ESP-NOW update rate while polling
static const uint32_t STATUS_INTERVAL_MS = 1000;  // ping rate while not polling
static const uint8_t  MAX_FAILURES = 3;        // consecutive failures before a value is cleared
static const uint32_t DTC_SCAN_MS = 60000;     // fault code rescan interval
static const uint32_t DTC_SEND_MS = 5000;      // repeat rate of the last fault code report
static const uint32_t ELM_REINIT_MS = 20000;   // no ECU answer this long: reset and set up the ELM again
static const uint8_t  MAX_ELM_TIMEOUTS = 3;    // ELM silent (no prompt) this many times in a row: reconnect
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
static volatile uint32_t lastSendMs = 0;

// The loop blocks for many seconds at a time (BLE scan + connect attempts, ELM timeouts, the
// fault code scan), longer than the dashboard's 15 s link timeout. A timer therefore repeats
// the last published packet every second, and restarts the board if the loop hangs outright.
static const uint32_t STALE_MS = 5000;        // published values older than this are not resent as OK
static const uint32_t LOOP_HANG_MS = 90000;   // no loop iteration this long: restart
static portMUX_TYPE publishMux = portMUX_INITIALIZER_UNLOCKED;
static ObdPacket published;                  // copy of the last packet the loop sent
static volatile uint32_t publishedMs = 0;
static volatile uint32_t lastLoopMs = 0;

static float mapScale = 1.0f;   // from 014F, 1.0 if not supported
static int16_t currentHeader = 0;
static uint8_t elmTimeouts = 0;    // consecutive commands without a '>' prompt
static uint32_t lastInitMs = 0;    // last ELM setup, or last time the vehicle answered
static bool resetupLogged = false; // one EV_ELM_RESETUP per quiet spell, not one every 20 s

struct Poll {
  uint32_t lastMs;
  uint8_t failures;
};
static Poll pollGear = {};
static Poll pollAtf = {};
static Poll pollMap = {};
static Poll pollBaro = {};
static Poll pollEngine = {};   // coolant + battery

static DtcPacket dtc;
static bool dtcValid = false;       // dtc holds a scan from the current ignition cycle
static uint32_t lastDtcScanMs = 0;
static uint32_t lastDtcSendMs = 0;

// ---------- Event log (survives restarts) ----------
// Connection events kept in flash, so a trip can be analysed without a laptop attached the whole
// time. Read with "LOG" on the USB serial port (also printed at boot), "LOG CLEAR" empties it.
enum EventCode : uint8_t {
  EV_BOOT = 1,           // detail: esp_reset_reason()
  EV_HANG_RESTART,       // the loop stopped for LOOP_HANG_MS; logged on the next boot
  EV_CONNECTED,          // detail: connect attempt that succeeded
  EV_CONNECT_FAILED,     // detail: NimBLE error of the last attempt
  EV_BLE_DISCONNECT,     // detail: disconnect reason (0x213 dongle ended it, 0x208 supervision timeout, 0x216 we ended it)
  EV_ELM_SILENT,         // reconnecting after MAX_ELM_TIMEOUTS timeouts in a row
  EV_ELM_SETUP_FAILED,   // detail: 0 first setup, 1 re-setup with no ECU answer
  EV_ECU_LOST,           // vehicle stopped answering (ignition off, or a problem)
  EV_ECU_BACK,
  EV_ELM_RESETUP,        // first re-setup after the ECU went quiet
  EV_RELEASED,           // phone switch on
};

struct PACKED LogEntry {
  uint16_t boot;       // boot counter
  uint32_t sec;        // seconds since that boot
  uint8_t code;
  int16_t detail;
  uint16_t timeouts;   // ELM timeouts since boot, at the time of the event
  uint16_t repeats;    // identical consecutive events folded into this one
};
static const int LOG_SIZE = 96;
struct PACKED EventLog {
  uint16_t head;       // next slot
  uint16_t count;
  LogEntry entries[LOG_SIZE];
};
static EventLog eventLog;
static Preferences logPrefs;
static uint16_t bootCount = 0;
static uint16_t timeoutsSinceBoot = 0;
RTC_NOINIT_ATTR static uint32_t hangMarker;   // set just before a hang restart
static const uint32_t HANG_MAGIC = 0x48414E47;

static const char *eventName(uint8_t code) {
  switch (code) {
  case EV_BOOT:             return "boot (reset reason)";
  case EV_HANG_RESTART:     return "restarted after the loop hung";
  case EV_CONNECTED:        return "connected (attempt)";
  case EV_CONNECT_FAILED:   return "connect failed (error)";
  case EV_BLE_DISCONNECT:   return "BLE disconnect (reason)";
  case EV_ELM_SILENT:       return "ELM silent, reconnecting";
  case EV_ELM_SETUP_FAILED: return "ELM setup failed (0 first, 1 re-setup)";
  case EV_ECU_LOST:         return "ECU stopped answering";
  case EV_ECU_BACK:         return "ECU answering";
  case EV_ELM_RESETUP:      return "ELM set up again (no ECU answer)";
  case EV_RELEASED:         return "released for the phone";
  default:                  return "?";
  }
}

static void logEvent(uint8_t code, int16_t detail = 0) {
  uint32_t sec = millis() / 1000;
  Serial.printf("[%lu] EVENT %s %d\n", millis(), eventName(code), detail);
  if (eventLog.count > 0) {
    LogEntry &last = eventLog.entries[(eventLog.head + LOG_SIZE - 1) % LOG_SIZE];
    if (last.boot == bootCount && last.code == code && last.detail == detail) {
      if (last.repeats < 0xFFFF) last.repeats++;
      last.sec = sec;
      last.timeouts = timeoutsSinceBoot;
      logPrefs.putBytes("log", &eventLog, sizeof(eventLog));
      return;
    }
  }
  eventLog.entries[eventLog.head] = { bootCount, sec, code, detail, timeoutsSinceBoot, 0 };
  eventLog.head = (eventLog.head + 1) % LOG_SIZE;
  if (eventLog.count < LOG_SIZE) eventLog.count++;
  logPrefs.putBytes("log", &eventLog, sizeof(eventLog));
}

static void printLog() {
  Serial.printf("Event log, %u entries (boot #, time since that boot, event, detail, ELM timeouts so far):\n",
                eventLog.count);
  for (int i = 0; i < eventLog.count; i++) {
    const LogEntry &e = eventLog.entries[(eventLog.head + LOG_SIZE - eventLog.count + i) % LOG_SIZE];
    Serial.printf("  boot %u  %2lu:%02lu:%02lu  %s %d  timeouts=%u%s", e.boot, e.sec / 3600,
                  e.sec / 60 % 60, e.sec % 60, eventName(e.code), e.detail, e.timeouts,
                  e.repeats ? "" : "\n");
    if (e.repeats) Serial.printf("  (x%u, last shown)\n", e.repeats + 1);
  }
}

static void initLog() {
  logPrefs.begin("obdlog", false);
  bootCount = logPrefs.getUShort("boots", 0) + 1;
  logPrefs.putUShort("boots", bootCount);
  if (logPrefs.getBytes("log", &eventLog, sizeof(eventLog)) != sizeof(eventLog)
      || eventLog.head >= LOG_SIZE || eventLog.count > LOG_SIZE) {
    memset(&eventLog, 0, sizeof(eventLog));
  }
  if (hangMarker == HANG_MAGIC) logEvent(EV_HANG_RESTART);
  hangMarker = 0;
  logEvent(EV_BOOT, (int16_t)esp_reset_reason());
  printLog();
}

// ---------- ESP-NOW ----------
void onEspNowSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
}

// Phone switch on the dashboard: while it repeats OBD_CMD_RELEASE, the connector stays off the
// dongle so the phone app can connect; once the repeats stop it reconnects after PHONE_RELEASE_MS
static volatile uint32_t releaseUntilMs = 0;   // 0 = not released

// Runs in the Wi-Fi task
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (Maintenance::onEspNow(info, data, len)) return;   // OTA maintenance command
  if (!info || len != sizeof(ObdCommandPacket)) return;
  if (memcmp(info->src_addr, Vehicle::MODEM_MAC, 6) != 0) return;
  ObdCommandPacket cmd;
  memcpy(&cmd, data, sizeof(cmd));
  if (cmd.type != TYPE_OBD_COMMAND || cmd.command != OBD_CMD_RELEASE) return;
  uint32_t until = cmd.arg ? millis() + PHONE_RELEASE_MS : 0;
  releaseUntilMs = cmd.arg && until == 0 ? 1 : until;
}

static bool released() {
  static bool was = false;
  uint32_t until = releaseUntilMs;
  bool now = until != 0 && (int32_t)(millis() - until) < 0;
  if (now != was) Serial.println(now ? "Releasing the dongle for the phone app" : "Release over, reconnecting");
  was = now;
  return now;
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
  esp_now_register_recv_cb(onEspNowRecv);
  Maintenance::begin("ObdConnector", MAINT_OBD);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, Vehicle::DISPLAY_MAC, 6);
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
  portENTER_CRITICAL(&publishMux);
  published = packet;
  publishedMs = millis();
  portEXIT_CRITICAL(&publishMux);
  esp_now_send(Vehicle::DISPLAY_MAC, (uint8_t *)&packet, sizeof(packet));
  lastSendMs = millis();
}

// Runs in the esp_timer task every second
static void pingTimerCb(void *) {
  if (Maintenance::requested) return;   // the loop is waiting for an OTA upload, not hung
  uint32_t now = millis();
  if (now - lastLoopMs > LOOP_HANG_MS) {
    Serial.printf("[%lu] Loop stuck for %lu s, restarting\n", now, (now - lastLoopMs) / 1000);
    Serial.flush();
    hangMarker = HANG_MAGIC;   // logged on the next boot; no flash writes from this task
    esp_restart();
  }
  if (now - lastSendMs < STATUS_INTERVAL_MS) return;

  ObdPacket copy;
  portENTER_CRITICAL(&publishMux);
  copy = published;
  uint32_t age = now - publishedMs;
  portEXIT_CRITICAL(&publishMux);
  if (copy.type != TYPE_OBD) return;   // nothing published yet

  // The loop is stuck in a slow request: the values are old, so don't present them as live
  if (age > STALE_MS && copy.status == OBD_STATUS_OK) {
    copy.status = OBD_STATUS_NO_ECU;
    copy.gear = copy.lockup = copy.atf_pan = copy.atf_outlet = OBD_NO_VALUE;
    copy.map_kpa = copy.baro_kpa = copy.boost = copy.coolant = copy.battery_cv = OBD_NO_VALUE;
  }
  esp_now_send(Vehicle::DISPLAY_MAC, (uint8_t *)&copy, sizeof(copy));
  lastSendMs = now;
}

void clearValues() {
  packet.gear = OBD_NO_VALUE;
  packet.lockup = OBD_NO_VALUE;
  packet.atf_pan = OBD_NO_VALUE;
  packet.atf_outlet = OBD_NO_VALUE;
  packet.map_kpa = OBD_NO_VALUE;
  packet.baro_kpa = OBD_NO_VALUE;
  packet.boost = OBD_NO_VALUE;
  packet.coolant = OBD_NO_VALUE;
  packet.battery_cv = OBD_NO_VALUE;
}

// ---------- BLE ----------
static volatile int disconnectReason = 0;   // set by onDisconnect, 0 = nothing to log

class ClientCallbacks : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient *c, int reason) override {
    Serial.printf("[%lu] Dongle disconnected (reason %d)\n", millis(), reason);
    disconnectReason = reason ? reason : -1;   // logged by loop(); flash writes don't belong in the BLE task
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

  // A named dongle wins over a device matched by its service alone; then the strongest signal
  const NimBLEAdvertisedDevice *found = nullptr;
  bool foundNamed = false;
  for (int i = 0; i < results.getCount(); i++) {
    const NimBLEAdvertisedDevice *dev = results.getDevice(i);
    bool candidate = isDongle(dev);
    if (dev->haveName() || candidate) {
      Serial.printf("  seen %s \"%s\" rssi=%d%s", dev->getAddress().toString().c_str(),
                    dev->getName().c_str(), dev->getRSSI(), candidate ? " CANDIDATE" : "");
      for (int s = 0; s < dev->getServiceUUIDCount(); s++) {
        Serial.printf(" svc %s", dev->getServiceUUID(s).toString().c_str());
      }
      Serial.println();
    }
    if (!candidate) continue;
    bool named = dev->haveName() && nameMatches(dev->getName());
    if (!found || (named && !foundNamed) || (named == foundNamed && dev->getRSSI() > found->getRSSI())) {
      found = dev;
      foundNamed = named;
    }
  }
  if (!found) return false;

  // A failed attempt is retried on the same address straight away, before rescanning
  NimBLEAddress addr = found->getAddress();
  Serial.printf("Connecting to %s \"%s\"\n", addr.toString().c_str(), found->getName().c_str());
  bool connected = false;
  int attempt = 1, lastErr = 0;
  for (; attempt <= 3; attempt++) {
    // No MTU exchange from our side: the Vgate starts its own, and NimBLE's then fails the
    // connect with BLE_HS_EALREADY. ELM replies arrive in several notifications either way.
    connected = client->connect(addr, true, false, false);
    if (connected) break;
    lastErr = client->getLastError();
    Serial.printf("[%lu] Connect attempt %d failed (error %d)\n", millis(), attempt, lastErr);
    if (lastErr == BLE_HS_EALREADY) ble_gap_conn_cancel();   // a previous attempt is still pending
    delay(500);
  }
  if (!connected) {
    logEvent(EV_CONNECT_FAILED, (int16_t)lastErr);
    return false;
  }
  if (!subscribeSerialService()) {
    logEvent(EV_CONNECT_FAILED, -1);   // connected, but no serial service
    client->disconnect();
    return false;
  }
  Serial.println("Connected and subscribed");
  logEvent(EV_CONNECTED, (int16_t)attempt);
  return true;
}

// ---------- ELM327 ----------
// Sends a command and collects the reply up to the '>' prompt.
// Echo is on (ATE1), so the reply starts after the echoed command. Anything before the echo
// belongs to an earlier command, e.g. the '>' of a request that timed out and finished late;
// without this, one late prompt shifted every later reply onto the next command.
static bool elmCommand(const char *cmd, String &reply, uint32_t timeoutMs = 1500, bool expectEcho = true) {
  reply = "";
  if (!client->isConnected()) return false;

  xStreamBufferReset(rxStream);
  String line = String(cmd) + "\r";
  if (!chWrite->writeValue((const uint8_t *)line.c_str(), line.length(), !chWrite->canWriteNoResponse())) {
    return false;
  }

  bool echoSeen = !expectEcho;
  String pending;   // current line while waiting for the echo
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    uint8_t buf[64];
    size_t n = xStreamBufferReceive(rxStream, buf, sizeof(buf), pdMS_TO_TICKS(20));
    for (size_t i = 0; i < n; i++) {
      if (!echoSeen) {
        char c = (char)buf[i];
        if (c == '\r' || c == '\n') {
          pending.trim();
          if (pending.equalsIgnoreCase(cmd)) echoSeen = true;
          else if (pending.length()) Serial.printf("ELM stale output before %s: %s\n", cmd, pending.c_str());
          pending = "";
        } else if (c == '>') {
          pending = "";   // prompt of an earlier command
        } else if (c) {
          pending += c;
        }
        continue;
      }
      if (buf[i] == '>') {
        elmTimeouts = 0;
        return true;
      }
      if (buf[i] != 0) reply += (char)buf[i];
    }
    if (!client->isConnected()) return false;
  }
  if (elmTimeouts < 255) elmTimeouts++;
  if (timeoutsSinceBoot < 0xFFFF) timeoutsSinceBoot++;
  String compact = reply;
  compact.replace("\r", " | ");
  Serial.printf("[%lu] ELM timeout on %s (%u in a row), partial reply: %s\n", millis(), cmd,
                elmTimeouts, compact.c_str());
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
// with headers off. pid < 0 matches the service byte alone (e.g. 43 for mode 03).
// Returns the number of data bytes, or -1 if there is no positive response.
static int parseResponse(const String &reply, uint8_t service, int pid, uint8_t *out, int maxLen) {
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

  int skip = pid < 0 ? 1 : 2;
  for (int i = 0; i + skip <= count; i++) {
    if (bytes[i] == service && (pid < 0 || (i + 1 < count && bytes[i + 1] == pid))) {
      int n = 0;
      for (int j = i + skip; j < count && n < maxLen; j++) out[n++] = bytes[j];
      return n;
    }
  }
  return -1;
}

// Addresses requests to an ECU. The ELM filters replies to 7E8-7EF by itself;
// other modules (e.g. ABS 7B0 -> 7B8) need an explicit receive address.
static bool setHeader(int16_t header) {
  if (header == currentHeader) return true;
  String reply;
  char cmd[12];
  snprintf(cmd, sizeof(cmd), "ATSH%03X", header);
  if (!elmCommand(cmd, reply)) return false;
  if (header == 0x7DF || (header & 0xFF8) == 0x7E0) {
    snprintf(cmd, sizeof(cmd), "ATAR");
  } else {
    snprintf(cmd, sizeof(cmd), "ATCRA%03X", header + 8);
  }
  if (!elmCommand(cmd, reply)) return false;
  currentHeader = header;
  return true;
}

// Sends cmd and returns the data bytes after the response service byte (and the
// echoed pid when pid >= 0), or -1 on failure
static int requestRaw(int16_t header, const char *cmd, uint8_t service, int pid,
                      uint8_t *out, int maxLen, uint32_t timeoutMs = 1500) {
  String reply;
  if (!setHeader(header)) return -1;
  if (!elmCommand(cmd, reply, timeoutMs)) return -1;

  int n = parseResponse(reply, service, pid, out, maxLen);
  if (n < 0) {
    String compact = reply;
    compact.replace("\r", " | ");
    Serial.printf("%03X %s -> %s\n", header, cmd, compact.c_str());
  }
  return n;
}

// Sends a service + pid request (e.g. 21D9) and returns its data bytes, or -1 on failure
static int request(int16_t header, const char *cmd, uint8_t *out, int maxLen, uint32_t timeoutMs = 1500) {
  uint8_t service = (uint8_t)(hexValue(cmd[0]) << 4 | hexValue(cmd[1])) + 0x40;
  uint8_t pid = (uint8_t)(hexValue(cmd[2]) << 4 | hexValue(cmd[3]));
  return requestRaw(header, cmd, service, pid, out, maxLen, timeoutMs);
}

static bool initElm() {
  static const char *const INIT[] = {
    "ATE1",   // echo on: elmCommand matches each reply to its command by the echo
    "ATL0",   // no linefeeds
    "ATS1",   // spaces between bytes
    "ATH0",   // no headers
    "ATAT1",  // adaptive timing
    "ATSP6",  // ISO 15765-4 CAN 11-bit 500 kbps
    "ATCAF1", // CAN auto formatting (ISO-TP)
  };
  String reply;

  // ATZ resets the ELM, it takes up to a second
  // Echo may be on or off before the reset, so ATZ doesn't wait for one
  if (!elmCommand("ATZ", reply, 3000, false)) return false;
  delay(200);
  Serial.printf("ELM: %s\n", reply.c_str());

  for (const char *cmd : INIT) {
    if (!elmCommand(cmd, reply)) return false;
  }
  currentHeader = 0x7DF;
  elmTimeouts = 0;
  lastInitMs = millis();
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

  // Coolant and starter battery, both slow
  if (pollEngine.lastMs == 0 || now - pollEngine.lastMs >= ENGINE_POLL_MS) {
    pollEngine.lastMs = now;
    int c = request(0x7E0, "2105", d, sizeof(d));
    int b = request(0x7E0, "2142", d + 4, sizeof(d) - 4);
    if (c >= 1) packet.coolant = (int16_t)d[0] - 40;
    if (b >= 2) packet.battery_cv = (int16_t)(((d[4] << 8) | d[5]) / 10);
    if (c >= 1 || b >= 2) {
      pollEngine.failures = 0;
    } else if (pollFailed(pollEngine)) {
      packet.coolant = OBD_NO_VALUE;
      packet.battery_cv = OBD_NO_VALUE;
    }
  }

  bool anyOk = pollGear.failures == 0 || pollAtf.failures == 0 || pollMap.failures == 0;
  packet.status = anyOk ? OBD_STATUS_OK : OBD_STATUS_NO_ECU;
}

static void resetPolls() {
  pollGear = {};
  pollAtf = {};
  pollMap = {};
  pollBaro = {};
  pollEngine = {};
  clearValues();
}

// ---------- Fault codes ----------
static void dtcAdd(uint16_t code, uint8_t source) {
  for (int i = 0; i < DTC_MAX_CODES; i++) {
    if (dtc.source[i] == DTC_UNUSED) {
      dtc.codes[i] = code;
      dtc.source[i] = source;
      return;
    }
  }
}

// OBD-II mode 03 (stored) or 07 (pending) on CAN: a count byte, then 2 bytes per code
static int16_t readObdCodes(int16_t header, const char *cmd, uint8_t source) {
  uint8_t d[64];
  uint8_t service = strcmp(cmd, "03") == 0 ? 0x43 : 0x47;
  int n = requestRaw(header, cmd, service, -1, d, sizeof(d), 2000);
  if (n < 1) return DTC_NO_ANSWER;
  int count = d[0];
  for (int i = 0; i < count && 2 + 2 * i < n; i++) {
    dtcAdd((uint16_t)(d[1 + 2 * i] << 8 | d[2 + 2 * i]), source);
  }
  return count;
}

// Toyota ABS/VSC ECU (KWP2000 on CAN). Untested on the VDJ200: tries readDTCByStatus,
// then readDiagnosticTroubleCodes, and logs what answers.
static int16_t readAbsCodes() {
  uint8_t d[64];
  int n = requestRaw(0x7B0, "1800FF00", 0x58, -1, d, sizeof(d), 2000);
  if (n >= 1) {
    // count, then code high, code low, status. The status should tell current codes
    // from history; logged until its bits are known for this ECU.
    for (int i = 0; i < d[0] && 3 + 3 * i <= n; i++) {
      dtcAdd((uint16_t)(d[1 + 3 * i] << 8 | d[2 + 3 * i]), DTC_ABS);
      Serial.printf("  ABS code %02X%02X status %02X\n", d[1 + 3 * i], d[2 + 3 * i], d[3 + 3 * i]);
    }
    Serial.printf("ABS 18 -> %d bytes, %d codes\n", n, d[0]);
    return d[0];
  }
  n = requestRaw(0x7B0, "13", 0x53, -1, d, sizeof(d), 2000);
  if (n >= 1) {
    // count, then 2 bytes per code
    for (int i = 0; i < d[0] && 2 + 2 * i < n; i++) {
      dtcAdd((uint16_t)(d[1 + 2 * i] << 8 | d[2 + 2 * i]), DTC_ABS);
    }
    Serial.printf("ABS 13 -> %d bytes, %d codes\n", n, d[0]);
    return d[0];
  }
  return DTC_NO_ANSWER;
}

static void scanDtc() {
  uint16_t scans = dtc.scan;
  dtc = {};
  dtc.type = TYPE_DTC;
  dtc.scan = scans + 1;
  memset(dtc.source, DTC_UNUSED, sizeof(dtc.source));

  uint8_t d[8];
  int n = request(0x7E0, "0101", d, sizeof(d));
  dtc.mil = n >= 1 ? (d[0] >> 7) : OBD_NO_VALUE;

  dtc.stored[DTC_ENGINE] = readObdCodes(0x7E0, "03", DTC_ENGINE);
  dtc.pending[DTC_ENGINE] = readObdCodes(0x7E0, "07", DTC_ENGINE | DTC_PENDING);
  dtc.stored[DTC_GEARBOX] = readObdCodes(0x7E1, "03", DTC_GEARBOX);
  dtc.pending[DTC_GEARBOX] = readObdCodes(0x7E1, "07", DTC_GEARBOX | DTC_PENDING);
  dtc.stored[DTC_ABS] = readAbsCodes();
  dtc.pending[DTC_ABS] = DTC_NO_ANSWER;

  Serial.printf("DTC scan %u: mil=%d engine=%d/%d gearbox=%d/%d abs=%d\n", dtc.scan, dtc.mil,
                dtc.stored[DTC_ENGINE], dtc.pending[DTC_ENGINE],
                dtc.stored[DTC_GEARBOX], dtc.pending[DTC_GEARBOX], dtc.stored[DTC_ABS]);
  for (int i = 0; i < DTC_MAX_CODES && dtc.source[i] != DTC_UNUSED; i++) {
    Serial.printf("  code %04X module %d%s\n", dtc.codes[i], dtc.source[i] & 0x7F,
                  dtc.source[i] & DTC_PENDING ? " pending" : "");
  }
}

static void sendDtc() {
  esp_now_send(Vehicle::DISPLAY_MAC, (uint8_t *)&dtc, sizeof(dtc));
  lastDtcSendMs = millis();
}

// ---------- Serial passthrough (for probing the dongle and the ECUs) ----------
// "@7E0 2104" sends 2104 to the engine ECU; "@ ATI" or "@ 0100" sends to the current header.
// The raw reply is printed as "RAW <cmd> -> <reply>". Runs between polls, once connected.
static void pollSerial() {
  static char line[48];
  static size_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') {
      if (len < sizeof(line) - 1) line[len++] = c;
      continue;
    }
    line[len] = 0;
    len = 0;
    if (strcasecmp(line, "LOG") == 0) {
      printLog();
      continue;
    }
    if (strcasecmp(line, "LOG CLEAR") == 0) {
      memset(&eventLog, 0, sizeof(eventLog));
      logPrefs.putBytes("log", &eventLog, sizeof(eventLog));
      Serial.println("Event log cleared");
      continue;
    }
    if (line[0] != '@') continue;
    if (state != State::Polling) {
      Serial.println("RAW not connected to the dongle");
      continue;
    }
    char *cmd = line + 1;
    unsigned header = 0;
    if (*cmd != ' ' && sscanf(cmd, "%x", &header) == 1) {
      if (!setHeader((int16_t)header)) {
        Serial.println("RAW header failed");
        continue;
      }
      while (*cmd && *cmd != ' ') cmd++;
    }
    while (*cmd == ' ') cmd++;
    String reply;
    bool ok = elmCommand(cmd, reply, 3000);
    reply.replace("\r", " | ");
    Serial.printf("RAW %03X %s -> %s%s\n", currentHeader, cmd, reply.c_str(), ok ? "" : " (timeout)");
  }
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
  initLog();

  lastLoopMs = millis();
  sendPacket();
  const esp_timer_create_args_t pingArgs = { .callback = pingTimerCb, .name = "obd_ping" };
  esp_timer_handle_t pingTimer;
  esp_timer_create(&pingArgs, &pingTimer);
  esp_timer_start_periodic(pingTimer, STATUS_INTERVAL_MS * 1000ULL);

  NimBLEDevice::init("OBD-Connector");
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);

  client = NimBLEDevice::createClient();
  client->setClientCallbacks(&clientCallbacks, false);
  client->setConnectTimeout(5000);
  // 30-50 ms interval, 4 s supervision timeout: relaxed for slow dongle BLE chips
  client->setConnectionParams(24, 40, 0, 400);
}

void loop() {
  Maintenance::loop();
  uint32_t now = millis();
  lastLoopMs = now;

  pollSerial();
  if (disconnectReason) {
    logEvent(EV_BLE_DISCONNECT, (int16_t)disconnectReason);
    disconnectReason = 0;
  }
  // Vehicle answering or not, logged on changes only
  static bool ecuAnswering = false;
  if ((packet.status == OBD_STATUS_OK) != ecuAnswering) {
    ecuAnswering = packet.status == OBD_STATUS_OK;
    logEvent(ecuAnswering ? EV_ECU_BACK : EV_ECU_LOST, packet.status);
  }

  static bool releaseLogged = false;
  if (released()) {
    if (!releaseLogged) logEvent(EV_RELEASED);
    releaseLogged = true;
    static uint32_t lastDisconnectMs = 0;   // the disconnect takes a moment to complete
    if (client->isConnected() && now - lastDisconnectMs > 2000) {
      Serial.println("Disconnecting from the dongle");
      client->disconnect();
      lastDisconnectMs = now;
    }
    state = State::Scanning;
    resetPolls();
    dtcValid = false;
    packet.status = OBD_STATUS_RELEASED;
    if (now - lastSendMs >= STATUS_INTERVAL_MS) sendPacket();
    delay(10);
    return;
  }
  releaseLogged = false;

  if (state != State::Scanning && !client->isConnected()) {
    Serial.println("Dongle lost, scanning again");
    state = State::Scanning;
  }

  switch (state) {
  case State::Scanning:
    packet.status = OBD_STATUS_NO_DONGLE;
    resetPolls();
    dtcValid = false;
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
      logEvent(EV_ELM_SETUP_FAILED, 0);
      client->disconnect();
      state = State::Scanning;
    }
    break;

  case State::Polling:
    pollOnce();
    // The ELM stopped answering at all (it reset or hung after a voltage dip): start over
    if (elmTimeouts >= MAX_ELM_TIMEOUTS) {
      Serial.printf("[%lu] ELM not answering, reconnecting\n", millis());
      logEvent(EV_ELM_SILENT, elmTimeouts);
      client->disconnect();
      state = State::Scanning;
      break;
    }
    if (now - lastSendMs >= SEND_INTERVAL_MS) {
      sendPacket();
      Serial.printf("status=%d gear=%d lockup=%d atf=%d/%d map=%d baro=%d boost=%d coolant=%d batt=%d\n",
                    packet.status, packet.gear, packet.lockup, packet.atf_pan,
                    packet.atf_outlet, packet.map_kpa, packet.baro_kpa, packet.boost,
                    packet.coolant, packet.battery_cv);
    }
    // Vehicle not answering (ignition off): poll slowly but keep pinging the monitor
    if (packet.status != OBD_STATUS_OK) {
      dtcValid = false;   // rescan on the next ignition
      // The ELM may have lost its setup (reset by a voltage dip while cranking, or a receive
      // filter left behind): set it up again rather than polling a confused ELM forever
      if (millis() - lastInitMs >= ELM_REINIT_MS) {
        Serial.printf("[%lu] No ECU answer for %lu s, setting up the ELM again\n", millis(),
                      ELM_REINIT_MS / 1000);
        if (!resetupLogged) logEvent(EV_ELM_RESETUP);
        resetupLogged = true;
        if (!initElm()) {
          Serial.println("ELM setup failed, reconnecting");
          logEvent(EV_ELM_SETUP_FAILED, 1);
          client->disconnect();
          state = State::Scanning;
          break;
        }
      }
      delay(STATUS_INTERVAL_MS);
      sendPacket();
      break;
    }
    lastInitMs = millis();
    resetupLogged = false;
    // Fault codes: scan when the vehicle starts answering, then every minute
    if (!dtcValid || now - lastDtcScanMs >= DTC_SCAN_MS) {
      if (!dtcValid) readMapScaling();   // 014F only answers with the ignition on
      scanDtc();
      dtcValid = true;
      lastDtcScanMs = millis();
      sendDtc();
    } else if (millis() - lastDtcSendMs >= DTC_SEND_MS) {
      sendDtc();
    }
    break;
  }
}
