// Heater connector: polls an HCalory diesel heater over BLE and forwards its status over
// ESP-NOW to the ToyotaMonitor (via the C6 modem). Runs the HeaterCommandPackets the dashboard
// sends through the modem (start, stop, mode, level / temperature) and acknowledges each one in
// HeaterPacket.ack_seq, and HEATER_CMD_RELEASE, which keeps it off the heater so the phone app can
// connect. Keeps the time of day on a DS1302 RTC (set it over USB serial with
// "T HH:MM" or "T HH:MM:SS") and runs the armed morning run: every day at run_at it starts the
// heater, unless the battery is low, and stops it HEATER_RUN_LENGTH_MIN after ignition.
//
// Protocol (same as github.com/evanfoster/hcalory-control, verified on the 8 kW unit):
//   service fff0, write fff2, notify fff1, no PIN
//   command 00 02 00 01 00 01 00 0E 04 00 00 09 + 8 x 00 + argument + checksum
//           argument 00 query, 01 stop, 02 start, 03 up, 04 down, 06 thermostat, 07 level
//   reply   39 bytes, from offset 20: state, mode, setting, ?, -, volts, sign?, case temp
//           (2 bytes, deci-degrees), sign?, cabin temp (2 bytes, deci-degrees), ...
//   Checksum = sum of the bytes from offset 8, both ways.
//
// Target board: standard ESP32.

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <Preferences.h>
#include "ContractsInclude.hpp"
#include "../Shared/Maintenance.hpp"
#include "../Shared/CarHubPeer.hpp"
#include "ds1302.hpp"

using namespace Contracts;

// -------------------- USER CONFIG --------------------
// The heater's BLE address is Vehicle::HEATER_BLE_MAC (Shared/Vehicles/Toyota.hpp)
static const uint32_t QUERY_INTERVAL_MS = 1000;   // the HCalory app also asks every second
static const uint32_t SEND_INTERVAL_MS = 1000;    // ESP-NOW ping rate, also while searching
static const uint32_t REPLY_TIMEOUT_MS = 10000;   // no reply for this long: reconnect
static const uint32_t SCAN_MS = 5000;
static const uint32_t WRITE_GAP_MS = 300;         // between BLE writes; back-to-back frames get lost
static const uint32_t QUERY_AFTER_CMD_MS = 500;   // first status query after a command
static const int16_t DEFAULT_RUN_AT = 7 * 60;      // morning run, minutes after midnight
static const int RTC_CLK_PIN = 25;
static const int RTC_DAT_PIN = 26;
static const int RTC_RST_PIN = 27;
static const uint32_t CLOCK_READ_MS = 1000;
static const int LED_PIN = 2;                      // blue LED on most DevKit boards
static const uint32_t IDENTIFY_MS = 10000;         // "I" over serial blinks it, to find this board
static const uint32_t RUN_CONNECT_WINDOW_MS = 10 * 60 * 1000;   // heater must be reachable by then
static const uint32_t RUN_RETRY_MS = 20000;        // resend start / stop if the state doesn't change
static const int RUN_START_TRIES = 3;
// -----------------------------------------------------

static const NimBLEUUID HEATER_SERVICE("fff0");
static const NimBLEUUID HEATER_WRITE("fff2");
static const NimBLEUUID HEATER_NOTIFY("fff1");

static const size_t FRAME_LEN = 39;
static const size_t CHECKSUM_FROM = 8;

struct Frame {
  uint8_t data[FRAME_LEN];
};

static NimBLEClient *client = nullptr;
static NimBLERemoteCharacteristic *chWrite = nullptr;
static QueueHandle_t frames = nullptr;   // replies from the notify callback

static HeaterPacket packet;
static uint32_t lastSendMs = 0;
static uint32_t lastQueryMs = 0;
static uint32_t lastReplyMs = 0;
static uint32_t lastWriteMs = 0;

static QueueHandle_t commands = nullptr;   // HeaterCommandPackets from the dashboard
static Preferences prefs;

// ---------- ESP-NOW ----------
// Runs in the Wi-Fi task: only commands from the modem or the CarHub are queued for loop()
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (Maintenance::onEspNow(info, data, len)) return;   // OTA maintenance command
  if (!info || len != sizeof(HeaterCommandPacket)) return;
  if (memcmp(info->src_addr, Vehicle::MODEM_MAC, 6) != 0 && !CarHubPeer::isCarHub(info->src_addr)) return;
  HeaterCommandPacket cmd;
  memcpy(&cmd, data, sizeof(cmd));
  if (cmd.type == TYPE_HEATER_COMMAND) xQueueSend(commands, &cmd, 0);
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

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, Vehicle::DISPLAY_MAC, 6);
  peerInfo.channel = 0;   // 0 = current Wi-Fi channel
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer!");
    return false;
  }
  if (!CarHubPeer::add()) Serial.println("Failed to add the CarHub peer");
  esp_now_register_recv_cb(onEspNowRecv);
  Maintenance::begin("HeaterConnector", MAINT_HEATER);
  return true;
}

void sendPacket() {
  packet.type = TYPE_HEATER;
  esp_now_send(Vehicle::DISPLAY_MAC, (uint8_t *)&packet, sizeof(packet));
  CarHubPeer::send(&packet, sizeof(packet));
  lastSendMs = millis();
}

// ---------- HCalory protocol ----------
static uint8_t checksum(const uint8_t *data, size_t len) {
  uint8_t sum = 0;
  for (size_t i = CHECKSUM_FROM; i < len; i++) sum += data[i];
  return sum;
}

static bool sendCommand(uint8_t arg) {
  uint8_t cmd[] = { 0x00, 0x02, 0x00, 0x01, 0x00, 0x01, 0x00, 0x0E, 0x04, 0x00, 0x00, 0x09,
                    0, 0, 0, 0, 0, 0, 0, 0, arg, 0x00 };
  cmd[sizeof(cmd) - 1] = checksum(cmd, sizeof(cmd) - 1);

  // The heater drops a frame that follows another one too closely
  uint32_t since = millis() - lastWriteMs;
  if (since < WRITE_GAP_MS) delay(WRITE_GAP_MS - since);
  bool ok = chWrite->writeValue(cmd, sizeof(cmd), !chWrite->canWriteNoResponse());
  lastWriteMs = millis();
  if (arg != 0x00) {
    Serial.printf("TX %s ", ok ? "ok" : "FAILED");
    for (uint8_t b : cmd) Serial.printf("%02X ", b);
    Serial.println();
  }
  return ok;
}

static bool sendQuery() { return sendCommand(0x00); }

// Temperature with a sign byte in front (01 = negative, as in diesel-heater-ble;
// not yet seen below zero)
static int16_t signedTemp(const uint8_t *f, int signAt) {
  int16_t v = (int16_t)((f[signAt + 1] << 8) | f[signAt + 2]);
  return f[signAt] == 0x01 ? -v : v;
}

static bool parseFrame(const uint8_t *f) {
  if (f[0] != 0x00 || f[1] != 0x03) return false;
  if (checksum(f, FRAME_LEN - 1) != f[FRAME_LEN - 1]) return false;

  packet.state = f[20];
  packet.mode = f[21];
  packet.setting = f[22];
  packet.volts = f[25];
  packet.case_temp = signedTemp(f, 26);
  packet.cabin_temp = signedTemp(f, 29);
  return true;
}

// ---------- BLE ----------
class ClientCallbacks : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient *c, int reason) override {
    Serial.printf("Heater disconnected (reason %d)\n", reason);
  }
};
static ClientCallbacks clientCallbacks;

static void onNotify(NimBLERemoteCharacteristic *, uint8_t *data, size_t len, bool) {
  if (len != FRAME_LEN) return;
  Frame frame;
  memcpy(frame.data, data, FRAME_LEN);
  xQueueSend(frames, &frame, 0);
}

static bool findAndConnect() {
  Serial.printf("Scanning for heater %s...\n", Vehicle::HEATER_BLE_MAC);
  NimBLEScan *scan = NimBLEDevice::getScan();
  NimBLEScanResults results = scan->getResults(SCAN_MS, false);

  const NimBLEAdvertisedDevice *found = nullptr;
  for (int i = 0; i < results.getCount(); i++) {
    const NimBLEAdvertisedDevice *dev = results.getDevice(i);
    if (strcasecmp(dev->getAddress().toString().c_str(), Vehicle::HEATER_BLE_MAC) == 0) found = dev;
  }
  if (!found) {
    Serial.println("Heater not found (out of range, or the HCalory app is connected)");
    return false;
  }

  Serial.printf("Connecting to \"%s\" rssi=%d\n", found->getName().c_str(), found->getRSSI());
  if (!client->connect(found)) {
    Serial.println("Connect failed");
    return false;
  }

  NimBLERemoteService *svc = client->getService(HEATER_SERVICE);
  NimBLERemoteCharacteristic *notify = svc ? svc->getCharacteristic(HEATER_NOTIFY) : nullptr;
  chWrite = svc ? svc->getCharacteristic(HEATER_WRITE) : nullptr;
  if (!notify || !chWrite || !notify->subscribe(true, onNotify)) {
    Serial.println("Heater service not found");
    client->disconnect();
    return false;
  }
  Serial.println("Connected and subscribed");
  return true;
}

static bool heaterLinked() { return client->isConnected() && packet.link == HEATER_LINK_OK; }

// Sends a HEATER_CMD_* to the heater, then asks for its new state soon
static bool heaterCommand(int16_t command) {
  if (!sendCommand((uint8_t)command)) return false;
  lastQueryMs = millis() - QUERY_INTERVAL_MS + QUERY_AFTER_CMD_MS;
  return true;
}

// Igniting, heating or running (the state bytes with bit 7 set, except the error state)
static bool heaterBurning() {
  return packet.state != HEATER_STATE_ERROR && (packet.state & 0x80);
}

// ---------- Clock ----------
// Reads the RTC into packet.clock / packet.time, logging when its state changes
static void readClock() {
  ds1302::Time t;
  ds1302::Status s = ds1302::read(t);
  int16_t clock = s == ds1302::Status::Ok ? HEATER_CLOCK_OK
                : s == ds1302::Status::NotSet ? HEATER_CLOCK_NOT_SET : HEATER_CLOCK_NONE;
  if (clock != packet.clock) {
    static const char *const names[] = { "not found", "OK", "not set (send T HH:MM)" };
    Serial.printf("RTC %s\n", names[clock]);
  }
  packet.clock = clock;
  packet.time = clock == HEATER_CLOCK_OK ? t.hour * 60 + t.minute : -1;
}

static void setClock(int hour, int minute, int second) {
  ds1302::set({ (uint8_t)hour, (uint8_t)minute, (uint8_t)second });
  readClock();
  Serial.printf("RTC set to %02d:%02d:%02d\n", hour, minute, second);
}

static uint32_t identifyUntilMs = 0;

static void blinkIdentify() {
  if (!identifyUntilMs) return;
  if ((int32_t)(millis() - identifyUntilMs) >= 0) {
    identifyUntilMs = 0;
    digitalWrite(LED_PIN, LOW);
    return;
  }
  digitalWrite(LED_PIN, (millis() / 150) % 2);
}

// "T HH:MM" or "T HH:MM:SS" sets the clock, "T" prints it, "I" blinks the LED,
// "D" dumps the RTC registers
static void pollSerial() {
  static char line[24];
  static size_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') {
      if (len < sizeof(line) - 1) line[len++] = c;
      continue;
    }
    line[len] = 0;
    len = 0;
    if (line[0] == 'I' || line[0] == 'i') {
      pinMode(LED_PIN, OUTPUT);
      identifyUntilMs = millis() + IDENTIFY_MS;
      Serial.println("Blinking the LED");
      continue;
    }
    if (line[0] == 'D' || line[0] == 'd') {
      ds1302::dump(Serial);
      continue;
    }
    if (line[0] != 'T' && line[0] != 't') continue;

    int h, m, s = 0;
    if (sscanf(line + 1, "%d:%d:%d", &h, &m, &s) >= 2) {
      if (h >= 0 && h < 24 && m >= 0 && m < 60 && s >= 0 && s < 60) setClock(h, m, s);
      else Serial.println("Usage: T HH:MM[:SS]");
    } else {
      ds1302::Time t;
      if (ds1302::read(t) == ds1302::Status::Ok) Serial.printf("RTC %02d:%02d:%02d\n", t.hour, t.minute, t.second);
      else Serial.println("RTC not available");
    }
  }
}

// ---------- Morning run ----------
static uint32_t runPhaseMs = 0;   // when the current run state began
static uint32_t runTryMs = 0;     // last start / stop sent
static int runTries = 0;

static void setRunState(int16_t s, const char *why) {
  Serial.printf("Morning run: %s\n", why);
  packet.run_state = s;
  runPhaseMs = millis();
  runTryMs = 0;
  runTries = 0;
}

static void morningRun() {
  // Starts when the clock enters the run minute, so it fires once a day, and not again
  // after a reboot within that minute
  static int16_t prevTime = -1;
  bool due = packet.run_armed && prevTime >= 0 && packet.time != prevTime && packet.time == packet.run_at;
  prevTime = packet.time;
  if (due && packet.run_state != HEATER_RUN_RUNNING && packet.run_state != HEATER_RUN_STOPPING) {
    setRunState(HEATER_RUN_STARTING, "due");
  }

  uint32_t now = millis();
  switch (packet.run_state) {
  case HEATER_RUN_STARTING:
    if (!packet.run_armed) {
      setRunState(HEATER_RUN_IDLE, "disarmed before the heater started");
    } else if (!heaterLinked()) {
      if (now - runPhaseMs > RUN_CONNECT_WINDOW_MS) setRunState(HEATER_RUN_FAILED, "heater not reachable");
    } else if (heaterBurning() && runTries == 0) {
      setRunState(HEATER_RUN_IDLE, "heater already on, leaving it alone");
    } else if (heaterBurning()) {
      setRunState(HEATER_RUN_RUNNING, "heater ignited");   // the run length counts from here
    } else if (packet.state == HEATER_STATE_ERROR) {
      setRunState(HEATER_RUN_FAILED, "heater reports an error");
    } else if (packet.volts < HEATER_RUN_MIN_DECIVOLTS) {
      Serial.printf("Morning run: battery at %d.%d V\n", packet.volts / 10, packet.volts % 10);
      setRunState(HEATER_RUN_SKIPPED_LOW_BATTERY, "skipped, battery low");
    } else if (runTryMs == 0 || now - runTryMs > RUN_RETRY_MS) {
      if (runTries >= RUN_START_TRIES) {
        setRunState(HEATER_RUN_FAILED, "heater didn't ignite");
      } else {
        Serial.println("Morning run: starting the heater");
        heaterCommand(HEATER_CMD_START);
        runTries++;
        runTryMs = now;
      }
    }
    break;

  case HEATER_RUN_RUNNING:
    if (heaterLinked() && !heaterBurning()) {
      setRunState(HEATER_RUN_DONE, "heater stopped before the end (manually?)");
    } else if (now - runPhaseMs >= HEATER_RUN_LENGTH_MIN * 60000UL) {
      setRunState(HEATER_RUN_STOPPING, "run over");
    }
    break;

  case HEATER_RUN_STOPPING:
    // Keeps trying until the heater is reachable and confirms it is cooling down or off
    if (!heaterLinked()) break;
    if (!heaterBurning()) {
      setRunState(HEATER_RUN_DONE, "heater stopped");
    } else if (runTryMs == 0 || now - runTryMs > RUN_RETRY_MS) {
      Serial.println("Morning run: stopping the heater");
      heaterCommand(HEATER_CMD_STOP);
      runTryMs = now;
    }
    break;
  }
}

// ---------- Dashboard commands ----------
// While the dashboard's phone switch is on, it repeats HEATER_CMD_RELEASE and the connector stays
// off the heater, so the phone app can connect. Once the repeats stop (switch off, or the
// dashboard lost power), the connector takes the heater back after HEATER_RELEASE_MS.
static uint32_t releaseUntilMs = 0;   // 0 = not released

static bool released() {
  if (releaseUntilMs && (int32_t)(millis() - releaseUntilMs) >= 0) {
    releaseUntilMs = 0;
    Serial.println("Release over, reconnecting to the heater");
  }
  return releaseUntilMs != 0;
}

static void runRelease(int16_t arg) {
  if (!arg) {
    if (releaseUntilMs) Serial.println("Released heater taken back");
    releaseUntilMs = 0;
    return;
  }
  if (!releaseUntilMs) Serial.println("Releasing the heater for the phone app");
  releaseUntilMs = millis() + HEATER_RELEASE_MS;
  if (!releaseUntilMs) releaseUntilMs = 1;
}

static const char *commandName(int16_t command) {
  switch (command) {
  case HEATER_CMD_STOP:            return "stop";
  case HEATER_CMD_START:           return "start";
  case HEATER_CMD_UP:              return "up";
  case HEATER_CMD_DOWN:            return "down";
  case HEATER_CMD_MODE_THERMOSTAT: return "thermostat mode";
  case HEATER_CMD_MODE_LEVEL:      return "level mode";
  case HEATER_CMD_RUN_AT:          return "run at";
  case HEATER_CMD_RUN_ARM:         return "arm run";
  case HEATER_CMD_SET_TIME:        return "set time";
  default:                         return "unknown";
  }
}

static int16_t runCommand(const HeaterCommandPacket &cmd) {
  switch (cmd.command) {
  case HEATER_CMD_RUN_AT:
    if (cmd.arg < 0 || cmd.arg >= 24 * 60) return HEATER_ACK_FAILED;
    packet.run_at = cmd.arg;
    prefs.putShort("run_at", packet.run_at);
    return HEATER_ACK_OK;

  case HEATER_CMD_RUN_ARM:
    packet.run_armed = cmd.arg ? 1 : 0;
    prefs.putShort("run_armed", packet.run_armed);
    return HEATER_ACK_OK;

  case HEATER_CMD_SET_TIME:
    if (cmd.arg < 0 || cmd.arg >= 24 * 60 || packet.clock == HEATER_CLOCK_NONE) return HEATER_ACK_FAILED;
    setClock(cmd.arg / 60, cmd.arg % 60, 0);
    return HEATER_ACK_OK;

  case HEATER_CMD_STOP:
  case HEATER_CMD_START:
  case HEATER_CMD_UP:
  case HEATER_CMD_DOWN:
  case HEATER_CMD_MODE_THERMOSTAT:
  case HEATER_CMD_MODE_LEVEL:
    if (!heaterLinked()) return HEATER_ACK_NO_LINK;
    return heaterCommand(cmd.command) ? HEATER_ACK_OK : HEATER_ACK_FAILED;

  default:
    return HEATER_ACK_FAILED;
  }
}

// Runs queued commands once each (a resend of the last one is only acknowledged again)
static void handleCommands() {
  HeaterCommandPacket cmd;
  while (xQueueReceive(commands, &cmd, 0) == pdTRUE) {
    if (cmd.seq == 0) {   // unacknowledged: leaves ack_seq alone for a command in flight
      if (cmd.command == HEATER_CMD_RELEASE) runRelease(cmd.arg);
      continue;
    }
    if (cmd.seq != packet.ack_seq) {
      packet.ack_result = runCommand(cmd);
      packet.ack_seq = cmd.seq;
      Serial.printf("Command %d %s(%d): result %d\n", cmd.seq, commandName(cmd.command), cmd.arg,
                    packet.ack_result);
    }
    sendPacket();
  }
}

// ---------- Arduino ----------
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Heater connector: HCalory BLE -> ESP-NOW");

  packet.link = HEATER_LINK_NONE;
  packet.ack_result = HEATER_ACK_NONE;
  packet.clock = HEATER_CLOCK_NONE;
  packet.time = -1;
  ds1302::begin(RTC_CLK_PIN, RTC_DAT_PIN, RTC_RST_PIN);
  readClock();
  if (packet.clock == HEATER_CLOCK_NONE) Serial.println("RTC not found");

  prefs.begin("heater");
  packet.run_at = prefs.getShort("run_at", DEFAULT_RUN_AT);
  packet.run_armed = prefs.getShort("run_armed", 0);

  commands = xQueueCreate(8, sizeof(HeaterCommandPacket));
  if (!initEspNow()) {
    delay(1000);
    ESP.restart();
  }

  frames = xQueueCreate(4, sizeof(Frame));

  NimBLEDevice::init("Heater-Connector");
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);

  client = NimBLEDevice::createClient();
  client->setClientCallbacks(&clientCallbacks, false);
  client->setConnectTimeout(5000);
}

void loop() {
  Maintenance::loop();
  handleCommands();
  pollSerial();
  blinkIdentify();

  static uint32_t lastClockMs = 0;
  if (millis() - lastClockMs >= CLOCK_READ_MS) {
    lastClockMs = millis();
    readClock();
  }
  morningRun();

  if (released()) {
    static uint32_t lastDisconnectMs = 0;   // the disconnect takes a moment to complete
    if (client->isConnected() && millis() - lastDisconnectMs > 2000) {
      Serial.println("Disconnecting from the heater");
      client->disconnect();
      lastDisconnectMs = millis();
    }
    packet.link = HEATER_LINK_RELEASED;
    if (millis() - lastSendMs >= SEND_INTERVAL_MS) sendPacket();
    delay(10);
    return;
  }

  if (!client->isConnected()) {
    packet.link = HEATER_LINK_NONE;
    if (identifyUntilMs) {   // the 5 s scan would stop the blinking
      if (millis() - lastSendMs >= SEND_INTERVAL_MS) sendPacket();
      delay(10);
      return;
    }
    sendPacket();
    if (findAndConnect()) {
      lastReplyMs = millis();
      lastQueryMs = 0;
    }
    return;
  }

  uint32_t now = millis();
  if (now - lastQueryMs >= QUERY_INTERVAL_MS) {
    lastQueryMs = now;
    if (!sendQuery()) Serial.println("Query write failed");
  }

  Frame frame;
  while (xQueueReceive(frames, &frame, 0) == pdTRUE) {
    if (!parseFrame(frame.data)) {
      Serial.println("Bad frame");
      continue;
    }
    lastReplyMs = millis();
    packet.link = HEATER_LINK_OK;
    sendPacket();
    Serial.printf("state=%02X mode=%d setting=%d volts=%.1f case=%.1f cabin=%.1f\n",
                  packet.state, packet.mode, packet.setting, packet.volts / 10.0f,
                  packet.case_temp / 10.0f, packet.cabin_temp / 10.0f);
  }

  if (millis() - lastReplyMs > REPLY_TIMEOUT_MS) {
    Serial.println("Heater stopped answering, reconnecting");
    client->disconnect();
    return;
  }

  // Replies late: tell the dashboard, and keep pinging it
  if (millis() - lastReplyMs > 3 * QUERY_INTERVAL_MS) packet.link = HEATER_LINK_NONE;
  if (millis() - lastSendMs >= SEND_INTERVAL_MS) sendPacket();
  delay(10);
}
