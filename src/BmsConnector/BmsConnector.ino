// BMS connector: reads a JBD / Xiaoxiang BMS over BLE and sends the battery state over
// ESP-NOW every second. Packets are only sent while the BMS answers, so a dashboard sees the
// link drop (and dims the values) instead of receiving zeros.
//
// Target board: standard ESP32.

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "esp_system.h"
#include <esp_mac.h>
#include <WiFi.h>
#include <esp_now.h>
#include "ContractsInclude.hpp"
#include "../Shared/Maintenance.hpp"

using namespace Contracts;

// -------------------- USER CONFIG --------------------
// BMS address and ESP-NOW destination come from the vehicle profile (Shared/Vehicles/*.hpp)
static const uint32_t POLL_MS = 1000;            // basic info request
static const uint32_t SEND_MS = 1000;            // ESP-NOW packet
static const uint32_t CONNECT_TIMEOUT_MS = 10000;
static const uint32_t RETRY_MS = 5000;           // after a failed or lost connection
static const uint32_t REPLY_TIMEOUT_MS = 5000;   // no valid reply for this long: reconnect
// -----------------------------------------------------

static NimBLEUUID SVC_FF00("0000ff00-0000-1000-8000-00805f9b34fb");
static NimBLEUUID CH_FF01("0000ff01-0000-1000-8000-00805f9b34fb");
static NimBLEUUID CH_FF02("0000ff02-0000-1000-8000-00805f9b34fb");

static NimBLEClient* client = nullptr;
static NimBLERemoteCharacteristic* chNotify = nullptr;
static NimBLERemoteCharacteristic* chWrite = nullptr;

// Written by the notify callback (BLE host task), read by loop()
static portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;
static BmsPacket latest;
static volatile uint32_t lastReplyMs = 0;
static volatile bool hasReply = false;

static std::vector<uint8_t> rxBuf;   // notify callback only

enum class BleState {
  Idle,
  Connecting,
  Discovering,
  Subscribing,
  Ready,
  Failed
};

static const char* stateName(BleState s) {
  switch (s) {
    case BleState::Idle:        return "Idle";
    case BleState::Connecting:  return "Connecting";
    case BleState::Discovering: return "Discovering";
    case BleState::Subscribing: return "Subscribing";
    case BleState::Ready:       return "Ready";
    case BleState::Failed:      return "Failed";
  }
  return "?";
}

static BleState bleState = BleState::Idle;
static uint32_t stateSinceMs = 0;
static uint32_t nextBleAttemptMs = 0;

static void setState(BleState s) {
  Serial.printf("BLE %s -> %s\n", stateName(bleState), stateName(s));
  bleState = s;
  stateSinceMs = millis();
}

static void fail(uint32_t now) {
  nextBleAttemptMs = now + RETRY_MS;
  setState(BleState::Failed);
}

void onEspNowSent(const wifi_tx_info_t* tx_info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) Serial.println("ESP-NOW send failed");
}

// ==== Setup ==== //
void setup() {
  Serial.begin(115200);

  Serial.println();
  Serial.printf("BMS gateway (%s)\n", Vehicle::NAME);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  WiFi.setSleep(false);

  // Read the real STA MAC from efuse
  uint8_t staMac[6];
  esp_read_mac(staMac, ESP_MAC_WIFI_STA);
  Serial.printf("ESP-NOW (STA) MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                staMac[0], staMac[1], staMac[2], staMac[3], staMac[4], staMac[5]);

  if (esp_now_init() != ESP_OK) {
      Serial.println("ESP-NOW init failed!");
      ESP.restart();
  }
  esp_now_register_send_cb(onEspNowSent);
  esp_now_register_recv_cb(Maintenance::onEspNowRecv);   // OTA maintenance command from the dashboard
  Maintenance::begin("BmsConnector", MAINT_BMS);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, Vehicle::BMS_DEST_MAC, 6);
  peerInfo.channel = 0;   // 0 = current Wi-Fi channel
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add ESP-NOW peer!");
      ESP.restart();
  }

  NimBLEDevice::init("ESP32-BMS");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  client = NimBLEDevice::createClient();
  client->setConnectionParams(24, 40, 0, 2000);
  client->setConnectTimeout(CONNECT_TIMEOUT_MS);

  Serial.println("Setup complete.");
}

// ---------- JBD protocol ----------
// Frame: DD <cmd> <status> <len> <payload:len> <checksum:2> 77,
// checksum = 0x10000 - sum(status, len, payload)
static const size_t FRAME_OVERHEAD = 7;

static bool checksumOk(const uint8_t* frame, size_t len) {
  uint16_t sum = 0;
  for (size_t i = 2; i < len - 3; i++) sum += frame[i];
  uint16_t expected = (uint16_t)(0x10000 - sum);
  return ((uint16_t(frame[len - 3]) << 8) | frame[len - 2]) == expected;
}

// Basic info (0x03). Payload offsets:
// 0..1 pack voltage (0.01 V), 2..3 current (0.01 A, signed), 4..5 remaining capacity
// (0.01 Ah), 6..17 nominal capacity, cycles, date, balance and protection flags,
// 18 software version, 19 SOC, 20 MOSFET state, 21 cell count, 22 NTC count,
// 23..24 first temperature (0.1 K)
static bool parseBasicInfo03(const uint8_t* pl, size_t len, BmsPacket& out) {
  if (len < 25) return false;
  out.type = TYPE_BMS;
  out.volts = (int32_t)((uint16_t(pl[0]) << 8) | pl[1]);
  out.amps = (int32_t)(int16_t((uint16_t(pl[2]) << 8) | pl[3]));
  out.remaining = (int32_t)((uint16_t(pl[4]) << 8) | pl[5]);
  out.soc_percent = pl[19];
  out.temp = pl[22] > 0 ? (int32_t)((uint16_t(pl[23]) << 8) | pl[24]) - 2731 : 0;   // 0.1 K -> 0.1 °C
  return true;
}

// Takes complete frames off rxBuf, using the length byte (0x77 and 0xDD can occur in data)
static void consumeFrames() {
  while (true) {
    auto itStart = std::find(rxBuf.begin(), rxBuf.end(), 0xDD);
    rxBuf.erase(rxBuf.begin(), itStart);   // junk before the start
    if (rxBuf.size() < 4) return;

    size_t frameLen = rxBuf[3] + FRAME_OVERHEAD;
    if (rxBuf.size() < frameLen) return;   // wait for more data

    const uint8_t* f = rxBuf.data();
    if (f[frameLen - 1] != 0x77 || !checksumOk(f, frameLen)) {
      rxBuf.erase(rxBuf.begin());   // not a frame after all: look for the next DD
      continue;
    }

    BmsPacket pkt;
    if (f[1] == 0x03 && f[2] == 0x00 && parseBasicInfo03(f + 4, f[3], pkt)) {
      portENTER_CRITICAL(&dataMux);
      latest = pkt;
      portEXIT_CRITICAL(&dataMux);
      lastReplyMs = millis();
      hasReply = true;
    }
    rxBuf.erase(rxBuf.begin(), rxBuf.begin() + frameLen);
  }
}

static void onNotify(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
  rxBuf.insert(rxBuf.end(), data, data + len);
  if (rxBuf.size() > 512) rxBuf.clear();   // runaway garbage
  consumeFrames();
}

static void sendBasic03() {
  static const uint8_t cmd[] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
  chWrite->writeValue(cmd, sizeof(cmd), !chWrite->canWriteNoResponse());
}

// ---------- ESP-NOW ----------
// Sends the latest reading, but only while the BMS answers
static void sendLatest(uint32_t now) {
  if (!hasReply || now - lastReplyMs > REPLY_TIMEOUT_MS) return;

  BmsPacket pkt;
  portENTER_CRITICAL(&dataMux);
  pkt = latest;
  portEXIT_CRITICAL(&dataMux);

  esp_now_send(Vehicle::BMS_DEST_MAC, (uint8_t*)&pkt, sizeof(pkt));
  Serial.printf("BMS: V=%ld.%02ldV  I=%ld.%02ldA  SOC=%u%%  Rem=%ld.%02ldAh  T=%ld.%1ldC\n",
                pkt.volts / 100, abs(pkt.volts % 100),
                pkt.amps / 100, abs(pkt.amps % 100),
                pkt.soc_percent,
                pkt.remaining / 100, abs(pkt.remaining % 100),
                pkt.temp / 10, abs(pkt.temp % 10));
}

void loop() {
  Maintenance::loop();
  uint32_t now = millis();

  static uint32_t lastPoll = 0;
  static uint32_t lastSend = 0;

  if (now - lastSend >= SEND_MS) {
    lastSend = now;
    sendLatest(now);
  }

  switch (bleState) {
    case BleState::Idle:
      if ((int32_t)(now - nextBleAttemptMs) >= 0) {
        Serial.printf("Connecting to %s\n", Vehicle::BMS_BLE_MAC);
        NimBLEAddress addr(std::string(Vehicle::BMS_BLE_MAC), BLE_ADDR_PUBLIC);
        // Asynchronous, so the ESP-NOW packets keep going while it connects
        if (client->connect(addr, true, true, false)) setState(BleState::Connecting);
        else fail(now);
      }
      break;

    case BleState::Connecting:
      if (client->isConnected()) setState(BleState::Discovering);
      else if (now - stateSinceMs > CONNECT_TIMEOUT_MS + 2000) {
        Serial.println("Connect timed out (BMS out of range, or another app is connected)");
        fail(now);
      }
      break;

    case BleState::Discovering: {
      NimBLERemoteService* svc = client->getService(SVC_FF00);
      chNotify = svc ? svc->getCharacteristic(CH_FF01) : nullptr;
      chWrite = svc ? svc->getCharacteristic(CH_FF02) : nullptr;
      if (!chNotify || !chWrite) {
        Serial.println("FF00 service or characteristics not found");
        fail(now);
      } else {
        setState(BleState::Subscribing);
      }
      break;
    }

    case BleState::Subscribing:
      if (chNotify->canNotify() && chNotify->subscribe(true, onNotify)) {
        rxBuf.clear();
        lastReplyMs = now;   // give the first reply REPLY_TIMEOUT_MS
        hasReply = false;
        lastPoll = 0;
        setState(BleState::Ready);
      } else {
        Serial.println("Subscribe failed");
        fail(now);
      }
      break;

    case BleState::Ready:
      if (!client->isConnected()) {
        Serial.println("BMS disconnected");
        fail(now);
      } else if (now - lastReplyMs > REPLY_TIMEOUT_MS) {
        Serial.println("BMS stopped answering, reconnecting");
        fail(now);
      } else if (now - lastPoll >= POLL_MS) {
        lastPoll = now;
        sendBasic03();
      }
      break;

    case BleState::Failed:
      if (client->isConnected()) client->disconnect();
      if ((int32_t)(now - nextBleAttemptMs) >= 0) setState(BleState::Idle);
      break;
  }

  delay(10);
}
