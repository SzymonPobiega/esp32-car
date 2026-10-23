#include <Arduino.h>
#include <NimBLEDevice.h>
#include "esp_system.h"
#include <esp_mac.h> 
#include <WiFi.h>
#include <esp_now.h>
#include "ContractsInclude.hpp"
#include <HardwareSerial.h>

static const int UART_RX_PIN = 16;
static const int UART_TX_PIN = 17;
static const uint32_t UART_BAUD = 115200;

HardwareSerial& U = Serial1;

using namespace Contracts;

NimBLEScan* pBLEScan = nullptr;

//BMS BLE
static NimBLEUUID SVC_FF00("0000ff00-0000-1000-8000-00805f9b34fb");
static NimBLEUUID CH_FF01("0000ff01-0000-1000-8000-00805f9b34fb");
static NimBLEUUID CH_FF02("0000ff02-0000-1000-8000-00805f9b34fb");

//WIT BLE
static NimBLEUUID SVC_FFE5("0000FFE5-0000-1000-8000-00805F9A34FB");
static NimBLEUUID CH_FFE4("0000FFE4-0000-1000-8000-00805F9A34FB");

//Volvo
//static const char* TARGET_MAC = "A5:C2:37:62:EF:6E";

//Toyoya
static const char* TARGET_MAC = "A5:C2:37:2B:DC:A1";

static const uint8_t broadcast_mac[] = {0xff,0xff,0xff,0xff,0xff,0xff};

static NimBLEClient* client = nullptr;
static NimBLERemoteCharacteristic* chNotify = nullptr;
static NimBLERemoteCharacteristic* chWrite = nullptr;

static std::vector<uint8_t> rxBuf;
static BmsPacket pendingMessage;
static bool dataReady = false;

enum class BleState {
  Idle,
  Connecting,
  Discovering,
  Subscribing,
  Ready,
  Failed
};

BleState bleState = BleState::Idle;
uint32_t nextBleAttemptMs = 0;

void onEspNowSent(const wifi_tx_info_t* tx_info, esp_now_send_status_t status) {
    Serial.print("ESP-NOW send status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

static void sendBmsOverUart(const BmsPacket& pkt) {
    const uint8_t len = sizeof(BmsPacket);
    U.write(&len, 1);
    U.write((const uint8_t*)&pkt, len);
    U.flush();
    //Serial.print("Date sent over UART");
}

// ==== Setup ==== //
void setup() {
  Serial.begin(115200);
  U.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  Serial.println();
  Serial.println("BMS gateway");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  WiFi.setSleep(false);

  // Read the real STA MAC from efuse
  uint8_t staMac[6];
  esp_read_mac(staMac, ESP_MAC_WIFI_STA);

  char macStr[18];
  snprintf(macStr, sizeof(macStr),
          "%02X:%02X:%02X:%02X:%02X:%02X",
          staMac[0], staMac[1], staMac[2],
          staMac[3], staMac[4], staMac[5]);

  Serial.print("ESP-NOW (STA) MAC: ");
  Serial.println(macStr);

  // Now init ESP-NOW
  if (esp_now_init() != ESP_OK) {
      Serial.println("❌ ESP-NOW init failed!");
      ESP.restart();
  }

  // New-style callback signature for your core:
  esp_now_register_send_cb(onEspNowSent);

  // Add peer (your ESP32-S3)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcast_mac, 6);
  peerInfo.channel = 0;   // 0 = current Wi-Fi channel
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("❌ Failed to add ESP-NOW peer!");
      ESP.restart();
  }

  NimBLEDevice::init("ESP32-BMS");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  // if (!connectBMS()) {
  //   Serial.println("❌ Failed to connect to BMS");
  //   ESP.restart();
  // }

  Serial.println("Setup complete.");
}

// ---------- Helpers ----------

static uint16_t be16(const uint8_t* p) {
  return (uint16_t(p[0]) << 8) | p[1];
}

static int16_t be16s(const uint8_t* p) {
  return int16_t((uint16_t(p[0]) << 8) | p[1]);
}

bool parseBasicInfo03(const uint8_t* frame, size_t len) {
  if (len < 41) return false;
  if (frame[0] != 0xDD || frame[1] != 0x03 || frame[len - 1] != 0x77) return false;

  const uint8_t status = frame[2];
  const uint8_t payloadLen = frame[3];
  if (status != 0x00 || payloadLen != 0x22) return false;

  // Payload begins at frame[4]
  const uint8_t* pl = &frame[4];

  // Offsets inside payload (JBD 0x03):
  // 0..1  pack voltage (0.01V)
  // 2..3  current (0.01A signed)
  // 4..5  remaining cap (0.01Ah)
  // 16    SW version
  // 17    SOC
  // 18    MOS state
  // 19    cell count
  // 20    temp sensor count
  // 21..22 temperature (0.1K)

  pendingMessage.volts = (int32_t)((uint16_t(pl[0]) << 8) | pl[1]);
  pendingMessage.amps     = (int32_t)(int16_t((uint16_t(pl[2]) << 8) | pl[3]));
  pendingMessage.remaining  = (int32_t)((uint16_t(pl[4]) << 8) | pl[5]);
  pendingMessage.soc_percent    = pl[19];

  const uint16_t temp_raw = (uint16_t(pl[23]) << 8) | pl[24];
  pendingMessage.temp = (int32_t)temp_raw - 2731; // 0.1K -> 0.1°C

  return true;
}


static void printHex(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    Serial.printf("%02X", data[i]);
    if (i + 1 < len) Serial.print(' ');
  }
}

static void consumeFrames() {
  while (true) {
    // Find frame start
    auto itStart = std::find(rxBuf.begin(), rxBuf.end(), 0xDD);
    if (itStart == rxBuf.end()) {
      rxBuf.clear();
      return;
    }

    // Drop junk before start
    if (itStart != rxBuf.begin())
      rxBuf.erase(rxBuf.begin(), itStart);

    // Find frame end
    auto itEnd = std::find(rxBuf.begin() + 1, rxBuf.end(), 0x77);
    if (itEnd == rxBuf.end())
      return; // wait for more data

    std::vector<uint8_t> frame(rxBuf.begin(), itEnd + 1);
    rxBuf.erase(rxBuf.begin(), itEnd + 1);

    // ---- Handle frame ----
    if (frame.size() >= 2 && frame[1] == 0x03) {
      
      if (parseBasicInfo03(frame.data(), frame.size())) {
        // Serial.printf(
        //   "BMS: V=%ld.%02ldV  I=%ld.%02ldA  SOC=%u%%  Rem=%ld.%02ldAh  T=%ld.%1ldC\n",
        //   pendingMessage.volts / 100, abs(pendingMessage.volts % 100),
        //   pendingMessage.amps / 100, abs(pendingMessage.amps % 100),
        //   pendingMessage.soc_percent,
        //   pendingMessage.remaining / 100, abs(pendingMessage.remaining % 100),
        //   pendingMessage.temp / 10, abs(pendingMessage.temp % 10)
        // );
        pendingMessage.type = TYPE_BMS;
      }
    }
  }
}


// ---------- Notify callback ----------
static void onNotify(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool b) {
  //Serial.print("[RX chunk] ");
  //printHex(data, len);
  //Serial.println();

  rxBuf.insert(rxBuf.end(), data, data + len);
  consumeFrames();
}

// ---------- Connect ----------
static bool connectBMS() {
  NimBLEAddress addr(std::string(TARGET_MAC), BLE_ADDR_PUBLIC);

  if (!client) {
    client = NimBLEDevice::createClient();
    client->setConnectionParams(24, 40, 0, 2000);
  }

  Serial.print("Connecting to ");
  Serial.println(TARGET_MAC);

  if (!client->connect(addr, true, true, false)) {
    Serial.println("❌ Connect failed");
    return false;
  }

  client->setConnectionParams(24, 40, 0, 2000);
  Serial.printf("MTU=%u RSSI=%d\n", client->getMTU(), client->getRssi());

  NimBLERemoteService* svc = client->getService(SVC_FF00);
  if (!svc) {
    Serial.println("❌ FF00 service not found");
    return false;
  }

  chNotify = svc->getCharacteristic(CH_FF01);
  chWrite  = svc->getCharacteristic(CH_FF02);

  if (!chNotify || !chWrite) {
    Serial.println("❌ Characteristics not found");
    return false;
  }

  if (!chNotify->canNotify()) {
    Serial.println("❌ FF01 has no notify");
    return false;
  }

  Serial.println("Subscribing to FF01 notify...");
  if (!chNotify->subscribe(true, onNotify)) {
    Serial.println("❌ Subscribe failed");
    return false;
  }

  Serial.println("✅ Connected + subscribed");
  return true;
}

// ---------- Send command ----------
static void sendCmd(const uint8_t* cmd, size_t len) {
  if (!client || !client->isConnected()) return;

//   Serial.print("[TX] ");
//   printHex(cmd, len);
//   Serial.println();

  bool noResp = chWrite->canWriteNoResponse();
  chWrite->writeValue(cmd, len, !noResp);
}

// ---------- Known-good commands ----------
static void sendBasic03() {
  const uint8_t cmd[] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
  sendCmd(cmd, sizeof(cmd));
}

bool startBleConnect() {
  NimBLEAddress addr(std::string(TARGET_MAC), BLE_ADDR_PUBLIC);

  if (!client) {
    client = NimBLEDevice::createClient();
    client->setConnectionParams(24, 40, 0, 2000);
  }

  // async connect
  return client->connect(addr, true, true, false);
}

void loop() {
  uint32_t now = millis();

  static uint32_t lastPoll = 0;
  static uint32_t lastBroadcast = 0;

  if (now - lastBroadcast >= 1000) {
    lastBroadcast = now;
    pendingMessage.type = TYPE_BMS;
    esp_err_t res = esp_now_send(broadcast_mac, (uint8_t*)&pendingMessage, sizeof(pendingMessage));
    Serial.print("  |  ESP-NOW send: ");
    Serial.println(res == ESP_OK ? "OK" : String("ERR ") + res);

    sendBmsOverUart(pendingMessage);

    Serial.printf(
          "BMS: V=%ld.%02ldV  I=%ld.%02ldA  SOC=%u%%  Rem=%ld.%02ldAh  T=%ld.%1ldC\n",
          pendingMessage.volts / 100, abs(pendingMessage.volts % 100),
          pendingMessage.amps / 100, abs(pendingMessage.amps % 100),
          pendingMessage.soc_percent,
          pendingMessage.remaining / 100, abs(pendingMessage.remaining % 100),
          pendingMessage.temp / 10, abs(pendingMessage.temp % 10)
        );
  }

  if (now - lastPoll >= 1000 && bleState == BleState::Ready) {
      lastPoll = now;
      sendBasic03();
  }

  switch (bleState) {
    case BleState::Idle:
      if (now >= nextBleAttemptMs) {
        if (startBleConnect()) {
          bleState = BleState::Discovering;
        } else {
          bleState = BleState::Failed;
          nextBleAttemptMs = now + 5000;
        }
      }
      break;

    case BleState::Discovering:
      if (client && client->isConnected()) {
        NimBLERemoteService* svc = client->getService(SVC_FF00);
        if (!svc) {
          bleState = BleState::Failed;
          nextBleAttemptMs = now + 5000;
          break;
        }

        chNotify = svc->getCharacteristic(CH_FF01);
        chWrite  = svc->getCharacteristic(CH_FF02);
        if (!chNotify || !chWrite) {
          bleState = BleState::Failed;
          nextBleAttemptMs = now + 5000;
          break;
        }

        bleState = BleState::Subscribing;
      }
      break;

    case BleState::Subscribing:
      if (chNotify->canNotify() && chNotify->subscribe(true, onNotify)) {
        bleState = BleState::Ready;
      } else {
        bleState = BleState::Failed;
        nextBleAttemptMs = now + 5000;
      }
      break;

    case BleState::Ready:
      // normal BLE work here
      break;

    case BleState::Failed:
      if (client && client->isConnected()) {
        client->disconnect();
      }
      bleState = BleState::Idle;
      break;

    default:
      break;
  }

  delay(10);
}

// void loop() {
  

//   // if (!client || !client->isConnected()) {
//   //   delay(1000);
//   //   return;
//   // }
  
  

//   delay(1000);
// }
