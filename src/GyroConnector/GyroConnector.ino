#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEScan.h>
#include "esp_system.h"
#include <esp_mac.h> 
#include <WiFi.h>
#include <esp_now.h>
#include "ContractsInclude.hpp"

using namespace Contracts;

NimBLEScan* pBLEScan = nullptr;

//WIT BLE
static NimBLEUUID SVC_FFE5("0000FFE5-0000-1000-8000-00805F9A34FB");
static NimBLEUUID CH_FFE4("0000FFE4-0000-1000-8000-00805F9A34FB");
static NimBLEUUID CH_FFE9("0000FFE9-0000-1000-8000-00805F9A34FB");


static NimBLEClient* client = nullptr;
static NimBLERemoteCharacteristic* chNotify = nullptr;
static NimBLERemoteCharacteristic* chWrite = nullptr;

static uint32_t lastGyroPublishMs = 0;
static constexpr uint32_t GYRO_PUBLISH_INTERVAL_MS = 1000;

static std::vector<uint8_t> rxBuf;
static int16_t height;

void onEspNowSent(const wifi_tx_info_t* tx_info, esp_now_send_status_t status) {
    Serial.print("ESP-NOW send status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.printf("GYRO gateway (%s)\n", Vehicle::NAME);

    WiFi.mode(WIFI_STA);        // required for ESP-NOW
    WiFi.disconnect();          // just to be safe

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
        return;
    }

    // New-style callback signature for your core:
    esp_now_register_send_cb(onEspNowSent);

    // Add peer (your ESP32-S3)
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, Vehicle::DISPLAY_MAC, 6);
    peerInfo.channel = 0;   // 0 = current Wi-Fi channel
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add ESP-NOW peer!");
        while (true) { delay(1000); }
    }

    NimBLEDevice::init("GYRO-Gateway");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    if (!connectWT901()) {
      Serial.println("❌ Failed to connect to WT901");
      return;
    }
    //startScan();
    Serial.println("✅ Setup complete.");
}

// ---------- Helpers ----------

static inline int16_t le_i16(const uint8_t* p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline uint32_t le_u32(const uint8_t* p) {
  return (uint32_t)p[0]
      | ((uint32_t)p[1] << 8)
      | ((uint32_t)p[2] << 16)
      | ((uint32_t)p[3] << 24);
}

static bool checksum_ok_11(const uint8_t* f) {
  uint8_t sum = 0;
  for (int i = 0; i < 10; i++) sum = (uint8_t)(sum + f[i]);
  return sum == f[10];
}

static void printHex(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    Serial.printf("%02X", data[i]);
    if (i + 1 < len) Serial.print(' ');
  }
  Serial.println();
}

static void consumeFrames() {
  constexpr size_t FRAME_LEN = 20;

  while (rxBuf.size() >= FRAME_LEN) {

    // resync to 0x55
    if (rxBuf[0] != 0x55) {
      rxBuf.erase(rxBuf.begin());
      continue;
    }

    uint8_t flag = rxBuf[1];

    // If it's not a frame type we handle, drop one byte and resync.
    if (!(flag == 0x61 || flag == 0x71)) {
      rxBuf.erase(rxBuf.begin());
      continue;
    }

    const uint8_t* f = rxBuf.data();

    if (flag == 0x61) {

      // Decode angles at fixed offsets in the 0x61 frame:
      int16_t r_raw = le_i16(&f[14]);
      int16_t p_raw = le_i16(&f[16]);
      int16_t y_raw = le_i16(&f[18]);

      constexpr float SCALE = 180.0f / 32768.0f;
      float roll  = r_raw * SCALE;
      float pitch = p_raw * SCALE;
      float yaw   = y_raw * SCALE;

      uint32_t now = millis();

      //Serial.printf("RPY: %.2f %.2f %.2f\n", roll, pitch, yaw);

      // publish at most once per second
      if (now - lastGyroPublishMs >= GYRO_PUBLISH_INTERVAL_MS) {

        lastGyroPublishMs = now;

        GyroAnglePacket pkt;
        pkt.type = Contracts::TYPE_GYRO_ANGLE;
        pkt.roll_deg  = (int16_t)(roll  * 100.0f);
        pkt.pitch_deg = (int16_t)(pitch * 100.0f);
        pkt.yaw_deg   = (int16_t)(yaw   * 100.0f);
        pkt.height = height;
        esp_now_send(Vehicle::DISPLAY_MAC, (uint8_t*)&pkt, sizeof(pkt));
      }

      rxBuf.erase(rxBuf.begin(), rxBuf.begin() + FRAME_LEN);
      continue;
    }

    if (flag == 0x71) {
      const uint8_t* f = rxBuf.data();

      uint8_t regL = f[2];
      uint8_t regH = f[3]; // usually 0x00 here

      // Air pressure + altitude request starts at reg 0x45
      if (regL == 0x45 && regH == 0x00) {
        // layout: 55 71 45 00  P0 P1 P2 P3  H0 H1 H2 H3  (rest = other regs / padding)
        uint32_t pressure_pa = le_u32(&f[4]);   // P0..P3
        int32_t  height_cm   = (int32_t)le_u32(&f[8]);  // H0..H3 (signed in case)

        float height_m = height_cm / 100.0f;

        Serial.printf("Pressure: %lu Pa, Alt: %.2f m\n",
                      (unsigned long)pressure_pa, height_m);

        height = (int16_t)height_m;
        // GyroHeightPacket pkt;
        // pkt.type = Contracts::TYPE_GYRO_HEIGHT;
        // pkt.height  = (int16_t)height_m;
        // esp_now_send(Vehicle::DISPLAY_MAC, (uint8_t*)&pkt, sizeof(pkt));
      }

      rxBuf.erase(rxBuf.begin(), rxBuf.begin() + FRAME_LEN);
      continue;
    }
  }
}


static void onNotify(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
  //printHex(data, len);
  rxBuf.insert(rxBuf.end(), data, data + len);
  consumeFrames();
}

static void requestAltitude() {
    if (!chWrite) return;

    uint8_t cmd[] = {0xFF, 0xAA, 0x27, 0x45, 0x00};
    chWrite->writeValue(cmd, sizeof(cmd), false);

    Serial.println("Requested pressure + altitude");
}

static bool connectWT901() {
  NimBLEAddress addr(std::string(Vehicle::GYRO_BLE_MAC), BLE_ADDR_RANDOM );
  if (!client) {
    client = NimBLEDevice::createClient();
    client->setConnectionParams(24, 40, 0, 2000);
  }

  Serial.print("Connecting to ");
  Serial.println(Vehicle::GYRO_BLE_MAC);

  client->connect(addr);
  if (!client->isConnected()) {
    Serial.println("❌ Connect failed");
    return false;
  }

  NimBLERemoteService* svc = client->getService(SVC_FFE5);
  if (!svc) {
    Serial.println("❌ FFE5 service not found");
    return false;
  }

  chNotify = svc->getCharacteristic(CH_FFE4);
  if (!chNotify) {
    Serial.println("❌ FFE4 notify characteristic not found");
    return false;
  }

  if (!chNotify->canNotify()) {
    Serial.println("❌ FFE4 has no notify");
    return false;
  }

  Serial.println("Subscribing to FFE4 notify...");
  if (!chNotify->subscribe(true, onNotify)) {
    Serial.println("❌ Subscribe failed");
    return false;
  }

  Serial.println("✅ Connected + subscribed (WT901)");

  chWrite = svc->getCharacteristic("0000FFE9-0000-1000-8000-00805F9A34FB");
  if (!chWrite) {
      Serial.println("❌ FFE9 write characteristic not found");
      return false;
  }

  if (!chWrite->canWrite()) {
      Serial.println("❌ FFE9 not writable");
      return false;
  }

  return true;
}


void loop() {
  // No gyro installed in this vehicle yet
  if (Vehicle::GYRO_BLE_MAC[0] == '\0') {
    static bool reported = false;
    if (!reported) {
      Serial.printf("No gyro configured for the %s, idle\n", Vehicle::NAME);
      reported = true;
    }
    delay(1000);
    return;
  }

  if (!client || !client->isConnected()) {
     connectWT901();
  } else {
    requestAltitude();   // ask sensor
  }
  delay(1000);
}
