#pragma once
// Maintenance mode: over-the-air updates for the connectors.
//
// The dashboard sends a MaintenanceCommandPacket over ESP-NOW. The connector then stops its
// normal work (BLE and ESP-NOW), joins the maintenance Wi-Fi network from Shared/Secrets.hpp
// and waits for an ArduinoOTA upload (flash.ps1 -Ota) for MAINTENANCE_WINDOW_MS. Without an
// upload it restarts into normal operation. Nothing runs before the command arrives.
//
// In a sketch:
//   setup():  Maintenance::begin("ObdConnector", Contracts::MAINT_OBD);
//   ESP-NOW receive callback: if (Maintenance::onEspNow(info, data, len)) return;
//             (or register Maintenance::onEspNowRecv when the sketch has no callback of its own)
//   loop():   Maintenance::loop();   // first thing; never returns once requested
//
// Host name on the network: <vehicle>-<sketch>-<last 3 MAC bytes>, e.g.
// toyota-obdconnector-692d78.local, which flash.ps1 derives from boards.json.

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <esp_now.h>
#include <esp_mac.h>
#include <NimBLEDevice.h>
#include "Contracts.hpp"
#include "Vehicle.hpp"

#if __has_include("Secrets.hpp")
#include "Secrets.hpp"
#define MAINTENANCE_CONFIGURED 1
#else
#define MAINTENANCE_CONFIGURED 0
#endif

namespace Maintenance {
  // Wi-Fi network name filled in (Secrets.hpp exists but may still be the empty template)
  inline bool configured() {
#if MAINTENANCE_CONFIGURED
    return Secrets::MAINTENANCE_WIFI_SSID[0] != 0;
#else
    return false;
#endif
  }
}

namespace Maintenance {

  inline constexpr uint32_t MAINTENANCE_WINDOW_MS = 10 * 60 * 1000UL;   // wait for an upload
  inline constexpr uint32_t WIFI_JOIN_MS = 30000;

  inline uint16_t role = 0;
  inline const char *sketchName = "";
  inline volatile bool requested = false;
  inline char hostName[48] = "";

  inline void begin(const char *sketch, uint16_t connectorRole) {
    sketchName = sketch;
    role = connectorRole;
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    String host = String(Vehicle::NAME) + "-" + sketch;
    host.toLowerCase();
    snprintf(hostName, sizeof(hostName), "%s-%02x%02x%02x", host.c_str(), mac[3], mac[4], mac[5]);
    if (configured()) Serial.printf("Maintenance mode available as %s.local\n", hostName);
    else Serial.println("Maintenance mode not configured (Wi-Fi name empty or no Shared/Secrets.hpp)");
  }

  // Call first in the sketch's ESP-NOW receive callback. True when the packet was a maintenance
  // command (whether or not it was for this connector), so the sketch can ignore it.
  inline bool onEspNow(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (!info || len != sizeof(Contracts::MaintenanceCommandPacket)) return false;
    Contracts::MaintenanceCommandPacket cmd;
    memcpy(&cmd, data, sizeof(cmd));
    if (cmd.type != Contracts::TYPE_MAINTENANCE_COMMAND) return false;
    // Only the dashboard (through the modem) may start it
    if (memcmp(info->src_addr, Vehicle::MODEM_MAC, 6) != 0) return true;
    if ((cmd.targets & role) && cmd.command == Contracts::MAINT_CMD_ENTER && configured()) requested = true;
    return true;
  }

  // For sketches without an ESP-NOW receive callback of their own
  inline void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    onEspNow(info, data, len);
  }

  [[noreturn]] inline void run() {
    Serial.println("Maintenance mode: stopping BLE and ESP-NOW");
#if !MAINTENANCE_CONFIGURED
    ESP.restart();   // not reached: requests are ignored when not configured
#else
    if (NimBLEDevice::isInitialized()) {
      NimBLEScan *scan = NimBLEDevice::getScan();
      if (scan->isScanning()) scan->stop();
      for (NimBLEClient *c : NimBLEDevice::getConnectedClients()) c->disconnect();
      delay(300);
      NimBLEDevice::deinit(true);
    }
    esp_now_deinit();

    Serial.printf("Maintenance mode: joining \"%s\"\n", Secrets::MAINTENANCE_WIFI_SSID);
    WiFi.disconnect();
    WiFi.setHostname(hostName);
    WiFi.begin(Secrets::MAINTENANCE_WIFI_SSID, Secrets::MAINTENANCE_WIFI_PASSWORD);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED) {
      if (millis() - start > WIFI_JOIN_MS) {
        Serial.println("Maintenance mode: Wi-Fi not reachable, restarting");
        delay(100);
        ESP.restart();
      }
      delay(100);
    }
    Serial.printf("Maintenance mode: %s.local at %s, waiting %lu min for an upload\n", hostName,
                  WiFi.localIP().toString().c_str(), MAINTENANCE_WINDOW_MS / 60000);

    ArduinoOTA.setHostname(hostName);
    ArduinoOTA.setPassword(Secrets::OTA_PASSWORD);
    ArduinoOTA.onStart([]() { Serial.println("OTA: receiving firmware"); });
    ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
      static int lastPct = -10;
      int pct = total ? done * 100 / total : 0;
      if (pct >= lastPct + 10) {
        Serial.printf("OTA: %d%%\n", pct);
        lastPct = pct;
      }
    });
    ArduinoOTA.onEnd([]() { Serial.println("OTA: done, restarting"); });
    ArduinoOTA.onError([](ota_error_t e) { Serial.printf("OTA: error %u\n", e); });
    ArduinoOTA.begin();
    MDNS.addServiceTxt("arduino", "tcp", "sketch", sketchName);

    start = millis();
    while (millis() - start < MAINTENANCE_WINDOW_MS) {
      ArduinoOTA.handle();   // restarts the board after a successful upload
      delay(10);
    }
    Serial.println("Maintenance mode: no upload, restarting");
    delay(100);
    ESP.restart();
#endif
    while (true) {}
  }

  // First thing in loop(): enters maintenance mode once the dashboard asked for it
  inline void loop() {
    if (requested) run();
  }
}
