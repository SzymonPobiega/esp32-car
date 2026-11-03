#pragma once
#include <stdint.h>

// Toyota Land Cruiser profile. Include Shared/Vehicle.hpp instead of this file.
namespace Vehicle {

  inline constexpr const char *NAME = "Toyota";

  // ---- ESP-NOW nodes (Wi-Fi STA MACs) ----

  // EspNowUartBridge on the C6 next to the P4; forwards everything to ToyotaMonitor
  inline constexpr uint8_t MODEM_MAC[] = { 0x9C, 0x13, 0x9E, 0xC3, 0xFF, 0x60 };

  // Where TPMS, gyro and OBD connectors send
  inline constexpr const uint8_t (&DISPLAY_MAC)[6] = MODEM_MAC;

  // Single display, so BMS goes straight to the modem (acknowledged, with retries)
  inline constexpr const uint8_t (&BMS_DEST_MAC)[6] = MODEM_MAC;

  // HeaterConnector; the dashboard sends it heater commands through the modem
  inline constexpr uint8_t HEATER_CONNECTOR_MAC[] = { 0xD4, 0xE9, 0xF4, 0xE6, 0xCC, 0x90 };

  // ---- BLE sensors ----

  inline constexpr const char *BMS_BLE_MAC = "A5:C2:37:2B:DC:A1";

  // WitMotion WT901BLE68 (static random address); empty = GyroConnector stays idle
  inline constexpr const char *GYRO_BLE_MAC = "db:dd:a8:10:0b:fa";

  // OBD dongle; empty = auto-detect by name / known ELM service
  inline constexpr const char *OBD_DONGLE_MAC = "";

  // HCalory 8 kW diesel heater ("Heater5664")
  inline constexpr const char *HEATER_BLE_MAC = "ec:b1:b6:05:fe:5e";

  // ---- TPMS sensor IDs, in gauge order ----

  inline constexpr uint32_t TPMS_SENSORS[] = {
    7549971,
    1126691,
    14554163,
    4592707,
  };
}
