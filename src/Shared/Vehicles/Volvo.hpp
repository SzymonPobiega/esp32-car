#pragma once
#include <stdint.h>

// Volvo camper profile. Include Shared/Vehicle.hpp instead of this file.
namespace Vehicle {

  inline constexpr const char *NAME = "Volvo";

  // ---- ESP-NOW nodes (Wi-Fi STA MACs) ----

  // VolvoMonitor (5" S3): receives TPMS and gyro
  inline constexpr uint8_t DISPLAY_MAC[] = { 0x50, 0x78, 0x7D, 0x13, 0x22, 0xF8 };

  // 7" S3 display, not referenced by any sketch
  inline constexpr uint8_t S3_7_INCH_MAC[] = { 0xD0, 0xCF, 0x13, 0x06, 0x79, 0xA0 };

  // VolvoCameraController LilyGO
  inline constexpr uint8_t LILYGO_CAMERA_MAC[] = { 0x94, 0x51, 0xDC, 0x47, 0xAE, 0xC4 };

  // VolvoWorkLightsController LilyGO
  inline constexpr uint8_t LILYGO_WORKLIGHT_MAC[] = { 0xE0, 0x8C, 0xFE, 0x75, 0xDA, 0x54 };

  // ShellyConnectorUartBridge, ESP-NOW side of the Shelly lights
  inline constexpr uint8_t SHELLY_BRIDGE_MAC[] = { 0xD4, 0xE9, 0xF4, 0xA8, 0x7E, 0x34 };

  // BmsConnector broadcasts so both displays receive it
  inline constexpr uint8_t BMS_DEST_MAC[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

  // ---- BLE sensors ----

  inline constexpr const char *BMS_BLE_MAC = "A5:C2:37:62:EF:6E";
  inline constexpr const char *GYRO_BLE_MAC = "e0:3b:b5:fe:27:8e";

  // ---- TPMS sensor IDs, in gauge order ----

  inline constexpr uint32_t TPMS_SENSORS[] = {
    8924691,   // TPMS1_132E88
    9580339,   // TPMS3_332F92
    9449267,   // TPMS3_332F90
    6499363,   // TPMS2_232C63
    14692163,  // TPMS4_432FE0
    2240579,   // TPMS4_433022
  };
}
