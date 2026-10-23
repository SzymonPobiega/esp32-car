#pragma once
#include <stdint.h>

#if defined(__GNUC__)
  #define PACKED __attribute__((packed))
#else
  #define PACKED
#endif

namespace Contracts {

  static constexpr uint8_t SYNC1 = 0xAA;
  static constexpr uint8_t SYNC2 = 0x55;
  static constexpr int16_t TYPE_TPMS = 0x02;
  static constexpr int16_t TYPE_GYRO_ANGLE = 0x03;
  static constexpr int16_t TYPE_GYRO_HEIGHT = 0x04;
  static constexpr int16_t TYPE_BMS = 0x05;
  static constexpr int16_t TYPE_CAMERA_SELECT = 0x06;
  static constexpr int16_t TYPE_WORKLIGHT_SWITCH = 0x07;
  static constexpr int16_t TYPE_WORKLIGHT_STATUS = 0x08;
  static constexpr int16_t TYPE_LIGHT_SWITCH = 0x09;
  static constexpr int16_t TYPE_LIGHT_STATUS = 0x0A;
  static constexpr int16_t TYPE_OBD = 0x0B;
  static constexpr int16_t TYPE_SHELLY_BRIDGE_KEEPALIVE = 0x41;

  //S3
  inline constexpr uint8_t S3_5_1_MAC[] = { 0x50, 0x78, 0x7D, 0x13, 0x22, 0xF8 };

  //S3 7 inch
  inline constexpr uint8_t S3_7_1_MAC[] = { 0xD0, 0xCF, 0x13, 0x06, 0x79, 0xA0 };

  //Volvo camera control LilyGO
  inline constexpr uint8_t LILYGO_CAMERA_MAC[] = { 0x94, 0x51, 0xDC, 0x47, 0xAE, 0xC4 };

  //Volvo work lights control LilyGO
  inline constexpr uint8_t LILYGO_WORKLIGHT_MAC[] = { 0xE0, 0x8C, 0xFE, 0x75, 0xDA, 0x54 };

  //Bridge from ESP-NOW to Shelly connector via UART
  inline constexpr uint8_t SHELLY_BRIDGE_MAC[] = { 0xD4, 0xE9, 0xF4, 0xA8, 0x7E, 0x34 };

  //P4
  inline constexpr uint8_t P4_4_1_MAC[] = { 0x9C, 0x13, 0x9E, 0xC3, 0xFF, 0x60 };

  struct PACKED GyroAnglePacket {
    int16_t type;
    int16_t roll_deg;   // centi-degrees
    int16_t pitch_deg;  // centi-degrees
    int16_t yaw_deg;    // centi-degrees
    int16_t height;     // centi-meters
  };

  struct PACKED CameraSelectPacket {
    int16_t type;
    int16_t input;
  };

  struct PACKED LightsSwitchPacket {
    int16_t type;
    int16_t zone;
    int16_t state;
  };

  struct PACKED LightsStatusPacket {
    int16_t type;
    int16_t zone1;
    int16_t zone2;
    int16_t zone3;
    int16_t zone4;
  };

  struct PACKED WorkLightsSwitchPacket {
    int16_t type;
    int16_t input;
    int16_t state;
  };

  struct PACKED WorkLightsStatusPacket {
    int16_t type;
    int16_t relay1;
    int16_t relay2;
    int16_t relay3;
    int16_t relay4;
  };

  struct PACKED BmsPacket {
    int16_t type;
    int16_t volts;   // centi-volts
    int16_t amps;  // centi-amps
    int16_t soc_percent;    // percent
    int16_t remaining;     // centi-amp-h
    int16_t temp; //deci-deg
  };

  struct PACKED WakeUpPacket {
    int16_t type;
    int16_t volts;   // centi-volts
    int16_t amps;  // centi-amps
    int16_t soc_percent;    // percent
    int16_t remaining;     // centi-amp-h
    int16_t temp; //deci-deg
  };

  struct PACKED GyroHeightPacket {
    int16_t type;
    int16_t height;     // centi-meters
  };

  struct PACKED TpmsPacket {
    int16_t type;
    uint32_t sequence;
    uint32_t sensorId;
    uint32_t pressure;
    int16_t temp;
  };

  // ObdPacket.status
  static constexpr int16_t OBD_STATUS_NO_DONGLE = 0;  // looking for / connecting to the BLE dongle
  static constexpr int16_t OBD_STATUS_NO_ECU = 1;     // dongle connected, vehicle not answering
  static constexpr int16_t OBD_STATUS_OK = 2;         // vehicle answering

  // Value not available (request failed or not supported)
  static constexpr int16_t OBD_NO_VALUE = -32768;

  struct PACKED ObdPacket {
    int16_t type;
    int16_t status;
    int16_t gear;        // raw gear code from 21DA (A & 7)
    int16_t lockup;      // torque converter lockup flag from 21DA (bit 4 of A)
    int16_t atf_pan;     // deg C
    int16_t atf_outlet;  // deg C
    int16_t map_kpa;     // manifold absolute pressure, kPa
    int16_t baro_kpa;    // barometric pressure, kPa
    int16_t boost;       // centi-bar (MAP - baro)
  };

  static_assert(sizeof(GyroAnglePacket) == 10, "GyroAnglePacket must be 10 bytes");
  static_assert(sizeof(ObdPacket) == 18, "ObdPacket must be 18 bytes");
  static_assert(sizeof(GyroHeightPacket) == 4, "GyroHeightPacket must be 4 bytes");
  static_assert(sizeof(TpmsPacket) == 16, "TpmsPacket must be 16 bytes");
  static_assert(sizeof(BmsPacket) == 12, "BmsPacket must be 12 bytes");
}
