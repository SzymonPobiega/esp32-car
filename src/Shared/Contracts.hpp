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
  static constexpr int16_t TYPE_DTC = 0x0C;
  static constexpr int16_t TYPE_HEATER = 0x0D;
  static constexpr int16_t TYPE_HEATER_COMMAND = 0x0E;
  static constexpr int16_t TYPE_OBD_COMMAND = 0x0F;
  static constexpr int16_t TYPE_SHELLY_BRIDGE_KEEPALIVE = 0x41;

  // Node addresses and other per-vehicle values live in Shared/Vehicle.hpp

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
  static constexpr int16_t OBD_STATUS_RELEASED = 3;   // staying off the dongle so a phone can connect
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
    int16_t coolant;     // deg C, from 2105 @ 7E0 (A - 40)
    int16_t battery_cv;  // starter battery as the engine ECU sees it, centi-volts (2142 @ 7E0, mV / 10)
  };

  // DtcPacket modules (index into stored/pending, low bits of source)
  static constexpr uint8_t DTC_ENGINE = 0;    // 7E0, OBD-II mode 03 / 07
  static constexpr uint8_t DTC_GEARBOX = 1;   // 7E1, OBD-II mode 03 / 07
  static constexpr uint8_t DTC_ABS = 2;       // 7B0, Toyota KWP 18 / 13
  static constexpr uint8_t DTC_MODULES = 3;
  static constexpr uint8_t DTC_PENDING = 0x80;   // set in source for a pending code
  static constexpr uint8_t DTC_UNUSED = 0xFF;    // empty slot in codes / source
  static constexpr int16_t DTC_NO_ANSWER = -1;   // module did not answer the request
  static constexpr int DTC_MAX_CODES = 8;

  // Fault code report. The OBD connector scans when the vehicle starts answering and
  // then every minute, and repeats the latest report every few seconds.
  struct PACKED DtcPacket {
    int16_t type;
    uint16_t scan;                    // number of completed scans since the dongle connected
    int16_t mil;                      // check engine lamp from 0101 (0/1), OBD_NO_VALUE if unknown
    int16_t stored[DTC_MODULES];      // confirmed codes per module, or DTC_NO_ANSWER
    int16_t pending[DTC_MODULES];     // pending codes per module, or DTC_NO_ANSWER
    uint16_t codes[DTC_MAX_CODES];    // first codes found, SAE 2-byte format (e.g. 0x0087 = P0087)
    uint8_t source[DTC_MAX_CODES];    // module of each code, | DTC_PENDING; DTC_UNUSED if empty
  };

  // HeaterPacket.link
  static constexpr int16_t HEATER_LINK_NONE = 0;   // looking for / connecting to the heater
  static constexpr int16_t HEATER_LINK_OK = 1;     // heater answering status queries
  static constexpr int16_t HEATER_LINK_RELEASED = 2;   // stays off BLE so a phone can connect (HEATER_CMD_RELEASE)

  // HeaterPacket.state: the HCalory state byte as reported
  static constexpr int16_t HEATER_STATE_OFF = 0x00;
  static constexpr int16_t HEATER_STATE_COOLDOWN = 0x41;
  static constexpr int16_t HEATER_STATE_COOLDOWN_STARTING = 0x43;
  static constexpr int16_t HEATER_STATE_COOLDOWN_RECEIVED = 0x45;
  static constexpr int16_t HEATER_STATE_IGNITION_RECEIVED = 0x80;
  static constexpr int16_t HEATER_STATE_IGNITION_STARTING = 0x81;
  static constexpr int16_t HEATER_STATE_IGNITING = 0x83;
  static constexpr int16_t HEATER_STATE_RUNNING = 0x85;
  static constexpr int16_t HEATER_STATE_HEATING = 0x87;
  static constexpr int16_t HEATER_STATE_ERROR = 0xFF;

  // HeaterPacket.mode
  static constexpr int16_t HEATER_MODE_OFF = 0;
  static constexpr int16_t HEATER_MODE_THERMOSTAT = 1;   // setting = target deg C
  static constexpr int16_t HEATER_MODE_LEVEL = 2;        // setting = level 1-6
  static constexpr int16_t HEATER_MODE_IGNITION_FAILED = 8;

  // HeaterPacket.ack_result
  static constexpr int16_t HEATER_ACK_NONE = 0;      // no command received since the connector started
  static constexpr int16_t HEATER_ACK_OK = 1;        // sent to the heater
  static constexpr int16_t HEATER_ACK_NO_LINK = 2;   // heater not connected
  static constexpr int16_t HEATER_ACK_FAILED = 3;    // BLE write failed, or unknown command

  // HeaterPacket.clock
  static constexpr int16_t HEATER_CLOCK_NONE = 0;    // no RTC: the morning run is stored but never fires
  static constexpr int16_t HEATER_CLOCK_OK = 1;
  static constexpr int16_t HEATER_CLOCK_NOT_SET = 2; // RTC found, but its time was never set or was lost

  // HeaterPacket.run_state: the morning run, as the connector executes it
  static constexpr int16_t HEATER_RUN_IDLE = 0;          // waiting for the run time (or not armed)
  static constexpr int16_t HEATER_RUN_STARTING = 1;      // run time reached, starting the heater
  static constexpr int16_t HEATER_RUN_RUNNING = 2;
  static constexpr int16_t HEATER_RUN_STOPPING = 3;      // run over, stopping the heater
  static constexpr int16_t HEATER_RUN_DONE = 4;          // last run completed
  static constexpr int16_t HEATER_RUN_SKIPPED_LOW_BATTERY = 5;
  static constexpr int16_t HEATER_RUN_FAILED = 6;        // heater not reachable, or it didn't ignite

  static constexpr int16_t HEATER_RUN_LENGTH_MIN = 30;   // from ignition
  static constexpr int16_t HEATER_RUN_MIN_DECIVOLTS = 110;   // whole volts: skips at a 10 V reading, i.e. under ~11 V

  // HCalory diesel heater status, sent every second. Values from state to cabin_temp are
  // only meaningful while link is HEATER_LINK_OK; the rest come from the connector itself.
  struct PACKED HeaterPacket {
    int16_t type;
    int16_t link;
    int16_t state;
    int16_t mode;
    int16_t setting;
    int16_t volts;       // deci-volts (the heater reports whole volts)
    int16_t case_temp;   // deci-degrees, heat exchanger
    int16_t cabin_temp;  // deci-degrees, the heater's air intake sensor
    int16_t ack_seq;     // seq of the last HeaterCommandPacket received
    int16_t ack_result;  // HEATER_ACK_*
    int16_t run_at;      // morning run start, minutes after midnight
    int16_t run_armed;   // 0/1
    int16_t clock;       // HEATER_CLOCK_*
    int16_t time;        // connector's clock, minutes after midnight; -1 unless clock is OK
    int16_t run_state;   // HEATER_RUN_*
  };

  // HeaterCommandPacket.command, with the HCalory command byte each one sends
  static constexpr int16_t HEATER_CMD_STOP = 1;             // 01
  static constexpr int16_t HEATER_CMD_START = 2;            // 02
  static constexpr int16_t HEATER_CMD_UP = 3;               // 03, level or target temperature
  static constexpr int16_t HEATER_CMD_DOWN = 4;             // 04
  static constexpr int16_t HEATER_CMD_MODE_THERMOSTAT = 6;  // 06
  static constexpr int16_t HEATER_CMD_MODE_LEVEL = 7;       // 07
  // Handled by the connector: arg = minutes after midnight, or 0/1 to disarm/arm
  static constexpr int16_t HEATER_CMD_RUN_AT = 0x10;
  static constexpr int16_t HEATER_CMD_RUN_ARM = 0x11;
  static constexpr int16_t HEATER_CMD_SET_TIME = 0x12;      // arg = minutes after midnight, seconds = 0
  // arg 1: disconnect from the heater and stay off it for HEATER_RELEASE_MS, so the phone app can
  // connect; the dashboard repeats it every HEATER_RELEASE_REFRESH_MS while its switch is on.
  // arg 0: reconnect now. Sent with seq 0: not acknowledged, and run every time it arrives.
  static constexpr int16_t HEATER_CMD_RELEASE = 0x13;
  // Phone switches (heater and OBD): a connector stays off its BLE device this long after
  // each release request; the dashboard repeats the request this often while the switch is on
  static constexpr uint32_t PHONE_RELEASE_MS = 30000;
  static constexpr uint32_t PHONE_RELEASE_REFRESH_MS = 15000;
  static constexpr uint32_t HEATER_RELEASE_MS = PHONE_RELEASE_MS;
  static constexpr uint32_t HEATER_RELEASE_REFRESH_MS = PHONE_RELEASE_REFRESH_MS;

  // Dashboard -> OBD connector (through the modem). Fire and forget, repeated while needed.
  static constexpr int16_t OBD_CMD_RELEASE = 1;   // arg 1: stay off the dongle for PHONE_RELEASE_MS; 0: reconnect now
  struct PACKED ObdCommandPacket {
    int16_t type;
    int16_t command;   // OBD_CMD_*
    int16_t arg;
  };

  // Dashboard -> heater connector (through the modem). The connector echoes seq in
  // HeaterPacket.ack_seq and runs a repeated seq only once, so a lost ack can be resent
  // (seq 0 is never acknowledged: HEATER_CMD_RELEASE only).
  struct PACKED HeaterCommandPacket {
    int16_t type;
    int16_t seq;         // 0 only for HEATER_CMD_RELEASE
    int16_t command;     // HEATER_CMD_*
    int16_t arg;
  };

  static_assert(sizeof(GyroAnglePacket) == 10, "GyroAnglePacket must be 10 bytes");
  static_assert(sizeof(HeaterPacket) == 30, "HeaterPacket must be 30 bytes");
  static_assert(sizeof(HeaterCommandPacket) == 8, "HeaterCommandPacket must be 8 bytes");
  static_assert(sizeof(ObdPacket) == 22, "ObdPacket must be 22 bytes");
  static_assert(sizeof(ObdCommandPacket) == 6, "ObdCommandPacket must be 6 bytes");
  static_assert(sizeof(DtcPacket) == 42, "DtcPacket must be 42 bytes");
  static_assert(sizeof(GyroHeightPacket) == 4, "GyroHeightPacket must be 4 bytes");
  static_assert(sizeof(TpmsPacket) == 16, "TpmsPacket must be 16 bytes");
  static_assert(sizeof(BmsPacket) == 12, "BmsPacket must be 12 bytes");
}
