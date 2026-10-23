#pragma once

#include <Arduino.h>
#include "ContractsInclude.hpp"

// Link to the ESP-NOW modem (EspNowUartBridge) running on the on-board C6.
namespace modem {

struct Message {
  uint8_t  srcMac[6];
  int8_t   rssi;
  uint16_t len;
  uint8_t  data[Modem::MAX_ESPNOW_DATA];
};

struct Stats {
  uint32_t bytesReceived;
  uint32_t framesOk;
  uint32_t crcErrors;
  uint32_t lengthErrors;
  uint32_t lastHeartbeatMs;
  bool     hasHeartbeat;
  Modem::Heartbeat heartbeat;  // last heartbeat received from the modem
};

void begin();

// Non-blocking. Parses whatever bytes are available and returns true when an
// ESP-NOW message is ready in out. Call repeatedly until it returns false.
bool poll(Message& out);

// True if a heartbeat arrived within Modem::HEARTBEAT_TIMEOUT_MS
bool modemAlive();

const Stats& stats();

} // namespace modem
