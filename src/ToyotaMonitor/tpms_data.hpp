#pragma once

#include <Arduino.h>

namespace tpms {

constexpr size_t kMaxSensors = 6;

// Sensor ID in TpmsConnector pings sent before any sensor has been received
constexpr uint32_t kNoSensor = 0;

extern volatile uint32_t sequence;
extern volatile uint32_t latestPressure[kMaxSensors];
extern volatile int16_t latestTemp[kMaxSensors];
extern volatile bool hasData;
extern volatile uint32_t lastUpdated[kMaxSensors];
extern volatile uint32_t last_update;

struct TpmsPacket {
  uint32_t sequence;
  uint32_t sensorId;
  uint32_t pressure;
  int16_t temp;
} __attribute__((packed));

// uint32_t getIndex(uint32_t sensor);
bool handleTpms(const TpmsPacket& pkt);
}
