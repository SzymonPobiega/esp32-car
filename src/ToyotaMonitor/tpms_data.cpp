#include "tpms_data.hpp"

namespace tpms {

volatile uint32_t sequence = 0;
volatile uint32_t latestPressure[kMaxSensors] = {0};
volatile int16_t latestTemp[kMaxSensors]     = {0};
volatile bool     hasData                 = false;
volatile uint32_t lastUpdated[kMaxSensors]    = {0};
volatile uint32_t last_update                 = 0;

uint32_t getIndex(uint32_t sensor)
{
    if (sensor == 7549971)  return 0;
    else if (sensor == 1126691)  return 1;
    else if (sensor == 14554163) return 2;
    else if (sensor == 4592707)  return 3;
    return 0;
}

bool handleTpms(const TpmsPacket& pkt) {
    uint32_t now = millis();

    // Connector ping before any sensor has been received: link is alive, nothing to show
    if (pkt.sensorId == kNoSensor) {
        last_update = now;
        return true;
    }

    uint32_t idx = getIndex(pkt.sensorId);
    if (idx >= kMaxSensors) {
        return false;
    }

    if (pkt.sequence > sequence) {
        sequence = pkt.sequence;
        latestPressure[idx] = pkt.pressure;
        latestTemp[idx]     = pkt.temp;
        lastUpdated[idx] = now;
        hasData = true;
        Serial.print("Dupa!");
    }
    
    last_update = now;
    return true;
}

} // namespace tpms
