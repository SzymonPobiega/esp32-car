#include "tpms_data.hpp"

namespace tpms {

volatile uint32_t sequence = 0;
volatile uint32_t latestPressure[kMaxSensors] = {0};
volatile int16_t latestTemp[kMaxSensors]     = {0};
volatile bool     hasData                 = false;
volatile uint32_t lastUpdated[kMaxSensors]    = {0};
volatile uint32_t totalPeriods[kMaxSensors]   = {0};
volatile uint32_t last_update                 = 0;
volatile uint32_t start_time                  = 0;

static_assert(sizeof(Vehicle::TPMS_SENSORS) / sizeof(Vehicle::TPMS_SENSORS[0]) <= kMaxSensors,
              "More TPMS sensors in the vehicle profile than gauges");

// Gauge index of a sensor (Vehicle::TPMS_SENSORS order), kMaxSensors if it is not ours
uint32_t getIndex(uint32_t sensor)
{
    for (uint32_t i = 0; i < sizeof(Vehicle::TPMS_SENSORS) / sizeof(Vehicle::TPMS_SENSORS[0]); i++) {
        if (Vehicle::TPMS_SENSORS[i] == sensor) return i;
    }
    return kMaxSensors;
}

bool processTpms(Contracts::TpmsPacket& pkt) {
    
    uint32_t now = millis();
    uint32_t idx = getIndex(pkt.sensorId);
    if (idx >= kMaxSensors) {
        return false;
    }

    if (pkt.sequence > sequence) {
        sequence = pkt.sequence;
        latestPressure[idx] = pkt.pressure;
        latestTemp[idx]     = pkt.temp;
        //We only increase the total number of periods per sensor if the packet is a new one (not a retransmission)
        totalPeriods[idx]++;
        lastUpdated[idx] = now;
        hasData = true;
        //Serial.print("Dupa!");
    }
    
    last_update = now;
    return true;
}

} // namespace tpms
