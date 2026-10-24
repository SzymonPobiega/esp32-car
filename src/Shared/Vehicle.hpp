#pragma once

// Selects the vehicle profile: everything that differs between the Volvo and the Toyota
// (ESP-NOW destinations, BLE sensor addresses, TPMS sensor IDs).
//
// - Sketches used in only one vehicle define it themselves in their ContractsInclude.hpp.
// - Shared connectors (BMS, Gyro, TPMS) get it from the build: flash.ps1 passes
//   -DVEHICLE_TOYOTA or -DVEHICLE_VOLVO. For Arduino IDE builds, see VehicleSelect.hpp.

#include "VehicleSelect.hpp"

#if defined(VEHICLE_TOYOTA) && defined(VEHICLE_VOLVO)
  #error "Both VEHICLE_TOYOTA and VEHICLE_VOLVO are defined"
#elif defined(VEHICLE_TOYOTA)
  #include "Vehicles/Toyota.hpp"
#elif defined(VEHICLE_VOLVO)
  #include "Vehicles/Volvo.hpp"
#else
  #error "No vehicle selected: build with flash.ps1, or uncomment one in Shared/VehicleSelect.hpp"
#endif
