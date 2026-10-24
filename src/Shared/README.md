# Shared

Headers shared by all sketches. A sketch includes them through its own `ContractsInclude.hpp`, which points at `../Shared`. See the [main README](../../README.md#concepts) for the concepts.

| File | Contents |
| --- | --- |
| `Contracts.hpp` | Packet types and packed structs carried over ESP-NOW and UART |
| `ModemProtocol.hpp` | UART framing between the Toyota ESP-NOW modem (C6) and ToyotaMonitor (P4): sync, kinds, heartbeat, CRC-16 |
| `Vehicle.hpp` | Selects the vehicle profile from `VEHICLE_TOYOTA` / `VEHICLE_VOLVO`; `#error` if none or both are defined |
| `VehicleSelect.hpp` | Vehicle for Arduino IDE builds of the shared connectors (leave commented out when using `flash.ps1`) |
| `Vehicles/Volvo.hpp`, `Vehicles/Toyota.hpp` | Per-vehicle values: ESP-NOW destinations and node MACs, BLE sensor addresses, TPMS sensor IDs |

When adding a vehicle-specific value, add it to both profiles under the same name, so the shared connectors compile for either vehicle.
