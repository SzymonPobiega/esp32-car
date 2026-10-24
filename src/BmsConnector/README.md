# BmsConnector

Reads a JBD / Xiaoxiang battery management system over BLE and sends the house battery state over ESP-NOW every second. Used in both vehicles.

- **Board:** ESP32 (classic), `esp32:esp32:esp32`.
- **Vehicle:** selected at build time (`-DVEHICLE_TOYOTA` / `-DVEHICLE_VOLVO`, see the [main README](../../README.md#vehicle-profiles)).

## Configuration

From the vehicle profile ([Volvo](../Shared/Vehicles/Volvo.hpp), [Toyota](../Shared/Vehicles/Toyota.hpp)):

| Setting | Volvo | Toyota |
| --- | --- | --- |
| `Vehicle::BMS_BLE_MAC` | `A5:C2:37:62:EF:6E` | `A5:C2:37:2B:DC:A1` |
| `Vehicle::BMS_DEST_MAC` | broadcast (both displays) | the ESP-NOW modem (unicast) |

## How it works

1. A non-blocking state machine connects to the BMS as a GATT client: Idle → Discovering → Subscribing → Ready, with a 5 s back-off after a failure. It uses service `FF00`, notify characteristic `FF01` and write characteristic `FF02`.
2. Once Ready, it sends the basic-info command `DD A5 03 00 FF FD 77` every second.
3. It reassembles the `DD … 77` reply from the notifications and extracts voltage, current, remaining capacity, state of charge and temperature.
4. Every second it sends the latest `BmsPacket` to `Vehicle::BMS_DEST_MAC` and prints it to the serial log.

## Output

`BmsPacket` (type `0x05`, 12 bytes): `volts` (0.01 V), `amps` (0.01 A, signed), `soc_percent`, `remaining` (0.01 Ah), `temp` (0.1 °C). See [Contracts.hpp](../Shared/Contracts.hpp).
