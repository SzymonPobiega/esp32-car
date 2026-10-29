# ESP32 vehicle dashboards

ESP32 firmware for in-vehicle dashboards in two vehicles: a **Volvo camper** and a **Toyota Land Cruiser**. Small headless ESP32 boards read sensors (tyre pressure, attitude, house battery, OBD-II) and send the readings over ESP-NOW to touchscreen dashboards. On the Volvo, the dashboards also switch lights and camera inputs.

![Monitor](tpms-screen.jpg)

## Why?

- because off-the-shelf systems look ugly
- because I want to have full control over what kind of alarms I have
- because I own a 6x6 truck and there are no systems out there that support vehicles (yes, there are some for twin tires on the rear axle but they look ugly)
- because it is fun!

## Vehicles

| Vehicle | Architecture | Dashboard |
| --- | --- | --- |
| Volvo camper | [VOLVO.md](VOLVO.md) | VolvoMonitor (ESP32-S3) and VolvoControlPanel (ESP32-S3) |
| Toyota Land Cruiser | [TOYOTA.md](TOYOTA.md) | ToyotaMonitor (ESP32-P4) with an ESP32-C6 ESP-NOW modem |

## Modules

Every module is an Arduino sketch in `src/`. Each has its own README.

| Module | Role | Volvo | Toyota |
| --- | --- | :---: | :---: |
| [TpmsConnector](src/TpmsConnector/README.md) | BLE TPMS sensors → ESP-NOW | ✓ | ✓ |
| [BmsConnector](src/BmsConnector/README.md) | JBD battery BMS (BLE) → ESP-NOW | ✓ | ✓ |
| [GyroConnector](src/GyroConnector/README.md) | WitMotion WT901 IMU (BLE) → ESP-NOW | ✓ | ✓ |
| [ObdConnector](src/ObdConnector/README.md) | ELM327 BLE OBD-II dongle → ESP-NOW | | ✓ |
| [ToyotaMonitor](src/ToyotaMonitor/README.md) | Toyota dashboard (ESP32-P4) | | ✓ |
| [EspNowUartBridge](src/EspNowUartBridge/README.md) | ESP-NOW modem on the C6 next to the P4 | | ✓ |
| [VolvoMonitor](src/VolvoMonitor/README.md) | Volvo dashboard (ESP32-S3) | ✓ | |
| [VolvoControlPanel](src/VolvoControlPanel/README.md) | Volvo camper control touchscreen (ESP32-S3) | ✓ | |
| [VolvoCameraController](src/VolvoCameraController/README.md) | Camera input selector relays (LilyGO) | ✓ | |
| [VolvoWorkLightsController](src/VolvoWorkLightsController/README.md) | Exterior work light relays and buttons (LilyGO) | ✓ | |
| [ShellyConnectorUartBridge](src/ShellyConnectorUartBridge/README.md) | ESP-NOW ↔ UART bridge for the Shelly lights | ✓ | |
| [ShellyConnector](src/ShellyConnector/README.md) | UART ↔ Shelly Plus RGBW PM (Wi-Fi HTTP RPC) | ✓ | |

`src/Monitor` and `src/SimpleMonitor` are legacy and no longer maintained. `src/Shared` holds the code all sketches share (see [Packet contracts](#packet-contracts) and [Vehicle profiles](#vehicle-profiles)).

## Concepts

### Transports

- **ESP-NOW is the backbone.** Every board-to-board link uses it, except where a board is busy with Wi-Fi or has no radio.
- **BLE** is used only to read third-party sensors (TPMS sensors, BMS, IMU, OBD dongle). The connector boards run BLE and ESP-NOW side by side on the same radio. BLE never carries traffic between this project's own boards.
- **UART** is used only where one board must bridge ESP-NOW to a board that cannot take part in it: the Shelly client, which is joined to the Shelly's Wi-Fi network, and the ESP32-P4, which has no radio of its own.

### ESP-NOW

ESP-NOW sends short frames (up to 250 bytes) directly between ESP32s, with no access point.

- **Setup.** Every node calls `WiFi.mode(WIFI_STA)`, disconnects, calls `esp_now_init()` and adds peers with `encrypt = false`. Nodes never join an access point, so all of them stay on the default Wi-Fi channel (1). The Toyota modem and the OBD connector set channel 1 explicitly.
- **Unicast vs broadcast.** Unicast frames are acknowledged at the MAC layer and retried by the radio; the send callback reports success or failure. Broadcast (`FF:FF:FF:FF:FF:FF`) reaches every listener but is never acknowledged. Use unicast when there is one receiver; broadcast only when several boards need the same data (for example the Volvo status messages that both Volvo displays show).
- **Application retries.** The Volvo displays send commands through `espnow_retry`: it keeps the last packet per peer and, when the send callback reports a failure, resends it with exponential back-off (30, 60, 120, 240, 480 ms, at most 5 times). A newer command to the same peer replaces the pending one.
- **Pings.** Connectors repeat their last packet periodically (TPMS every 5 s, BMS every 1 s, OBD at least every 1 s), so dashboards can show whether each connector is reachable even when no new reading arrives.
- **No security.** Traffic is unencrypted and unauthenticated; any nearby ESP32 can send packets. Receivers only check the packet type and size.

### Packet contracts

All packets are defined in [`src/Shared/Contracts.hpp`](src/Shared/Contracts.hpp). Every sketch includes it through its own `ContractsInclude.hpp`, which points at `../Shared`.

Every packet is a `PACKED` struct of little-endian integers whose first field is `int16_t type`. It is sent byte for byte, with no header or checksum. Receivers identify a packet by its type and its exact size. Physical values are scaled integers (centi-volts, centi-degrees and so on), so no floats cross the wire.

| Type | Struct | Size (bytes) | Fields (units) | Sent by |
| --- | --- | --- | --- | --- |
| `0x02` TPMS | `TpmsPacket` | 16 | sequence, sensorId, pressure (Pa), temp (0.01 °C) | TpmsConnector |
| `0x03` GYRO_ANGLE | `GyroAnglePacket` | 10 | roll, pitch, yaw (0.01°), height (whole metres; the comment says cm) | GyroConnector |
| `0x04` GYRO_HEIGHT | `GyroHeightPacket` | 4 | height (cm) | not sent (code commented out) |
| `0x05` BMS | `BmsPacket` | 12 | volts (0.01 V), amps (0.01 A, signed), soc (%), remaining (0.01 Ah), temp (0.1 °C) | BmsConnector |
| `0x06` CAMERA_SELECT | `CameraSelectPacket` | 4 | input 1–4 | VolvoMonitor |
| `0x07` WORKLIGHT_SWITCH | `WorkLightsSwitchPacket` | 6 | input 1–4, state 0/1 | Volvo displays |
| `0x08` WORKLIGHT_STATUS | `WorkLightsStatusPacket` | 10 | relay1–relay4 (0/1) | VolvoWorkLightsController |
| `0x09` LIGHT_SWITCH | `LightsSwitchPacket` | 6 | zone 1–4, state 0/1 | Volvo displays |
| `0x0A` LIGHT_STATUS | `LightsStatusPacket` | 10 | zone1–zone4 (0/1) | ShellyConnector |
| `0x0B` OBD | `ObdPacket` | 18 | status, gear, lockup, ATF pan/outlet (°C), MAP, baro (kPa), boost (0.01 bar) | ObdConnector |

### UART links

Both UART links run at 115200 baud, 8N1.

- **Shelly bridge ↔ ShellyConnector** (Volvo): `[len:1][payload:len]`, where the payload is the same packed struct ESP-NOW carries. The receiver reads the length byte, collects exactly that many bytes, and drops a frame that stalls for more than 50 ms, so it resynchronises on the next frame.
- **ESP-NOW modem → ToyotaMonitor** (Toyota): `[AA 55][kind:1][len:2][payload][crc16:2]`, defined in [`src/Shared/ModemProtocol.hpp`](src/Shared/ModemProtocol.hpp). The CRC lets the receiver discard noise, such as the C6's boot messages on the same wire. Besides received ESP-NOW packets (with the sender's MAC and RSSI), the modem sends a heartbeat every second.

### Vehicle profiles

Everything that differs between the two vehicles lives in one profile per vehicle:

- [`src/Shared/Vehicles/Volvo.hpp`](src/Shared/Vehicles/Volvo.hpp)
- [`src/Shared/Vehicles/Toyota.hpp`](src/Shared/Vehicles/Toyota.hpp)

A profile holds the ESP-NOW destination of each connector (`DISPLAY_MAC`, `BMS_DEST_MAC`), the BLE addresses of the vehicle's sensors (`BMS_BLE_MAC`, `GYRO_BLE_MAC`, `OBD_DONGLE_MAC`), the MACs of the Volvo actuator boards, and the TPMS sensor IDs in gauge order. Sketches include [`src/Shared/Vehicle.hpp`](src/Shared/Vehicle.hpp) and use `Vehicle::...`; they never hard-code addresses.

The vehicle is chosen at build time:

- **Single-vehicle sketches** (ToyotaMonitor, ObdConnector, VolvoMonitor, VolvoControlPanel) define their vehicle in their `ContractsInclude.hpp`.
- **Shared connectors** (TpmsConnector, BmsConnector, GyroConnector) get it from the build: `-DVEHICLE_TOYOTA` or `-DVEHICLE_VOLVO`. Building one without a vehicle fails with an `#error`, so a connector can't be flashed with the wrong car's addresses by accident. For Arduino IDE builds, uncomment one line in [`src/Shared/VehicleSelect.hpp`](src/Shared/VehicleSelect.hpp).

Each shared connector prints its vehicle at boot, e.g. `BMS gateway (Toyota)`.

### Building and flashing

[`src/flash.ps1`](src/flash.ps1) builds a sketch for the right vehicle and flashes it. It reads the MAC of the board on the given port and looks it up in [`src/boards.json`](src/boards.json), which lists every physical board with its sketch, vehicle and board settings (FQBN).

```powershell
cd src
./flash.ps1 -Port COM7                                   # identify the board, build its sketch for its vehicle, flash
./flash.ps1 -Port COM7 -Sketch BmsConnector -Vehicle Volvo -Fqbn esp32:esp32:esp32   # board not in boards.json yet
./flash.ps1 -Sketch TpmsConnector -Vehicle Toyota -BuildOnly                          # build only
```

The script refuses to flash a board with another vehicle's build unless `-Force` is given. It uses the `arduino-cli` and `esptool` that come with Arduino IDE 2 and the esp32 core. Add each new board to `boards.json`; the Volvo entries still need their `fqbn` filled in.
