# GyroConnector

Reads attitude (roll, pitch, yaw) and altitude from a WitMotion WT901 BLE IMU and sends them over ESP-NOW at most once a second. Used in both vehicles.

- **Board:** ESP32 (classic), `esp32:esp32:esp32`.
- **Vehicle:** selected at build time (`-DVEHICLE_TOYOTA` / `-DVEHICLE_VOLVO`, see the [main README](../../README.md#vehicle-profiles)).

## Configuration

`Vehicle::GYRO_BLE_MAC` in the vehicle profile: `e0:3b:b5:fe:27:8e` on the Volvo, `db:dd:a8:10:0b:fa` (WT901BLE68) on the Toyota. Both are random static addresses. When it is empty the connector logs `No gyro configured` once and does nothing else. Packets go to `Vehicle::DISPLAY_MAC`.

## How it works

1. It connects as a GATT client to the WT901 (random address), subscribes to notify characteristic `FFE4` on service `FFE5`, and looks up the write characteristic `FFE9`. If the link drops, `loop()` reconnects.
2. Once a second it writes `FF AA 27 45 00` to `FFE9` to request pressure and altitude.
3. It parses the 20-byte WitMotion frames: `55 61` carries roll, pitch and yaw; `55 71` with register `0x45` carries pressure and altitude.
4. At most once a second it sends a `GyroAnglePacket` with the angles and the latest altitude.

## Output

`GyroAnglePacket` (type `0x03`, 10 bytes): `roll_deg`, `pitch_deg`, `yaw_deg` (0.01°), `height` (whole metres, although the comment in Contracts.hpp says centimetres). See [Contracts.hpp](../Shared/Contracts.hpp).

The separate `GyroHeightPacket` sender is commented out.
