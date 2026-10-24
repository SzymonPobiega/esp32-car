# TpmsConnector

Reads Chinese BLE TPMS sensors (LEEPEE type) and forwards each reading over ESP-NOW. Used in both vehicles: it sends to VolvoMonitor on the Volvo and to the ESP-NOW modem on the Toyota.

- **Board:** ESP32 (classic), `esp32:esp32:esp32`. A separate board is used because some ESP32-S3 display boards have poor Bluetooth reception (the antenna sits under the screen).
- **Vehicle:** selected at build time (`-DVEHICLE_TOYOTA` / `-DVEHICLE_VOLVO`, see the [main README](../../README.md#vehicle-profiles)).

## How it works

1. A NimBLE active scan runs continuously (120 s scans, restarted every 5 s) with duplicates enabled, and keeps only devices whose name starts with `TPMS`.
2. From each advert of 32 bytes or more it reads the sensor ID, pressure and temperature (below), increments `sequence` and sends a `TpmsPacket` to `Vehicle::DISPLAY_MAC`.
3. Every 5 s it re-sends the last packet as a ping. Before any sensor has been heard, the ping has sensor ID 0, which dashboards treat as "connected, no reading yet". Repeats carry the same `sequence`, so dashboards ignore them as readings.

## BLE protocol

The sensors publish their readings in BLE advertisements (byte numbers are 0-based, counted in the raw advert):

- bytes 14–16: unique part of the sensor ID (bytes 11–13 are common to all sensors)
- bytes 17–20 (little-endian): pressure in pascals
- bytes 21–22 (little-endian): signed temperature in 0.01 °C

Each sensor has a 3-byte unique ID that is also part of its name. In the [nRF Connect](https://www.nordicsemi.com/) app, sensor `233111` appears as `TPMS2_233111`, with this advertisement data:

| LEN | TYPE | VALUE |
|-----|------|-------|
| 2   | 0x01 | 0x05 |
| 3   | 0x03 | 0xB0FB |
| 19  | 0xFF | 0x000181EACA233111CDD80300FD0800005F00 |
| 13  | 0x09 | 0x54504D53325F323333313131 |

The type `0x09` field is the name, `TPMS2_233111`, in ASCII. In the manufacturer field (`0xFF`), bytes 7–9 hold the same number (233111) as a 24-bit value. Its most significant digit is the intended position (1 front left, 2 front right, 3 rear left, 4 rear right); the remaining 5 digits are the sensor's unique ID. The next 4 bytes hold the pressure in pascals and the next 2 the temperature in 0.01 °C.

![Sensor app](../../tpms-app.png)

## Output

`TpmsPacket` (type `0x02`, 16 bytes): `sequence`, `sensorId`, `pressure` (Pa), `temp` (0.01 °C). See [Contracts.hpp](../Shared/Contracts.hpp).

The mapping from sensor ID to wheel position lives in the dashboards, from `Vehicle::TPMS_SENSORS` in the vehicle profile.
