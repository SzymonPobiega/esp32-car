# VolvoMonitor

The Volvo dashboard: an ESP32-S3 with a 480×800 LVGL touchscreen. It shows tyre pressures, attitude, the house battery and the reversing cameras, and switches the interior and exterior lights. Volvo only.

- **Board:** ESP32-S3 with PSRAM (display driven through ESP32_Display_Panel). The firmware is about 1.9 MB, so it needs a larger app partition than the 1.25 MB default.
- **MAC:** `50:78:7D:13:22:F8` (`Vehicle::DISPLAY_MAC` in the [Volvo profile](../Shared/Vehicles/Volvo.hpp)), where TpmsConnector and GyroConnector send.

## Tabs

| Tab | Shows | Sends |
| --- | --- | --- |
| Tires | 6 tyre gauges (dual rear wheels), mapped from `Vehicle::TPMS_SENSORS` | |
| Gyro | Pitch and roll indicators and a compass | |
| Power | House battery; interior and exterior light toggles | `LightsSwitchPacket` to `SHELLY_BRIDGE_MAC` (interior), `WorkLightsSwitchPacket` to `LILYGO_WORKLIGHT_MAC` (exterior) |
| Camera | Camera input buttons | `CameraSelectPacket` (input 1–4) to `LILYGO_CAMERA_MAC`; input 1 (rear high) at boot |

## Receiving

The ESP-NOW callback dispatches on the packet type and exact size:

| Packet | Handling |
| --- | --- |
| `TPMS` | Stored only if `sequence` is higher than the last one seen, which drops the connector's 5 s repeats. Unknown sensor IDs are ignored. |
| `GYRO_ANGLE` | Centi-degrees to degrees; yaw becomes a compass heading, `(360 - yaw + 90) % 360` |
| `GYRO_HEIGHT` | Height (no sender at present) |
| `BMS` | Battery widget |
| `WORKLIGHT_STATUS`, `LIGHT_STATUS` | Light toggles follow the actual relay / Shelly state |

The callback runs on the Wi-Fi task, so BMS and light status go into buffers protected by a critical section, and `loop()` applies them to LVGL.

## Sending

All commands go through `espnow_retry` (`espnow_retry.cpp`): the last packet per peer (up to 8 peers) is kept and, on a failed send callback, resent from `loop()` with back-off of 30, 60, 120, 240 and 480 ms, at most 5 times. A newer command to the same peer replaces the pending one. Toggles flip immediately and are corrected by the next status broadcast.

The lights can be switched only with the ignition on: the actuator boards are behind the ignition relay (see [VOLVO.md](../../VOLVO.md#power)).
