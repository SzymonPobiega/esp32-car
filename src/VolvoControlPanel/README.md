# VolvoControlPanel

The camper control panel: an ESP32-S3 Waveshare RGB LCD with GT911 touch, running the `CamperUI` LVGL screen. It shows the house battery and switches the interior and exterior lights. It also powers the headless ESP boards down when it goes to sleep. Volvo only.

- **Board:** ESP32-S3 Waveshare RGB LCD with PSRAM (`src/waveshare` holds the LVGL port).

## Screen

- Battery widget, updated from `BMS` broadcasts.
- Interior light toggles: `LightsSwitchPacket` to `Vehicle::SHELLY_BRIDGE_MAC`, updated from `LIGHT_STATUS` broadcasts.
- Exterior light toggles: `WorkLightsSwitchPacket` to `Vehicle::LILYGO_WORKLIGHT_MAC`, updated from `WORKLIGHT_STATUS` broadcasts.
- Hatch and roof actuator buttons are placeholders.

Commands go through the same `espnow_retry` layer as [VolvoMonitor](../VolvoMonitor/README.md#sending).

## Sleep and power gating

After 60 s without a touch (`SLEEP_AFTER_MS`), the panel:

1. turns off the backlight,
2. shuts down ESP-NOW and Wi-Fi,
3. drives GPIO 6 (`POWER_RELAY_GPIO`) low, which cuts power to the headless ESP boards behind that relay.

A touch reverses the steps; ESP-NOW is restarted before the boards are powered back on. See [VOLVO.md](../../VOLVO.md#power) for the second, ignition-driven relay.
