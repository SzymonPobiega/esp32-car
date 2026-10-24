# ToyotaMonitor

The Toyota dashboard: an ESP32-P4 board with a 480×800 MIPI-DSI touchscreen (ST7701 panel, GT911 touch) running LVGL 8. It receives every reading through the [ESP-NOW modem](../EspNowUartBridge/README.md) on the C6 next to it. Toyota only.

## Board settings

`esp32:esp32:esp32p4:PSRAM=enabled,USBMode=hwcdc,CDCOnBoot=cdc` (ESP32 core 3.3.x).

- **PSRAM: Enabled** is required. The display frame buffer and the two LVGL draw buffers (about 750 KB each) are allocated in PSRAM; without it the panel fails with `ESP_ERR_NO_MEM` at boot.
- **USB Mode: Hardware CDC and JTAG** and **USB CDC On Boot: Enabled** send `Serial` to the USB port, so the log is readable there. Opening that port from a PC resets the P4.
- The firmware uses about 89% of the default 1.25 MB app partition.

## Screen

- **Gauges tab.** Four tyre pressure gauges (kPa) with temperature and the seconds since the last reading (`--` before the first one). Sensor IDs map to gauges through `Vehicle::TPMS_SENSORS`; packets from unknown sensors are ignored.
- **Tab2.** Pitch and roll indicators from `GYRO_ANGLE` packets (fixed at +20° / −15° until the first one).
- **Transmission tab.** Gear, converter lockup, ATF pan and outlet temperatures (orange from 105 °C, red from 115 °C), boost with a 0–2 bar fill bar, MAP, and an OBD status line. Values go to `--` after 10 s without OBD packets.
- **Status bar** (all tabs, 88 px). House battery values in whole numbers (V, A, %, Ah, °C), grey after 10 s without data, and four connector lights (TPMS, OBD, BMS, GYRO): grey before the first packet, green with a flash on each packet, red when packets stop (15 s for TPMS and OBD, 10 s for BMS and gyro).

Bar heights are `tab_bar_h` and `bms_bar_h` in `setup()`. The gauge and indicator rows size themselves to the space left.

## Code

| File | Contents |
| --- | --- |
| `ToyotaMonitor.ino` | Display, touch and LVGL setup, tab layout, `loop()`, and `handleModemMessage()` which dispatches packets by type and size |
| `modem_link.cpp/.hpp` | Non-blocking parser for the modem's UART frames (UART1, P4 GPIO 32 RX / 35 TX); tracks modem heartbeats and logs `ESP-NOW modem up` / `lost` and counters every 10 s |
| `tpms_data.cpp/.hpp` | Latest pressure and temperature per gauge; accepts a reading only if its `sequence` is newer; sensor ID 0 = connector ping |
| `gauge_ui.cpp/.hpp` | Tyre gauge widget and its 100 ms refresh timer |
| `attitude_ui.cpp/.hpp`, `src/pitch.c`, `src/roll.c` | Pitch / roll indicator widget and its images |
| `obd_ui.cpp/.hpp` | Transmission tab |
| `bms_ui.cpp/.hpp` | Status bar with battery values |
| `link_indicator.cpp/.hpp` | Connector status light |
| `src/lcd`, `src/touch` | ST7701 MIPI-DSI panel and GT911 touch drivers |
| `lv_conf.h` | LVGL configuration |

`loop()` runs every 5 ms. Keep it short: LVGL reads the touch panel only when `lv_timer_handler()` runs, and taps are missed if the loop is slow.

## LVGL configuration

LVGL 8.4. Required `lv_conf.h` settings (from the display vendor's sample):

```c
#define LV_COLOR_DEPTH 16
#define LV_COLOR_16_SWAP 0
#define LV_MEM_CUSTOM 1
#define LV_TICK_CUSTOM 1
#define LV_MEMCPY_MEMSET_STD 1
#define LV_ATTRIBUTE_FAST_MEM IRAM_ATTR
```

Fonts in use: Montserrat 14, 20, 36 and 48. `LV_SPRINTF_USE_FLOAT` is off, so format floats with `snprintf` before passing text to LVGL.
