# EspNowUartBridge

ESP-NOW modem for the Toyota dashboard. The ESP32-P4 running [ToyotaMonitor](../ToyotaMonitor/README.md) has no radio, so this sketch runs on the ESP32-C6 on the same board: it receives ESP-NOW packets and forwards them to the P4 over UART. Receive only; sending from the P4 can be added later. Toyota only.

- **Board:** ESP32-C6, `esp32:esp32:esp32c6`. Keep Core Debug Level at None.
- **Its MAC** (`9C:13:9E:C3:FF:60`) is the address every Toyota connector sends to (`Vehicle::MODEM_MAC`).

## Wiring

| C6 | P4 | Use |
| --- | --- | --- |
| GPIO 16 (U0TXD) | GPIO 32 | Modem → P4 (required) |
| GPIO 17 (U0RXD) | GPIO 35 | P4 → modem (not used yet; leave disconnected while flashing the C6) |
| GND | GND | Shared on the board |

UART0 is the link, so nothing else may print to it: IDF logging is switched off at startup. Debug output goes to the C6's USB Serial/JTAG port, and only when "USB CDC On Boot" is enabled.

## Flashing

The C6 is flashed through its UART0 header with a USB-UART adapter (3.3 V logic). The adapter can't reset the C6, so put it in download mode by hand: hold BOOT, press RESET, release BOOT, then upload.

## How it works

1. Wi-Fi in station mode, channel fixed at `Modem::ESPNOW_CHANNEL` (1), power saving off, ESP-NOW receive callback registered.
2. The callback only copies each packet (with the sender's MAC and RSSI) into a 32-entry FreeRTOS queue and counts it. Packets larger than 250 bytes, or arriving with the queue full, are counted as dropped.
3. `loop()` drains the queue and writes one frame per packet to UART (4 KB transmit buffer).
4. Every second it sends a heartbeat with its MAC, channel, uptime and the received / dropped / oversize counters.

## UART protocol

Defined in [`Shared/ModemProtocol.hpp`](../Shared/ModemProtocol.hpp), shared with ToyotaMonitor. 115200 baud, 8N1.

```
[AA][55][kind:1][len:2 LE][payload:len][crc:2 LE]      crc = CRC-16/CCITT-FALSE over kind, len, payload
```

| Kind | Payload |
| --- | --- |
| `0x01` ESPNOW_RX | `EspNowRxHeader` (sender MAC, RSSI) + the ESP-NOW data |
| `0x02` HEARTBEAT | `Heartbeat`: MAC, channel, uptime, rx count, rx dropped, rx oversize |

The CRC and sync bytes let the P4 resynchronise after noise, such as the C6's boot messages, which appear on the same pins.
