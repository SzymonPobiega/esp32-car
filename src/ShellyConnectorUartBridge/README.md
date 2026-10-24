# ShellyConnectorUartBridge

The ESP-NOW side of the Volvo's Shelly light path. [ShellyConnector](../ShellyConnector/README.md) has to join the Shelly's own Wi-Fi network, which ties its radio to that network's channel and keeps it out of ESP-NOW. This board stays on ESP-NOW and relays packets to it over UART. Volvo only.

- **Board:** ESP32, MAC `D4:E9:F4:A8:7E:34` (`Vehicle::SHELLY_BRIDGE_MAC`).
- **UART:** UART1, 115200 8N1, GPIO 16 RX / 17 TX, crossed over to ShellyConnector.

## Behaviour

- **ESP-NOW → UART.** Accepts only `LIGHT_SWITCH` packets and writes each to UART as `[len:1][packet]`.
- **UART → ESP-NOW.** Reads `[len:1][payload]` frames from UART (a `LightsStatusPacket` from ShellyConnector), strips the length byte and broadcasts the payload, so both displays receive it.
- The ESP-NOW callbacks only fill two fixed 8-entry ring buffers; `loop()` drains them, with at most one ESP-NOW send in flight.

The keep-alive fields `bridgeEnabled` and `KEEPALIVE_TIMEOUT_MS` are declared but unused.
