# ShellyConnector

Controls the Volvo's interior lights through a Shelly Plus RGBW PM. It receives switch commands from [ShellyConnectorUartBridge](../ShellyConnectorUartBridge/README.md) over UART, calls the Shelly's HTTP RPC over Wi-Fi, and reports the light state back. Volvo only.

- **Board:** ESP32.
- **UART:** UART1, 115200 8N1, GPIO 16 RX / 17 TX, crossed over to the bridge.
- **Shelly:** joins the Shelly's own access point (`WIFI_SSID`, `ShellyPlusRGBWPM-…`) and talks to `192.168.33.1`.

## Behaviour

1. **Sleep until needed.** It boots into light sleep and wakes when the UART RX pin (GPIO 16) goes low. It discards whatever woke it, so the command that caused the wake-up is lost; the display corrects itself from the next status broadcast.
2. **Connect.** Joins the Shelly's Wi-Fi (reboots if that fails within 1 minute) and calls `Shelly.GetDeviceInfo` to learn the profile: `light`, `rgbw` or `rgb`.
3. **Commands.** For each `LightsSwitchPacket` it calls `Light.Set`, `RGBW.Set` or `RGB.Set` for the zone.
4. **Status.** Every second it polls `Shelly.GetStatus` and sends a `LightsStatusPacket` back over UART.
5. **Idle.** After 5 minutes without UART traffic it reboots, back into sleep.

## UART framing

`[len:1][payload:len]`, where the payload is the packet struct, starting with its 2-byte type. The receiver reads the length byte (0 or more than 64 is treated as noise and skipped), collects exactly that many bytes, and then checks type and size. A frame that stalls for more than 50 ms is dropped, so the next byte starts a fresh frame. Frames that aren't a valid `LightsSwitchPacket` are logged and ignored.

## Messages

| Direction | Packet |
| --- | --- |
| In | `LightsSwitchPacket` (type `0x09`, 6 bytes): `zone` 1–4, `state` 0/1 |
| Out | `LightsStatusPacket` (type `0x0A`, 10 bytes): `zone1`–`zone4` (0/1) |
