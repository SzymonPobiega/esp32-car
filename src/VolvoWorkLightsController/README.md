# VolvoWorkLightsController

Switches the four exterior work lights, from ESP-NOW commands or from physical push-buttons, and broadcasts their state so both Volvo displays stay in sync. Volvo only.

- **Board:** LilyGO ESP32 relay board, MAC `E0:8C:FE:75:DA:54` (`Vehicle::LILYGO_WORKLIGHT_MAC`).
- **Relays:** GPIO 21, 19, 18, 5 (off at boot).
- **Buttons:** GPIO 22, 23, 32, 33 (pull-up, active low), one per relay.

## Behaviour

- A `WorkLightsSwitchPacket` sets relay `input` (1–4) to `state` (0/1).
- A button press toggles its relay and broadcasts the status immediately.
- The status is also broadcast every 5 s.

## Messages

| Direction | Packet |
| --- | --- |
| In | `WorkLightsSwitchPacket` (type `0x07`, 6 bytes): `input` 1–4, `state` 0/1, from both displays |
| Out | `WorkLightsStatusPacket` (type `0x08`, 10 bytes): `relay1`–`relay4` (0/1), ESP-NOW broadcast |
