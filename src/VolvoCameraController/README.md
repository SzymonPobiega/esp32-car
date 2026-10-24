# VolvoCameraController

Selects which reversing camera feeds the display: a LilyGO relay board that energises exactly one of four relays on command. Volvo only.

- **Board:** LilyGO ESP32 relay board, MAC `94:51:DC:47:AE:C4` (`Vehicle::LILYGO_CAMERA_MAC`).
- **Relays:** GPIO 21, 19, 18, 5 (active high, `RELAY_ACTIVE_LOW = false`).
- **Power:** currently powered constantly; it will move to ignition-switched power.

## Behaviour

- Relay 1 is on at boot.
- On every ESP-NOW packet it reads `CameraSelectPacket.input` (1–4), switches that relay on and the others off.
- It does not check the packet's `type` or length, so any ESP-NOW packet addressed to it changes the selection.

## Input

`CameraSelectPacket` (type `0x06`, 4 bytes): `input` 1–4, sent by [VolvoMonitor](../VolvoMonitor/README.md).
