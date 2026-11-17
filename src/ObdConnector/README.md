# ObdConnector

Talks to a BLE ELM327 OBD-II dongle, polls transmission and boost readings from the Toyota's ECUs, and sends them over ESP-NOW to the dashboard. Toyota only.

- **Board:** ESP32 (classic), `esp32:esp32:esp32`.
- **Background:** the requests and formulas come from [`Toyota Techstream PIDs.pdf`](../Toyota%20Techstream%20PIDs.pdf).

## Readings

| Reading | Request | ECU header | Decoding | Polled |
| --- | --- | --- | --- | --- |
| Gear | `21DA` | `7E1` (transmission) | `A & 7` (raw gear code) | every cycle |
| Torque converter lockup | `21DA` | `7E1` | bit 4 of `A` | every cycle |
| ATF pan temperature | `21D9` | `7E1` | `E - 40` °C | every 2 s |
| ATF converter outlet temperature | `21D9` | `7E1` | `G - 40` °C | every 2 s |
| Manifold pressure (MAP) | `010B` | `7DF` | `A` kPa, scaled by `10·D/255` when `014F` byte D is non-zero | every cycle |
| Barometric pressure | `0133` | `7DF` | `A` kPa | every 30 s |
| Boost | | | MAP − baro, in 0.01 bar | |
| Coolant temperature | `2105` | `7E0` (engine) | `A - 40` °C | every 3 s |
| Starter battery voltage | `2142` | `7E0` | `A·B` mV | every 3 s |

The other engine blocks in the [Right to Readout](https://github.com/Right-to-Readout/right-to-readout/tree/main/dist/toyota-land-cruiser-200-vdj200-1vdftv-eu-to-2015-08) file (`2146`, `2162`, `216D`, `2170`, `2171`, `2177`, `2178`) get no answer from this truck's engine ECU, with the engine running or not.

Dongle: Vgate iCar Pro BLE ("IOS-Vlink", ELM327 v2.3, service `18F0`). It only connects when the ESP doesn't start an MTU exchange (NimBLE's connect is called with `exchangeMTU = false`). For probing, a line such as `@7E0 2105` on the USB serial port sends that request once connected and logs the raw reply as `RAW ...`.

`A`, `E`, `G` are the data bytes after the positive response (`61 DA`, `61 D9`, `41 0B`). Gear and lockup decoding comes from related Toyota gearboxes and is unverified on the VDJ200. `014F` is read once after connecting.

## How it works

1. **Finding the dongle.** A 3 s BLE scan logs every named device it sees. The dongle is the device whose address equals `Vehicle::OBD_DONGLE_MAC` if that is set; otherwise the first one whose name contains OBD, ELM, Vlink, Vgate, Veepeak, Konnwei, `IOS-` or `ANDROID-`, or that advertises a known ELM serial service (`FFF0`, `FFE0`, `18F0`, vLinker's `E7810A71-…`).
2. **Serial port.** After connecting, it picks a service with a notify (or indicate) and a write characteristic, preferring the known ELM services, and subscribes. Notifications go into a stream buffer.
3. **ELM327 setup.** `ATZ`, then `ATE0 ATL0 ATS1 ATH0 ATAT1 ATSP6 ATCAF1`: echo off, spaces on, headers off, adaptive timing, ISO 15765-4 CAN 11-bit 500 kbps, CAN auto-formatting (ISO-TP, so multi-frame replies such as `21D9` are reassembled). Each command waits for the `>` prompt.
4. **Polling.** Each cycle switches the header as needed (`ATSH7E1` / `ATSH7DF`) and sends the requests in the table. The reply parser handles single frames, multi-frame `0:` / `1:` lines, several ECUs answering, and text lines such as `NO DATA` or `CAN ERROR`. Failed requests are logged with their raw reply. A value is cleared after 3 failed reads in a row.
5. **Sending.** An `ObdPacket` goes to `Vehicle::DISPLAY_MAC` up to 4 times a second while polling. While searching for the dongle or when the vehicle doesn't answer (ignition off), it is sent about once a second as a ping, with the status field saying which.

If the dongle disconnects, the connector rescans. If the dongle stops answering (3 commands in a row without a `>` prompt), the connector disconnects and starts over; if the vehicle hasn't answered for 20 s, it resets the ELM and runs the setup again. A 1 s timer repeats the last packet while the loop is busy (scanning, connecting, slow requests), so the dashboard's link light stays green; values more than 5 s old are sent as "no ECU answer" instead. If the loop doesn't come round for 90 s, the board restarts.

## Output

`ObdCommandPacket` (type `0x0F`, 6 bytes, from the modem): `OBD_CMD_RELEASE` with arg 1 keeps the connector off the dongle for 30 s, so a phone app can connect; the dashboard repeats it every 15 s while its FREE OBD switch is on. Arg 0 reconnects at once.

`ObdPacket` (type `0x0B`, 22 bytes): `status` (0 no dongle, 1 dongle but no ECU answer, 2 OK, 3 released for the phone), `gear`, `lockup`, `atf_pan`, `atf_outlet` (°C), `map_kpa`, `baro_kpa`, `boost` (0.01 bar), `coolant` (°C), `battery_cv` (0.01 V). A value that isn't available is `OBD_NO_VALUE` (-32768). See [Contracts.hpp](../Shared/Contracts.hpp).

## Troubleshooting

An event log of the last 96 connection events (boots, connects, BLE disconnects with the reason code, reconnects after the ELM went silent, ECU lost / answering, ELM setup failures, hang restarts) is kept in flash, so it survives restarts and power cuts. Each entry carries the boot number, the time since that boot and the ELM timeouts counted so far; identical events in a row are folded into one with a count. It is printed at boot; type `LOG` on the serial port to print it, `LOG CLEAR` to empty it.

The serial log (115200 baud) shows the scan, the ELM version reply, one `status=… gear=… atf=…` line per update, and `<request> -> <raw reply>` for each failed request. Cheap ELM clones sometimes fail on multi-frame replies like `21D9`.
