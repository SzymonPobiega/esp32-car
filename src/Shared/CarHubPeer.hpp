#pragma once
// The CarHub (always-on LoRa gateway, see Vehicle::CARHUB_MAC) gets a copy of every connector
// packet. Its send failures (CarHub off or out of range) only show up in the send callback's log.
#include <esp_now.h>
#include <string.h>

namespace CarHubPeer {

// Call after esp_now_init()
inline bool add() {
  if (!Vehicle::HAS_CARHUB) return true;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, Vehicle::CARHUB_MAC, 6);
  peer.channel = 0;   // current Wi-Fi channel
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

inline void send(const void *data, size_t len) {
  if (Vehicle::HAS_CARHUB) esp_now_send(Vehicle::CARHUB_MAC, (const uint8_t *)data, len);
}

inline bool isCarHub(const uint8_t *mac) {
  return Vehicle::HAS_CARHUB && memcmp(mac, Vehicle::CARHUB_MAC, 6) == 0;
}

}  // namespace CarHubPeer
