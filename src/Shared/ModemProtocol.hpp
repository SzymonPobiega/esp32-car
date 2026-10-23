#pragma once
#include <stdint.h>
#include <stddef.h>

#if defined(__GNUC__)
  #define PACKED __attribute__((packed))
#else
  #define PACKED
#endif

// UART protocol between the ESP-NOW modem (EspNowUartBridge on the C6) and the host (P4).
//
// Frame on the wire:
//   [SYNC1][SYNC2][kind:1][len:2 LE][payload:len][crc:2 LE]
// crc = CRC-16/CCITT-FALSE over kind, len and payload.
namespace Modem {

  static constexpr uint8_t  SYNC1 = 0xAA;
  static constexpr uint8_t  SYNC2 = 0x55;
  static constexpr uint32_t UART_BAUD = 115200;

  static constexpr uint8_t ESPNOW_CHANNEL = 1;
  static constexpr size_t  MAX_ESPNOW_DATA = 250;

  // Modem -> host
  static constexpr uint8_t KIND_ESPNOW_RX = 0x01;  // payload: EspNowRxHeader + ESP-NOW data
  static constexpr uint8_t KIND_HEARTBEAT = 0x02;  // payload: Heartbeat

  // How often the modem sends a heartbeat, and when the host considers it gone
  static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 1000;
  static constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 3500;

  struct PACKED EspNowRxHeader {
    uint8_t srcMac[6];
    int8_t  rssi;
  };

  struct PACKED Heartbeat {
    uint8_t  mac[6];       // modem STA MAC, use it as the ESP-NOW destination in senders
    uint8_t  channel;
    uint32_t uptimeMs;
    uint32_t rxCount;      // ESP-NOW packets received
    uint32_t rxDropped;    // dropped because the queue to UART was full
    uint32_t rxOversize;   // dropped because they exceed MAX_ESPNOW_DATA
  };

  static constexpr size_t HEADER_LEN = 5;  // sync1, sync2, kind, len(2)
  static constexpr size_t CRC_LEN = 2;
  static constexpr size_t MAX_PAYLOAD = sizeof(EspNowRxHeader) + MAX_ESPNOW_DATA;
  static constexpr size_t MAX_FRAME = HEADER_LEN + MAX_PAYLOAD + CRC_LEN;

  inline uint16_t crc16(uint16_t crc, const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
      crc ^= (uint16_t)data[i] << 8;
      for (int b = 0; b < 8; b++) {
        crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
      }
    }
    return crc;
  }

  static constexpr uint16_t CRC_INIT = 0xFFFF;

  // Builds a complete frame into out (must hold HEADER_LEN + len + CRC_LEN bytes).
  // Returns the number of bytes written.
  inline size_t buildFrame(uint8_t* out, uint8_t kind, const uint8_t* payload, uint16_t len) {
    out[0] = SYNC1;
    out[1] = SYNC2;
    out[2] = kind;
    out[3] = (uint8_t)(len & 0xFF);
    out[4] = (uint8_t)(len >> 8);
    for (uint16_t i = 0; i < len; i++) out[HEADER_LEN + i] = payload[i];

    uint16_t crc = crc16(CRC_INIT, out + 2, 3 + len);
    out[HEADER_LEN + len] = (uint8_t)(crc & 0xFF);
    out[HEADER_LEN + len + 1] = (uint8_t)(crc >> 8);
    return HEADER_LEN + len + CRC_LEN;
  }

  static_assert(sizeof(EspNowRxHeader) == 7, "EspNowRxHeader must be 7 bytes");
  static_assert(sizeof(Heartbeat) == 23, "Heartbeat must be 23 bytes");
}
