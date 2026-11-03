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

  // Host -> modem
  static constexpr uint8_t KIND_ESPNOW_TX = 0x81;  // payload: EspNowTxHeader + ESP-NOW data

  // How often the modem sends a heartbeat, and when the host considers it gone
  static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 1000;
  static constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 3500;

  struct PACKED EspNowRxHeader {
    uint8_t srcMac[6];
    int8_t  rssi;
  };

  struct PACKED EspNowTxHeader {
    uint8_t dstMac[6];
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

  // Reassembles frames from a byte stream, one byte at a time
  struct FrameParser {
    enum Result { NONE, FRAME, CRC_ERROR, LENGTH_ERROR };

    uint8_t kind = 0;
    uint16_t len = 0;
    uint8_t payload[MAX_PAYLOAD];

    // FRAME when b completes a frame with a good CRC; kind, len and payload hold it
    Result feed(uint8_t b) {
      switch (state) {
      case 0: if (b == SYNC1) state = 1; break;
      case 1: state = b == SYNC2 ? 2 : b == SYNC1 ? 1 : 0; break;
      case 2: kind = b; state = 3; break;
      case 3: len = b; state = 4; break;
      case 4:
        len |= (uint16_t)b << 8;
        if (len > MAX_PAYLOAD) {
          state = 0;
          return LENGTH_ERROR;
        }
        pos = 0;
        state = len == 0 ? 6 : 5;
        break;
      case 5:
        payload[pos++] = b;
        if (pos == len) state = 6;
        break;
      case 6: rxCrc = b; state = 7; break;
      case 7: {
        rxCrc |= (uint16_t)b << 8;
        state = 0;
        uint8_t hdr[3] = { kind, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
        uint16_t crc = crc16(crc16(CRC_INIT, hdr, sizeof(hdr)), payload, len);
        return crc == rxCrc ? FRAME : CRC_ERROR;
      }
      }
      return NONE;
    }

  private:
    uint8_t state = 0;   // 0 sync1, 1 sync2, 2 kind, 3-4 len, 5 payload, 6-7 crc
    uint16_t pos = 0;
    uint16_t rxCrc = 0;
  };

  static_assert(sizeof(EspNowRxHeader) == 7, "EspNowRxHeader must be 7 bytes");
  static_assert(sizeof(EspNowTxHeader) == 6, "EspNowTxHeader must be 6 bytes");
  static_assert(sizeof(Heartbeat) == 23, "Heartbeat must be 23 bytes");
}
