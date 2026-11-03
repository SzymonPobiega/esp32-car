#include "modem_link.hpp"

namespace modem {

static constexpr int P4_RX_PIN = 32; // wired from C6_U0TXD
static constexpr int P4_TX_PIN = 35; // wired to C6_U0RXD
static constexpr size_t RX_BUFFER = 4096;

HardwareSerial Link(1);

static Modem::FrameParser parser;
static uint8_t txFrame[Modem::MAX_FRAME];

static Stats st = {};
static bool wasAlive = false;
static uint32_t lastMissingReportMs = 0;
static uint32_t lastStatsReportMs = 0;

static constexpr uint32_t MISSING_REPORT_INTERVAL_MS = 5000;
static constexpr uint32_t STATS_REPORT_INTERVAL_MS = 10000;

void begin() {
  Link.setRxBufferSize(RX_BUFFER);
  Link.begin(Modem::UART_BAUD, SERIAL_8N1, P4_RX_PIN, P4_TX_PIN);
}

const Stats& stats() {
  return st;
}

bool modemAlive() {
  return st.hasHeartbeat && (millis() - st.lastHeartbeatMs) < Modem::HEARTBEAT_TIMEOUT_MS;
}

bool send(const uint8_t dstMac[6], const void* data, size_t len) {
  if (len > Modem::MAX_ESPNOW_DATA) return false;
  uint8_t payload[sizeof(Modem::EspNowTxHeader) + Modem::MAX_ESPNOW_DATA];
  memcpy(payload, dstMac, sizeof(Modem::EspNowTxHeader));
  memcpy(payload + sizeof(Modem::EspNowTxHeader), data, len);
  size_t n = Modem::buildFrame(txFrame, Modem::KIND_ESPNOW_TX, payload,
                               (uint16_t)(sizeof(Modem::EspNowTxHeader) + len));
  return Link.write(txFrame, n) == n;
}

static void handleHeartbeat() {
  if (parser.len != sizeof(Modem::Heartbeat)) {
    st.lengthErrors++;
    return;
  }
  memcpy(&st.heartbeat, parser.payload, sizeof(st.heartbeat));
  st.lastHeartbeatMs = millis();
  st.hasHeartbeat = true;

  if (!wasAlive) {
    const uint8_t* m = st.heartbeat.mac;
    Serial.printf("ESP-NOW modem up: MAC %02X:%02X:%02X:%02X:%02X:%02X, channel %u\n",
                  m[0], m[1], m[2], m[3], m[4], m[5], st.heartbeat.channel);
  }
}

// Returns true if the completed frame produced a Message
static bool handleFrame(Message& out) {
  st.framesOk++;

  switch (parser.kind) {
  case Modem::KIND_HEARTBEAT:
    handleHeartbeat();
    return false;

  case Modem::KIND_ESPNOW_RX: {
    if (parser.len < sizeof(Modem::EspNowRxHeader)) {
      st.lengthErrors++;
      return false;
    }
    Modem::EspNowRxHeader h;
    memcpy(&h, parser.payload, sizeof(h));
    memcpy(out.srcMac, h.srcMac, sizeof(out.srcMac));
    out.rssi = h.rssi;
    out.len = parser.len - sizeof(h);
    memcpy(out.data, parser.payload + sizeof(h), out.len);
    return true;
  }

  default:
    // Unknown kind from a newer modem firmware, ignore
    return false;
  }
}

bool poll(Message& out) {
  bool alive = modemAlive();
  if (wasAlive && !alive) {
    Serial.println("ESP-NOW modem lost (no heartbeat)");
  }
  wasAlive = alive;

  // Help diagnose wiring/baud problems while the modem is not detected
  uint32_t now = millis();
  if (!alive && now - lastMissingReportMs >= MISSING_REPORT_INTERVAL_MS) {
    lastMissingReportMs = now;
    Serial.printf("ESP-NOW modem not detected: %lu bytes, %lu frames, %lu CRC errors, %lu length errors\n",
                  (unsigned long)st.bytesReceived, (unsigned long)st.framesOk,
                  (unsigned long)st.crcErrors, (unsigned long)st.lengthErrors);
  }

  // Counters reported by the modem itself, cumulative since the modem booted
  if (alive && now - lastStatsReportMs >= STATS_REPORT_INTERVAL_MS) {
    lastStatsReportMs = now;
    Serial.printf("ESP-NOW modem: rx=%lu dropped=%lu oversize=%lu, link CRC errors=%lu\n",
                  (unsigned long)st.heartbeat.rxCount, (unsigned long)st.heartbeat.rxDropped,
                  (unsigned long)st.heartbeat.rxOversize, (unsigned long)st.crcErrors);
  }

  while (Link.available()) {
    st.bytesReceived++;
    switch (parser.feed((uint8_t)Link.read())) {
    case Modem::FrameParser::FRAME:
      if (handleFrame(out)) return true;
      break;
    case Modem::FrameParser::CRC_ERROR:
      st.crcErrors++;
      break;
    case Modem::FrameParser::LENGTH_ERROR:
      st.lengthErrors++;
      break;
    case Modem::FrameParser::NONE:
      break;
    }
  }
  return false;
}

} // namespace modem
