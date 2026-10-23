#include "modem_link.hpp"

namespace modem {

static constexpr int P4_RX_PIN = 32; // wired from C6_U0TXD
static constexpr int P4_TX_PIN = 35; // wired to C6_U0RXD
static constexpr size_t RX_BUFFER = 4096;

HardwareSerial Link(1);

enum class State : uint8_t { Sync1, Sync2, Kind, Len1, Len2, Payload, Crc1, Crc2 };

static State state = State::Sync1;
static uint8_t kind = 0;
static uint16_t len = 0;
static uint16_t pos = 0;
static uint16_t rxCrc = 0;
static uint8_t payload[Modem::MAX_PAYLOAD];

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

static void handleHeartbeat() {
  if (len != sizeof(Modem::Heartbeat)) {
    st.lengthErrors++;
    return;
  }
  memcpy(&st.heartbeat, payload, sizeof(st.heartbeat));
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
  uint8_t hdr[3] = { kind, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
  uint16_t crc = Modem::crc16(Modem::CRC_INIT, hdr, sizeof(hdr));
  crc = Modem::crc16(crc, payload, len);
  if (crc != rxCrc) {
    st.crcErrors++;
    return false;
  }
  st.framesOk++;

  switch (kind) {
  case Modem::KIND_HEARTBEAT:
    handleHeartbeat();
    return false;

  case Modem::KIND_ESPNOW_RX: {
    if (len < sizeof(Modem::EspNowRxHeader)) {
      st.lengthErrors++;
      return false;
    }
    Modem::EspNowRxHeader h;
    memcpy(&h, payload, sizeof(h));
    memcpy(out.srcMac, h.srcMac, sizeof(out.srcMac));
    out.rssi = h.rssi;
    out.len = len - sizeof(h);
    memcpy(out.data, payload + sizeof(h), out.len);
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
    uint8_t b = (uint8_t)Link.read();
    st.bytesReceived++;

    switch (state) {
    case State::Sync1:
      if (b == Modem::SYNC1) state = State::Sync2;
      break;

    case State::Sync2:
      if (b == Modem::SYNC2) state = State::Kind;
      else if (b != Modem::SYNC1) state = State::Sync1;
      break;

    case State::Kind:
      kind = b;
      state = State::Len1;
      break;

    case State::Len1:
      len = b;
      state = State::Len2;
      break;

    case State::Len2:
      len |= (uint16_t)b << 8;
      if (len > Modem::MAX_PAYLOAD) {
        st.lengthErrors++;
        state = State::Sync1;
      } else {
        pos = 0;
        state = (len == 0) ? State::Crc1 : State::Payload;
      }
      break;

    case State::Payload:
      payload[pos++] = b;
      if (pos == len) state = State::Crc1;
      break;

    case State::Crc1:
      rxCrc = b;
      state = State::Crc2;
      break;

    case State::Crc2:
      rxCrc |= (uint16_t)b << 8;
      state = State::Sync1;
      if (handleFrame(out)) return true;
      break;
    }
  }
  return false;
}

} // namespace modem
