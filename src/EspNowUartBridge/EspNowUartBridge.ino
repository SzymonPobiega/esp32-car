// ESP-NOW modem for the P4 dashboard, running on the ESP32-C6 that sits next to the P4.
//
// Receives ESP-NOW packets and forwards them to the P4 over UART0, framed as described
// in Shared/ModemProtocol.hpp. Sends a heartbeat every second so the P4 can tell the
// modem is alive and learn its MAC address.
//
// UART0 is the link to the P4, so nothing else may print to it. Debug output goes to the
// USB Serial/JTAG port, and only when "USB CDC On Boot" is enabled.

#include <WiFi.h>
#include <esp_now.h>
#include <esp_mac.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <atomic>
#include "ContractsInclude.hpp"

// -------------------- USER CONFIG --------------------
static const int UART_TX_PIN = 16;     // C6 U0TXD -> P4 GPIO32
static const int UART_RX_PIN = 17;     // C6 U0RXD <- P4 GPIO35

static const size_t RX_QUEUE_LEN = 32;          // ESP-NOW packets buffered between callback and UART
static const size_t UART_TX_BUFFER = 4096;
// -----------------------------------------------------

#if ARDUINO_USB_CDC_ON_BOOT
  #define DBG(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG(...) do {} while (0)
#endif

HardwareSerial &Link = Serial0;

struct RxItem {
  Modem::EspNowRxHeader header;
  uint8_t len;
  uint8_t data[Modem::MAX_ESPNOW_DATA];
};

static QueueHandle_t rxQueue = nullptr;

static std::atomic<uint32_t> rxCount{0};
static std::atomic<uint32_t> rxDropped{0};
static std::atomic<uint32_t> rxOversize{0};

static uint8_t staMac[6];
static uint8_t frameBuf[Modem::MAX_FRAME];
static uint32_t lastHeartbeatMs = 0;

// ---------- ESP-NOW callback ----------
// Runs in the Wi-Fi task: copy into the queue and return.
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!info || !data || len <= 0) return;

  if ((size_t)len > Modem::MAX_ESPNOW_DATA) {
    rxOversize++;
    return;
  }

  RxItem item;
  memcpy(item.header.srcMac, info->src_addr, 6);
  item.header.rssi = info->rx_ctrl ? (int8_t)info->rx_ctrl->rssi : 0;
  item.len = (uint8_t)len;
  memcpy(item.data, data, len);

  rxCount++;
  if (xQueueSend(rxQueue, &item, 0) != pdTRUE) {
    rxDropped++;
  }
}

// ---------- Setup ESP-NOW ----------
bool initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);

  if (esp_wifi_set_channel(Modem::ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    DBG("esp_wifi_set_channel failed\n");
    return false;
  }

  esp_read_mac(staMac, ESP_MAC_WIFI_STA);
  DBG("ESP-NOW (STA) MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
      staMac[0], staMac[1], staMac[2], staMac[3], staMac[4], staMac[5]);

  if (esp_now_init() != ESP_OK) {
    DBG("ESP-NOW init failed!\n");
    return false;
  }

  esp_now_register_recv_cb(onEspNowRecv);
  return true;
}

// ---------- UART output ----------
void sendFrame(uint8_t kind, const uint8_t *payload, uint16_t len) {
  size_t n = Modem::buildFrame(frameBuf, kind, payload, len);
  Link.write(frameBuf, n);
}

void forwardRxItem(const RxItem &item) {
  uint8_t payload[Modem::MAX_PAYLOAD];
  memcpy(payload, &item.header, sizeof(item.header));
  memcpy(payload + sizeof(item.header), item.data, item.len);
  sendFrame(Modem::KIND_ESPNOW_RX, payload, (uint16_t)(sizeof(item.header) + item.len));
}

void sendHeartbeat() {
  Modem::Heartbeat hb;
  memcpy(hb.mac, staMac, 6);

  uint8_t primary = 0;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&primary, &second);
  hb.channel = primary;

  hb.uptimeMs = millis();
  hb.rxCount = rxCount;
  hb.rxDropped = rxDropped;
  hb.rxOversize = rxOversize;
  sendFrame(Modem::KIND_HEARTBEAT, (const uint8_t *)&hb, sizeof(hb));
}

// ---------- Arduino ----------
void setup() {
  // Keep IDF logs off UART0, it carries the link to the P4
  esp_log_level_set("*", ESP_LOG_NONE);

#if ARDUINO_USB_CDC_ON_BOOT
  Serial.begin(115200);
#endif

  Link.setTxBufferSize(UART_TX_BUFFER);
  Link.begin(Modem::UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  rxQueue = xQueueCreate(RX_QUEUE_LEN, sizeof(RxItem));
  if (!rxQueue || !initEspNow()) {
    DBG("Init failed, restarting...\n");
    delay(1000);
    ESP.restart();
  }

  DBG("ESP-NOW modem ready on channel %d\n", Modem::ESPNOW_CHANNEL);
  sendHeartbeat();
  lastHeartbeatMs = millis();
}

void loop() {
  RxItem item;

  // Block briefly for the first packet, then drain whatever else is queued
  if (xQueueReceive(rxQueue, &item, pdMS_TO_TICKS(10)) == pdTRUE) {
    do {
      forwardRxItem(item);
    } while (xQueueReceive(rxQueue, &item, 0) == pdTRUE);
  }

  uint32_t now = millis();
  if (now - lastHeartbeatMs >= Modem::HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = now;
    sendHeartbeat();
    DBG("rx=%lu dropped=%lu oversize=%lu\n",
        (unsigned long)rxCount.load(), (unsigned long)rxDropped.load(), (unsigned long)rxOversize.load());
  }
}
