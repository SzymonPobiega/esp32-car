#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "ContractsInclude.hpp"
#include <esp_sleep.h>
#include <esp_wifi.h>

using namespace Contracts;

// ====== YOUR SETTINGS ======
const char* WIFI_SSID = "ShellyPlusRGBWPM-D8132ADD3F00";
const char* WIFI_PASS = "";

// Shelly IP or hostname on your LAN
const char* SHELLY_HOST = "192.168.33.1";

// If Shelly local auth is enabled, fill these in.
// Otherwise leave empty strings.
const char* SHELLY_USER = "";
const char* SHELLY_PASS = "";
// ===========================

// ====== UART SETTINGS ======
static const int UART_RX_PIN = 16;
static const int UART_TX_PIN = 17;
static const uint32_t UART_BAUD = 115200;
HardwareSerial& U = Serial1;
// ===========================

bool systemAwake = false;
unsigned long lastUartActivityMs = 0;
const unsigned long inactivityTimeoutMs = 5UL * 60UL * 1000UL; // 5 min
const unsigned long connectionTimeoutMs = 1UL * 60UL * 1000UL; // 1 min

String shellyProfile = "";
unsigned long lastPollMs = 0;
const unsigned long pollIntervalMs = 1000;

// UART parser: frames from the bridge are [len][payload], payload starts with the int16 type
static uint8_t uartRxBuf[64];
static size_t uartRxLen = 0;
static size_t uartFrameLen = 0;           // 0 = waiting for the length byte
static unsigned long uartLastByteMs = 0;
// A frame arrives in one burst; a gap this long inside a frame means we lost sync
static const unsigned long uartFrameGapMs = 50;

void stopWiFiForSleep() {
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  btStop(); // optional, reduces power further if BT stack is on
}

void enterSleepUntilUartActivity() {
  Serial.println("Entering light sleep, waiting for UART...");

  stopWiFiForSleep();

  // On classic ESP32, GPIO16/17 belong to VDDSDIO domain.
  // Keep it powered during light sleep if the pin must stay valid.
  esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_ON);

  // Wake on RX pin going LOW (UART start bit)
  gpio_wakeup_enable((gpio_num_t)UART_RX_PIN, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  Serial.flush();
  delay(20);

  esp_light_sleep_start();

  Serial.println("Woke from UART/GPIO activity");
}

bool httpGetJson(const String& url, DynamicJsonDocument& doc) {
  HTTPClient http;
  http.begin(url);

  if (strlen(SHELLY_USER) > 0) {
    http.setAuthorization(SHELLY_USER, SHELLY_PASS);
  }

  int code = http.GET();
  if (code <= 0) {
    Serial.printf("HTTP error: %s\n", http.errorToString(code).c_str());
    http.end();
    return false;
  }

  if (code != 200) {
    Serial.printf("HTTP status: %d\n", code);
    String body = http.getString();
    Serial.println(body);
    http.end();
    return false;
  }

  String body = http.getString();
  http.end();

  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    Serial.print("JSON parse error: ");
    Serial.println(err.c_str());
    Serial.println(body);
    return false;
  }

  return true;
}

bool httpCallJson(const String& url, DynamicJsonDocument& doc) {
  HTTPClient http;
  http.begin(url);

  if (strlen(SHELLY_USER) > 0) {
    http.setAuthorization(SHELLY_USER, SHELLY_PASS);
  }

  int code = http.GET();
  if (code <= 0) {
    Serial.printf("HTTP error: %s\n", http.errorToString(code).c_str());
    http.end();
    return false;
  }

  String body = http.getString();
  http.end();

  if (code != 200) {
    Serial.printf("HTTP status: %d\n", code);
    Serial.println(body);
    return false;
  }

  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    Serial.print("JSON parse error: ");
    Serial.println(err.c_str());
    Serial.println(body);
    return false;
  }

  return true;
}

void sendPacketFramed(const uint8_t* data, uint8_t len) {
  U.write(&len, 1);
  U.write(data, len);
}

void sendLightsStatusPacket(int16_t z1, int16_t z2, int16_t z3, int16_t z4) {
  LightsStatusPacket pkt;
  pkt.type = TYPE_LIGHT_STATUS;
  pkt.zone1 = z1;
  pkt.zone2 = z2;
  pkt.zone3 = z3;
  pkt.zone4 = z4;

  sendPacketFramed(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

bool getShellyDeviceInfo() {
  String url = String("http://") + SHELLY_HOST +
               "/rpc/Shelly.GetDeviceInfo?id=1";

  DynamicJsonDocument doc(2048);
  if (!httpGetJson(url, doc)) {
    return false;
  }

  JsonObject result = doc.as<JsonObject>();
  if (result.isNull()) {
    Serial.println("No result in Shelly.GetDeviceInfo response");
    serializeJsonPretty(doc, Serial);
    Serial.println();
    return false;
  }

  String model = result["model"] | "";
  String app = result["app"] | "";
  shellyProfile = result["profile"] | "";

  Serial.println("Shelly device info:");
  Serial.printf("  model   : %s\n", model.c_str());
  Serial.printf("  app     : %s\n", app.c_str());
  Serial.printf("  profile : %s\n", shellyProfile.c_str());

  return true;
}

bool setShellyLightState(int zone, bool on) {
  String url;

  if (shellyProfile == "light") {
    if (zone < 1 || zone > 4) {
      Serial.printf("Invalid light zone: %d\n", zone);
      return false;
    }

    // Shelly light channels are 0..3, packet zones are 1..4
    int shellyId = zone - 1;

    url = String("http://") + SHELLY_HOST +
          "/rpc/Light.Set?id=" + String(shellyId) +
          "&on=" + String(on ? "true" : "false");
  } else if (shellyProfile == "rgbw") {
    // Only one logical output in rgbw mode
    if (zone != 1) {
      Serial.printf("RGBW profile supports only zone 1, got zone %d\n", zone);
      return false;
    }

    url = String("http://") + SHELLY_HOST +
          "/rpc/RGBW.Set?id=0&on=" + String(on ? "true" : "false");
  } else if (shellyProfile == "rgb") {
    // Only one logical output in rgb mode
    if (zone != 1) {
      Serial.printf("RGB profile supports only zone 1, got zone %d\n", zone);
      return false;
    }

    url = String("http://") + SHELLY_HOST +
          "/rpc/RGB.Set?id=0&on=" + String(on ? "true" : "false");
  } else {
    Serial.printf("Unsupported Shelly profile for switching: %s\n", shellyProfile.c_str());
    return false;
  }

  DynamicJsonDocument doc(2048);
  if (!httpCallJson(url, doc)) {
    Serial.println("Failed to switch Shelly output");
    return false;
  }

  Serial.printf("Shelly switch: zone=%d -> %s\n", zone, on ? "ON" : "OFF");
  return true;
}

void printLightProfileStatus(JsonObject result, int16_t& z1, int16_t& z2, int16_t& z3, int16_t& z4) {
  int16_t zones[4] = {0, 0, 0, 0};

  for (int i = 0; i < 4; i++) {
    String key = "light:" + String(i);
    JsonObject ch = result[key];

    if (ch.isNull()) {
      Serial.printf("Channel %d: missing\n", i);
      zones[i] = 0;
      continue;
    }

    bool output = ch["output"] | false;
    int brightness = ch["brightness"] | -1;
    float apower = ch["apower"] | 0.0;

    zones[i] = output ? 1 : 0;

    Serial.printf("Channel %d -> output=%s, brightness=%d, power=%.2f W\n",
                  i,
                  output ? "ON" : "OFF",
                  brightness,
                  apower);
  }

  z1 = zones[0];
  z2 = zones[1];
  z3 = zones[2];
  z4 = zones[3];
}

void printRgbwProfileStatus(JsonObject result, int16_t& z1, int16_t& z2, int16_t& z3, int16_t& z4) {
  JsonObject rgbw = result["rgbw:0"];
  if (rgbw.isNull()) {
    Serial.println("rgbw:0 missing");
    z1 = z2 = z3 = z4 = 0;
    return;
  }

  bool output = rgbw["output"] | false;
  int brightness = rgbw["brightness"] | -1;

  Serial.printf("RGBW -> output=%s, brightness=%d",
                output ? "ON" : "OFF",
                brightness);

  if (rgbw["rgb"].is<JsonArray>()) {
    JsonArray rgb = rgbw["rgb"].as<JsonArray>();
    if (rgb.size() >= 3) {
      Serial.printf(", rgb=[%d,%d,%d]",
                    rgb[0].as<int>(),
                    rgb[1].as<int>(),
                    rgb[2].as<int>());
    }
  }

  if (!rgbw["white"].isNull()) {
    Serial.printf(", white=%d", rgbw["white"].as<int>());
  }

  Serial.println();

  z1 = output ? 1 : 0;
  z2 = 0;
  z3 = 0;
  z4 = 0;
}

void printRgbProfileStatus(JsonObject result, int16_t& z1, int16_t& z2, int16_t& z3, int16_t& z4) {
  JsonObject rgb = result["rgb:0"];
  if (rgb.isNull()) {
    Serial.println("rgb:0 missing");
    z1 = z2 = z3 = z4 = 0;
    return;
  }

  bool output = rgb["output"] | false;
  int brightness = rgb["brightness"] | -1;

  Serial.printf("RGB -> output=%s, brightness=%d\n",
                output ? "ON" : "OFF",
                brightness);

  z1 = output ? 1 : 0;
  z2 = 0;
  z3 = 0;
  z4 = 0;
}

void pollShellyStatus() {
  String url = String("http://") + SHELLY_HOST +
               "/rpc/Shelly.GetStatus?id=1";

  DynamicJsonDocument doc(8192);
  if (!httpGetJson(url, doc)) {
    return;
  }

  JsonObject result = doc.as<JsonObject>();
  if (result.isNull()) {
    Serial.println("No result in Shelly.GetStatus response");
    serializeJsonPretty(doc, Serial);
    Serial.println();
    return;
  }

  Serial.println("----- Shelly status -----");

  int16_t z1 = 0;
  int16_t z2 = 0;
  int16_t z3 = 0;
  int16_t z4 = 0;

  if (shellyProfile == "light") {
    printLightProfileStatus(result, z1, z2, z3, z4);
  } else if (shellyProfile == "rgbw") {
    printRgbwProfileStatus(result, z1, z2, z3, z4);
  } else if (shellyProfile == "rgb") {
    printRgbProfileStatus(result, z1, z2, z3, z4);
  } else {
    Serial.println("Unknown profile, raw payload:");
    serializeJsonPretty(result, Serial);
    Serial.println();
    return;
  }

  sendLightsStatusPacket(z1, z2, z3, z4);
}

void handleLightsSwitchPacket(const LightsSwitchPacket& pkt) {
  if (pkt.type != TYPE_LIGHT_SWITCH) {
    return;
  }

  int zone = pkt.zone;
  bool state = (pkt.state != 0);

  Serial.printf("UART LightsSwitchPacket: zone=%d state=%d\n", zone, pkt.state);
  setShellyLightState(zone, state);
}

void handleUartFrame(const uint8_t* data, size_t len) {
  int16_t type;
  if (len < sizeof(type)) {
    return;
  }
  memcpy(&type, data, sizeof(type));

  if (type == TYPE_LIGHT_SWITCH && len == sizeof(LightsSwitchPacket)) {
    LightsSwitchPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));
    handleLightsSwitchPacket(pkt);
  } else {
    Serial.printf("UART frame ignored: type=%d len=%u\n", type, (unsigned)len);
  }
}

void processUartRx() {
  unsigned long now = millis();

  // Drop a partial frame that stalled, the next byte starts a new frame
  if (uartFrameLen != 0 && now - uartLastByteMs > uartFrameGapMs) {
    Serial.printf("UART partial frame dropped (%u of %u bytes)\n",
                  (unsigned)uartRxLen, (unsigned)uartFrameLen);
    uartFrameLen = 0;
    uartRxLen = 0;
  }

  while (U.available() > 0) {
    int b = U.read();
    if (b < 0) {
      break;
    }
    lastUartActivityMs = now;
    uartLastByteMs = now;

    if (uartFrameLen == 0) {
      // Length byte; one that cannot be a frame we handle is noise, skip it
      if (b == 0 || (size_t)b > sizeof(uartRxBuf)) {
        continue;
      }
      uartFrameLen = (size_t)b;
      uartRxLen = 0;
      continue;
    }

    uartRxBuf[uartRxLen++] = (uint8_t)b;
    if (uartRxLen == uartFrameLen) {
      handleUartFrame(uartRxBuf, uartRxLen);
      uartFrameLen = 0;
      uartRxLen = 0;
    }
  }
}

bool connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting to WiFi");
  unsigned long connectionAttemptStart = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");

    if (millis() - connectionAttemptStart >= connectionTimeoutMs) {
      return false;
    }
  }
  Serial.println();
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  return true;
}

void setup() {
  Serial.begin(115200);
  U.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  delay(1000);

  Serial.println("Shelly connector");

  // Boot into low-power waiting mode
  systemAwake = false;
  lastPollMs = millis();
  lastUartActivityMs = millis();

  enterSleepUntilUartActivity();

  // After wake, become fully active
  systemAwake = true;

  // Optional: discard garbage that woke us
  while (U.available()) {
    U.read();
  }

  if (!connectWiFi())
  {
    Serial.println("Could not connect to Shelly, rebooting...");
    delay(100);
    ESP.restart();
  }

  if (!getShellyDeviceInfo()) {
    Serial.println("Failed to read Shelly device info.");
  }

  lastUartActivityMs = millis();
}

void loop() {
  if (!systemAwake) {
    return;
  }

  processUartRx();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, reconnecting...");
    if (!connectWiFi())
    {
      Serial.println("Could not connect to Shelly, rebooting...");
      delay(100);
      ESP.restart();
    }
  }

  if (millis() - lastPollMs >= pollIntervalMs) {
    lastPollMs = millis();
    pollShellyStatus();
  }

  if (millis() - lastUartActivityMs >= inactivityTimeoutMs) {
    Serial.println("No UART activity for 5 minutes, rebooting...");
    delay(100);
    ESP.restart();
  }
}