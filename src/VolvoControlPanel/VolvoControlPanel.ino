#include "src\waveshare/lvgl_port.h"           // LVGL porting functions for integration with ESP32
#include "ContractsInclude.hpp"
#include <esp_mac.h> 
#include <WiFi.h>
#include <esp_now.h>
#include "CamperUI.hpp"
#include "espnow_retry.hpp"

using namespace CamperUI;

static constexpr uint32_t SLEEP_AFTER_MS = 60000;
static constexpr gpio_num_t POWER_RELAY_GPIO = GPIO_NUM_6;

static uint32_t lastActivityMs = 0;
static bool sleeping = false;

static lv_obj_t* wakeLayer = nullptr;

static void wakeLayerEvent(lv_event_t* e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        Serial.println("Waking up!");
        exitUiSleep();
    }
}

static void showWakeLayer()
{
    if (wakeLayer) return;

    wakeLayer = lv_obj_create(lv_layer_top());   // important: top layer, not lv_scr_act()
    lv_obj_remove_style_all(wakeLayer);

    lv_obj_set_pos(wakeLayer, 0, 0);
    lv_obj_set_size(wakeLayer, LV_HOR_RES, LV_VER_RES);

    lv_obj_add_flag(wakeLayer, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(wakeLayer, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_ext_click_area(wakeLayer, 0);

    lv_obj_add_event_cb(wakeLayer, wakeLayerEvent, LV_EVENT_RELEASED, nullptr);
    lv_obj_add_event_cb(wakeLayer, wakeLayerEvent, LV_EVENT_PRESS_LOST, nullptr);

    lv_obj_move_foreground(wakeLayer);
}

static void hideWakeLayer()
{
    if (!wakeLayer) return;

    lv_obj_del(wakeLayer);
    wakeLayer = nullptr;
}

static bool initEspNow()
{
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("❌ ESP-NOW init failed!");
        return false;
    }

    esp_now_register_recv_cb(onEspNowRecv);
    esp_now_register_send_cb(onEspNowSent);

    esp_now_peer_info_t peerInfo = {};
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    memcpy(peerInfo.peer_addr, Vehicle::LILYGO_WORKLIGHT_MAC, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("❌ Failed to add LILYGO peer!");
        return false;
    }

    memcpy(peerInfo.peer_addr, Vehicle::SHELLY_BRIDGE_MAC, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("❌ Failed to add SHELLY peer!");
        return false;
    }

    return true;
}

static void enterUiSleep()
{
    if (sleeping) return;

    Serial.println("Entering UI sleep");

    sleeping = true;

    // Turn OFF external ESP32s
    digitalWrite(POWER_RELAY_GPIO, LOW);

    showWakeLayer();

    // Turn off LCD backlight
    wavesahre_rgb_lcd_bl_off();

    // Pause LVGL if your LVGL version supports this
    //lv_timer_enable(false);

    // Stop ESP-NOW and Wi-Fi
    esp_now_deinit();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
}

static void exitUiSleep()
{
    if (!sleeping) return;

    Serial.println("Waking UI");

    // Restart Wi-Fi / ESP-NOW
    initEspNow();

    // Power ON external ESP32s first
    digitalWrite(POWER_RELAY_GPIO, HIGH);

    // Resume LVGL
    //lv_timer_enable(true);
    hideWakeLayer();

    // Turn LCD backlight back on
    wavesahre_rgb_lcd_bl_on();

    sleeping = false;
    lastActivityMs = millis();
}

void onEspNowRecv(const esp_now_recv_info_t *info,
                  const uint8_t *data, int len)
{
    if (len < 1) {
      return;
    }
    uint8_t type = data[0];
    Serial.printf("ESP-NOW packet received. Type: %d Length: %d\n", type, len);
    if (type == Contracts::TYPE_WORKLIGHT_STATUS && len == (int)sizeof(Contracts::WorkLightsStatusPacket))
    {
        Contracts::WorkLightsStatusPacket pkt;
        memcpy(&pkt, data, sizeof(pkt));

        setLightState(LightGroup::Exterior, 0, pkt.relay1);
        setLightState(LightGroup::Exterior, 1, pkt.relay2);
        setLightState(LightGroup::Exterior, 2, pkt.relay3);
        setLightState(LightGroup::Exterior, 3, pkt.relay4);

        return;
    }
    if (type == Contracts::TYPE_BMS && len == (int)sizeof(Contracts::BmsPacket))
    {
        Contracts::BmsPacket pkt;
        memcpy(&pkt, data, sizeof(pkt));

        // Serial.println(pkt.volts);
        // Serial.println(pkt.amps);
        // Serial.println(pkt.soc_percent);
        // Serial.println(pkt.remaining);
        // Serial.println(pkt.temp);
        setSocPercent(pkt.soc_percent);            // 0..100
        setRemainingAh(pkt.remaining / (float)100);              // e.g. 62.0
        setChargeCurrentA(pkt.amps / (float)100);            // e.g. 12.4

        return;
    }
    if (type == Contracts::TYPE_LIGHT_STATUS && len == (int)sizeof(Contracts::LightsStatusPacket))
    {
        Contracts::LightsStatusPacket pkt;
        memcpy(&pkt, data, sizeof(pkt));

        setLightState(LightGroup::Interior, 0, pkt.zone1);
        setLightState(LightGroup::Interior, 1, pkt.zone2);
        setLightState(LightGroup::Interior, 2, pkt.zone3);
        setLightState(LightGroup::Interior, 3, pkt.zone4);

        return;
    }
}

void onEspNowSent(const wifi_tx_info_t* tx_info, esp_now_send_status_t status) {
    Serial.print("ESP-NOW send status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

static void noteActivity()
{
    if (sleeping) {
        exitUiSleep();
        return;
    }

    lastActivityMs = millis();
}

static void onLightChanged(CamperUI::LightGroup group, uint8_t idx, bool on) {
    
    noteActivity();

    Serial.printf("Light group=%u idx=%u -> %s\n",
                  (unsigned)group, (unsigned)idx, on ? "ON" : "OFF");
    // TODO: your IO / ESP-NOW logic here

    if (group == CamperUI::LightGroup::Interior) {
        Contracts::LightsSwitchPacket pkt;
        pkt.type = Contracts::TYPE_LIGHT_SWITCH;
        pkt.zone = (int16_t)idx+1;
        pkt.state = (int16_t)on;

        esp_err_t res = espnow_send_cached(Vehicle::SHELLY_BRIDGE_MAC, (uint8_t*)&pkt, sizeof(pkt));
        if (res != ESP_OK) {
            Serial.print("ESP-NOW send: ");
            Serial.println(res == ESP_OK ? "OK" : String("ERR ") + res);
        }

    } else {
        Contracts::WorkLightsSwitchPacket pkt;
        pkt.type = Contracts::TYPE_WORKLIGHT_SWITCH;
        pkt.input = (int16_t)idx+1;
        pkt.state = (int16_t)on;

        esp_err_t res = espnow_send_cached(Vehicle::LILYGO_WORKLIGHT_MAC, (uint8_t*)&pkt, sizeof(pkt));
        if (res != ESP_OK) {
            Serial.print("ESP-NOW send: ");
            Serial.println(res == ESP_OK ? "OK" : String("ERR ") + res);
        }
    }
}

static void onAllOff() {

    noteActivity();

    Serial.println("ALL OFF triggered");
    // TODO: your IO / ESP-NOW logic here
}

static void onActPressed(CamperUI::Actuator a, CamperUI::Direction d) {

    noteActivity();

    Serial.printf("PRESS act=%u dir=%u\n", (unsigned)a, (unsigned)d);
    // start motor/relay
}

static void onActReleased(CamperUI::Actuator a, CamperUI::Direction d) {

    noteActivity();

    Serial.printf("RELEASE act=%u dir=%u\n", (unsigned)a, (unsigned)d);
    // stop motor/relay
}

static void build_ui()
{
    CamperUI::Callbacks cb;
    cb.onLightChanged = onLightChanged;
    cb.onAllOff = onAllOff;
    cb.onActuatorPressed = onActPressed;
    cb.onActuatorReleased = onActReleased;
    CamperUI::init(cb);

    // Example: set some live values
    CamperUI::setSocPercent(78);
    CamperUI::setRemainingAh(62.0f);
    CamperUI::setChargeCurrentA(12.4f);
    CamperUI::setDischargeCurrentA(6.8f);
}

void setup() {
    Serial.begin(115200);

    Serial.println();
    Serial.println("Control panel");

    if (!initEspNow()) {
        return;
    }

    pinMode(POWER_RELAY_GPIO, OUTPUT);

    // HIGH = relay ON
    digitalWrite(POWER_RELAY_GPIO, HIGH);

    static esp_lcd_panel_handle_t panel_handle = NULL;
    static esp_lcd_touch_handle_t tp_handle = NULL;

    tp_handle = touch_gt911_init();
    panel_handle = waveshare_esp32_s3_rgb_lcd_init();
    wavesahre_rgb_lcd_bl_on();

    ESP_ERROR_CHECK(lvgl_port_init(panel_handle, tp_handle));

    ESP_LOGI(TAG, "Display LVGL UI");

    if (lvgl_port_lock(-1)) {
        build_ui();
        lvgl_port_unlock();
    }

    Serial.println("Setup complete");
    lastActivityMs = millis();
}

void loop() {
    if (!sleeping && millis() - lastActivityMs > SLEEP_AFTER_MS) {
        enterUiSleep();
    }

    delay(100);
}