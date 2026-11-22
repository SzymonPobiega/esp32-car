// Bench rig for the CarHub: a fake heater connector (ESP-NOW, HeaterPacket every second, runs and
// acknowledges HeaterCommandPackets) and a fake JBD BMS (BLE FF00 / FF01 notify / FF02 write).
// Flash it to a spare ESP32 and the CarHub firmware built as tbeam-carhub-toyota-bench, which
// points at this board (CarHubVehicle.h, CARHUB_BENCH). Not part of either car.
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "../Shared/Contracts.hpp"

using namespace Contracts;

static const uint8_t CARHUB_MAC[6] = {0xA4, 0xF0, 0x0F, 0xD8, 0xB4, 0x40};

static HeaterPacket heater = {};
static uint32_t stateUntil = 0; // igniting / cooling down end
static volatile bool sendNow = false;
static QueueHandle_t commands;

static NimBLECharacteristic *bmsNotify;
static volatile bool bmsRequested = false;

static void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len == sizeof(HeaterCommandPacket)) {
        HeaterCommandPacket c;
        memcpy(&c, data, sizeof(c));
        if (c.type == TYPE_HEATER_COMMAND) xQueueSend(commands, &c, 0);
    }
}

class BmsWrite : public NimBLECharacteristicCallbacks
{
    void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &) override
    {
        std::string v = c->getValue();
        if (v.size() >= 3 && (uint8_t)v[0] == 0xDD && (uint8_t)v[1] == 0xA5 && (uint8_t)v[2] == 0x03) bmsRequested = true;
    }
};

static void sendBmsFrame()
{
    uint8_t pl[27] = {};
    uint16_t volts = 1321;   // 13.21 V
    int16_t amps = -140;     // -1.40 A
    uint16_t remaining = 8650;
    uint16_t kelvin = 2731 + 125; // 12.5 C
    pl[0] = volts >> 8; pl[1] = volts;
    pl[2] = (uint16_t)amps >> 8; pl[3] = (uint16_t)amps;
    pl[4] = remaining >> 8; pl[5] = remaining;
    pl[19] = 87;  // SOC
    pl[21] = 4;   // cells
    pl[22] = 1;   // NTCs
    pl[23] = kelvin >> 8; pl[24] = kelvin;
    uint8_t f[4 + sizeof(pl) + 3];
    f[0] = 0xDD; f[1] = 0x03; f[2] = 0x00; f[3] = sizeof(pl);
    memcpy(f + 4, pl, sizeof(pl));
    uint16_t sum = f[2] + f[3];
    for (uint8_t b : pl) sum += b;
    uint16_t chk = (uint16_t)(0x10000 - sum);
    f[4 + sizeof(pl)] = chk >> 8;
    f[5 + sizeof(pl)] = chk;
    f[6 + sizeof(pl)] = 0x77;
    // Two notifications, like a real JBD (20-byte MTU chunks)
    bmsNotify->notify(f, 20);
    bmsNotify->notify(f + 20, sizeof(f) - 20);
    Serial.println("BMS: answered basic info");
}

static void apply(const HeaterCommandPacket &c)
{
    if (c.seq != 0 && c.seq == heater.ack_seq) return; // repeat
    bool levelMode = heater.mode == HEATER_MODE_LEVEL;
    switch (c.command) {
    case HEATER_CMD_START: heater.state = HEATER_STATE_IGNITING; stateUntil = millis() + 4000; break;
    case HEATER_CMD_STOP: heater.state = HEATER_STATE_COOLDOWN; stateUntil = millis() + 4000; break;
    case HEATER_CMD_UP: heater.setting = min<int>(heater.setting + 1, levelMode ? 6 : 35); break;
    case HEATER_CMD_DOWN: heater.setting = max<int>(heater.setting - 1, levelMode ? 1 : 5); break;
    case HEATER_CMD_MODE_LEVEL: heater.mode = HEATER_MODE_LEVEL; heater.setting = 3; break;
    case HEATER_CMD_MODE_THERMOSTAT: heater.mode = HEATER_MODE_THERMOSTAT; heater.setting = 20; break;
    case HEATER_CMD_RUN_AT: heater.run_at = c.arg; break;
    case HEATER_CMD_RUN_ARM: heater.run_armed = c.arg; break;
    }
    heater.ack_seq = c.seq;
    heater.ack_result = HEATER_ACK_OK;
    Serial.printf("Heater: command %d arg %d seq %d -> state 0x%02X mode %d setting %d\n", c.command, c.arg, c.seq,
                  heater.state, heater.mode, heater.setting);
    sendNow = true;
}

void setup()
{
    Serial.begin(115200);
    commands = xQueueCreate(8, sizeof(HeaterCommandPacket));

    NimBLEDevice::init("FAKEBMS");
    NimBLEServer *server = NimBLEDevice::createServer();
    server->advertiseOnDisconnect(true);
    NimBLEService *svc = server->createService("FF00");
    bmsNotify = svc->createCharacteristic("FF01", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    NimBLECharacteristic *w = svc->createCharacteristic("FF02", NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    static BmsWrite cb;
    w->setCallbacks(&cb);
    svc->start();
    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->setName("FAKEBMS");
    adv->addServiceUUID("FF00");
    adv->start();

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_now_init();
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, CARHUB_MAC, 6);
    esp_now_add_peer(&peer);
    esp_now_register_recv_cb(onRecv);

    heater.type = TYPE_HEATER;
    heater.link = HEATER_LINK_OK;
    heater.state = HEATER_STATE_OFF;
    heater.mode = HEATER_MODE_LEVEL;
    heater.setting = 3;
    heater.volts = 128;
    heater.case_temp = 215;
    heater.cabin_temp = 182;
    heater.run_at = 6 * 60 + 30;
    heater.clock = HEATER_CLOCK_OK;
    heater.time = 14 * 60;
    Serial.printf("CarHub rig up, BLE %s\n", NimBLEDevice::getAddress().toString().c_str());
}

void loop()
{
    static uint32_t lastSend = 0;
    HeaterCommandPacket c;
    while (xQueueReceive(commands, &c, 0) == pdTRUE) apply(c);

    uint32_t now = millis();
    if (stateUntil && now > stateUntil) {
        stateUntil = 0;
        heater.state = heater.state == HEATER_STATE_IGNITING ? HEATER_STATE_RUNNING : HEATER_STATE_OFF;
        heater.case_temp = heater.state == HEATER_STATE_RUNNING ? 1800 : 215;
        Serial.printf("Heater: now 0x%02X\n", heater.state);
        sendNow = true;
    }
    if (sendNow || now - lastSend >= 1000) {
        sendNow = false;
        lastSend = now;
        esp_now_send(CARHUB_MAC, (uint8_t *)&heater, sizeof(heater));
    }
    if (bmsRequested) {
        bmsRequested = false;
        sendBmsFrame();
    }
    delay(10);
}
