#include "sound.hpp"
#include <Arduino.h>
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "freertos/queue.h"

// Pins from Guition's mp3_player example and the board schematic
static const int PIN_MCLK = 13;
static const int PIN_BCLK = 12;
static const int PIN_WS = 10;
static const int PIN_DOUT = 9;
static const int PIN_AMP = 11;     // NS4150 CTRL, high = on
static const uint16_t ES8311_ADDR = 0x18;

// MCLK = 256 * sound::SAMPLE_RATE = 4.096 MHz
static const uint8_t DAC_VOLUME = 0xB3;      // 0xBF = 0 dB, 0.5 dB per step
using sound::SAMPLE_RATE;
using sound::GAP_MS;

static i2c_master_dev_handle_t codec;
static i2s_chan_handle_t tx;
static QueueHandle_t queue;

static bool codec_write(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(codec, buf, sizeof(buf), 100) == ESP_OK;
}

// ES8311 as I2S slave, MCLK from its pin, 16-bit I2S, DAC only.
// Register values follow Espressif's es8311 driver for 4.096 MHz / 16 kHz.
static bool codec_init()
{
    static const uint8_t init[][2] = {
        { 0x00, 0x1F },   // reset
        { 0x00, 0x00 },
        { 0x00, 0x80 },   // power on, slave mode
        { 0x01, 0x3F },   // all clocks on, MCLK from the pin
        { 0x02, 0x00 },   // pre-divider 1, multiplier 1
        { 0x03, 0x10 },   // ADC single speed, OSR 16
        { 0x04, 0x10 },   // DAC OSR 16
        { 0x05, 0x00 },   // ADC and DAC dividers 1
        { 0x06, 0x03 },   // BCLK divider 4
        { 0x07, 0x00 },   // LRCK divider 256
        { 0x08, 0xFF },
        { 0x09, 0x0C },   // serial in: I2S, 16 bit
        { 0x0A, 0x0C },   // serial out: I2S, 16 bit
        { 0x10, 0x1F },   // analog references up
        { 0x11, 0x7F },
        { 0x0D, 0x01 },   // power up the analog circuitry
        { 0x0E, 0x02 },
        { 0x12, 0x00 },   // power up the DAC
        { 0x13, 0x10 },   // enable the output driver
        { 0x1C, 0x6A },
        { 0x37, 0x08 },   // bypass the DAC equalizer
        { 0x31, 0x00 },   // unmute
        { 0x32, DAC_VOLUME },
    };
    for (size_t i = 0; i < sizeof(init) / sizeof(init[0]); i++) {
        if (!codec_write(init[i][0], init[i][1])) return false;
        if (i == 0) delay(20);
    }
    return true;
}

static void write_silence(uint32_t ms)
{
    static const int16_t zeros[256] = {};
    size_t left = SAMPLE_RATE * ms / 1000, written;
    while (left) {
        size_t n = left < 256 ? left : 256;
        i2s_channel_write(tx, zeros, n * sizeof(int16_t), &written, portMAX_DELAY);
        left -= n;
    }
}

static void sound_task(void *)
{
    const Sound *s;
    for (;;) {
        xQueueReceive(queue, &s, portMAX_DELAY);
        digitalWrite(PIN_AMP, HIGH);
        for (;;) {
            size_t written;
            i2s_channel_write(tx, s->pcm, s->samples * sizeof(int16_t), &written, portMAX_DELAY);
            write_silence(GAP_MS);
            if (xQueueReceive(queue, &s, 0) != pdTRUE) break;
        }
        // Let the DMA drain before switching the amp off
        write_silence(100);
        digitalWrite(PIN_AMP, LOW);
    }
}

namespace sound {

bool begin()
{
    pinMode(PIN_AMP, OUTPUT);
    digitalWrite(PIN_AMP, LOW);

    i2c_master_bus_handle_t bus;
    if (i2c_master_get_bus_handle(I2C_NUM_1, &bus) != ESP_OK) return false;
    i2c_device_config_t dev = {};
    dev.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev.device_address = ES8311_ADDR;
    dev.scl_speed_hz = 100000;
    if (i2c_master_bus_add_device(bus, &dev, &codec) != ESP_OK) return false;

    // MCLK has to run before the codec is configured
    i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan.auto_clear = true;   // send zeros when there is nothing to play
    if (i2s_new_channel(&chan, &tx, nullptr) != ESP_OK) return false;

    i2s_std_config_t std = {};
    std.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE);
    std.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
    std.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;   // same sample on both channels
    std.gpio_cfg.mclk = (gpio_num_t)PIN_MCLK;
    std.gpio_cfg.bclk = (gpio_num_t)PIN_BCLK;
    std.gpio_cfg.ws = (gpio_num_t)PIN_WS;
    std.gpio_cfg.dout = (gpio_num_t)PIN_DOUT;
    std.gpio_cfg.din = I2S_GPIO_UNUSED;
    if (i2s_channel_init_std_mode(tx, &std) != ESP_OK) return false;
    if (i2s_channel_enable(tx) != ESP_OK) return false;

    if (!codec_init()) {
        Serial.println("ES8311 not responding");
        return false;
    }

    queue = xQueueCreate(8, sizeof(const Sound *));
    xTaskCreate(sound_task, "sound", 4096, nullptr, 3, nullptr);
    return true;
}

void play(const Sound *const *sequence)
{
    if (!queue) return;
    for (; *sequence; sequence++) xQueueSend(queue, sequence, 0);
}

}  // namespace sound
