#include "bms_ui.hpp"
#include "fo_theme.hpp"
#include <Arduino.h>

// Values are dimmed when no BMS packet arrived for this long
static const uint32_t BMS_STALE_MS = 10000;

struct BmsBarUI {
    lv_obj_t *bar;
    lv_obj_t *label_volts;
    lv_obj_t *label_amps;
    lv_obj_t *label_soc;
    lv_obj_t *label_temp;
};

static BmsBarUI ui;
volatile uint32_t bms_last_update = 0;
static bool fresh = false;   // values shown in bright amber

static void set_values_color(uint32_t color)
{
    lv_obj_t *labels[] = { ui.label_volts, ui.label_amps, ui.label_soc, ui.label_temp };
    for (lv_obj_t *l : labels) {
        lv_obj_set_style_text_color(l, fo::hex(color), 0);
    }
}

static void bms_stale_timer_cb(lv_timer_t *t)
{
    if (fresh && millis() - bms_last_update > BMS_STALE_MS) {
        fresh = false;
        set_values_color(fo::AMBER_OFF);
    }
}

lv_obj_t *create_bms_bar(lv_obj_t *parent, int width, int height)
{
    ui.bar = fo::metal_panel(parent, 0, fo::SCREEN_H - height, width, height);
    lv_obj_set_style_border_color(ui.bar, fo::hex(fo::EDGE), 0);
    lv_obj_set_style_border_width(ui.bar, 3, 0);
    lv_obj_set_style_pad_hor(ui.bar, 12, 0);
    lv_obj_set_style_pad_ver(ui.bar, 8, 0);
    lv_obj_set_style_pad_column(ui.bar, 8, 0);
    lv_obj_set_flex_flow(ui.bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui.bar, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Four amber counters share the space left of the status lamps
    lv_obj_t *counters = fo::box(ui.bar);
    lv_obj_set_flex_grow(counters, 1);
    lv_obj_set_height(counters, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(counters, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(counters, 6, 0);

    ui.label_volts     = fo::counter(counters, "VOLTS");
    ui.label_amps      = fo::counter(counters, "AMPS");
    ui.label_soc       = fo::counter(counters, "CHARGE");
    ui.label_temp      = fo::counter(counters, "TEMP");
    set_values_color(fo::AMBER_OFF);

    lv_timer_create(bms_stale_timer_cb, 1000, nullptr);
    return ui.bar;
}

void bms_bar_update(const Contracts::BmsPacket &b)
{
    char buf[16];

    // Volts to 0.1 V (the resting voltage tells the charge state), the rest whole
    // numbers; units are on the captions
    snprintf(buf, sizeof(buf), "%.1f", b.volts / 100.0f);
    lv_label_set_text(ui.label_volts, buf);

    snprintf(buf, sizeof(buf), "%+ld", lroundf(b.amps / 100.0f));
    lv_label_set_text(ui.label_amps, buf);

    snprintf(buf, sizeof(buf), "%d%%", b.soc_percent);
    lv_label_set_text(ui.label_soc, buf);

    snprintf(buf, sizeof(buf), "%ld°", lroundf(b.temp / 10.0f));
    lv_label_set_text(ui.label_temp, buf);

    bms_last_update = millis();
    if (!fresh) {
        fresh = true;
        set_values_color(fo::AMBER);
    }
}
