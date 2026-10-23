#include "bms_ui.hpp"
#include <Arduino.h>

// Values are greyed out when no BMS packet arrived for this long
static const uint32_t BMS_STALE_MS = 10000;

static const lv_color_t COLOR_BG    = lv_color_make(32, 32, 32);
static const lv_color_t COLOR_FRESH = lv_color_white();
static const lv_color_t COLOR_STALE = lv_color_make(110, 110, 110);

struct BmsBarUI {
    lv_obj_t *bar;
    lv_obj_t *label_volts;
    lv_obj_t *label_amps;
    lv_obj_t *label_soc;
    lv_obj_t *label_remaining;
    lv_obj_t *label_temp;
};

static BmsBarUI ui;
static uint32_t lastUpdateMs = 0;
static bool hasData = false;

static lv_obj_t *create_value_label(lv_obj_t *parent)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(label, COLOR_STALE, 0);
    lv_label_set_text(label, "--");
    return label;
}

static void set_values_color(lv_color_t color)
{
    lv_obj_t *labels[] = { ui.label_volts, ui.label_amps, ui.label_soc, ui.label_remaining, ui.label_temp };
    for (lv_obj_t *l : labels) {
        lv_obj_set_style_text_color(l, color, 0);
    }
}

static void bms_stale_timer_cb(lv_timer_t *t)
{
    if (hasData && millis() - lastUpdateMs > BMS_STALE_MS) {
        set_values_color(COLOR_STALE);
    }
}

lv_obj_t *create_bms_bar(lv_obj_t *parent, int width, int height)
{
    ui.bar = lv_obj_create(parent);
    lv_obj_remove_style_all(ui.bar);
    lv_obj_set_size(ui.bar, width, height);
    lv_obj_align(ui.bar, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(ui.bar, COLOR_BG, 0);
    lv_obj_set_style_bg_opa(ui.bar, LV_OPA_COVER, 0);
    lv_obj_clear_flag(ui.bar, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_flex_flow(ui.bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui.bar, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    ui.label_volts     = create_value_label(ui.bar);
    ui.label_amps      = create_value_label(ui.bar);
    ui.label_soc       = create_value_label(ui.bar);
    ui.label_remaining = create_value_label(ui.bar);
    ui.label_temp      = create_value_label(ui.bar);

    lv_timer_create(bms_stale_timer_cb, 1000, nullptr);
    return ui.bar;
}

void bms_bar_update(const Contracts::BmsPacket &b)
{
    char buf[16];

    snprintf(buf, sizeof(buf), "%.2f V", b.volts / 100.0f);
    lv_label_set_text(ui.label_volts, buf);

    snprintf(buf, sizeof(buf), "%+.2f A", b.amps / 100.0f);
    lv_label_set_text(ui.label_amps, buf);

    snprintf(buf, sizeof(buf), "%d %%", b.soc_percent);
    lv_label_set_text(ui.label_soc, buf);

    snprintf(buf, sizeof(buf), "%.1f Ah", b.remaining / 100.0f);
    lv_label_set_text(ui.label_remaining, buf);

    snprintf(buf, sizeof(buf), "%.1f °C", b.temp / 10.0f);
    lv_label_set_text(ui.label_temp, buf);

    lastUpdateMs = millis();
    hasData = true;
    set_values_color(COLOR_FRESH);
}
