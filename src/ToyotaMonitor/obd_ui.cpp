#include "obd_ui.hpp"
#include <Arduino.h>

using namespace Contracts;

volatile uint32_t obd_last_update = 0;

// Values are cleared when the connector has been silent for this long
// (BLE scanning and connecting can pause it for several seconds)
static const uint32_t OBD_STALE_MS = 10000;

// ATF temperature colouring
static const int ATF_WARN_C = 105;
static const int ATF_CRIT_C = 115;

// Boost bar range, centi-bar
static const int BOOST_BAR_MAX = 200;

static const lv_color_t COLOR_TILE    = lv_color_make(0x20, 0x20, 0x20);
static const lv_color_t COLOR_CAPTION = lv_color_make(170, 170, 170);
static const lv_color_t COLOR_VALUE   = lv_color_white();
static const lv_color_t COLOR_NONE    = lv_color_make(110, 110, 110);
static const lv_color_t COLOR_OK      = lv_color_make(80, 220, 100);
static const lv_color_t COLOR_WARN    = lv_color_make(255, 165, 0);
static const lv_color_t COLOR_CRIT    = lv_color_make(230, 40, 40);

struct ObdUI {
    lv_obj_t *gear;
    lv_obj_t *lockup;
    lv_obj_t *atf_pan;
    lv_obj_t *atf_outlet;
    lv_obj_t *boost;
    lv_obj_t *boost_bar;
    lv_obj_t *map;
    lv_obj_t *status;
};

static ObdUI ui;
static bool stale = true;

static lv_obj_t *create_tile(lv_obj_t *parent, const char *caption, int col, int col_span, int row)
{
    lv_obj_t *tile = lv_obj_create(parent);
    lv_obj_remove_style_all(tile);
    lv_obj_set_style_bg_color(tile, COLOR_TILE, 0);
    lv_obj_set_style_bg_opa(tile, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(tile, 12, 0);
    lv_obj_set_style_pad_all(tile, 10, 0);
    lv_obj_clear_flag(tile, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_grid_cell(tile, LV_GRID_ALIGN_STRETCH, col, col_span, LV_GRID_ALIGN_STRETCH, row, 1);

    lv_obj_t *label = lv_label_create(tile);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(label, COLOR_CAPTION, 0);
    lv_label_set_text(label, caption);
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, 0);

    return tile;
}

static lv_obj_t *create_value(lv_obj_t *tile, const lv_font_t *font)
{
    lv_obj_t *label = lv_label_create(tile);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, COLOR_NONE, 0);
    lv_label_set_text(label, "--");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 8);
    return label;
}

static void set_value(lv_obj_t *label, const char *text, lv_color_t color)
{
    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(label, color, 0);
}

static void set_temperature(lv_obj_t *label, int16_t value)
{
    if (value == OBD_NO_VALUE) {
        set_value(label, "-- °C", COLOR_NONE);
        return;
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%d °C", value);
    lv_color_t color = value >= ATF_CRIT_C ? COLOR_CRIT : value >= ATF_WARN_C ? COLOR_WARN : COLOR_VALUE;
    set_value(label, buf, color);
}

static void clear_values()
{
    set_value(ui.gear, "--", COLOR_NONE);
    set_value(ui.lockup, "--", COLOR_NONE);
    set_temperature(ui.atf_pan, OBD_NO_VALUE);
    set_temperature(ui.atf_outlet, OBD_NO_VALUE);
    set_value(ui.boost, "-- bar", COLOR_NONE);
    lv_bar_set_value(ui.boost_bar, 0, LV_ANIM_OFF);
    set_value(ui.map, "MAP -- kPa", COLOR_CAPTION);
}

static void obd_stale_timer_cb(lv_timer_t *t)
{
    if (obd_last_update == 0) {
        lv_label_set_text(ui.status, "Waiting for OBD connector");
        return;
    }
    if (!stale && millis() - obd_last_update > OBD_STALE_MS) {
        stale = true;
        clear_values();
        lv_label_set_text(ui.status, "No data from OBD connector");
    }
}

void create_obd_tab(lv_obj_t *parent)
{
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_remove_style_all(cont);
    lv_obj_set_size(cont, lv_pct(100), lv_pct(100));
    lv_obj_set_style_pad_all(cont, 10, 0);
    lv_obj_set_style_pad_row(cont, 10, 0);
    lv_obj_set_style_pad_column(cont, 10, 0);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

    static lv_coord_t col_dsc[] = { LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };
    static lv_coord_t row_dsc[] = {
        LV_GRID_FR(4),        // gear, lockup
        LV_GRID_FR(3),        // ATF temperatures
        LV_GRID_FR(3),        // boost
        LV_GRID_CONTENT,      // status line
        LV_GRID_TEMPLATE_LAST
    };
    lv_obj_set_grid_dsc_array(cont, col_dsc, row_dsc);

    ui.gear       = create_value(create_tile(cont, "CURRENT GEAR", 0, 1, 0), &lv_font_montserrat_48);
    ui.lockup     = create_value(create_tile(cont, "CONVERTER", 1, 1, 0), &lv_font_montserrat_36);
    ui.atf_pan    = create_value(create_tile(cont, "ATF PAN", 0, 1, 1), &lv_font_montserrat_36);
    ui.atf_outlet = create_value(create_tile(cont, "ATF OUTLET", 1, 1, 1), &lv_font_montserrat_36);

    lv_obj_t *boost_tile = create_tile(cont, "BOOST", 0, 2, 2);
    ui.boost = create_value(boost_tile, &lv_font_montserrat_36);
    lv_obj_align(ui.boost, LV_ALIGN_CENTER, 0, -6);

    ui.map = lv_label_create(boost_tile);
    lv_obj_set_style_text_font(ui.map, &lv_font_montserrat_14, 0);
    lv_obj_align(ui.map, LV_ALIGN_TOP_RIGHT, 0, 0);

    ui.boost_bar = lv_bar_create(boost_tile);
    lv_obj_set_size(ui.boost_bar, lv_pct(100), 14);
    lv_obj_align(ui.boost_bar, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_bar_set_range(ui.boost_bar, 0, BOOST_BAR_MAX);

    ui.status = lv_label_create(cont);
    lv_obj_set_style_text_font(ui.status, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(ui.status, lv_color_black(), 0);
    lv_obj_set_grid_cell(ui.status, LV_GRID_ALIGN_CENTER, 0, 2, LV_GRID_ALIGN_CENTER, 3, 1);

    clear_values();
    lv_label_set_text(ui.status, "Waiting for OBD connector");
    lv_timer_create(obd_stale_timer_cb, 500, nullptr);
}

void obd_ui_update(const ObdPacket &p)
{
    obd_last_update = millis();
    stale = false;

    switch (p.status) {
    case OBD_STATUS_NO_DONGLE:
        lv_label_set_text(ui.status, "Connector is searching for the OBD dongle");
        break;
    case OBD_STATUS_NO_ECU:
        lv_label_set_text(ui.status, "Dongle connected, vehicle not answering (ignition off?)");
        break;
    default:
        lv_label_set_text(ui.status, "");
        break;
    }

    char buf[24];

    // Raw gear code from 21DA, not yet verified against the VDJ200 gearbox
    if (p.gear == OBD_NO_VALUE) {
        set_value(ui.gear, "--", COLOR_NONE);
    } else {
        snprintf(buf, sizeof(buf), "%d", p.gear);
        set_value(ui.gear, buf, COLOR_VALUE);
    }

    if (p.lockup == OBD_NO_VALUE) {
        set_value(ui.lockup, "--", COLOR_NONE);
    } else if (p.lockup) {
        set_value(ui.lockup, "LOCKED", COLOR_OK);
    } else {
        set_value(ui.lockup, "OPEN", COLOR_VALUE);
    }

    set_temperature(ui.atf_pan, p.atf_pan);
    set_temperature(ui.atf_outlet, p.atf_outlet);

    if (p.boost == OBD_NO_VALUE) {
        set_value(ui.boost, "-- bar", COLOR_NONE);
        lv_bar_set_value(ui.boost_bar, 0, LV_ANIM_OFF);
    } else {
        snprintf(buf, sizeof(buf), "%.2f bar", p.boost / 100.0f);
        set_value(ui.boost, buf, COLOR_VALUE);
        int v = p.boost < 0 ? 0 : p.boost > BOOST_BAR_MAX ? BOOST_BAR_MAX : p.boost;
        lv_bar_set_value(ui.boost_bar, v, LV_ANIM_OFF);
    }

    if (p.map_kpa == OBD_NO_VALUE) {
        set_value(ui.map, "MAP -- kPa", COLOR_CAPTION);
    } else {
        snprintf(buf, sizeof(buf), "MAP %d kPa", p.map_kpa);
        set_value(ui.map, buf, COLOR_CAPTION);
    }
}
