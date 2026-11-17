#include "obd_ui.hpp"
#include "fo_theme.hpp"
#include "modem_link.hpp"
#include <Arduino.h>

using namespace Contracts;

volatile uint32_t obd_last_update = 0;

// Values are cleared when the connector has been silent for this long
// (BLE scanning and connecting can pause it for several seconds)
static const uint32_t OBD_STALE_MS = 10000;

// ATF temperatures: inverted from WARN, inverted and blinking from CRIT
static const int ATF_WARN_C = 105;
static const int ATF_CRIT_C = 115;
// Coolant: same, from these
static const int COOLANT_WARN_C = 105;
static const int COOLANT_CRIT_C = 110;
// Temperature bars (ATF, coolant): 18 segments of 5 °C from 40 °C
static const int ATF_BAR_MIN = 40;
static const int ATF_BAR_STEP = 5;
static const int ATF_SEGS = 18;

// Boost bar: 20 segments of 0.1 bar, centi-bar
static const int BOOST_BAR_MAX = 200;
static const int BOOST_SEGS = 20;

// Vertical nudge for the gear digit in its frame (the 120 px font holds digits only,
// so its line box is the digit itself)
static const lv_coord_t GEAR_NUDGE_Y = 0;

static const lv_coord_t ROW_W = fo::CRT_W - 16 - 2 * 14;   // page width minus padding

struct ObdUI {
    lv_obj_t *gear;
    lv_obj_t *lockup_box;
    lv_obj_t *lockup;
    lv_obj_t *map;
    lv_obj_t *battery;
    lv_obj_t *phone;   // switch: keeps the connector off the dongle
    fo::ReadingRow coolant;
    fo::ReadingRow atf_pan;
    fo::ReadingRow atf_outlet;
    fo::ReadingRow boost;
    lv_obj_t *status;
    lv_obj_t *cursor;
};

static ObdUI ui;
static bool stale = true;

// ---- Phone switch: while on, the connector is told every PHONE_RELEASE_REFRESH_MS to stay off
// the dongle, so a phone app can use it. Off, or the dashboard losing power, stops the repeats
// and the connector takes the dongle back within PHONE_RELEASE_MS. ----

static bool phone_on = false;
static uint32_t release_sent_ms = 0;

static void send_release(int16_t arg)
{
    ObdCommandPacket p = { TYPE_OBD_COMMAND, OBD_CMD_RELEASE, arg };
    modem::send(Vehicle::OBD_CONNECTOR_MAC, &p, sizeof(p));
    release_sent_ms = millis();
}

static void phone_cb(lv_event_t *e)
{
    phone_on = !phone_on;
    send_release(phone_on ? 1 : 0);   // off: reconnect now rather than after the release runs out
    if (phone_on) lv_obj_add_state(ui.phone, LV_STATE_CHECKED);
    else lv_obj_clear_state(ui.phone, LV_STATE_CHECKED);
    lv_label_set_text(lv_obj_get_child(ui.phone, 0), phone_on ? "OBD FREED" : "FREE OBD");
    Serial.printf("OBD phone switch %s\n", phone_on ? "on" : "off");
}

static void set_temperature(fo::ReadingRow &r, int16_t value, int warn = ATF_WARN_C, int crit = ATF_CRIT_C)
{
    if (value == OBD_NO_VALUE) {
        fo::reading_row_set(r, "--", false, false);
        fo::seg_bar_set(r.bar, 0, fo::PHOSPHOR);
        return;
    }
    char buf[16];
    snprintf(buf, sizeof(buf), value >= warn ? "%d°C HOT" : "%d°C", value);
    fo::reading_row_set(r, buf, value >= warn, value >= crit);
    fo::seg_bar_set(r.bar, (value - ATF_BAR_MIN + ATF_BAR_STEP - 1) / ATF_BAR_STEP, fo::PHOSPHOR);
}

static void set_lockup(int16_t lockup)
{
    const char *text = lockup == OBD_NO_VALUE ? "--" : lockup ? "LOCKED" : "OPEN";
    lv_label_set_text(ui.lockup, text);
    bool locked = lockup != OBD_NO_VALUE && lockup;
    lv_obj_set_style_bg_opa(ui.lockup_box, locked ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_color(ui.lockup, fo::hex(locked ? fo::CRT_BG : fo::PHOSPHOR), 0);
}

static void set_status(const char *text)
{
    if (strcmp(lv_label_get_text(ui.status), text) != 0) lv_label_set_text(ui.status, text);
}

static void clear_values()
{
    lv_label_set_text(ui.gear, "-");
    set_lockup(OBD_NO_VALUE);
    set_temperature(ui.atf_pan, OBD_NO_VALUE);
    set_temperature(ui.atf_outlet, OBD_NO_VALUE);
    fo::reading_row_set(ui.boost, "--", false, false);
    fo::seg_bar_set(ui.boost.bar, 0, fo::PHOSPHOR);
    lv_label_set_text(ui.map, "MAP -- kPa");
    set_temperature(ui.coolant, OBD_NO_VALUE);
    lv_label_set_text(ui.battery, "BATTERY -- V");
}

static void obd_timer_cb(lv_timer_t *t)
{
    // Blinking cursor and critical values
    static bool phase = false;
    phase = !phase;
    if (phase) lv_obj_add_flag(ui.cursor, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_clear_flag(ui.cursor, LV_OBJ_FLAG_HIDDEN);

    fo::ReadingRow *rows[] = { &ui.atf_pan, &ui.atf_outlet, &ui.coolant };
    for (fo::ReadingRow *r : rows) {
        fo::reading_row_blink(*r, phase);
    }

    if (phone_on && millis() - release_sent_ms >= PHONE_RELEASE_REFRESH_MS) send_release(1);

    if (obd_last_update == 0) {
        set_status("> WAITING FOR OBD CONNECTOR");
        return;
    }
    if (!stale && millis() - obd_last_update > OBD_STALE_MS) {
        stale = true;
        clear_values();
        set_status("> NO DATA FROM OBD CONNECTOR");
    }
}

void create_obd_tab(lv_obj_t *parent)
{
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(parent, 12, 0);

    fo::label(parent, &fo_vt323_24, fo::PHOSPHOR_DIM, "> DRIVETRAIN STATUS");

    // Gear in a double frame, converter state and MAP beside it
    lv_obj_t *top = fo::box(parent);
    lv_obj_set_size(top, ROW_W, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(top, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(top, 16, 0);

    lv_obj_t *gear_box = fo::box(top);
    lv_obj_set_size(gear_box, 150, 150);
    lv_obj_set_style_border_color(gear_box, fo::hex(fo::PHOSPHOR), 0);
    lv_obj_set_style_border_width(gear_box, 2, 0);
    lv_obj_t *inner = fo::box(gear_box);
    lv_obj_set_size(inner, 150 - 12, 150 - 12);
    lv_obj_center(inner);
    lv_obj_set_style_border_color(inner, fo::hex(fo::PHOSPHOR), 0);
    lv_obj_set_style_border_width(inner, 2, 0);
    // Raw gear code from 21DA, not yet verified against the VDJ200 gearbox
    ui.gear = fo::label(gear_box, &fo_vt323_120, fo::PHOSPHOR, "-");
    lv_obj_align(ui.gear, LV_ALIGN_CENTER, 0, GEAR_NUDGE_Y);

    lv_obj_t *side = fo::box(top);
    lv_obj_set_flex_grow(side, 1);
    lv_obj_set_height(side, 150);
    lv_obj_set_flex_flow(side, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(side, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(side, 4, 0);

    fo::label(side, &fo_vt323_24, fo::PHOSPHOR_DIM, "TORQUE CONVERTER");
    // Converter state, and the phone switch beside it
    lv_obj_t *lock_row = fo::box(side);
    lv_obj_set_size(lock_row, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(lock_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(lock_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    ui.lockup_box = fo::box(lock_row);
    lv_obj_set_size(ui.lockup_box, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_color(ui.lockup_box, fo::hex(fo::PHOSPHOR), 0);
    lv_obj_set_style_border_color(ui.lockup_box, fo::hex(fo::PHOSPHOR), 0);
    lv_obj_set_style_border_width(ui.lockup_box, 2, 0);
    lv_obj_set_style_pad_hor(ui.lockup_box, 10, 0);
    ui.lockup = fo::label(ui.lockup_box, &fo_vt323_40, fo::PHOSPHOR, "--");
    ui.phone = fo::crt_button(lock_row, "FREE OBD", 108, 40, &fo_vt323_24, false);
    lv_obj_add_event_cb(ui.phone, phone_cb, LV_EVENT_CLICKED, nullptr);
    ui.map = fo::label(side, &fo_vt323_24, fo::PHOSPHOR_DIM, "MAP -- kPa");
    ui.battery = fo::label(side, &fo_vt323_24, fo::PHOSPHOR_DIM, "BATTERY -- V");

    fo::reading_row(parent, ui.atf_pan, "ATF PAN", ATF_SEGS, ROW_W);
    fo::reading_row(parent, ui.atf_outlet, "ATF OUTLET", ATF_SEGS, ROW_W);
    fo::reading_row(parent, ui.coolant, "COOLANT", ATF_SEGS, ROW_W);
    fo::reading_row(parent, ui.boost, "BOOST", BOOST_SEGS, ROW_W);

    lv_obj_t *scale = fo::box(parent);
    lv_obj_set_size(scale, ROW_W, LV_SIZE_CONTENT);
    lv_obj_set_style_pad_top(scale, -8, 0);
    lv_obj_t *l = fo::label(scale, &fo_vt323_20, fo::PHOSPHOR_DIM, "0");
    lv_obj_align(l, LV_ALIGN_LEFT_MID, 0, 0);
    l = fo::label(scale, &fo_vt323_20, fo::PHOSPHOR_DIM, "1.0");
    lv_obj_align(l, LV_ALIGN_CENTER, 0, 0);
    l = fo::label(scale, &fo_vt323_20, fo::PHOSPHOR_DIM, "2.0 BAR");
    lv_obj_align(l, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_set_height(scale, 20);

    // Status line at the bottom, with a blinking cursor
    lv_obj_t *spacer = fo::box(parent);
    lv_obj_set_flex_grow(spacer, 1);

    lv_obj_t *status_row = fo::box(parent);
    lv_obj_set_size(status_row, ROW_W, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(status_row, LV_FLEX_FLOW_ROW);
    ui.status = fo::label(status_row, &fo_vt323_24, fo::PHOSPHOR, "");
    ui.cursor = fo::label(status_row, &fo_vt323_24, fo::PHOSPHOR, "_");

    clear_values();
    set_status("> WAITING FOR OBD CONNECTOR");
    lv_timer_create(obd_timer_cb, 500, nullptr);
}

void obd_ui_update(const ObdPacket &p)
{
    obd_last_update = millis();
    stale = false;

    switch (p.status) {
    case OBD_STATUS_NO_DONGLE:
        set_status("> SEARCHING FOR OBD DONGLE");
        break;
    case OBD_STATUS_NO_ECU:
        set_status("> NO ECU ANSWER (IGNITION OFF?)");
        break;
    case OBD_STATUS_RELEASED:
        set_status("> OBD DONGLE FREE FOR THE PHONE APP");
        break;
    default:
        set_status("> ECU LINK OK");
        break;
    }

    char buf[24];

    if (p.gear == OBD_NO_VALUE) {
        lv_label_set_text(ui.gear, "-");
    } else {
        snprintf(buf, sizeof(buf), "%d", p.gear);
        lv_label_set_text(ui.gear, buf);
    }

    set_lockup(p.lockup);
    set_temperature(ui.atf_pan, p.atf_pan);
    set_temperature(ui.atf_outlet, p.atf_outlet);
    set_temperature(ui.coolant, p.coolant, COOLANT_WARN_C, COOLANT_CRIT_C);

    if (p.battery_cv == OBD_NO_VALUE) {
        lv_label_set_text(ui.battery, "BATTERY -- V");
    } else {
        snprintf(buf, sizeof(buf), "BATTERY %.1f V", p.battery_cv / 100.0f);
        lv_label_set_text(ui.battery, buf);
    }

    if (p.boost == OBD_NO_VALUE) {
        fo::reading_row_set(ui.boost, "--", false, false);
        fo::seg_bar_set(ui.boost.bar, 0, fo::PHOSPHOR);
    } else {
        snprintf(buf, sizeof(buf), "%.2f BAR", p.boost / 100.0f);
        fo::reading_row_set(ui.boost, buf, false, false);
        int v = p.boost < 0 ? 0 : p.boost > BOOST_BAR_MAX ? BOOST_BAR_MAX : p.boost;
        fo::seg_bar_set(ui.boost.bar, (v * BOOST_SEGS + BOOST_BAR_MAX / 2) / BOOST_BAR_MAX, fo::PHOSPHOR);
    }

    if (p.map_kpa == OBD_NO_VALUE) {
        lv_label_set_text(ui.map, "MAP -- kPa");
    } else {
        snprintf(buf, sizeof(buf), "MAP %d kPa", p.map_kpa);
        lv_label_set_text(ui.map, buf);
    }
}
