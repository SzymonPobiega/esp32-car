#include "heater_ui.hpp"
#include "fo_theme.hpp"
#include "modem_link.hpp"
#include <Arduino.h>

using namespace Contracts;

volatile uint32_t heater_last_update = 0;

// Values are cleared when the connector has been silent for this long
// (scanning for the heater pauses it for several seconds)
static const uint32_t HEATER_STALE_MS = 15000;

// Case temperature bar: 20 segments of 10 °C
static const int CASE_BAR_STEP = 10;
static const int CASE_SEGS = 20;
// Cabin temperature bar: 20 segments of 2 °C from -10 °C
static const int CABIN_BAR_MIN = -10;
static const int CABIN_BAR_STEP = 2;
static const int CABIN_SEGS = 20;

// Commands: resent until the connector echoes the seq (it runs each seq once)
static const uint32_t RESEND_MS = 1500;
static const int MAX_SENDS = 4;               // the connector can be busy scanning for 5 s
static const uint32_t FEEDBACK_MS = 3000;     // how long a command result stays on the status line
static const uint32_t HOLD_MS = 1000;         // start / stop need a long press
static const uint32_t PENDING_SHOW_MS = 6000; // a changed run time shows until echoed, or this long

static const int RUN_STEP_MIN = 15;
static const int RUN_LENGTH_MIN = 30;

static const lv_coord_t ROW_W = fo::CRT_W - 16 - 2 * 14;   // page width minus padding
static const lv_coord_t SETTING_BOX = 120;
static const lv_coord_t GAP = 6;

// Start-up and shutdown phases, highlighted in the strip under the state
enum Phase { PHASE_NONE = -1, PHASE_IGNITE, PHASE_WARM_UP, PHASE_RUN, PHASE_COOL, PHASE_COUNT };
static const char *const kPhaseNames[PHASE_COUNT] = { "IGNITE", "WARM UP", "RUN", "COOL" };

struct StateInfo {
    const char *name;   // nullptr for a state code we don't know
    Phase phase;
};

static StateInfo state_info(int16_t state)
{
    switch (state) {
    case HEATER_STATE_OFF:               return { "OFF", PHASE_NONE };
    case HEATER_STATE_IGNITION_RECEIVED:
    case HEATER_STATE_IGNITION_STARTING:
    case HEATER_STATE_IGNITING:          return { "IGNITING", PHASE_IGNITE };
    case HEATER_STATE_HEATING:           return { "WARMING UP", PHASE_WARM_UP };
    case HEATER_STATE_RUNNING:           return { "RUNNING", PHASE_RUN };
    case HEATER_STATE_COOLDOWN_RECEIVED:
    case HEATER_STATE_COOLDOWN_STARTING:
    case HEATER_STATE_COOLDOWN:          return { "COOLING DOWN", PHASE_COOL };
    case HEATER_STATE_ERROR:             return { "FAULT", PHASE_NONE };
    default:                             return { nullptr, PHASE_NONE };
    }
}

static const char *command_name(int16_t command)
{
    switch (command) {
    case HEATER_CMD_STOP:            return "STOP";
    case HEATER_CMD_START:           return "START";
    case HEATER_CMD_UP:              return "UP";
    case HEATER_CMD_DOWN:            return "DOWN";
    case HEATER_CMD_MODE_THERMOSTAT: return "THERMOSTAT";
    case HEATER_CMD_MODE_LEVEL:      return "LEVEL";
    case HEATER_CMD_RUN_AT:          return "RUN TIME";
    case HEATER_CMD_RUN_ARM:         return "ARM";
    default:                         return "COMMAND";
    }
}

struct HeaterUI {
    lv_obj_t *setting;
    lv_obj_t *state_box;
    lv_obj_t *state;
    lv_obj_t *mode;
    lv_obj_t *supply;
    lv_obj_t *phases[PHASE_COUNT];
    fo::ReadingRow case_temp;
    fo::ReadingRow cabin_temp;
    lv_obj_t *power;        // hold to start / stop
    lv_obj_t *power_fill;   // hold progress strip
    lv_obj_t *level;
    lv_obj_t *thermostat;
    lv_obj_t *down;
    lv_obj_t *up;
    lv_obj_t *earlier;
    lv_obj_t *run_at;
    lv_obj_t *later;
    lv_obj_t *arm;
    lv_obj_t *run_status;
    lv_obj_t *status;
    lv_obj_t *cursor;
};

static HeaterUI ui;
static HeaterPacket last = {};     // last packet from the connector
static bool stale = true;          // no heater data (link down or connector silent)
static bool connector_up = false;  // connector heard from recently
static bool fault = false;         // state box blinks
static char status_text[48] = "";  // normal status line, behind command feedback
static int16_t power_command = 0;  // what the power button sends, 0 = disabled

// The command in flight
static struct {
    HeaterCommandPacket packet;
    bool active;
    int sends;
    uint32_t sent_ms;
} pending;
static int16_t next_seq = 1;
static uint32_t feedback_until = 0;

// Run time / arm as last asked for, shown until the connector reports them
static int16_t asked_run_at = -1;
static int16_t asked_armed = -1;
static uint32_t asked_ms = 0;

static void set_text(lv_obj_t *label, const char *text)
{
    if (strcmp(lv_label_get_text(label), text) != 0) lv_label_set_text(label, text);
}

static void set_button_text(lv_obj_t *button, const char *text)
{
    set_text(lv_obj_get_child(button, 0), text);
}

static void set_enabled(lv_obj_t *obj, bool on)
{
    if (on) lv_obj_clear_state(obj, LV_STATE_DISABLED);
    else lv_obj_add_state(obj, LV_STATE_DISABLED);
}

static void set_checked(lv_obj_t *obj, bool on)
{
    if (on) lv_obj_add_state(obj, LV_STATE_CHECKED);
    else lv_obj_clear_state(obj, LV_STATE_CHECKED);
}

// ---- Status line: command feedback for a few seconds, otherwise the link status ----

static void show_status()
{
    if (pending.active || millis() < feedback_until) return;
    set_text(ui.status, status_text);
}

static void set_status(const char *text)
{
    snprintf(status_text, sizeof(status_text), "%s", text);
    show_status();
}

static void feedback(const char *text)
{
    set_text(ui.status, text);
    feedback_until = millis() + FEEDBACK_MS;
}

// ---- Commands ----

static void transmit()
{
    modem::send(Vehicle::HEATER_CONNECTOR_MAC, &pending.packet, sizeof(pending.packet));
    pending.sends++;
    pending.sent_ms = millis();
}

static void send_command(int16_t command, int16_t arg = 0)
{
    pending.packet = { TYPE_HEATER_COMMAND, next_seq, command, arg };
    next_seq = next_seq >= 0x7FFF ? 1 : next_seq + 1;
    pending.active = true;
    pending.sends = 0;
    transmit();

    char buf[48];
    snprintf(buf, sizeof(buf), "> SENDING %s...", command_name(command));
    set_text(ui.status, buf);
    Serial.printf("Heater command %d %s(%d)\n", pending.packet.seq, command_name(command), arg);
}

static void check_ack(const HeaterPacket &p)
{
    if (!pending.active || p.ack_seq != pending.packet.seq) return;
    pending.active = false;

    char buf[48];
    const char *name = command_name(pending.packet.command);
    switch (p.ack_result) {
    case HEATER_ACK_OK:      snprintf(buf, sizeof(buf), "> %s: SENT TO HEATER", name); break;
    case HEATER_ACK_NO_LINK: snprintf(buf, sizeof(buf), "> %s: HEATER NOT CONNECTED", name); break;
    default:                 snprintf(buf, sizeof(buf), "> %s: FAILED", name); break;
    }
    if (pending.packet.command == HEATER_CMD_RUN_AT || pending.packet.command == HEATER_CMD_RUN_ARM) {
        if (p.ack_result == HEATER_ACK_OK) snprintf(buf, sizeof(buf), "> %s SAVED", name);
    }
    feedback(buf);
}

static void resend_timer()
{
    if (!pending.active || millis() - pending.sent_ms < RESEND_MS) return;
    if (pending.sends < MAX_SENDS) {
        transmit();
        return;
    }
    pending.active = false;
    asked_run_at = -1;
    asked_armed = -1;
    feedback(modem::modemAlive() ? "> NO REPLY FROM HEATER CONNECTOR" : "> ESP-NOW MODEM NOT RUNNING");
}

// ---- Hold to start / stop: a strip fills along the bottom of the button ----

static void fill_width_cb(void *obj, int32_t w)
{
    lv_obj_set_width((lv_obj_t *)obj, w);
}

static void hold_done_cb(lv_anim_t *a)
{
    lv_obj_set_width(ui.power_fill, 0);
    if (power_command) send_command(power_command);
}

static void power_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED && power_command) {
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, ui.power_fill);
        lv_anim_set_exec_cb(&a, fill_width_cb);
        lv_anim_set_values(&a, 0, lv_obj_get_content_width(ui.power));
        lv_anim_set_time(&a, HOLD_MS);
        lv_anim_set_ready_cb(&a, hold_done_cb);
        lv_anim_start(&a);
    } else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        // Let go early: cancel
        lv_anim_del(ui.power_fill, fill_width_cb);
        lv_obj_set_width(ui.power_fill, 0);
    }
}

static void command_clicked_cb(lv_event_t *e)
{
    send_command((int16_t)(intptr_t)lv_event_get_user_data(e));
}

// ---- Morning run ----

static int16_t shown_run_at()
{
    return asked_run_at >= 0 ? asked_run_at : last.run_at;
}

static bool shown_armed()
{
    return asked_armed >= 0 ? asked_armed : last.run_armed;
}

static void show_run()
{
    // What was asked for shows until the connector reports it (or gives up)
    if (asked_run_at >= 0 && (last.run_at == asked_run_at || millis() - asked_ms > PENDING_SHOW_MS)) asked_run_at = -1;
    if (asked_armed >= 0 && (last.run_armed == asked_armed || millis() - asked_ms > PENDING_SHOW_MS)) asked_armed = -1;

    lv_obj_t *run_controls[] = { ui.earlier, ui.later, ui.arm };
    for (lv_obj_t *b : run_controls) set_enabled(b, connector_up);
    if (!connector_up) {
        set_text(ui.run_at, "--:--");
        set_checked(ui.arm, false);
        set_button_text(ui.arm, "ARM");
        set_text(ui.run_status, "> MORNING RUN: NO CONNECTOR");
        return;
    }

    char buf[48];
    int16_t at = shown_run_at();
    snprintf(buf, sizeof(buf), "%02d:%02d", at / 60, at % 60);
    set_text(ui.run_at, buf);

    bool armed = shown_armed();
    set_checked(ui.arm, armed);
    set_button_text(ui.arm, armed ? "ARMED" : "ARM");

    if (last.clock != HEATER_CLOCK_OK) {
        set_text(ui.run_status, armed ? "> ARMED, INACTIVE: NO RTC YET" : "> MORNING RUN INACTIVE: NO RTC");
    } else if (armed) {
        snprintf(buf, sizeof(buf), "> RUNS %d MIN FROM %02d:%02d", RUN_LENGTH_MIN, at / 60, at % 60);
        set_text(ui.run_status, buf);
    } else {
        snprintf(buf, sizeof(buf), "> %d MIN MORNING RUN NOT ARMED", RUN_LENGTH_MIN);
        set_text(ui.run_status, buf);
    }
}

static void run_step_cb(lv_event_t *e)
{
    int step = (int)(intptr_t)lv_event_get_user_data(e);
    int at = (shown_run_at() + step + 24 * 60) % (24 * 60);
    asked_run_at = (int16_t)(at - at % RUN_STEP_MIN);
    asked_ms = millis();
    send_command(HEATER_CMD_RUN_AT, asked_run_at);
    show_run();
}

static void arm_cb(lv_event_t *e)
{
    asked_armed = shown_armed() ? 0 : 1;
    asked_ms = millis();
    send_command(HEATER_CMD_RUN_ARM, asked_armed);
    show_run();
}

// ---- Heater state ----

// State in a framed box, inverted while the heater burns or has a fault
static void set_state_box(const char *text, bool inverted)
{
    set_text(ui.state, text);
    lv_obj_set_style_bg_opa(ui.state_box, inverted ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_color(ui.state, fo::hex(inverted ? fo::CRT_BG : fo::PHOSPHOR), 0);
}

static void set_phase(Phase phase)
{
    for (int i = 0; i < PHASE_COUNT; i++) {
        fo::set_inverted(ui.phases[i], i == phase);
        if (i != phase) lv_obj_set_style_text_color(ui.phases[i], fo::hex(fo::PHOSPHOR_OFF), 0);
    }
}

static void set_temperature(fo::ReadingRow &r, int16_t deci, int bar_min, int bar_step)
{
    char buf[16];
    long c = lroundf(deci / 10.0f);
    snprintf(buf, sizeof(buf), "%ld°C", c);
    fo::reading_row_set(r, buf, false, false);
    fo::seg_bar_set(r.bar, (c - bar_min + bar_step - 1) / bar_step, fo::PHOSPHOR);
}

// Controls follow what the heater reports, not what was sent
static void show_controls()
{
    bool link = !stale;
    StateInfo info = state_info(last.state);

    if (!link) {
        power_command = 0;
        set_button_text(ui.power, "HOLD TO START");
    } else if (info.phase == PHASE_COOL) {
        // Cooldown can't be interrupted
        power_command = 0;
        set_button_text(ui.power, "COOLING DOWN...");
    } else if (last.state == HEATER_STATE_OFF) {
        power_command = HEATER_CMD_START;
        set_button_text(ui.power, "HOLD TO START");
    } else {
        power_command = HEATER_CMD_STOP;
        set_button_text(ui.power, "HOLD TO STOP");
    }
    set_enabled(ui.power, power_command != 0);

    bool level = link && last.mode == HEATER_MODE_LEVEL;
    bool thermostat = link && last.mode == HEATER_MODE_THERMOSTAT;
    set_enabled(ui.level, link);
    set_enabled(ui.thermostat, link);
    set_checked(ui.level, level);
    set_checked(ui.thermostat, thermostat);
    set_enabled(ui.down, level || thermostat);
    set_enabled(ui.up, level || thermostat);
}

static void clear_values()
{
    set_text(ui.setting, "-");
    set_state_box("--", false);
    set_text(ui.mode, "MODE --");
    set_text(ui.supply, "SUPPLY -- V");
    set_phase(PHASE_NONE);
    fo::reading_row_set(ui.case_temp, "--", false, false);
    fo::seg_bar_set(ui.case_temp.bar, 0, fo::PHOSPHOR);
    fo::reading_row_set(ui.cabin_temp, "--", false, false);
    fo::seg_bar_set(ui.cabin_temp.bar, 0, fo::PHOSPHOR);
    fault = false;
}

static void heater_timer_cb(lv_timer_t *t)
{
    // Blinking cursor and fault
    static bool phase = false;
    phase = !phase;
    if (phase) lv_obj_add_flag(ui.cursor, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_clear_flag(ui.cursor, LV_OBJ_FLAG_HIDDEN);
    if (fault) set_state_box("FAULT", phase);

    resend_timer();

    if (heater_last_update == 0) {
        set_status("> WAITING FOR HEATER CONNECTOR");
    } else if (connector_up && millis() - heater_last_update > HEATER_STALE_MS) {
        connector_up = false;
        if (!stale) clear_values();
        stale = true;
        set_status("> NO DATA FROM HEATER CONNECTOR");
    }
    show_controls();
    show_run();
    show_status();
}

// ---- Layout ----

static lv_obj_t *row(lv_obj_t *parent)
{
    lv_obj_t *r = fo::box(parent);
    lv_obj_set_size(r, ROW_W, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(r, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    return r;
}

static lv_obj_t *command_button(lv_obj_t *parent, const char *text, lv_coord_t w, const lv_font_t *font,
                                int16_t command)
{
    lv_obj_t *b = fo::crt_button(parent, text, w, 44, font);
    lv_obj_add_event_cb(b, command_clicked_cb, LV_EVENT_CLICKED, (void *)(intptr_t)command);
    return b;
}

void create_heater_tab(lv_obj_t *parent)
{
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(parent, GAP, 0);
    next_seq = (int16_t)(esp_random() % 0x7FFE) + 1;   // a restarted dashboard doesn't repeat seqs

    fo::label(parent, &fo_vt323_24, fo::PHOSPHOR_DIM, "> DIESEL HEATER");

    // Level or target temperature in a double frame, state and readings beside it
    lv_obj_t *top = fo::box(parent);
    lv_obj_set_size(top, ROW_W, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(top, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(top, 16, 0);

    lv_obj_t *setting_box = fo::box(top);
    lv_obj_set_size(setting_box, SETTING_BOX, SETTING_BOX);
    lv_obj_set_style_border_color(setting_box, fo::hex(fo::PHOSPHOR), 0);
    lv_obj_set_style_border_width(setting_box, 2, 0);
    lv_obj_t *inner = fo::box(setting_box);
    lv_obj_set_size(inner, SETTING_BOX - 12, SETTING_BOX - 12);
    lv_obj_center(inner);
    lv_obj_set_style_border_color(inner, fo::hex(fo::PHOSPHOR), 0);
    lv_obj_set_style_border_width(inner, 2, 0);
    ui.setting = fo::label(setting_box, &fo_vt323_120, fo::PHOSPHOR, "-");
    lv_obj_align(ui.setting, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *side = fo::box(top);
    lv_obj_set_flex_grow(side, 1);
    lv_obj_set_height(side, SETTING_BOX);
    lv_obj_set_flex_flow(side, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(side, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(side, 4, 0);

    fo::label(side, &fo_vt323_24, fo::PHOSPHOR_DIM, "HCALORY 8KW");
    ui.state_box = fo::box(side);
    lv_obj_set_size(ui.state_box, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_color(ui.state_box, fo::hex(fo::PHOSPHOR), 0);
    lv_obj_set_style_border_color(ui.state_box, fo::hex(fo::PHOSPHOR), 0);
    lv_obj_set_style_border_width(ui.state_box, 2, 0);
    lv_obj_set_style_pad_hor(ui.state_box, 10, 0);
    ui.state = fo::label(ui.state_box, &fo_vt323_32, fo::PHOSPHOR, "--");
    ui.mode = fo::label(side, &fo_vt323_24, fo::PHOSPHOR_DIM, "");
    ui.supply = fo::label(side, &fo_vt323_24, fo::PHOSPHOR_DIM, "");

    // IGNITE > WARM UP > RUN > COOL
    lv_obj_t *strip = row(parent);
    for (int i = 0; i < PHASE_COUNT; i++) {
        if (i > 0) fo::label(strip, &fo_vt323_24, fo::PHOSPHOR_OFF, ">");
        ui.phases[i] = fo::label(strip, &fo_vt323_24, fo::PHOSPHOR_OFF, kPhaseNames[i]);
        lv_obj_set_style_bg_color(ui.phases[i], fo::hex(fo::PHOSPHOR), 0);
        lv_obj_set_style_pad_hor(ui.phases[i], 6, 0);
    }

    fo::reading_row(parent, ui.case_temp, "CASE", CASE_SEGS, ROW_W);
    fo::reading_row(parent, ui.cabin_temp, "CABIN", CABIN_SEGS, ROW_W);

    // Start / stop: a long press, the strip along the bottom shows how long
    ui.power = fo::crt_button(parent, "HOLD TO START", ROW_W, 48, &fo_vt323_32, false);
    lv_obj_add_event_cb(ui.power, power_event_cb, LV_EVENT_ALL, nullptr);
    ui.power_fill = fo::box(ui.power);
    lv_obj_set_size(ui.power_fill, 0, 6);
    lv_obj_align(ui.power_fill, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_set_style_bg_color(ui.power_fill, fo::hex(fo::PHOSPHOR), 0);
    lv_obj_set_style_bg_opa(ui.power_fill, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(ui.power, 2, 0);

    // Mode and level / target temperature
    lv_obj_t *modes = row(parent);
    ui.level = command_button(modes, "LEVEL", 116, &fo_vt323_24, HEATER_CMD_MODE_LEVEL);
    ui.thermostat = command_button(modes, "THERMOSTAT", 140, &fo_vt323_24, HEATER_CMD_MODE_THERMOSTAT);
    ui.down = command_button(modes, "-", 64, &fo_vt323_32, HEATER_CMD_DOWN);
    ui.up = command_button(modes, "+", 64, &fo_vt323_32, HEATER_CMD_UP);

    // Morning run: start time in 15 minute steps, armed for the next one
    lv_obj_t *run = row(parent);
    ui.earlier = fo::crt_button(run, "<", 56, 44, &fo_vt323_32);
    lv_obj_add_event_cb(ui.earlier, run_step_cb, LV_EVENT_CLICKED, (void *)(intptr_t)-RUN_STEP_MIN);
    ui.run_at = fo::label(run, &fo_vt323_32, fo::PHOSPHOR, "--:--");
    ui.later = fo::crt_button(run, ">", 56, 44, &fo_vt323_32);
    lv_obj_add_event_cb(ui.later, run_step_cb, LV_EVENT_CLICKED, (void *)(intptr_t)RUN_STEP_MIN);
    ui.arm = fo::crt_button(run, "ARM", 150, 44, &fo_vt323_24);
    lv_obj_add_event_cb(ui.arm, arm_cb, LV_EVENT_CLICKED, nullptr);

    ui.run_status = fo::label(parent, &fo_vt323_24, fo::PHOSPHOR_DIM, "");

    // Status line at the bottom, with a blinking cursor
    lv_obj_t *spacer = fo::box(parent);
    lv_obj_set_flex_grow(spacer, 1);

    lv_obj_t *status_row = fo::box(parent);
    lv_obj_set_size(status_row, ROW_W, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(status_row, LV_FLEX_FLOW_ROW);
    ui.status = fo::label(status_row, &fo_vt323_24, fo::PHOSPHOR, "");
    ui.cursor = fo::label(status_row, &fo_vt323_24, fo::PHOSPHOR, "_");

    clear_values();
    set_status("> WAITING FOR HEATER CONNECTOR");
    show_controls();
    show_run();
    lv_timer_create(heater_timer_cb, 500, nullptr);
}

void heater_ui_update(const HeaterPacket &p)
{
    heater_last_update = millis();
    last = p;
    connector_up = true;
    check_ack(p);

    if (p.link != HEATER_LINK_OK) {
        if (!stale) clear_values();
        stale = true;
        set_status("> SEARCHING FOR HEATER");
        show_controls();
        show_run();
        return;
    }
    stale = false;

    char buf[32];
    StateInfo info = state_info(p.state);
    fault = p.state == HEATER_STATE_ERROR;
    if (!info.name) {
        snprintf(buf, sizeof(buf), "STATE %02X", p.state);
        set_state_box(buf, false);
    } else if (!fault) {
        set_state_box(info.name, p.state != HEATER_STATE_OFF);
    }
    set_phase(info.phase);

    if (p.mode == HEATER_MODE_LEVEL) {
        snprintf(buf, sizeof(buf), "%d", p.setting);
        set_text(ui.setting, buf);
        snprintf(buf, sizeof(buf), "LEVEL %d OF 6", p.setting);
        set_text(ui.mode, buf);
    } else if (p.mode == HEATER_MODE_THERMOSTAT) {
        snprintf(buf, sizeof(buf), "%d", p.setting);
        set_text(ui.setting, buf);
        snprintf(buf, sizeof(buf), "THERMOSTAT %d°C", p.setting);
        set_text(ui.mode, buf);
    } else {
        set_text(ui.setting, "-");
        set_text(ui.mode, p.mode == HEATER_MODE_OFF ? "STANDBY" : "MODE --");
    }

    snprintf(buf, sizeof(buf), "SUPPLY %ld V", lroundf(p.volts / 10.0f));
    set_text(ui.supply, buf);

    set_temperature(ui.case_temp, p.case_temp, 0, CASE_BAR_STEP);
    set_temperature(ui.cabin_temp, p.cabin_temp, CABIN_BAR_MIN, CABIN_BAR_STEP);

    if (p.mode == HEATER_MODE_IGNITION_FAILED) {
        set_status("> FAULT: IGNITION FAILED");
    } else if (fault) {
        snprintf(buf, sizeof(buf), "> HEATER FAULT (MODE %d)", p.mode);
        set_status(buf);
    } else {
        set_status("> HEATER LINK OK");
    }
    show_controls();
    show_run();
}
