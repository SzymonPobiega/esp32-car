#include "diag_ui.hpp"
#include "fo_theme.hpp"
#include "modem_link.hpp"
#include "tpms_data.hpp"
#include "bms_ui.hpp"
#include "obd_ui.hpp"
#include "heater_ui.hpp"
#include "sound.hpp"
#include <Arduino.h>
#include <string.h>

using namespace Contracts;

static const uint32_t TICK_MS = 50;           // finale lines follow the voice clips closely
static const int BLINK_TICKS = 5;             // cursor blinks every 250 ms
static const uint32_t MIN_STEP_MS = 400;      // each line stays pending at least this long
static const uint32_t PASS_DELAY_MS = 1000;   // "ALL SYSTEMS NOMINAL" stays up after its clip
static const uint32_t LINK_TIMEOUT_MS = 20000;
static const uint32_t OBD_TIMEOUT_MS = 30000;
static const uint32_t OBD_FRESH_MS = 15000;   // the OBD connector counts as connected
static const uint32_t DTC_FRESH_MS = 15000;   // the connector repeats its report every 5 s
static const uint32_t HEATER_TIMEOUT_MS = 30000;   // scanning and connecting take a while
static const int MAX_CODE_LINES = 3;          // per module, the page has room for ~20 lines

static const lv_coord_t ROW_W = fo::CRT_W - 16 - 2 * 14;   // page width minus padding

enum Result { STEP_PENDING, STEP_OK, STEP_WARN, STEP_NA, STEP_FAIL, STEP_SKIP };
static const char *const kResultText[] = { "_", "OK", "WARN", "N/A", "FAIL", "SKIP" };

struct Step {
    const char *name;
    const char *ok_text;   // shown instead of OK, e.g. ONLINE for links
    uint32_t timeout_ms;
    Result (*check)(bool timed_out);
    int module;   // fault code module whose codes are listed under the line, or -1
};

// After the real checks pass: lines that appear as their voice clip starts and go
// ONLINE as it ends, then ALL SYSTEMS NOMINAL with the last clip
struct FinaleLine {
    const char *name;
    const Sound *sound;
};
static const FinaleLine kFinale[] = {
    { "REACTOR", &snd_reactor_online },
    { "SENSORS", &snd_sensors_online },
};
static const int kFinaleCount = sizeof(kFinale) / sizeof(kFinale[0]);

static lv_obj_t *log_box;
static lv_obj_t *cursor;          // blinking "_" of the pending line or the last line
static lv_timer_t *timer;
static const volatile uint32_t *gyro_last = nullptr;
static int line_chars;            // VT323 is monospaced: characters per line
static int max_lines;             // lines that fit on the page

static int16_t obd_status = -1;
static uint32_t obd_status_ms = 0;
static DtcPacket dtc;
static uint32_t dtc_ms = 0;
static int16_t heater_link = HEATER_LINK_NONE;
static int16_t heater_state = HEATER_STATE_OFF;
static uint32_t heater_ms = 0;

static void (*pass_cb)() = nullptr;
static bool running = false;
static int step = 0;
static uint32_t step_start = 0;
static Result results[16];

static bool in_finale = false;
static uint32_t finale_start = 0;
static int finale_event = 0;

// ---- Checks ----

static bool alive(uint32_t last, uint32_t timeout)
{
    return last != 0 && millis() - last < timeout;
}

static Result wait_for(bool ok, bool timed_out)
{
    return ok ? STEP_OK : timed_out ? STEP_FAIL : STEP_PENDING;
}

static Result check_modem(bool t)  { return wait_for(modem::modemAlive(), t); }
static Result check_tpms(bool t)   { return wait_for(alive(tpms::last_update, 15000), t); }
static Result check_bms(bool t)    { return wait_for(alive(bms_last_update, 10000), t); }
static Result check_gyro(bool t)   { return wait_for(alive(*gyro_last, 10000), t); }
static Result check_obd(bool t)    { return wait_for(alive(obd_last_update, OBD_FRESH_MS), t); }
static Result check_heater_connector(bool t) { return wait_for(alive(heater_last_update, 10000), t); }

// A heater fault is a warning: it shouldn't keep the dashboard on this page
static Result check_heater(bool t)
{
    bool ok = alive(heater_ms, 10000) && heater_link == HEATER_LINK_OK;
    if (ok && heater_state == HEATER_STATE_ERROR) return STEP_WARN;
    return wait_for(ok, t);
}

static Result check_dongle(bool t)
{
    bool ok = alive(obd_status_ms, OBD_FRESH_MS) && obd_status >= OBD_STATUS_NO_ECU;
    return wait_for(ok, t);
}

static Result check_ecu(bool t)
{
    bool ok = alive(obd_status_ms, OBD_FRESH_MS) && obd_status == OBD_STATUS_OK;
    return wait_for(ok, t);
}

static Result check_scan(bool t)   { return wait_for(alive(dtc_ms, DTC_FRESH_MS), t); }

// Unverified modules (verified = false) never fail the run: no answer is N/A and
// codes are a warning
static Result module_result(int m, bool verified)
{
    if (dtc.stored[m] == DTC_NO_ANSWER) return verified ? STEP_FAIL : STEP_NA;
    if (dtc.stored[m] > 0) return verified ? STEP_FAIL : STEP_WARN;
    if (dtc.pending[m] > 0) return STEP_WARN;
    return STEP_OK;
}

static Result check_engine(bool)
{
    if (dtc.mil == 1) return STEP_FAIL;
    return module_result(DTC_ENGINE, true);
}

static Result check_gearbox(bool) { return module_result(DTC_GEARBOX, true); }
// The ABS request reads history codes too, and its decoding is unverified on the VDJ200
static Result check_abs(bool)     { return module_result(DTC_ABS, false); }

static const Step kSteps[] = {
    { "ESP-NOW MODEM",    "ONLINE", 5000,            check_modem,   -1 },
    { "TPMS CONNECTOR",   "ONLINE", LINK_TIMEOUT_MS, check_tpms,    -1 },
    { "BMS CONNECTOR",    "ONLINE", LINK_TIMEOUT_MS, check_bms,     -1 },
    { "GYRO CONNECTOR",   "ONLINE", LINK_TIMEOUT_MS, check_gyro,    -1 },
    { "HEATER CONNECTOR", "ONLINE", LINK_TIMEOUT_MS, check_heater_connector, -1 },
    { "DIESEL HEATER",    "ONLINE", HEATER_TIMEOUT_MS, check_heater, -1 },
    { "OBD CONNECTOR",    "ONLINE", LINK_TIMEOUT_MS, check_obd,     -1 },
    { "OBD DONGLE",       "ONLINE", OBD_TIMEOUT_MS,  check_dongle,  -1 },
    { "ECU LINK",         "ONLINE", OBD_TIMEOUT_MS,  check_ecu,     -1 },
    { "FAULT CODE SCAN",  "OK",     OBD_TIMEOUT_MS,  check_scan,    -1 },
    { "ENGINE CODES",     "OK",     0,               check_engine,  DTC_ENGINE },
    { "GEARBOX CODES",    "OK",     0,               check_gearbox, DTC_GEARBOX },
    { "ABS/VSC CODES",    "OK",     0,               check_abs,     DTC_ABS },
};
static const int kStepCount = sizeof(kSteps) / sizeof(kSteps[0]);

// A step is skipped when the step it depends on did not pass
static int depends_on(int i)
{
    switch (i) {
    case 5: return 4;            // heater <- heater connector
    case 7: return 6;            // dongle <- OBD connector
    case 8: return 7;            // ECU <- dongle
    case 9: return 8;            // scan <- ECU
    case 10: case 11: case 12: return 9;   // codes <- scan
    default: return -1;
    }
}

// ---- Log lines ----

// Like a console, the oldest lines scroll away when the page is full
static void make_room()
{
    while ((int)lv_obj_get_child_cnt(log_box) >= max_lines) {
        lv_obj_del(lv_obj_get_child(log_box, 0));
    }
}

static lv_obj_t *add_line(const char *text, uint32_t color)
{
    make_room();
    lv_obj_t *l = fo::label(log_box, &fo_vt323_24, color, text);
    lv_label_set_long_mode(l, LV_LABEL_LONG_CLIP);
    lv_obj_set_width(l, ROW_W);
    return l;
}

// "> NAME ........      OK": the result is right-aligned in its own label so the
// column lines up and FAIL can be inverted
static const int RESULT_CHARS = 8;   // " ONLINE" plus the inverted block's padding

static lv_obj_t *add_step_line(const char *name)
{
    make_room();
    lv_obj_t *row = fo::box(log_box);
    lv_obj_set_size(row, ROW_W, LV_SIZE_CONTENT);

    char buf[64];
    int len = snprintf(buf, sizeof(buf), "> %s ", name);
    int width = line_chars - RESULT_CHARS;
    while (len < width && len < (int)sizeof(buf) - 1) buf[len++] = '.';
    buf[len] = 0;
    lv_obj_t *text = fo::label(row, &fo_vt323_24, fo::PHOSPHOR, buf);
    lv_obj_align(text, LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_t *result = fo::label(row, &fo_vt323_24, fo::PHOSPHOR, "_");
    lv_obj_set_style_bg_color(result, fo::hex(fo::PHOSPHOR), 0);
    lv_obj_set_style_pad_hor(result, 3, 0);
    lv_obj_align(result, LV_ALIGN_RIGHT_MID, 0, 0);
    return result;
}

static void set_result(lv_obj_t *label, Result r, const char *ok_text = "OK")
{
    lv_label_set_text(label, r == STEP_OK ? ok_text : kResultText[r]);
    bool inverted = r == STEP_FAIL;
    lv_obj_set_style_bg_opa(label, inverted ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_color(label, fo::hex(inverted ? fo::CRT_BG : r == STEP_OK ? fo::PHOSPHOR : fo::PHOSPHOR_DIM), 0);
}

// Common 1VD-FTV / A750 / VSC codes; anything else is shown as the bare code
struct CodeName { uint16_t code; const char *name; };
static const CodeName kCodeNames[] = {
    { 0x0045, "TURBO BOOST CONTROL" },
    { 0x0087, "RAIL PRESSURE LOW" },
    { 0x0088, "RAIL PRESSURE HIGH" },
    { 0x0093, "FUEL SYSTEM LEAK" },
    { 0x0100, "MAF SENSOR" },
    { 0x0115, "COOLANT TEMP SENSOR" },
    { 0x0190, "RAIL PRESSURE SENSOR" },
    { 0x0234, "TURBO OVERBOOST" },
    { 0x0299, "TURBO UNDERBOOST" },
    { 0x0335, "CRANK SENSOR" },
    { 0x0340, "CAM SENSOR" },
    { 0x0400, "EGR FLOW" },
    { 0x0500, "SPEED SENSOR" },
    { 0x0705, "SHIFT RANGE SENSOR" },
    { 0x0710, "ATF TEMP SENSOR" },
    { 0x0750, "SHIFT SOLENOID A" },
    { 0x2563, "TURBO POSITION SENSOR" },
    { 0x4201, "ENGINE FAULT (VSC)" },     // C1201
    { 0x4241, "LOW SUPPLY VOLTAGE" },     // C1241
};

static void format_code(char *buf, size_t size, uint16_t code)
{
    static const char kLetters[] = "PCBU";
    snprintf(buf, size, "%c%d%03X", kLetters[code >> 14], (code >> 12) & 3, code & 0xFFF);
}

static void add_code_lines(int module)
{
    int shown = 0, more = 0;
    for (int i = 0; i < DTC_MAX_CODES; i++) {
        if (dtc.source[i] == DTC_UNUSED || (dtc.source[i] & 0x7F) != module) continue;
        if (shown == MAX_CODE_LINES) {
            more++;
            continue;
        }
        char code[8], buf[64];
        format_code(code, sizeof(code), dtc.codes[i]);
        const char *name = "";
        for (const CodeName &c : kCodeNames) {
            if (c.code == dtc.codes[i]) name = c.name;
        }
        snprintf(buf, sizeof(buf), "    %s %s%s", code, name,
                 dtc.source[i] & DTC_PENDING ? " (PENDING)" : "");
        add_line(buf, fo::PHOSPHOR_DIM);
        shown++;
    }
    if (more) {
        char buf[24];
        snprintf(buf, sizeof(buf), "    +%d MORE", more);
        add_line(buf, fo::PHOSPHOR_DIM);
    }
}

// ---- Run ----

static lv_obj_t *pending_result = nullptr;

// Last line with the blinking cursor after it
static void final_line(const char *text)
{
    make_room();
    lv_obj_t *row = fo::box(log_box);
    lv_obj_set_size(row, ROW_W, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    fo::label(row, &fo_vt323_24, fo::PHOSPHOR, text);
    cursor = fo::label(row, &fo_vt323_24, fo::PHOSPHOR, "_");
}

static void begin_step()
{
    step_start = millis();
    pending_result = add_step_line(kSteps[step].name);
    cursor = pending_result;
}

// ---- Finale ----
// Events: for each finale line, "add the line" at its clip's start and "ONLINE" at its
// end; then ALL SYSTEMS NOMINAL with the last clip, and the pass callback after it.

static void start_finale()
{
    static const Sound *sequence[kFinaleCount + 2];
    for (int i = 0; i < kFinaleCount; i++) sequence[i] = kFinale[i].sound;
    sequence[kFinaleCount] = &snd_all_systems_nominal;
    sequence[kFinaleCount + 1] = nullptr;
    sound::play(sequence);

    add_line("", fo::PHOSPHOR);
    in_finale = true;
    finale_start = millis();
    finale_event = 0;
}

// Time of a finale event since the finale started
static uint32_t finale_event_ms(int event)
{
    uint32_t t = 0;
    for (int i = 0; i < event / 2 && i < kFinaleCount; i++) {
        t += sound::duration_ms(*kFinale[i].sound) + sound::GAP_MS;
    }
    if (event < 2 * kFinaleCount && event % 2 == 1) {
        t += sound::duration_ms(*kFinale[event / 2].sound);
    }
    if (event == 2 * kFinaleCount + 1) {
        t += sound::duration_ms(snd_all_systems_nominal) + PASS_DELAY_MS;
    }
    return t;
}

static void finale_tick()
{
    while (finale_event <= 2 * kFinaleCount + 1 && millis() - finale_start >= finale_event_ms(finale_event)) {
        int e = finale_event++;
        if (e < 2 * kFinaleCount) {
            if (e % 2 == 0) {
                pending_result = add_step_line(kFinale[e / 2].name);
                cursor = pending_result;
            } else {
                lv_obj_clear_flag(pending_result, LV_OBJ_FLAG_HIDDEN);
                set_result(pending_result, STEP_OK, "ONLINE");
            }
        } else if (e == 2 * kFinaleCount) {
            add_line("", fo::PHOSPHOR);
            final_line("> ALL SYSTEMS NOMINAL");
        } else {
            in_finale = false;
            running = false;
            if (pass_cb) pass_cb();
            return;
        }
    }
}

static void diag_timer_cb(lv_timer_t *)
{
    // Blink the cursor
    static int ticks = 0;
    if (++ticks >= BLINK_TICKS) {
        ticks = 0;
        static bool phase = false;
        phase = !phase;
        if (cursor) {
            if (phase) lv_obj_add_flag(cursor, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_clear_flag(cursor, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (!running) return;

    if (in_finale) {
        finale_tick();
        return;
    }

    uint32_t elapsed = millis() - step_start;
    if (elapsed < MIN_STEP_MS) return;

    const Step &s = kSteps[step];
    int dep = depends_on(step);
    Result r;
    if (dep >= 0 && results[dep] != STEP_OK && results[dep] != STEP_WARN) {
        r = STEP_SKIP;
    } else {
        r = s.check(elapsed >= s.timeout_ms);
        if (r == STEP_PENDING) return;
    }

    results[step] = r;
    lv_obj_clear_flag(pending_result, LV_OBJ_FLAG_HIDDEN);
    set_result(pending_result, r, s.ok_text);
    if (s.module >= 0 && r != STEP_SKIP) add_code_lines(s.module);

    if (++step < kStepCount) {
        begin_step();
        return;
    }

    bool passed = true;
    for (int i = 0; i < kStepCount; i++) {
        if (results[i] == STEP_FAIL || results[i] == STEP_SKIP) passed = false;
    }
    if (passed) {
        start_finale();
    } else {
        add_line("", fo::PHOSPHOR);
        final_line("> CHECK FAILED. TAP COG TO RETRY");
        running = false;
    }
}

void create_diag_page(lv_obj_t *page, const volatile uint32_t *gyro_last_update)
{
    gyro_last = gyro_last_update;
    line_chars = ROW_W / lv_font_get_glyph_width(&fo_vt323_24, '.', 0);
    lv_obj_update_layout(page);
    max_lines = lv_obj_get_content_height(page) / lv_font_get_line_height(&fo_vt323_24);
    if (max_lines < 10) max_lines = 22;

    log_box = fo::box(page);
    lv_obj_set_size(log_box, lv_pct(100), lv_pct(100));
    lv_obj_set_flex_flow(log_box, LV_FLEX_FLOW_COLUMN);

    timer = lv_timer_create(diag_timer_cb, TICK_MS, nullptr);
}

void diag_start(void (*on_pass)())
{
    pass_cb = on_pass;
    lv_obj_clean(log_box);
    cursor = nullptr;
    add_line("> LC200 SYSTEM DIAGNOSTICS", fo::PHOSPHOR_DIM);
    add_line("", fo::PHOSPHOR);

    memset(results, 0, sizeof(results));
    step = 0;
    in_finale = false;
    running = true;
    begin_step();
}

void diag_stop()
{
    if (!running) return;
    running = false;
    // Clear the cursor of a line that was still waiting
    if (pending_result && strcmp(lv_label_get_text(pending_result), "_") == 0) {
        lv_obj_clear_flag(pending_result, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(pending_result, "");
    }
    in_finale = false;
    add_line("", fo::PHOSPHOR);
    add_line("> INTERRUPTED. TAP COG TO RUN", fo::PHOSPHOR);
    cursor = nullptr;
}

bool diag_running()
{
    return running;
}

void diag_obd_update(const ObdPacket &p)
{
    obd_status = p.status;
    obd_status_ms = millis();
}

void diag_dtc_update(const DtcPacket &p)
{
    dtc = p;
    dtc_ms = millis();
}

void diag_heater_update(const HeaterPacket &p)
{
    heater_link = p.link;
    heater_state = p.state;
    heater_ms = millis();
}
