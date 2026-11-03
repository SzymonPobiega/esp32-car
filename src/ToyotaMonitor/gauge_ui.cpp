#include "gauge_ui.hpp"
#include "fo_theme.hpp"
#include "tpms_data.hpp"
#include <Arduino.h>   // for millis()
#include <string.h>

static const int kGauges = 4;
static const char *const kNames[kGauges] = { "FRONT LEFT", "FRONT RIGHT", "REAR LEFT", "REAR RIGHT" };

// Dial range and the hatched low-pressure zone, Pa
static const int32_t P_MIN = 100000;
static const int32_t P_MAX = 300000;
static const int32_t P_LOW = 180000;

static const lv_coord_t DIAL_W = 180;   // the dial is the top half of a square meter

// Lifts "BAR" so it sits on the pressure digits' baseline
static const lv_coord_t BAR_NUDGE_Y = 8;

struct GaugeUI {
    lv_obj_t *meter;
    lv_meter_indicator_t *needle;
    lv_obj_t *label_pressure;
    lv_obj_t *label_temp;
    lv_obj_t *label_age;
    int32_t shown_pressure;
};

static GaugeUI gauges[kGauges];

// lv_label_set_text always redraws, so skip unchanged text
static void set_text(lv_obj_t *label, const char *text)
{
    if (strcmp(lv_label_get_text(label), text) != 0) lv_label_set_text(label, text);
}

static GaugeUI create_gauge(lv_obj_t *parent, const char *name)
{
    GaugeUI g{};
    g.shown_pressure = -1;

    lv_obj_t *cell = fo::box(parent);
    lv_obj_set_style_border_color(cell, fo::hex(fo::PHOSPHOR), 0);
    lv_obj_set_style_border_opa(cell, LV_OPA_30, 0);
    lv_obj_set_style_border_width(cell, 1, 0);
    lv_obj_set_style_pad_all(cell, 6, 0);
    lv_obj_set_flex_flow(cell, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cell, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(cell, 2, 0);

    fo::label(cell, &fo_vt323_24, fo::PHOSPHOR, name);

    // Clip the square meter to its upper half
    lv_obj_t *dial = fo::box(cell);
    lv_obj_set_size(dial, DIAL_W, DIAL_W / 2 + 6);

    g.meter = lv_meter_create(dial);
    lv_obj_remove_style_all(g.meter);
    lv_obj_clear_flag(g.meter, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(g.meter, DIAL_W, DIAL_W);
    lv_obj_set_pos(g.meter, 0, 0);
    lv_obj_set_style_line_color(g.meter, fo::hex(fo::PHOSPHOR), LV_PART_ITEMS);
    lv_obj_set_style_text_opa(g.meter, LV_OPA_TRANSP, LV_PART_TICKS);
    // Needle hub
    lv_obj_set_style_bg_color(g.meter, fo::hex(fo::PHOSPHOR), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(g.meter, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(g.meter, LV_RADIUS_CIRCLE, LV_PART_INDICATOR);
    lv_obj_set_style_width(g.meter, 12, LV_PART_INDICATOR);
    lv_obj_set_style_height(g.meter, 12, LV_PART_INDICATOR);

    lv_meter_scale_t *scale = lv_meter_add_scale(g.meter);
    lv_meter_set_scale_range(g.meter, scale, P_MIN, P_MAX, 180, 180);
    lv_meter_set_scale_ticks(g.meter, scale, 9, 2, 7, fo::hex(fo::PHOSPHOR));
    lv_meter_set_scale_major_ticks(g.meter, scale, 2, 3, 13, fo::hex(fo::PHOSPHOR), 0);

    lv_meter_indicator_t *rim = lv_meter_add_arc(g.meter, scale, 2, fo::hex(fo::PHOSPHOR), 0);
    lv_meter_set_indicator_start_value(g.meter, rim, P_MIN);
    lv_meter_set_indicator_end_value(g.meter, rim, P_MAX);

    lv_meter_indicator_t *low = lv_meter_add_arc(g.meter, scale, 7, fo::hex(fo::PHOSPHOR_OFF), -3);
    lv_meter_set_indicator_start_value(g.meter, low, P_MIN);
    lv_meter_set_indicator_end_value(g.meter, low, P_LOW);

    g.needle = lv_meter_add_needle_line(g.meter, scale, 3, fo::hex(fo::PHOSPHOR), -12);
    lv_meter_set_indicator_value(g.meter, g.needle, P_MIN);

    // Pressure with its unit beside it, temperature large enough to read while driving
    lv_obj_t *reading = fo::box(cell);
    lv_obj_set_size(reading, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(reading, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(reading, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_END);
    lv_obj_set_style_pad_column(reading, 6, 0);
    g.label_pressure = fo::label(reading, &fo_vt323_60, fo::PHOSPHOR_OFF, "--");
    lv_obj_t *unit = fo::label(reading, &fo_vt323_24, fo::PHOSPHOR_DIM, "BAR");
    lv_obj_set_style_pad_bottom(unit, BAR_NUDGE_Y, 0);
    g.label_temp = fo::label(cell, &fo_vt323_32, fo::PHOSPHOR_OFF, "--°C");
    g.label_age = fo::label(cell, &fo_vt323_20, fo::PHOSPHOR_DIM, "NO SIGNAL");
    return g;
}

void create_tyres_page(lv_obj_t *page)
{
    static lv_coord_t col_dsc[] = { LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };
    static lv_coord_t row_dsc[] = { LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };
    lv_obj_set_grid_dsc_array(page, col_dsc, row_dsc);
    lv_obj_set_style_pad_row(page, 12, 0);
    lv_obj_set_style_pad_column(page, 12, 0);

    for (int i = 0; i < kGauges; i++) {
        gauges[i] = create_gauge(page, kNames[i]);
        lv_obj_t *cell = lv_obj_get_parent(lv_obj_get_parent(gauges[i].meter));
        lv_obj_set_grid_cell(cell, LV_GRID_ALIGN_STRETCH, i % 2, 1, LV_GRID_ALIGN_STRETCH, i / 2, 1);
    }
}

static void gauge_timer_cb(lv_timer_t *t)
{
    using namespace tpms;
    uint32_t now = millis();
    char buf[32];

    for (int gi = 0; gi < kGauges; gi++) {
        GaugeUI &g = gauges[gi];

        if (lastUpdated[gi] == 0) {
            set_text(g.label_age, "NO SIGNAL");
            continue;
        }

        int32_t p = latestPressure[gi];
        if (p != g.shown_pressure) {
            if (g.shown_pressure < 0) {
                lv_obj_set_style_text_color(g.label_pressure, fo::hex(fo::PHOSPHOR), 0);
                lv_obj_set_style_text_color(g.label_temp, fo::hex(fo::PHOSPHOR), 0);
            }
            g.shown_pressure = p;
            int32_t needle = p < P_MIN ? P_MIN : p > P_MAX ? P_MAX : p;
            lv_meter_set_indicator_value(g.meter, g.needle, needle);
            snprintf(buf, sizeof(buf), "%.2f", p / 100000.0f);
            set_text(g.label_pressure, buf);
        }

        snprintf(buf, sizeof(buf), "%.1f°C", latestTemp[gi] / 100.0f);
        set_text(g.label_temp, buf);

        snprintf(buf, sizeof(buf), "LAST %lus", (unsigned long)((now - lastUpdated[gi]) / 1000));
        set_text(g.label_age, buf);
    }
}

void init_gauge_timer()
{
    lv_timer_create(gauge_timer_cb, 100, nullptr);
}
