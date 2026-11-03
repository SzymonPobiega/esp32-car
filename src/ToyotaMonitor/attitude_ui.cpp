#include "attitude_ui.hpp"
#include "fo_theme.hpp"
#include <Arduino.h>
#include <math.h>

// Readouts dim when no gyro packet arrived for this long
static const uint32_t GYRO_STALE_MS = 10000;

// Images are tilted at most this far
static const float MAX_TILT_DEG = 45.0f;

// Below this the status line says LEVEL
static const float LEVEL_DEG = 0.5f;

static const lv_coord_t VIEW_W = 408;
static const lv_coord_t VIEW_H = 272;
static const lv_coord_t HORIZON_Y = 238;
static const lv_coord_t CALLOUT_Y = 118;   // height where the callout lines leave the screen edge

// One leader line from a screen edge to a point on the car, Fallout Tactics style:
// along the edge-side label, down, then across to the car
struct Callout {
    lv_obj_t *line;
    lv_point_t pts[4];
    lv_coord_t anchor_x;   // point on the car, image coordinates
    lv_coord_t anchor_y;
};

struct TargetView {
    lv_obj_t *car;
    lv_obj_t *value;
    lv_obj_t *status;
    Callout left;
    Callout right;
    lv_coord_t img_x, img_y;     // image position in the view
    const char *text_level;
    const char *text_positive;   // status for a positive angle
    const char *text_negative;
    float shown;
};

static TargetView pitch_view;
static TargetView roll_view;
static const volatile uint32_t *gyro_last_update = nullptr;
static bool fresh = false;

static void callout_init(Callout &c, lv_obj_t *parent, bool from_left,
                         lv_coord_t anchor_x, lv_coord_t anchor_y)
{
    c.anchor_x = anchor_x;
    c.anchor_y = anchor_y;
    lv_coord_t edge = from_left ? 0 : VIEW_W;
    lv_coord_t step = from_left ? 32 : VIEW_W - 32;
    c.pts[0] = { edge, CALLOUT_Y };
    c.pts[1] = { step, CALLOUT_Y };
    c.pts[2] = { step, CALLOUT_Y };
    c.pts[3] = { step, CALLOUT_Y };

    c.line = lv_line_create(parent);
    lv_obj_set_style_line_color(c.line, fo::hex(fo::PHOSPHOR), 0);
    lv_obj_set_style_line_width(c.line, 2, 0);
    lv_obj_set_pos(c.line, 0, 0);
}

// Re-aims a callout at its anchor after the image has been rotated
static void callout_aim(TargetView &v, Callout &c, float deg)
{
    const lv_img_dsc_t *img = (const lv_img_dsc_t *)lv_img_get_src(v.car);
    float px = img->header.w / 2.0f, py = img->header.h / 2.0f;
    float a = deg * (float)M_PI / 180.0f;
    float dx = c.anchor_x - px, dy = c.anchor_y - py;
    // lv_img angles are clockwise on screen (y grows downwards)
    lv_coord_t x = (lv_coord_t)lroundf(v.img_x + px + dx * cosf(a) - dy * sinf(a));
    lv_coord_t y = (lv_coord_t)lroundf(v.img_y + py + dx * sinf(a) + dy * cosf(a));

    c.pts[2].y = y;
    c.pts[3] = { x, y };
    lv_line_set_points(c.line, c.pts, 4);
}

static void create_view(TargetView &v, lv_obj_t *parent, const char *title, const lv_img_dsc_t *img,
                              const char *left_name, lv_coord_t left_x, lv_coord_t left_y,
                              const char *right_name, lv_coord_t right_x, lv_coord_t right_y)
{
    v.shown = NAN;

    lv_obj_t *view = fo::box(parent);   // clips the rotated image
    lv_obj_set_size(view, VIEW_W, VIEW_H);

    static const lv_coord_t dash_pts_len = 2;
    static lv_point_t horizon_pts[dash_pts_len] = { { 0, HORIZON_Y }, { VIEW_W, HORIZON_Y } };
    lv_obj_t *horizon = lv_line_create(view);
    lv_line_set_points(horizon, horizon_pts, dash_pts_len);
    lv_obj_set_style_line_color(horizon, fo::hex(fo::PHOSPHOR_DIM), 0);
    lv_obj_set_style_line_width(horizon, 2, 0);
    lv_obj_set_style_line_dash_width(horizon, 6, 0);
    lv_obj_set_style_line_dash_gap(horizon, 6, 0);

    v.img_x = (VIEW_W - img->header.w) / 2;
    v.img_y = HORIZON_Y - img->header.h;
    v.car = lv_img_create(view);
    lv_img_set_src(v.car, img);
    lv_obj_set_pos(v.car, v.img_x, v.img_y);
    lv_img_set_pivot(v.car, img->header.w / 2, img->header.h / 2);

    callout_init(v.left, view, true, left_x, left_y);
    callout_init(v.right, view, false, right_x, right_y);

    lv_obj_t *l = fo::label(view, &fo_vt323_24, fo::PHOSPHOR, left_name);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 4, CALLOUT_Y - 26);
    l = fo::label(view, &fo_vt323_24, fo::PHOSPHOR, right_name);
    lv_obj_align(l, LV_ALIGN_TOP_RIGHT, -4, CALLOUT_Y - 26);

    l = fo::label(view, &fo_vt323_32, fo::PHOSPHOR, title);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 4, 0);

    v.value = fo::label(view, &fo_vt323_60, fo::PHOSPHOR_OFF, "--");
    lv_obj_align(v.value, LV_ALIGN_TOP_RIGHT, -4, 0);

    v.status = fo::label(view, &fo_vt323_20, fo::PHOSPHOR_DIM, "NO GYRO DATA");
    lv_obj_align(v.status, LV_ALIGN_BOTTOM_LEFT, 4, -6);

    l = fo::label(view, &fo_vt323_20, fo::PHOSPHOR_DIM, "HORIZON");
    lv_obj_align(l, LV_ALIGN_BOTTOM_RIGHT, -4, -6);

    callout_aim(v, v.left, 0);
    callout_aim(v, v.right, 0);
}

static void view_set(TargetView &v, float deg)
{
    float d = deg > MAX_TILT_DEG ? MAX_TILT_DEG : deg < -MAX_TILT_DEG ? -MAX_TILT_DEG : deg;
    if (d == v.shown) return;
    v.shown = d;

    lv_img_set_angle(v.car, (int16_t)lroundf(d * 10.0f));
    callout_aim(v, v.left, d);
    callout_aim(v, v.right, d);

    char buf[16];
    snprintf(buf, sizeof(buf), "%+ld°", lroundf(deg));
    lv_label_set_text(v.value, buf);
    lv_label_set_text(v.status, fabsf(deg) < LEVEL_DEG ? v.text_level
                                : deg > 0 ? v.text_positive : v.text_negative);
}

static void set_fresh(bool f)
{
    fresh = f;
    uint32_t color = f ? fo::PHOSPHOR : fo::PHOSPHOR_OFF;
    lv_obj_set_style_text_color(pitch_view.value, fo::hex(color), 0);
    lv_obj_set_style_text_color(roll_view.value, fo::hex(color), 0);
}

static void stale_timer_cb(lv_timer_t *t)
{
    if (fresh && millis() - *gyro_last_update > GYRO_STALE_MS) set_fresh(false);
}

void create_level_page(lv_obj_t *page, const volatile uint32_t *last_update)
{
    gyro_last_update = last_update;

    lv_obj_set_flex_flow(page, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(page, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Anchors are in image pixels: front bumper and spare wheel on the side view
    // (car faces left), the outer tyre walls on the front view
    create_view(pitch_view, page, "PITCH", &fo_car_side,
                "NOSE", 6, 78, "TAIL", 290, 60);
    pitch_view.text_level = "LEVEL";
    pitch_view.text_positive = "NOSE UP";
    pitch_view.text_negative = "NOSE DOWN";

    // Seen from the front, the car's left side is on the right of the screen
    create_view(roll_view, page, "ROLL", &fo_car_front,
                "RIGHT", 10, 150, "LEFT", 195, 150);
    roll_view.text_level = "LEVEL";
    roll_view.text_positive = "LEFT SIDE LOW";
    roll_view.text_negative = "RIGHT SIDE LOW";

    lv_timer_create(stale_timer_cb, 1000, nullptr);
}

void attitude_update(float pitch_deg, float roll_deg)
{
    if (!fresh) set_fresh(true);
    view_set(pitch_view, pitch_deg);
    view_set(roll_view, roll_deg);
}
