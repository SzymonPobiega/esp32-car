#pragma once

// Fallout 2 look: rusty metal frame, green CRT screen, amber counters, red lamps.
// Shared colours, fonts, styles and widget helpers for all the UI modules.

#include "lvgl.h"

// ---- Fonts (src/fonts, generated with lv_font_conv) ----
// VT323 subsets: 20/24 full ASCII; 32 upper case; 40 "LOCKED/OPEN";
// 60 digits and +-.%°; 120 digits only
LV_FONT_DECLARE(fo_vt323_20);
LV_FONT_DECLARE(fo_vt323_24);
LV_FONT_DECLARE(fo_vt323_32);
LV_FONT_DECLARE(fo_vt323_40);
LV_FONT_DECLARE(fo_vt323_60);
LV_FONT_DECLARE(fo_vt323_120);
// Stardos Stencil Bold, upper case and digits
LV_FONT_DECLARE(fo_stencil_14);
LV_FONT_DECLARE(fo_stencil_19);

// ---- Images (src/images) ----
LV_IMG_DECLARE(fo_car_side);
LV_IMG_DECLARE(fo_car_front);
LV_IMG_DECLARE(fo_metal_tile);
LV_IMG_DECLARE(fo_scanline_tile);

namespace fo {

// ---- Colours ----
inline lv_color_t hex(uint32_t c) { return lv_color_hex(c); }

static const uint32_t FRAME_DARK   = 0x241b12;  // screen background, gaps
static const uint32_t EDGE         = 0x17110b;  // black-brown outlines
static const uint32_t BRASS        = 0x6b5639;  // CRT bezel
static const uint32_t ENGRAVED     = 0xc9b38a;  // stencilled labels on metal
static const uint32_t ENGRAVED_HI  = 0xf0dcb0;  // active tab label
static const uint32_t CRT_BG       = 0x0a1708;  // CRT glass
static const uint32_t PHOSPHOR     = 0x5cf25c;  // bright CRT text
static const uint32_t PHOSPHOR_DIM = 0x45b545;  // secondary CRT text
static const uint32_t PHOSPHOR_OFF = 0x2a5a2a;  // stale / no data
static const uint32_t AMBER        = 0xf5b52a;  // counter digits
static const uint32_t AMBER_OFF    = 0x6a5020;  // stale counter digits

// ---- Layout (480x800 portrait) ----
static const lv_coord_t SCREEN_W  = 480;
static const lv_coord_t SCREEN_H  = 800;
static const lv_coord_t NAV_H     = 84;
static const lv_coord_t FOOTER_H  = 88;
static const lv_coord_t MAIN_H    = SCREEN_H - NAV_H - FOOTER_H;
static const lv_coord_t CRT_INSET = 18;
// Inner size of the CRT (its content area after the inset and padding)
static const lv_coord_t CRT_W     = SCREEN_W - 2 * CRT_INSET;
static const lv_coord_t CRT_H     = MAIN_H - 2 * CRT_INSET;

void init();

// Riveted brushed-metal panel at a fixed place on the screen
lv_obj_t *metal_panel(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h);

// Four rivets in the corners of a panel
void add_rivets(lv_obj_t *panel);

// CRT glass filling a panel (minus CRT_INSET); children use its content area
lv_obj_t *crt(lv_obj_t *panel);

// Plain container without theme styles, scrolling or clicks
lv_obj_t *box(lv_obj_t *parent);

// Label in the given font and colour
lv_obj_t *label(lv_obj_t *parent, const lv_font_t *font, uint32_t color, const char *text);

// Amber digit counter with a stencilled caption under it; returns the value label
lv_obj_t *counter(lv_obj_t *parent, const char *caption);

// Segment bar ("▮▮▮▯▯") made of `count` blocks
struct SegBar {
    lv_obj_t *root;
    lv_obj_t *segs[24];
    uint8_t count;
};
SegBar seg_bar(lv_obj_t *parent, uint8_t count, lv_coord_t height);
void seg_bar_set(SegBar &bar, int lit, uint32_t color);

// Phosphor-outlined button on the CRT; its label is child 0. Inverted while pressed (unless
// invert_on_press is false) and while checked, dimmed when disabled.
lv_obj_t *crt_button(lv_obj_t *parent, const char *text, lv_coord_t w, lv_coord_t h,
                     const lv_font_t *font, bool invert_on_press = true);

// Inverted text (dark on phosphor) marks a warning without a second colour
void set_inverted(lv_obj_t *label, bool on);

// "ATF PAN ........ 92°C" row in VT323 32 with a segment bar under it (none for 0 segs)
struct ReadingRow {
    lv_obj_t *head;
    lv_obj_t *name;
    lv_obj_t *value;
    lv_obj_t *leader;
    lv_point_t leader_pts[2];
    SegBar bar;
    lv_coord_t width;
    bool inverted;
    bool blink;
};
void reading_row(lv_obj_t *parent, ReadingRow &r, const char *name, uint8_t segs, lv_coord_t width);
// Sets the value; a blinking value alternates with inverted on each reading_row_blink
void reading_row_set(ReadingRow &r, const char *text, bool inverted, bool blink);
void reading_row_blink(ReadingRow &r, bool phase);

// Shared styles
extern lv_style_t style_metal;
extern lv_style_t style_counter;

}  // namespace fo
