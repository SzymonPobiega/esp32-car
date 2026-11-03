#include "fo_theme.hpp"

namespace fo {

lv_style_t style_metal;
lv_style_t style_counter;
static lv_style_t style_rivet;
static lv_style_t style_seg;
static lv_style_t style_btn, style_btn_on, style_btn_off;

void init()
{
    lv_style_init(&style_metal);
    lv_style_set_bg_color(&style_metal, hex(0x45372a));
    lv_style_set_bg_opa(&style_metal, LV_OPA_COVER);
    lv_style_set_bg_img_src(&style_metal, &fo_metal_tile);
    lv_style_set_bg_img_tiled(&style_metal, true);
    lv_style_set_border_side(&style_metal, LV_BORDER_SIDE_TOP);
    lv_style_set_border_color(&style_metal, hex(0x6e5a42));
    lv_style_set_border_width(&style_metal, 2);

    lv_style_init(&style_rivet);
    lv_style_set_size(&style_rivet, 10);
    lv_style_set_radius(&style_rivet, LV_RADIUS_CIRCLE);
    lv_style_set_bg_opa(&style_rivet, LV_OPA_COVER);
    lv_style_set_bg_color(&style_rivet, hex(0xa68d6a));
    lv_style_set_bg_grad_color(&style_rivet, hex(0x3a2d20));
    lv_style_set_bg_grad_dir(&style_rivet, LV_GRAD_DIR_VER);
    lv_style_set_border_color(&style_rivet, hex(0x120d08));
    lv_style_set_border_width(&style_rivet, 1);

    lv_style_init(&style_counter);
    lv_style_set_bg_color(&style_counter, hex(0x0c0906));
    lv_style_set_bg_opa(&style_counter, LV_OPA_COVER);
    lv_style_set_border_color(&style_counter, hex(0x1a130c));
    lv_style_set_border_width(&style_counter, 2);
    lv_style_set_radius(&style_counter, 4);
    lv_style_set_text_font(&style_counter, &fo_vt323_32);
    lv_style_set_text_color(&style_counter, hex(AMBER));
    lv_style_set_text_align(&style_counter, LV_TEXT_ALIGN_CENTER);

    lv_style_init(&style_seg);
    lv_style_set_bg_opa(&style_seg, LV_OPA_COVER);
    lv_style_set_flex_grow(&style_seg, 1);

    lv_style_init(&style_btn);
    lv_style_set_bg_color(&style_btn, hex(PHOSPHOR));
    lv_style_set_bg_opa(&style_btn, LV_OPA_TRANSP);
    lv_style_set_border_color(&style_btn, hex(PHOSPHOR));
    lv_style_set_border_width(&style_btn, 2);
    lv_style_set_text_color(&style_btn, hex(PHOSPHOR));

    lv_style_init(&style_btn_on);
    lv_style_set_bg_opa(&style_btn_on, LV_OPA_COVER);
    lv_style_set_text_color(&style_btn_on, hex(CRT_BG));

    lv_style_init(&style_btn_off);
    lv_style_set_border_color(&style_btn_off, hex(PHOSPHOR_OFF));
    lv_style_set_text_color(&style_btn_off, hex(PHOSPHOR_OFF));
    lv_style_set_bg_opa(&style_btn_off, LV_OPA_TRANSP);
}

lv_obj_t *box(lv_obj_t *parent)
{
    lv_obj_t *o = lv_obj_create(parent);
    lv_obj_remove_style_all(o);
    lv_obj_clear_flag(o, (lv_obj_flag_t)(LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE));
    return o;
}

lv_obj_t *label(lv_obj_t *parent, const lv_font_t *font, uint32_t color, const char *text)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, hex(color), 0);
    lv_label_set_text(l, text);
    return l;
}

lv_obj_t *crt_button(lv_obj_t *parent, const char *text, lv_coord_t w, lv_coord_t h,
                     const lv_font_t *font, bool invert_on_press)
{
    lv_obj_t *b = lv_obj_create(parent);
    lv_obj_remove_style_all(b);
    lv_obj_clear_flag(b, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(b, w, h);
    lv_obj_add_style(b, &style_btn, 0);
    lv_obj_add_style(b, &style_btn_on, LV_STATE_CHECKED);
    if (invert_on_press) lv_obj_add_style(b, &style_btn_on, LV_STATE_PRESSED);
    lv_obj_add_style(b, &style_btn_off, LV_STATE_DISABLED);

    lv_obj_t *l = lv_label_create(b);
    lv_obj_set_style_text_font(l, font, 0);
    lv_label_set_text(l, text);
    lv_obj_center(l);
    return b;
}

lv_obj_t *metal_panel(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h)
{
    lv_obj_t *p = box(parent);
    lv_obj_add_style(p, &style_metal, 0);
    lv_obj_set_pos(p, x, y);
    lv_obj_set_size(p, w, h);
    return p;
}

void add_rivets(lv_obj_t *panel)
{
    static const lv_align_t corners[] = {
        LV_ALIGN_TOP_LEFT, LV_ALIGN_TOP_RIGHT, LV_ALIGN_BOTTOM_LEFT, LV_ALIGN_BOTTOM_RIGHT
    };
    for (lv_align_t a : corners) {
        lv_obj_t *r = box(panel);
        lv_obj_add_style(r, &style_rivet, 0);
        lv_coord_t dx = (a == LV_ALIGN_TOP_LEFT || a == LV_ALIGN_BOTTOM_LEFT) ? 4 : -4;
        lv_coord_t dy = (a == LV_ALIGN_TOP_LEFT || a == LV_ALIGN_TOP_RIGHT) ? 4 : -4;
        lv_obj_align(r, a, dx, dy);
    }
}

lv_obj_t *crt(lv_obj_t *panel)
{
    lv_obj_t *glass = box(panel);
    lv_obj_set_size(glass, CRT_W, CRT_H);
    lv_obj_align(glass, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(glass, hex(CRT_BG), 0);
    lv_obj_set_style_bg_opa(glass, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(glass, 22, 0);
    lv_obj_set_style_border_color(glass, hex(EDGE), 0);
    lv_obj_set_style_border_width(glass, 4, 0);
    lv_obj_set_style_outline_color(glass, hex(BRASS), 0);
    lv_obj_set_style_outline_width(glass, 3, 0);

    // Scanlines on an inner layer: background images ignore the corner radius
    lv_obj_t *scan = box(glass);
    lv_obj_set_size(scan, CRT_W - 16, CRT_H - 16);
    lv_obj_center(scan);
    lv_obj_set_style_bg_img_src(scan, &fo_scanline_tile, 0);
    lv_obj_set_style_bg_img_tiled(scan, true, 0);
    return scan;
}

lv_obj_t *counter(lv_obj_t *parent, const char *caption)
{
    lv_obj_t *col = box(parent);
    lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(col, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(col, 3, 0);
    lv_obj_set_flex_grow(col, 1);
    lv_obj_set_height(col, LV_SIZE_CONTENT);

    lv_obj_t *window = box(col);
    lv_obj_add_style(window, &style_counter, 0);
    lv_obj_set_size(window, lv_pct(100), 42);

    lv_obj_t *value = lv_label_create(window);
    lv_label_set_text(value, "--");
    lv_obj_center(value);

    label(col, &fo_stencil_14, ENGRAVED, caption);
    return value;
}

SegBar seg_bar(lv_obj_t *parent, uint8_t count, lv_coord_t height)
{
    SegBar bar{};
    bar.count = count > 24 ? 24 : count;
    bar.root = box(parent);
    lv_obj_set_size(bar.root, lv_pct(100), height);
    lv_obj_set_flex_flow(bar.root, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(bar.root, 3, 0);
    for (uint8_t i = 0; i < bar.count; i++) {
        bar.segs[i] = box(bar.root);
        lv_obj_add_style(bar.segs[i], &style_seg, 0);
        lv_obj_set_height(bar.segs[i], height);
    }
    seg_bar_set(bar, 0, PHOSPHOR);
    return bar;
}

void seg_bar_set(SegBar &bar, int lit, uint32_t color)
{
    for (uint8_t i = 0; i < bar.count; i++) {
        lv_obj_set_style_bg_color(bar.segs[i], hex(color), 0);
        lv_obj_set_style_bg_opa(bar.segs[i], i < lit ? LV_OPA_COVER : LV_OPA_20, 0);
    }
}

void set_inverted(lv_obj_t *label, bool on)
{
    lv_obj_set_style_bg_opa(label, on ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_color(label, hex(on ? CRT_BG : PHOSPHOR), 0);
}

// Dotted leader between the name and the value
static void row_fit_leader(ReadingRow &r)
{
    lv_point_t name_sz, value_sz;
    lv_txt_get_size(&name_sz, lv_label_get_text(r.name), &fo_vt323_32, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
    lv_txt_get_size(&value_sz, lv_label_get_text(r.value), &fo_vt323_32, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
    lv_coord_t x1 = name_sz.x + 10;
    lv_coord_t x2 = r.width - value_sz.x - 12 - 10;
    if (x2 < x1) x2 = x1;
    r.leader_pts[0] = { x1, 26 };
    r.leader_pts[1] = { x2, 26 };
    lv_line_set_points(r.leader, r.leader_pts, 2);
}

void reading_row(lv_obj_t *parent, ReadingRow &r, const char *name, uint8_t segs, lv_coord_t width)
{
    r.width = width;
    lv_obj_t *col = box(parent);
    lv_obj_set_size(col, width, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(col, 6, 0);

    r.head = box(col);
    lv_obj_set_size(r.head, width, 32);

    r.leader = lv_line_create(r.head);
    lv_obj_set_style_line_color(r.leader, hex(PHOSPHOR_DIM), 0);
    lv_obj_set_style_line_width(r.leader, 2, 0);
    lv_obj_set_style_line_dash_width(r.leader, 2, 0);
    lv_obj_set_style_line_dash_gap(r.leader, 4, 0);

    r.name = label(r.head, &fo_vt323_32, PHOSPHOR, name);
    lv_obj_align(r.name, LV_ALIGN_LEFT_MID, 0, 0);

    r.value = label(r.head, &fo_vt323_32, PHOSPHOR, "--");
    lv_obj_set_style_bg_color(r.value, hex(PHOSPHOR), 0);
    lv_obj_set_style_pad_hor(r.value, 6, 0);
    lv_obj_align(r.value, LV_ALIGN_RIGHT_MID, 0, 0);
    r.inverted = false;
    r.blink = false;
    row_fit_leader(r);

    r.bar = segs ? seg_bar(col, segs, 22) : SegBar{};
}

void reading_row_set(ReadingRow &r, const char *text, bool inverted, bool blink)
{
    if (strcmp(lv_label_get_text(r.value), text) != 0) {
        lv_label_set_text(r.value, text);
        row_fit_leader(r);
    }
    // Also restore the steady state when a blink ends half way
    if (inverted != r.inverted || r.blink) {
        r.inverted = inverted;
        set_inverted(r.value, inverted);
    }
    r.blink = blink;
}

void reading_row_blink(ReadingRow &r, bool phase)
{
    if (r.blink) set_inverted(r.value, phase);
}

}  // namespace fo
