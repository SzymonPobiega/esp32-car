#include "link_indicator.hpp"
#include <Arduino.h>

static const uint32_t FLASH_MS = 300;
static const lv_coord_t DOT_SIZE = 14;

static const lv_color_t COLOR_NEVER = lv_color_make(110, 110, 110);
static const lv_color_t COLOR_OK    = lv_color_make(0, 140, 40);
static const lv_color_t COLOR_FLASH = lv_color_make(80, 255, 120);
static const lv_color_t COLOR_LOST  = lv_color_make(220, 40, 40);

struct LinkIndicator {
    lv_obj_t *dot;
    lv_obj_t *label;
    const volatile uint32_t *last_seen_ms;
    uint32_t timeout_ms;
};

static void set_color(LinkIndicator *li, lv_color_t color)
{
    lv_obj_set_style_bg_color(li->dot, color, 0);
    lv_obj_set_style_text_color(li->label, color, 0);
}

static void link_indicator_timer_cb(lv_timer_t *t)
{
    LinkIndicator *li = (LinkIndicator *)t->user_data;
    uint32_t seen = *li->last_seen_ms;
    uint32_t now = millis();

    if (seen == 0) {
        set_color(li, COLOR_NEVER);
    } else if (now - seen > li->timeout_ms) {
        set_color(li, COLOR_LOST);
    } else if (now - seen < FLASH_MS) {
        set_color(li, COLOR_FLASH);
    } else {
        set_color(li, COLOR_OK);
    }
}

lv_obj_t *create_link_indicator(lv_obj_t *parent, const char *name,
                           const volatile uint32_t *last_seen_ms, uint32_t timeout_ms)
{
    LinkIndicator *li = new LinkIndicator{};
    li->last_seen_ms = last_seen_ms;
    li->timeout_ms = timeout_ms;

    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_remove_style_all(cont);
    lv_obj_set_size(cont, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(cont, 6, 0);

    li->dot = lv_obj_create(cont);
    lv_obj_remove_style_all(li->dot);
    lv_obj_set_size(li->dot, DOT_SIZE, DOT_SIZE);
    lv_obj_set_style_radius(li->dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(li->dot, LV_OPA_COVER, 0);

    li->label = lv_label_create(cont);
    lv_obj_set_style_text_font(li->label, &lv_font_montserrat_14, 0);
    lv_label_set_text(li->label, name);

    set_color(li, COLOR_NEVER);
    lv_timer_create(link_indicator_timer_cb, 50, li);
    return cont;
}
