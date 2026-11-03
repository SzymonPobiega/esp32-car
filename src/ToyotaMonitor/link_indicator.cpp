#include "link_indicator.hpp"
#include "fo_theme.hpp"
#include <Arduino.h>

static const uint32_t FLASH_MS = 300;
static const lv_coord_t LAMP_SIZE = 14;

enum LampState : uint8_t { LAMP_NEVER, LAMP_OK, LAMP_FLASH, LAMP_LOST, LAMP_UNSET };

struct LinkIndicator {
    lv_obj_t *lamp;
    const volatile uint32_t *last_seen_ms;
    uint32_t timeout_ms;
    LampState state;
};

// Indicator lamp: unlit, green, bright green flash, red
static void set_state(LinkIndicator *li, LampState state)
{
    if (li->state == state) return;   // restyling redraws the lamp, so only on change
    li->state = state;

    uint32_t color, glow;
    switch (state) {
    case LAMP_OK:    color = 0x3fd02a; glow = 0x5cff3a; break;
    case LAMP_FLASH: color = 0xc8ff9a; glow = 0x9cff7a; break;
    case LAMP_LOST:  color = 0xe0301e; glow = 0xff4a2a; break;
    default:         color = 0x4a3b2c; glow = 0;        break;
    }
    lv_obj_set_style_bg_color(li->lamp, fo::hex(color), 0);
    lv_obj_set_style_shadow_color(li->lamp, fo::hex(glow), 0);
    lv_obj_set_style_shadow_width(li->lamp, glow ? 10 : 0, 0);
}

static void link_indicator_timer_cb(lv_timer_t *t)
{
    LinkIndicator *li = (LinkIndicator *)t->user_data;
    uint32_t seen = *li->last_seen_ms;
    uint32_t now = millis();

    if (seen == 0) {
        set_state(li, LAMP_NEVER);
    } else if (now - seen > li->timeout_ms) {
        set_state(li, LAMP_LOST);
    } else if (now - seen < FLASH_MS) {
        set_state(li, LAMP_FLASH);
    } else {
        set_state(li, LAMP_OK);
    }
}

lv_obj_t *create_link_indicator(lv_obj_t *parent, const char *name,
                           const volatile uint32_t *last_seen_ms, uint32_t timeout_ms)
{
    LinkIndicator *li = new LinkIndicator{};
    li->last_seen_ms = last_seen_ms;
    li->timeout_ms = timeout_ms;
    li->state = LAMP_UNSET;

    lv_obj_t *cont = fo::box(parent);
    lv_obj_set_size(cont, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(cont, 6, 0);

    li->lamp = fo::box(cont);
    lv_obj_set_size(li->lamp, LAMP_SIZE, LAMP_SIZE);
    lv_obj_set_style_radius(li->lamp, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(li->lamp, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(li->lamp, fo::hex(0x1a130c), 0);
    lv_obj_set_style_border_width(li->lamp, 2, 0);
    lv_obj_set_style_shadow_spread(li->lamp, 1, 0);

    fo::label(cont, &fo_stencil_14, fo::ENGRAVED, name);

    set_state(li, LAMP_NEVER);
    lv_timer_create(link_indicator_timer_cb, 50, li);
    return cont;
}
