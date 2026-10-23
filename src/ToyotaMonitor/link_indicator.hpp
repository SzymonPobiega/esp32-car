#pragma once

#include "lvgl.h"
#include <stdint.h>

// Small "● NAME" status light for a connector:
//   grey  - nothing received since boot
//   green - receiving, flashes bright on every packet
//   red   - nothing received for timeout_ms
// last_seen_ms is the millis() timestamp of the last packet (0 = never).
// Returns the indicator object so the caller can position it.
lv_obj_t *create_link_indicator(lv_obj_t *parent, const char *name,
                           const volatile uint32_t *last_seen_ms, uint32_t timeout_ms);
