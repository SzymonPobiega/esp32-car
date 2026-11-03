#pragma once
#include "lvgl.h"
#include <stdint.h>

// LEVEL page: Fallout-style target views of the car, side (pitch) and
// front (roll), tilted by the gyro angles, with callouts and readouts.
// last_update is the millis() of the last gyro packet (0 = never); the
// readouts dim when it gets old.
void create_level_page(lv_obj_t *page, const volatile uint32_t *last_update);

// Angles in degrees, as sent by the gyro. Positive pitch tilts the side view
// clockwise (nose up), positive roll tilts the front view clockwise (left side low).
void attitude_update(float pitch_deg, float roll_deg);
