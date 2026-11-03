#pragma once
#include <lvgl.h>

// TYRES page: one CRT dial per tyre (1-3 bar) with pressure, temperature and
// the seconds since the last reading. Gauge order follows Vehicle::TPMS_SENSORS.
void create_tyres_page(lv_obj_t *page);

// Starts the timer that refreshes the dials from tpms:: data
void init_gauge_timer();
