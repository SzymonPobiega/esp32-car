#pragma once

#include "lvgl.h"
#include "ContractsInclude.hpp"

// millis() of the last BMS packet (0 = never)
extern volatile uint32_t bms_last_update;

// Status bar with battery (BMS) values, shown below the tabs.
// Returns the bar so other status items can be added to it.
lv_obj_t *create_bms_bar(lv_obj_t *parent, int width, int height);

void bms_bar_update(const Contracts::BmsPacket &b);
