#pragma once

#include "lvgl.h"
#include "ContractsInclude.hpp"

// millis() of the last packet from the heater connector (0 = never)
extern volatile uint32_t heater_last_update;

// HCalory diesel heater status from the heater connector (read-only)
void create_heater_tab(lv_obj_t *parent);

void heater_ui_update(const Contracts::HeaterPacket &p);
