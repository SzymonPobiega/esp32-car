#pragma once

#include "lvgl.h"
#include "ContractsInclude.hpp"

// millis() of the last packet from the OBD connector (0 = never)
extern volatile uint32_t obd_last_update;

// Transmission / boost readings from the OBD connector
void create_obd_tab(lv_obj_t *parent);

void obd_ui_update(const Contracts::ObdPacket &p);
