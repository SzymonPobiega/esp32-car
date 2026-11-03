#pragma once

#include "lvgl.h"
#include "ContractsInclude.hpp"

// Diagnostics page: a console log that checks the modem, every connector, the OBD
// dongle and ECU, and the fault codes the OBD connector reports.
void create_diag_page(lv_obj_t *page, const volatile uint32_t *gyro_last_update);

// Runs (or restarts) the checks; on_pass is called once everything passed
void diag_start(void (*on_pass)());

// Stops a run, e.g. when another tab is selected
void diag_stop();

bool diag_running();

void diag_obd_update(const Contracts::ObdPacket &p);
void diag_dtc_update(const Contracts::DtcPacket &p);
void diag_heater_update(const Contracts::HeaterPacket &p);
