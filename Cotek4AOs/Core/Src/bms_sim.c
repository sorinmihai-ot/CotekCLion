//
// Created by sorin.mihai on 16/12/2025.
//
#include "qpc.h"
#include "bms_app.h"
#include <string.h>

#ifdef ENABLE_BMS_SIM

/* Simple, plausible “fake battery” snapshot */
void BmsSim_tick(void) {
    BmsTelemetry t;
    memset(&t, 0, sizeof(t));

    t.battery_type_code   = 0x0600;   // e.g. 600s
    t.array_voltage_V     = 52.0f;
    t.high_cell_V         = 3.60f;
    t.low_cell_V          = 3.50f;
    t.soc_percent         = 75;
    t.bms_state           = 62;       // "Ready" in your mapper
    t.bms_fault           = 0;        // no fault

    /* Publish like real BMS does */
    BMS_publish_telemetry(&t);
}

#endif