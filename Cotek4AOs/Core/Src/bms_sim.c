//
// Created by sorin.mihai on 16/12/2025.
//
#include "qpc.h"
#include "bms_app.h"
#include <string.h>
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include "app_signals.h"


#ifdef ENABLE_BMS_SIM



// ------------------------------------------------------------
// SIM CONFIG (edit only these macros, then flash)
// ------------------------------------------------------------

// 0 = normal telemetry forever
// 1 = low cell undervoltage
// 2 = high temperature
// 3 = critical BMS error (last_error_class/code)
// 4 = comms lost (no telemetry + BMS_CONN_LOST_SIG once)
// 5 = pack voltage high (ONLY useful if Controller checks pack>PSU using Cotek reading)
#ifndef BMS_SIM_INJECT
#define BMS_SIM_INJECT  1
#endif

// Delay before fault injection starts
#ifndef BMS_SIM_INJECT_AFTER_MS
#define BMS_SIM_INJECT_AFTER_MS  5000U
#endif

// How long to keep the injected fault active (except comms lost behaviour)
#ifndef BMS_SIM_INJECT_WINDOW_MS
#define BMS_SIM_INJECT_WINDOW_MS  2000U
#endif

// How often BmsSim_tick() is called (you arm SIM_TICK at 500ms currently)
// Used only for optional print throttling.
#ifndef BMS_SIM_TICK_MS
#define BMS_SIM_TICK_MS  500U
#endif

// Nominal values
#ifndef BMS_SIM_NOM_PACK_V
#define BMS_SIM_NOM_PACK_V  47.0f
#endif
#ifndef BMS_SIM_NOM_HCELL_V
#define BMS_SIM_NOM_HCELL_V 3.60f
#endif
#ifndef BMS_SIM_NOM_LCELL_V
#define BMS_SIM_NOM_LCELL_V 3.50f
#endif
#ifndef BMS_SIM_NOM_TEMP_C
#define BMS_SIM_NOM_TEMP_C  25.0f
#endif

// Fault values
#ifndef BMS_SIM_FAULT_LCELL_UV_V
#define BMS_SIM_FAULT_LCELL_UV_V  2.20f
#endif
#ifndef BMS_SIM_FAULT_TEMP_C
#define BMS_SIM_FAULT_TEMP_C      52.0f
#endif
#ifndef BMS_SIM_FAULT_PACK_V
#define BMS_SIM_FAULT_PACK_V      52.0f
#endif

#ifdef ENABLE_BMS_SIM
static uint8_t  s_sim_charging = 0U;
static uint32_t s_charge_t0_ms = 0U;

void BmsSim_setCharging(uint8_t on) {
    if (on && !s_sim_charging) {
        s_sim_charging = 1U;
        s_charge_t0_ms = HAL_GetTick();   // start the “charging phase” timeline now
    } else if (!on && s_sim_charging) {
        s_sim_charging = 0U;
        s_charge_t0_ms = 0U;
    }
}
#endif


// ------------------------------------------------------------

static void fill_nominal(BmsTelemetry *t) {
    memset(t, 0, sizeof(*t));

    t->battery_type_code   = 0x0600;   // 600s
    t->array_voltage_V     = BMS_SIM_NOM_PACK_V;
    t->high_cell_V         = BMS_SIM_NOM_HCELL_V;
    t->low_cell_V          = BMS_SIM_NOM_LCELL_V;
    t->soc_percent         = 75;
    t->bms_state           = 62;       // "Ready" in your mapper
    t->bms_fault           = 0;
    t->bms_fault_raw       = 0;
    t->last_error_class    = 0;
    t->last_error_code     = 0;
    t->sys_temp_high_C     = BMS_SIM_NOM_TEMP_C;
    t->sys_temp_low_C      = BMS_SIM_NOM_TEMP_C - 3.0f;
}

static void apply_injection(BmsTelemetry *t) {
#if (BMS_SIM_INJECT == 1)
    t->low_cell_V = BMS_SIM_FAULT_LCELL_UV_V;

#elif (BMS_SIM_INJECT == 2)
    t->sys_temp_high_C = BMS_SIM_FAULT_TEMP_C;

#elif (BMS_SIM_INJECT == 3)
    // Controller stops if last_error_class != 0
    t->last_error_class = 2;     // "Fault" (example)
    t->last_error_code  = 123;
    t->bms_fault        = 1;
    t->bms_fault_raw    = 0x20;

#elif (BMS_SIM_INJECT == 5)
    // Only triggers stop if Controller checks pack > PSU Vout using Cotek reading
    t->array_voltage_V  = BMS_SIM_FAULT_PACK_V;

#else
    // 0 or unknown -> no injection
    (void)t;
#endif
}

void BmsSim_tick(void) {
    BmsTelemetry t;
    fill_nominal(&t);

    // Default: always publish nominal unless we are "armed" by charging
    if (s_sim_charging) {
        uint32_t now = HAL_GetTick();
        uint32_t dt  = now - s_charge_t0_ms;

        const uint8_t inject_now =
            (dt >= BMS_SIM_INJECT_AFTER_MS) &&
            (dt <  (BMS_SIM_INJECT_AFTER_MS + BMS_SIM_INJECT_WINDOW_MS));

#if (BMS_SIM_INJECT == 4)
        // "comms lost" mode: after AFTER_MS, stop publishing telemetry (and optionally emit event once)
        static uint8_t s_sent_lost_evt = 0U;
        if (dt >= BMS_SIM_INJECT_AFTER_MS) {
            if (!s_sent_lost_evt) {
                s_sent_lost_evt = 1U;
                // If you want: explicitly tell controller comms lost.
                // (Only do this if your real system posts BMS_CONN_LOST_SIG)
                // QACTIVE_PUBLISH(Q_NEW(QEvt, BMS_CONN_LOST_SIG), 0U);
            }
            return; // no telemetry published => controller watchdog/logic should see comms stale
        } else {
            s_sent_lost_evt = 0U;
        }
#else
        if (inject_now) {
            apply_injection(&t);
        }
#endif
    }

    BMS_publish_telemetry(&t);
}



/* Simple, plausible “fake battery” snapshot */
// void BmsSim_tick(void) {
//     BmsTelemetry t;
//     memset(&t, 0, sizeof(t));
//
//     t.battery_type_code   = 0x0600;   // e.g. 600s
//     t.array_voltage_V     = 47.0f;
//     t.high_cell_V         = 3.60f;
//     t.low_cell_V          = 3.50f;
//     t.soc_percent         = 75;
//     t.bms_state           = 62;       // "Ready" in your mapper
//     t.bms_fault           = 0;        // no fault
//
//     /* Publish like real BMS does */
//     BMS_publish_telemetry(&t);


#endif