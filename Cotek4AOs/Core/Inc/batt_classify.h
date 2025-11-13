//
// Created by sorin.mihai on 06/10/2025.
//

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "bms_app.h"   // for BmsTelemetry (already in your project)

/* Three classes (plus Unknown when inputs are insufficient or SIM active) */
typedef enum {
    BATT_CLASS_UNKNOWN = 0,
    BATT_CLASS_NOT_RECOVERABLE,
    BATT_CLASS_RECOVERABLE,
    BATT_CLASS_OPERATIONAL,
} BattClass;

/* Result with UI-friendly decorations */
typedef struct {
    BattClass   cls;
    const char *label;       // "Not Recoverable" / "Recoverable" / "Operational" / "Unknown"
    const char *reason;      // short reason string (e.g., "NR error 0x05", "minV<2.5", "maxV>3.8")
    uint16_t    color565;    // recommended color for HMI (red/amber/green/grey)
} BattClassResult;

/* Evaluate class. If bms_sim_active==true, returns UNKNOWN by design. */
BattClassResult batt_classify(const BmsTelemetry *t, bool bms_sim_active);
