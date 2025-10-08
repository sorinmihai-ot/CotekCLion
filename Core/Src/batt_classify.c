//
// Created by sorin.mihai on 06/10/2025.
//
#include "batt_classify.h"

#include <stdio.h>

/* UI colors (RGB565) */
#define COL_RED     0xF800u  /* Not Recoverable */
#define COL_AMBER   0xFD20u  /* Recoverable */
#define COL_GREEN   0x07E0u  /* Operational */
#define COL_GREY    0xC618u  /* Unknown */

/* Upper bound used for "recovery window" checks across families */
static inline float upper_recovery_maxV(void) { return 3.8f; }

/* Family-specific min-cell thresholds for the "Recoverable window" */
static inline float low_thresh_minV(uint16_t code) {
    switch (code) {
    case 0x0600: return 2.0f;  /* 600s */
    case 0x0501: return 2.5f;  /* 500s BMZ */
    case 0x0500: return 2.8f;  /* 500s Hyperdrive */
    case 0x0402: return 2.5f;  /* 400s Steatite */
    case 0x0401: return 2.5f;  /* 400s Dual-Zone Steatite */
    case 0x0400: return 2.7f;  /* 400s Hyperdrive */
    default:     return 0.0f;  /* unknown family → can’t judge */
    }
}

/* --- Not-Recoverable error code sets (present → immediately Not Recoverable)
   Note: we only have last_error_code/class in telemetry (no duration/severity),
   so existence of any critical/major code below is treated as NR.            */

/* 600s: NR codes */
static bool nr_code_600s(uint8_t code) {
    switch (code) {
        case 0x01: case 0x02: case 0x03: case 0x04:
        case 0x05: case 0x06: case 0x07: case 0x08:
        case 0x09: case 0x0A: /* 0x0B reserved */
        case 0x0C: case 0x0D:
        case 0x11: case 0x12:
        case 0x25:
        case 0x30:
            return true;
        default: return false;
    }
}

/* 500s BMZ: NR codes */
static bool nr_code_500s_BMZ(uint8_t code) {
    switch (code) {
        case 0x01: /* Overvoltage */
        case 0x0C: /* Deep Discharge */
        case 0x05: case 0x34: /* Over Temp (dis/charge) */
        case 0x06: case 0x32: /* Under Temp (dis/charge) */
        case 0x08:           /* PCB Over Temp */
        case 0x2F:           /* Cells Temp Gap */
        case 0x0A:           /* Too Unbalanced */
        case 0x21:           /* V-sum mismatch/absent cell/chain broken */
        case 0x12: case 0x11:/* Thermistor errors */
        case 0x19:           /* CAN error while charging */
            return true;
        default: return false;
    }
}

/* 500s Hyperdrive: NR codes */
static bool nr_code_500s_Hyper(uint8_t code) {
    switch (code) {
        case 0x02: /* Deep Discharge */
        case 0x05: /* High Cell Temp (C2) - no severity info, treat as NR if present */
        case 0x06: /* Low Cell Temp (C2) */
        case 0x08: /* High PCB Temp (C2) */
        case 0x07: /* High Fuse Temp (C2) */
        case 0x01: /* High Cell Voltage (C2/C3) */
        case 0x09: /* Charge mismatch (C2) */
        case 0x0C: /* Cell Deep Discharge (C2) */
        case 0x0D: /* Welded Contactor (C2) */
        case 0x20: /* Short Circuit Discharge (C3) */
        case 0x21: /* BMS HW error 1 (C3) */
        case 0x22: /* BMS HW error 2 (C3) */
            return true;
        default: return false;
    }
}

/* 400s: user will provide NR code list later → for now, do not NR-by-code. */
static bool nr_code_400s(uint8_t code) {
    (void)code;
    return false; /* placeholder, only voltage rules apply for now */
}

static bool has_not_recoverable_code(uint16_t type_code, uint8_t err_code) {
    if (err_code == 0u) return false;
    switch (type_code) {
        case 0x0600: return nr_code_600s(err_code);
        case 0x0501: return nr_code_500s_BMZ(err_code);
        case 0x0500: return nr_code_500s_Hyper(err_code);
        case 0x0400: case 0x0401: case 0x0402: return nr_code_400s(err_code);
        default: return false;
    }
}

BattClassResult batt_classify(const BmsTelemetry *t, bool bms_sim_active) {
    BattClassResult r = { BATT_CLASS_UNKNOWN, "Unknown", "SIM/insufficient data", COL_GREY };

    if (bms_sim_active) {
        r.reason = "BMS SIM active";
        return r;
    }
    if (!t) return r;

    const uint16_t fam = t->battery_type_code;
    const float vmin  = t->low_cell_V;
    const float vmax  = t->high_cell_V;

    /* Basic sanity */
    if (fam == 0u || vmin <= 0.01f || vmax <= 0.01f) {
        r.reason = "type/V missing";
        return r;
    }

    /* 1) Not-Recoverable by explicit error code */
    if (has_not_recoverable_code(fam, t->last_error_code)) {
        r.cls = BATT_CLASS_NOT_RECOVERABLE;
        r.label = "Not Recoverable";
        r.color565 = COL_RED;

        static char why[32];
        (void)snprintf(why, sizeof(why), "NR error 0x%02X", t->last_error_code);
        r.reason = why;
        return r;
    }

    /* Voltage windows */
    const float low_ok  = low_thresh_minV(fam);
    const float high_ok = upper_recovery_maxV();

    if (low_ok <= 0.0f) {
        r.reason = "unknown family";
        return r;
    }

    /* 2) Recoverable window: min >= family_low  && max <= 3.8, and no NR errors */
    if ( (vmin >= low_ok) && (vmax <= high_ok) ) {
        r.cls = BATT_CLASS_RECOVERABLE;
        r.label = "Recoverable";
        r.color565 = COL_AMBER;

        static char why[32];
        (void)snprintf(why, sizeof(why), "min>=%.1f & max<=3.8", low_ok);
        r.reason = why;
        return r;
    }

    /* 3) Operational: max > 3.8 and no NR errors */
    if (vmax > high_ok) {
        r.cls = BATT_CLASS_OPERATIONAL;
        r.label = "Operational";
        r.color565 = COL_GREEN;
        r.reason = "max>3.8";
        return r;
    }

    /* 4) Otherwise treat as Not Recoverable (e.g., min below family threshold) */
    r.cls = BATT_CLASS_NOT_RECOVERABLE;
    r.label = "Not Recoverable";
    r.color565 = COL_RED;

    if (vmin < low_ok) {
        static char why[24];
        (void)snprintf(why, sizeof(why), "min<%.1f", low_ok);
        r.reason = why;
    } else {
        r.reason = "out of window";
    }
    return r;
}
