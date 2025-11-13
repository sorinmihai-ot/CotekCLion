// batt_classify.c
#include "batt_classify.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* Colors (RGB565) */
#define COL_RED     0xF800u
#define COL_AMBER   0xFD20u
#define COL_GREEN   0x07E0u
#define COL_GREY    0xC618u

static inline float upper_recovery_maxV(void) { return 3.8f; }
static inline float low_thresh_minV(uint16_t code) {
    switch (code) {
    case 0x0600: return 2.0f;
    case 0x0501: return 2.5f;
    case 0x0500: return 2.8f;
    case 0x0402: return 2.5f;
    case 0x0401: return 2.5f;
    case 0x0400: return 2.7f;
    default:     return 0.0f;
    }
}

/* NR code tables (sparse; extend as needed) */
static bool nr_code_600s(uint8_t code) {
    switch (code) {
        case 0x01: case 0x02: case 0x03: case 0x04:
        case 0x05: case 0x06: case 0x07: case 0x08:
        case 0x09: case 0x0A: case 0x0C: case 0x0D:
        case 0x11: case 0x12: case 0x25: case 0x30:
            return true;
        default: return false;
    }
}
static bool nr_code_500s_BMZ(uint8_t code) {
    switch (code) {
        case 0x01: case 0x0C:
        case 0x05: case 0x34:
        case 0x06: case 0x32:
        case 0x08: case 0x2F: case 0x0A: case 0x21:
        case 0x11: case 0x12: case 0x19:
            return true;
        default: return false;
    }
}
static bool nr_code_500s_Hyper(uint8_t code) {
    switch (code) {
        case 0x02: case 0x05: case 0x06: case 0x08: case 0x07:
        case 0x01: case 0x09: case 0x0C: case 0x0D: case 0x20:
        case 0x21: case 0x22:
            return true;
        default: return false;
    }
}
static bool nr_code_400s(uint8_t code) { (void)code; return false; }

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

    if (fam == 0u || vmin <= 0.01f || vmax <= 0.01f) {
        r.reason = "type/V missing";
        return r;
    }

    if (has_not_recoverable_code(fam, t->last_error_code)) {
        r.cls = BATT_CLASS_NOT_RECOVERABLE;
        r.label = "Not Recoverable";
        r.color565 = COL_RED;
        static char why[32];
        snprintf(why, sizeof(why), "NR error 0x%02X", t->last_error_code);
        r.reason = why;
        return r;
    }

    const float low_ok  = low_thresh_minV(fam);
    const float high_ok = upper_recovery_maxV();
    if (low_ok <= 0.0f) { r.reason = "unknown family"; return r; }

    if ( (vmin >= low_ok) && (vmax <= high_ok) ) {
        r.cls = BATT_CLASS_RECOVERABLE;
        r.label = "Batt Recoverable";
        r.color565 = COL_AMBER;
        static char why[32]; snprintf(why, sizeof(why), "min>=%.1f & max<=3.8", low_ok);
        r.reason = why;
        return r;
    }

    if (vmax > high_ok) {
        r.cls = BATT_CLASS_OPERATIONAL;
        r.label = "Batt Operational";
        r.color565 = COL_GREEN;
        r.reason = "max>3.8";
        return r;
    }

    r.cls = BATT_CLASS_NOT_RECOVERABLE;
    r.label = "Batt Not Recoverable";
    r.color565 = COL_RED;
    if (vmin < low_ok) {
        static char why[24]; snprintf(why, sizeof(why), "min<%.1f", low_ok);
        r.reason = why;
    } else {
        r.reason = "out of window";
    }
    return r;
}
