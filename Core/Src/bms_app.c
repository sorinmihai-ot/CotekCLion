// bms_app.c
// BMS Active Object (AO_Bms) — multi-pack (400s / 500s Hyperdrive/BMZ / 600s)
// Combined: robust BMZ vs 600 discrimination + correct 500HYP Hi/Lo (from 0x18FF0600 only)

#include "bms_app.h"
#include "qpc_cfg.h"
#include <ao_controller.h>
#include "bsp.h"
#include "qpc.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>

#include "bms_fault_decode.h"
#include "bms_debug.h"

Q_DEFINE_THIS_FILE

/* ============================ Build-time config ============================ */

#ifndef BSP_TICKS_PER_SEC
#define BSP_TICKS_PER_SEC         100U
#endif

#ifndef BMS_TICK_HZ
#define BMS_TICK_HZ               10U   /* AO internal tick = 10 Hz */
#endif

#ifndef BMS_PUB_HZ
#define BMS_PUB_HZ                 2U   /* publish telemetry at 2 Hz */
#endif

#ifndef BMS_WATCH_MS
#define BMS_WATCH_MS            1500U   /* comms-loss watchdog (ms) */
#endif

/* =============================== ID constants ============================== */

/* 500s Hyperdrive (J1939-like) 0x18FFxx00 pattern and specific PGNs */
#define ID_500_MASK             0xFFFF00FFu
#define ID_500_BASE             0x18FF0000u
#define ID_500_0600             0x18FF0600u  /* authoritative state + Hi/Lo cell (mV) */
#define ID_500_0700             0x18FF0700u  /* pack V/SOC/current/min-to-full */
#define ID_500_0800             0x18FF0800u  /* temps */
#define ID_500_1900             0x18FF1900u  /* node IDs + discharge limit (optional) */
#define ID_500_0300             0x18FF0300u  /* (legacy/fallback) fault/state; DO NOT use for Hi/Lo */
#define ID_500_0E00             0x18FF0E00u  /* error */
#define ID_500_5000             0x18FF5000u  /* fan etc */
#define ID_500_4000             0x18FF4000u  /* serial/firmware */
#define ID_500_F000             0x18FFF000u  /* BFG voltage etc (optional) */
#define ID_500_E000             0x18FFE000u  /* SoC + currents etc */

/* 600s + 500 BMZ extended range 0x100000xx */
#define ID_EXT_MASK             0xFFFF0000u
#define ID_EXT_BASE             0x10000000u
#define ID_EXT_10               0x10000010u
#define ID_EXT_11               0x10000011u
#define ID_EXT_20               0x10000020u
#define ID_EXT_100              0x10000100u
#define ID_EXT_110              0x10000110u
#define ID_EXT_50               0x10000050u
#define ID_EXT_00               0x10000000u
#define ID_EXT_80               0x10000080u
#define ID_EXT_90               0x10000090u
#define ID_EXT_91               0x10000091u
#define ID_EXT_A0               0x100000A0u

/* 400s family */
#define ID_400_FAULT            0x18060800u
#define ID_400_PACK_SOC         0x18070800u
#define ID_400_TEMPS            0x180C0800u
#define ID_400_SN_FW            0x18040A00u
#define ID_400_CELL_A_MASK      0xFFFFFF00u
#define ID_400_CELL_A_BASE      0x18000800u
#define ID_400_CELL_B_MASK      0xFFFFFF00u
#define ID_400_CELL_B_BASE      0x18010800u

/* ============================ Type codes / names =========================== */

#define TYPE_600S               0x0600u
#define TYPE_500S_HYP           0x0500u
#define TYPE_500S_BMZ           0x0501u
#define TYPE_400S_HYP           0x0400u
#define TYPE_400S_DUAL          0x0401u
#define TYPE_400S_STEATITE      0x0402u

static inline const char *bms_type_str(uint16_t code) {
    switch (code) {
        case TYPE_600S:          return "600s";
        case TYPE_500S_HYP:      return "500s Hyperdrive";
        case TYPE_500S_BMZ:      return "500s BMZ";
        case TYPE_400S_HYP:      return "400s Hyperdrive";
        case TYPE_400S_DUAL:     return "400s Dual-Zone";
        case TYPE_400S_STEATITE: return "400s Steatite";
        default:                 return "Unknown";
    }
}

/* ================================ Thresholds =============================== */

/* Use the range that worked for you on HYP */
#define CELL_MIN_V              0.80f
#define CELL_MAX_V              5.00f
#define PACK_MIN_VALID_V        5.00f

#define SERIES_600_MIN          15   /* inclusive */
#define SERIES_600_MAX          17
#define SERIES_500_MIN          13
#define SERIES_500_MAX          15

/* =========================== Endian helper funcs =========================== */

static inline uint16_t be16(const uint8_t *d) {
    return (uint16_t)((((uint16_t)d[0]) << 8) | d[1]);
}
static inline int16_t be16s(const uint8_t *d) {
    return (int16_t)((((uint16_t)d[0]) << 8) | d[1]);
}
static inline uint32_t be32(const uint8_t *d) {
    return ((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) | ((uint32_t)d[2] << 8) | d[3];
}

/* ============================== Globals/externs ============================ */

extern volatile uint16_t g_lastSig;
extern volatile uint8_t  g_lastTag;

volatile uint32_t last_bms_ms = 0;

/* Family-detection hints (reset on comms-lost) */
typedef struct {
    uint8_t  seen_hyp500;    /* any 0x18FFxx00 observed */
    uint16_t lock;           /* 0=unlocked; else locked family code */

    /* BMZ vs 600 signature counters on 0x10000010 */
    uint8_t  sig600_hits;    /* d0..1==02,25 and d7==05 */
    uint8_t  sigBMZ_hits;    /* d0..1==01,C0 and d7==01 */
    uint8_t  saw_bmz_sig;    /* latched if we ever saw BMZ signature */
    uint8_t  saw_600_only;   /* latched if we ever saw a 600-only ID */
} BmsFamilyDetect;

static BmsFamilyDetect s_det;
static inline void det_reset(void) { memset(&s_det, 0, sizeof(s_det)); }

/* ================================ Utilities =================================*/

static inline int roundf_to_int(float x) {
    return (int)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

static inline float accept_cell_mv(uint16_t mv) {
    if (mv == 0u || mv == 0xFFFFu) return 0.0f;
    const float v = (float)mv * 0.001f;
    if (v < CELL_MIN_V || v > CELL_MAX_V) return 0.0f;
    return v;
}

static void log_type(uint16_t new_code) {
    static uint16_t s_last = 0;
    if (new_code && new_code != s_last) {
        s_last = new_code;
        printf("BMS: detected type: %s (0x%04" PRIX16 ")\r\n", bms_type_str(new_code), new_code);
    }
}

/* Clear snapshot when family changes (prevents sticky data between families) */
static void begin_family(BmsTelemetry *b, uint16_t newcode) {
    if (b->battery_type_code != newcode) {
        memset(b, 0, sizeof(*b));
        b->battery_type_code = newcode;
        log_type(newcode);
    }
}

/* Choose family with optional locking (ignore if locked elsewhere) */
static inline void choose_family(BmsTelemetry *b, uint16_t code, bool strong) {
    if (s_det.lock && s_det.lock != code) {
        return; /* locked to another family */
    }
    begin_family(b, code);
    if (strong) {
        s_det.lock = code;
    }
}

/* 500s subtype choice using hints (kept for completeness) */
static uint16_t choose_500_subtype(void) {
    /* Not used now for HYP vs BMZ (handled by extended IDs), but harmless */
    if (s_det.seen_hyp500)  return TYPE_500S_HYP;
    return TYPE_500S_BMZ; /* default if ambiguous */
}

/* Try reclassifying between 500s/600s using Vpack and ~Vcell (only if not locked) */
static uint8_t bms_try_reclassify_by_voltage(BmsTelemetry *b) {
    if (s_det.lock) return 0U;

    const float vpack = b->array_voltage_V;
    const float vhi   = b->high_cell_V;
    const float vlo   = b->low_cell_V;

    float vcell = 0.0f;
    const bool hi_ok = (vhi >= CELL_MIN_V && vhi <= CELL_MAX_V);
    const bool lo_ok = (vlo >= CELL_MIN_V && vlo <= CELL_MAX_V);

    if (hi_ok && lo_ok)      vcell = 0.5f * (vhi + vlo);
    else if (hi_ok)          vcell = vhi;
    else if (lo_ok)          vcell = vlo;
    else                     return 0U;

    if (!(vpack > PACK_MIN_VALID_V)) return 0U;

    const float series_f = vpack / vcell;
    const int   series   = roundf_to_int(series_f);

    uint16_t inferred = 0;
    if (series >= SERIES_600_MIN && series <= SERIES_600_MAX) {
        inferred = TYPE_600S; /* ≈16s */
    } else if (series >= SERIES_500_MIN && series <= SERIES_500_MAX) {
        inferred = choose_500_subtype(); /* ≈14s */
    } else {
        return 0U;
    }

    if (inferred != 0 && inferred != b->battery_type_code) {
        b->battery_type_code = inferred; /* reclassify without wiping snapshot */
        printf("BMS: reclassified by Vpack/Vcell: series=%d (%.2f/%.2f) => %s\r\n",
               series, vpack, vcell, bms_type_str(inferred));
        log_type(inferred);
        return 1U;
    }
    return 0U;
}

/* ============================ AO definition block ========================== */

typedef struct {
    QActive  super;
    QTimeEvt tick;          /* 10 Hz internal tick */
    BmsTelemetry snap;
    uint8_t  have_any_data;
    uint32_t tick10;
    uint32_t last_rx_ticks;
    uint16_t pub_div;       /* tick divider for publish cadence */
} BmsAO;

static BmsAO l_bms;
QActive *AO_Bms = &l_bms.super;

static QState Bms_initial(BmsAO *me, void const *par);
static QState Bms_active (BmsAO *me, QEvt const *e);

/* Public: simulator can publish a full snapshot */
void BMS_publish_telemetry(BmsTelemetry const *t) {
    l_bms.snap = *t;
    l_bms.have_any_data = 1U;

    BmsTelemetryEvt *be = Q_NEW(BmsTelemetryEvt, BMS_UPDATED_SIG);
    be->data = *t;
    if (!QACTIVE_POST_X(AO_Controller, &be->super, 1U, 0U)) {
        QF_gc(&be->super);
    }
}

/* ============================ Family parsers ============================= */

static inline void mark_strong_600(BmsTelemetry *b) {
    s_det.saw_600_only = 1U;
    choose_family(b, TYPE_600S, true); /* hard lock */
}

/* 600s + 500 BMZ “extended” range: 0x100000xx */
static int parse_ext_100000xx(uint32_t id, uint8_t dlc, const uint8_t *d, BmsTelemetry *b) {
    if ((id & ID_EXT_MASK) != ID_EXT_BASE) return 0;

    switch (id) {
        case ID_EXT_10: {
            /* common fields */
            if (dlc >= 4) {
                const uint16_t v10 = be16(&d[0]);     /* 0.1V */
                const int16_t  i10 = be16s(&d[2]);    /* 0.1A (signed) */
                b->array_voltage_V = (float)v10 * 0.1f;
                b->current_dA      = i10;
            }

            /* signature scoring (no 600 hard-lock from 0x10 alone) */
            if (dlc >= 8 && !s_det.lock) {
                const uint8_t d0 = d[0], d1 = d[1], d7 = d[7];
                /* BMZ: 01 C0 .... 01 */
                if (d0==0x01 && d1==0xC0 && d7==0x01) {
                    s_det.saw_bmz_sig = 1U;
                    if (s_det.sigBMZ_hits < 3U) s_det.sigBMZ_hits++;
                    s_det.sig600_hits = 0U;
                    choose_family(b, TYPE_500S_BMZ, (s_det.sigBMZ_hits >= 2U));
                    return 1;
                }
                /* 600-like: 02 25 .... 05 */
                if (d0==0x02 && d1==0x25 && d7==0x05) {
                    if (s_det.sig600_hits < 3U) s_det.sig600_hits++;
                    if (!s_det.saw_bmz_sig) {
                        /* provisional 600 until a 600-only ID appears */
                        choose_family(b, TYPE_600S, false);
                    }
                    return 1;
                }
            }
            return 1;
        }

        case ID_EXT_11: {
            /* neutral; some firmwares use mW + zero tail */
            return 1;
        }

        /* 600-only frames => hard lock */
        case ID_EXT_20:
        case ID_EXT_100:
        case ID_EXT_110:
        case ID_EXT_50:
        case ID_EXT_00: {
            if (id == ID_EXT_100 && dlc >= 4) {
                const float hi = accept_cell_mv(be16(&d[0]));
                const float lo = accept_cell_mv(be16(&d[2]));
                if (hi > 0.0f) b->high_cell_V = hi;
                if (lo > 0.0f) b->low_cell_V  = lo;
            } else if (id == ID_EXT_110 && dlc >= 4) {
                b->sys_temp_high_C = (float)be16s(&d[0]) * 0.1f;
                b->sys_temp_low_C  = (float)be16s(&d[2]) * 0.1f;
            } else if (id == ID_EXT_20 && dlc >= 4) {
                b->soc_percent = d[3];
            }
            mark_strong_600(b);
            return 1;
        }

        /* neutral metadata */
        case ID_EXT_80:
        case ID_EXT_90:
        case ID_EXT_91:
        case ID_EXT_A0: {
            if (id == ID_EXT_90 && dlc >= 8) {
                b->serial_number    = be32(&d[0]);
                b->firmware_version = be32(&d[4]);
            }
            return 1;
        }

        default:
            return 0;
    }
}

/* 500s Hyperdrive (J1939-like): 0x18FFxx00 */
static int parse_500HYP(uint32_t id, uint8_t dlc, const uint8_t *d, BmsTelemetry *b) {
    if ((id & ID_500_MASK) != ID_500_BASE) return 0;

    s_det.seen_hyp500 = 1U;
    choose_family(b, TYPE_500S_HYP, true);

    switch (id) {
        case ID_500_0600: { /* Array state + Hi/Lo cell (1mV/bit each) */
            if (dlc >= 8) {
                const uint8_t st = d[2];
                switch (st) {
                    case 0: case 1: case 2: case 4: case 8: case 16:
                        b->bms_state = st; break;
                    default: break;
                }
                const float hi = accept_cell_mv(be16(&d[4]));
                const float lo = accept_cell_mv(be16(&d[6]));
                if (hi > 0.0f) b->high_cell_V = hi;
                if (lo > 0.0f) b->low_cell_V  = lo;
                if (hi > 0.0f || lo > 0.0f) {
                    printf("BMS(0600): Hcell=%.3fV Lcell=%.3fV\r\n", b->high_cell_V, b->low_cell_V);
                }
            }
            return 1;
        }

        case ID_500_0700: { /* pack V (0.1V), SOC %, charger flag, current (A) */
            if (dlc >= 6) {
                b->array_voltage_V = (float)be16(&d[0]) * 0.1f;
                b->soc_percent     = d[2];
                /* d[3] charger connected (ignored) */
                b->current_dA      = be16s(&d[4]); /* 1 A/bit, signed */
            }
            return 1;
        }

        case ID_500_0800: { /* temps 0.1C (BE) */
            if (dlc >= 4) {
                b->sys_temp_high_C = (float)be16s(&d[0]) * 0.1f;
                b->sys_temp_low_C  = (float)be16s(&d[2]) * 0.1f;
            }
            return 1;
        }

        case ID_500_1900: { /* optional; ignore for now */
            return 1;
        }

        case ID_500_0300: { /* fault + state; DO NOT use for Hi/Lo to avoid conflicts */
            if (dlc >= 4) {
                b->bms_fault_raw = d[2];
                b->bms_fault     = (b->bms_fault_raw != 0U) ? 1U : 0U;

                const uint8_t st = d[3];
                switch (st) {
                    case 0: case 1: case 2: case 4: case 8: case 16:
                        if (b->bms_state == 0) b->bms_state = st; /* keep 0600 priority */
                        break;
                    default: break;
                }
            }
            return 1;
        }

        case ID_500_0E00: { if (dlc >= 2) b->last_error_code = d[1]; return 1; }
        case ID_500_5000: { return 1; }
        case ID_500_4000: {
            if (dlc >= 8) {
                b->serial_number    = be32(&d[0]);
                b->firmware_version = be32(&d[4]);
            }
            return 1;
        }
        case ID_500_F000: { /* optional diag */
            return 1;
        }
        case ID_500_E000: { /* SoC + currents (0.1A) */
            if (dlc >= 1) {
                b->soc_percent = d[0];
            }
            return 1;
        }
        default:
            return 0;
    }
}

/* 400s family (Hyperdrive / Dual-Zone / Steatite) */
static int parse_400(uint32_t id, uint8_t dlc, const uint8_t *d, BmsTelemetry *b) {
    const bool is_cell_A = ((id & ID_400_CELL_A_MASK) == ID_400_CELL_A_BASE);
    const bool is_cell_B = ((id & ID_400_CELL_B_MASK) == ID_400_CELL_B_BASE);

    if (!(id == ID_400_FAULT || id == ID_400_PACK_SOC || id == ID_400_TEMPS ||
          id == ID_400_SN_FW || is_cell_A || is_cell_B)) {
        return 0;
    }

    if (b->battery_type_code == 0) {
        begin_family(b, TYPE_400S_HYP); /* default */
    }

    switch (id) {
        case ID_400_FAULT: {
            if (dlc >= 5) {
                const uint8_t fault = d[0];
                b->bms_fault     = fault ? 1U : 0U;
                b->bms_fault_raw = fault;

                const float hi = (float)be16(&d[1]) * 0.0015f;
                const float lo = (float)be16(&d[3]) * 0.0015f;
                const float ahi = (hi >= CELL_MIN_V && hi <= CELL_MAX_V) ? hi : 0.0f;
                const float alo = (lo >= CELL_MIN_V && lo <= CELL_MAX_V) ? lo : 0.0f;

                if (ahi > 0.0f) b->high_cell_V = ahi;
                if (alo > 0.0f) {
                    if (b->low_cell_V == 0.0f || alo < b->low_cell_V) b->low_cell_V = alo;
                }
                if (ahi > 0.0f || alo > 0.0f) {
                    printf("BMS(400): Hcell=%.3fV Lcell=%.3fV\r\n", b->high_cell_V, b->low_cell_V);
                }
            }
            return 1;
        }

        case ID_400_PACK_SOC: {
            if (dlc >= 3) {
                b->array_voltage_V = (float)be16(&d[0]) * 0.0012f;
                b->soc_percent = d[2];
            }
            return 1;
        }

        case ID_400_TEMPS: {
            if (dlc >= 4) {
                b->sys_temp_high_C = (float)be16s(&d[0]) * 0.1f;
                b->sys_temp_low_C  = (float)be16s(&d[2]) * 0.1f;
            }
            return 1;
        }

        case ID_400_SN_FW: {
            if (dlc >= 6) {
                b->serial_number    = be32(&d[0]);
                b->firmware_version = be16(&d[4]);
            }
            if (dlc >= 8 && d[6] == 0x00 && d[7] == 0x20) {
                begin_family(b, TYPE_400S_STEATITE);
            }
            return 1;
        }

        default: {
            if (dlc >= 2) {
                for (int i = 0; i + 1 < dlc; i += 2) {
                    const float v = (float)be16(&d[i]) * 0.001f;
                    if (v > CELL_MIN_V && v <= CELL_MAX_V) {
                        if (v > b->high_cell_V) b->high_cell_V = v;
                        if (b->low_cell_V == 0.0f || v < b->low_cell_V) b->low_cell_V = v;
                    }
                }
            }
            if ((id & 0xFFu) == 0x01u && (b->battery_type_code & 0xFF00u) == TYPE_400S_HYP) {
                begin_family(b, TYPE_400S_DUAL);
            }
            return 1;
        }
    }
}

/* ============================== Unified entry ============================== */

int BMS_ParseFrame(CanFrameEvt const *f, BmsTelemetry *b) {
    const uint32_t id  = f->id;
    const uint8_t  dlc = f->dlc;
    const uint8_t *d   = f->data;

    if (parse_400(id, dlc, d, b))            return 1; /* exclusive set of IDs */
    if (parse_500HYP(id, dlc, d, b))         return 1; /* 0x18FFxx00 pattern */
    if (parse_ext_100000xx(id, dlc, d, b))   return 1; /* 600s + 500BMZ ext */

    return 0;
}

/* AO_Bms hook when a frame was accepted */
void bms_on_frame(uint32_t id, const uint8_t *d, uint8_t dlc) {
    (void)d; (void)dlc;
    const uint32_t now = tick_ms();
    const uint32_t gap = now - last_bms_ms;
    last_bms_ms = now;

#if BMS_DEBUG
    if (gap >= 500U) {
        BMS_DBG("BMSDBG: FRAME RESUME id=0x%08" PRIX32 " gap=%" PRIu32 " ms (fresh=%u)\r\n",
                id, gap, (unsigned)bms_is_fresh());
    }
#endif
}

/* ================================ AO wiring =================================*/

void BmsAO_ctor(void) {
    QActive_ctor(&l_bms.super, Q_STATE_CAST(&Bms_initial));
    QTimeEvt_ctorX(&l_bms.tick, &l_bms.super, BMS_TICK_SIG, 0U);
}

static QState Bms_initial(BmsAO * const me, void const * const par) {
    (void)par;
    memset(&me->snap, 0, sizeof(me->snap));
    det_reset();
    me->have_any_data = 0U;
    me->tick10        = 0U;
    me->last_rx_ticks = 0U;
    me->pub_div       = 0U;

    QActive_subscribe(&me->super, CAN_RX_SIG);

    printf("BMS: initial, arming tick\r\n");
    QTimeEvt_armX(&me->tick,
                  BSP_TICKS_PER_SEC / BMS_TICK_HZ,
                  BSP_TICKS_PER_SEC / BMS_TICK_HZ);
    return Q_TRAN(&Bms_active);
}

static uint8_t bms_try_reclassify_by_voltage(BmsTelemetry *b); /* fwd */

static QState Bms_active(BmsAO * const me, QEvt const * const e) {
    switch (e->sig) {

    case CAN_RX_SIG: {
        CanFrameEvt const *ce = Q_EVT_CAST(CanFrameEvt);
        if (BMS_ParseFrame(ce, &me->snap)) {
            printf("BMS: frame parsed (id=0x%08" PRIX32 ", ext=%u, dlc=%u)\r\n",
                   ce->id, ce->isExt, ce->dlc);
            me->have_any_data = 1U;
            me->last_rx_ticks = me->tick10;
            bms_on_frame(ce->id, ce->data, ce->dlc);
        }
        return Q_HANDLED();
    }

    case BMS_TICK_SIG: {
        me->tick10++;

        /* Late sanity: derive series & reclassify if needed (runs at 10 Hz) */
        if (me->have_any_data) {
            if (bms_try_reclassify_by_voltage(&me->snap)) {
                /* nudge publish sooner so HMI updates quickly */
                me->pub_div = (uint16_t)(BMS_PUB_HZ > 1 ? (BMS_TICK_HZ / BMS_PUB_HZ) : 0);
            }
        }

        /* Publish at BMS_PUB_HZ */
        const uint16_t pub_div_target = (uint16_t)(BMS_TICK_HZ / BMS_PUB_HZ);
        if (++me->pub_div >= pub_div_target) {
            me->pub_div = 0U;

            if (me->have_any_data) {
                BmsTelemetryEvt *be = Q_NEW(BmsTelemetryEvt, BMS_UPDATED_SIG);
                be->data = me->snap;
                if (!QACTIVE_POST_X(AO_Controller, &be->super, 1U, &me->super)) {
                    QF_gc(&be->super);
                }
            } else {
                (void)QACTIVE_POST_X(AO_Controller,
                                     Q_NEW(QEvt, BMS_NO_BATTERY_SIG), 1U, &me->super);
            }
        }

        /* Comms-loss detection (wall-clock) */
        {
            const uint32_t now = tick_ms();
            const uint32_t age = now - last_bms_ms;
            if (me->have_any_data && (age > BMS_WATCH_MS)) {
                printf("BMS: comms lost (no frames in %" PRIu32 " ms)\r\n", age);
                (void)QACTIVE_POST_X(AO_Controller,
                    Q_NEW(QEvt, BMS_CONN_LOST_SIG), 1U, &me->super);

                /* wipe the snapshot & detection hints to avoid stale UI */
                memset(&me->snap, 0, sizeof(me->snap));
                det_reset();
                me->have_any_data = 0U;
            }
        }
        return Q_HANDLED();
    }

    default:
        break;
    }
    return Q_SUPER(&QHsm_top);
}

/* ============================ Snapshot accessors =========================== */

void BMS_GetSnapshot(BmsTelemetry *dst) {
    QF_CRIT_STAT;
    QF_CRIT_ENTRY();
    *dst = l_bms.snap;
    QF_CRIT_EXIT();
}

/* ===================== Mapping (text/classification) ====================== */

const char *BMS_state_to_text(uint16_t batt_type, uint8_t raw_state) {
    (void)batt_type;
    switch (raw_state) {
        case 0:  return "Idle";
        case 1:  return "Precharge";
        case 2:  return "Charge";
        case 3:  return "Discharge";
        case 4:  return "Balancing";
        case 5:  return "Sleep";
        case 62: return "Ready";
        case 63: return "Active";
        default: return "Unknown";
    }
}

typedef enum { BMS_STATUS_OK = 0, BMS_STATUS_WARN, BMS_STATUS_FAULT } BmsStatus;
static inline uint32_t critical_mask_for(uint16_t batt_type) { (void)batt_type; return 0u; }

BmsStatus BMS_classify_fault(uint16_t batt_type, uint32_t rawFault, bool *recoverable_out) {
    const uint32_t crit = critical_mask_for(batt_type);
    const bool critical    = (rawFault & crit) != 0u;
    const bool recoverable = (rawFault & ~crit) != 0u;
    if (recoverable_out) *recoverable_out = recoverable && !critical;
    if (critical)    return BMS_STATUS_FAULT;
    if (recoverable) return BMS_STATUS_WARN;
    return BMS_STATUS_OK;
}
