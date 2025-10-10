//
// BMS Active Object (AO_Bms) -- multi-pack (400s/500s/600s)
//
#include "bms_app.h"
#include "qpc_cfg.h"
#include <ao_controller.h>
#include "bsp.h"
#include "qpc.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "bms_fault_decode.h"

Q_DEFINE_THIS_FILE

#ifndef BSP_TICKS_PER_SEC
#define BSP_TICKS_PER_SEC 100U
#endif

/* ================= Endian helpers (BE/Motorola for J1939 etc.) ================= */
#ifndef BMS_ENDIAN_HELPERS
#define BMS_ENDIAN_HELPERS
static inline uint16_t be16(const uint8_t *d) { return (uint16_t)((((uint16_t)d[0])<<8) | d[1]); }
static inline uint32_t be32(const uint8_t *d) { return  ((uint32_t)d[0]<<24) | ((uint32_t)d[1]<<16) | ((uint32_t)d[2]<<8) | d[3]; }
static inline int16_t  be16s(const uint8_t *d) { return (int16_t)((d[0] << 8) | d[1]); }
#endif

extern volatile uint16_t g_lastSig;
extern volatile uint8_t  g_lastTag;

/* ================= Derived status helpers (mapping for HMI) ================= */

typedef enum {
    BMS_STATUS_OK = 0,
    BMS_STATUS_WARN,
    BMS_STATUS_FAULT
} BmsStatus;

/* Exported helpers (declare in bms_app.h later if you want Controller to call them) */
static const char *bms_type_str(uint16_t code);
const char *BMS_state_to_text(uint16_t batt_type, uint8_t raw_state);
BmsStatus  BMS_classify_fault(uint16_t batt_type, uint32_t rawFault, bool *recoverable_out);

/* Conservative policy: treat unknown bits as WARN (recoverable), reserve a mask for critical when spec is known */
static inline uint32_t critical_mask_for(uint16_t batt_type) {
    (void)batt_type;
    /* TODO: fill in real masks per family; keep 0 for bring-up so you can charge */
    return 0u;
}

/* ================= AO definition ================= */

typedef struct {
    QActive  super;
    QTimeEvt tick;          /* 10Hz internal tick */
    BmsTelemetry snap;
    uint8_t  have_any_data;
    uint32_t tick10;        /* 10Hz ticks */
    uint32_t last_rx_ticks; /* age tracking */
    uint16_t pub_div;       /* downcounter for publish cadence */
} BmsAO;

static BmsAO l_bms;
QActive *AO_Bms = &l_bms.super;

/* ----- forward decls ----- */
static QState Bms_initial(BmsAO *me, void const *par);
static QState Bms_active (BmsAO *me, QEvt const *e);

/* Public helper: called by simulator to publish a complete snapshot */
void BMS_publish_telemetry(BmsTelemetry const *t) {
    /* keep the AO snapshot current (used by GetSnapshot, etc.) */
    l_bms.snap = *t;
    l_bms.have_any_data = 1U;

    BmsTelemetryEvt *be = Q_NEW(BmsTelemetryEvt, BMS_UPDATED_SIG);
    be->data = *t;  /* copy */

    /* post to Controller with margin=1; GC if queue is temporarily full */
    if (!QACTIVE_POST_X(AO_Controller, &be->super, 1U, 0U)) {
        QF_gc(&be->super);
    }
}

static const char *bms_type_str(uint16_t code) {
    switch (code) {
        case 0x0600: return "600s";
        case 0x0500: return "500s Hyperdrive";
        case 0x0501: return "500s BMZ";
        case 0x0400: return "400s Hyperdrive";
        case 0x0401: return "400s Dual-Zone";
        case 0x0402: return "400s Steatite";
        default:     return "Unknown";
    }
}

static void bms_maybe_log_type(uint16_t new_code) {
    static uint16_t s_last_logged_code = 0;
    if (new_code != 0 && new_code != s_last_logged_code) {
        s_last_logged_code = new_code;
        printf("BMS: detected battery type: %s (0x%04X)\r\n",
               bms_type_str(new_code), new_code);
    }
}


/* Return 1 if frame used/parsed (battery data seen), 0 otherwise. */
int BMS_ParseFrame(CanFrameEvt const *f, BmsTelemetry *b)
{
    uint32_t id  = f->id;
    uint8_t  dlc = f->dlc;
    uint8_t const *d = f->data;

    /* ===================== 600s family (Ocado 0x100000xx, extended) ===================== */
    /* Keep existing mapping; if we see 0x10000010 later we will classify as BMZ 500s. */
    if ((id & 0xFFFF0000u) == 0x10000000u) {
        /* If we haven't chosen a 500s-BMZ subtype yet, default this extended block to 600s. */
        if (b->battery_type_code == 0) {
            /* Will be overridden to 0x0501 if we see 0x10000010 (BMZ) */
            b->battery_type_code = 0x0600;
            bms_maybe_log_type(b->battery_type_code);
        }

        switch (id) {
            case 0x10000090u: /* Pack ID & FW (Serial + FW) */
                if (dlc >= 8) {
                    b->serial_number    = be32(&d[0]);
                    b->firmware_version = be32(&d[4]);
                }
                return 1;

            case 0x10000110u: /* Temperatures (0.1°C) */
                if (dlc >= 4) {
                    int16_t th = be16s(&d[0]);
                    int16_t tl = be16s(&d[2]);
                    b->sys_temp_high_C = (float)th * 0.1f;
                    b->sys_temp_low_C  = (float)tl * 0.1f;
                }
                return 1;

            case 0x10000020u: /* Charging params / SOC (%) */
                if (dlc >= 4) {
                    b->soc_percent = d[3]; /* 0..100 or 0xFF unknown */
                }
                return 1;

            default: ; /* fall-through to allow BMZ handling below */
        }
    }

    /* ===================== 500s BMZ (extended 0x100000xx) ===================== */
    /* Presence of 0x10000010 indicates the BMZ flavour of the extended scheme. */
    if (id == 0x10000010u) { /* Pack status & power + battery type */
        if (b->battery_type_code == 0 || b->battery_type_code == 0x0600) {
            b->battery_type_code = 0x0501; /* 500s BMZ sub-type */
            bms_maybe_log_type(b->battery_type_code);
        }
        if (dlc >= 8) {
            /* V, I, state, fault, type (per doc) */
            uint16_t v10 = be16(&d[0]);          /* 0.1 V/bit */
            int16_t  i10 = (int16_t)be16(&d[2]); /* 0.1 A/bit, sign: + = charging */
            (void)i10; /* placeholder for future current field */

            b->array_voltage_V = (float)v10 * 0.1f;
            b->bms_state       = d[4];                 /* enum */
            b->bms_fault       = (d[5] != 0U) ? 1U : 0U; /* any bit -> fault present */
            b->bms_fault_raw   = d[5];

            /* d[6] is "Battery Type" enumeration in BMZ doc; optional to store */
            printf("BMS: BMZ state=%s fault=%s (raw=0x%02X)\r\n",
                   BMS_state_to_text(b->battery_type_code, b->bms_state),
                   b->bms_fault ? "YES":"NO", (unsigned)b->bms_fault);
        }
        return 1;
    }
    if ((id & 0xFFFF0000u) == 0x10000000u && b->battery_type_code == 0x0501) {
        switch (id) {
            case 0x10000020u: /* SOC etc. */
                if (dlc >= 4) b->soc_percent = d[3];
                return 1;

            case 0x10000100u: /* Highest/Lowest cell mV */
                if (dlc >= 4) {
                    b->high_cell_V = (float)be16(&d[0]) * 0.001f;
                    b->low_cell_V  = (float)be16(&d[2]) * 0.001f;
                }
                return 1;

            case 0x10000110u: /* Temps 0.1°C */
                if (dlc >= 4) {
                    int16_t th = be16s(&d[0]);
                    int16_t tl = be16s(&d[2]);
                    b->sys_temp_high_C = (float)th * 0.1f;
                    b->sys_temp_low_C  = (float)tl * 0.1f;
                }
                return 1;

            case 0x10000090u: /* Serial + FW */
                if (dlc >= 8) {
                    b->serial_number    = be32(&d[0]);
                    b->firmware_version = be32(&d[4]);
                }
                return 1;

            default: ;
        }
    }

    /* ===================== 500s Hyperdrive (J1939 0x18FFxx00) ===================== */
    if ((id & 0xFFFF00FFu) == 0x18FF0000u) {
        if (b->battery_type_code == 0) {
            b->battery_type_code = 0x0500;
            bms_maybe_log_type(b->battery_type_code);
        }

        switch (id) {
            case 0x18FF4000u: /* Serial + FW (32/32-bit BE) */
                if (dlc >= 8) {
                    b->serial_number    = be32(&d[0]);
                    b->firmware_version = be32(&d[4]);
                }
                return 1;

            case 0x18FF0300u: /* State/fault */
                if (dlc >= 2) {
                    b->bms_fault = d[0];
                    b->bms_fault_raw = d[0];
                    uint8_t s = d[1];
                    switch (s) {
                        case 0: case 1: case 2: case 3:
                        case 4: case 5: case 62: case 63:
                            b->bms_state = s;                // accept known states
                            break;
                        default:
                            /* ignore unknown/placeholder states like 0x7F/0xFF */
                            /* leave b->bms_state unchanged */
                            break;
                    }

                    printf("BMS: state=%s fault=0x%02X\r\n",
                           BMS_state_to_text(b->battery_type_code, b->bms_state),
                           (unsigned)b->bms_fault);
                }
                return 1;

            case 0x18FF0600u: {           // High/Low cell voltage (mV) for 500s Hyperdrive
                if (dlc >= 8) {
                    uint16_t hi_mv = be16(&d[4]);
                    uint16_t lo_mv = be16(&d[6]);
                    float hi = hi_mv * 0.001f;   // mV -> V
                    float lo = lo_mv * 0.001f;

                    // sanity clamp (Li-ion)
                    if (hi >= 2.0f && hi <= 4.6f) b->high_cell_V = hi;
                    if (lo >= 2.0f && lo <= 4.6f) b->low_cell_V  = lo;
                }
                return 1;
            }


            case 0x18FF0700u: /* Array/pack voltage 0.1 V/LSB */
                if (dlc >= 2) b->array_voltage_V = (float)be16(&d[0]) * 0.1f;
                return 1;

            case 0x18FFE000u: /* SOC % */
                if (dlc >= 1) b->soc_percent = d[0];
                return 1;

            case 0x18FF0800u: /* System temps 0.1°C/LSB */
                if (dlc >= 4) {
                    b->sys_temp_high_C = (float)be16s(&d[0]) * 0.1f;
                    b->sys_temp_low_C  = (float)be16s(&d[2]) * 0.1f;
                }
                return 1;

            case 0x18FF5000u: /* Fan RPM */
                if (dlc >= 2) b->fan_rpm = be16(&d[0]);
                return 1;

            case 0x18FF0E00u: /* Last error class/code */
                if (dlc >= 2) {
                    b->last_error_class = d[0];
                    b->last_error_code  = d[1];
                }
                return 1;

            default: ;
        }
    }

    /* ===================== 400s family (Custom Power: Dual-Zone / Steatite) ===================== */
    if ( (id == 0x18070800u) || (id == 0x18060800u) || (id == 0x180C0800u)
      || (id == 0x18000800u) || (id == 0x18010800u)
      || (id == 0x18040A00u)
      || ((id & 0xFFFFFF00u) == 0x18000800u)
      || ((id & 0xFFFFFF00u) == 0x18010800u) ) {

        if (b->battery_type_code == 0) {
            b->battery_type_code = 0x0400;
            bms_maybe_log_type(b->battery_type_code);
        }

        if ((id & 0x000000FFu) == 0x01u) {
            if ((b->battery_type_code & 0xFF00u) == 0x0400u && b->battery_type_code != 0x0401) {
                b->battery_type_code = 0x0401; /* Dual-Zone tentative */
                bms_maybe_log_type(b->battery_type_code);
            }
        }

        switch (id) {
            case 0x18070800u: /* STATUS_1: SOC, etc. */
                if (dlc >= 7) {
                    b->soc_percent = d[6] ? d[6] : d[5];
                }
                return 1;

            case 0x18060800u: /* STATUS_0: min/max cell V overall (BE mV) */
                if (dlc >= 4) {
                    float vmax = (float)be16(&d[0]) * 0.001f;
                    float vmin = (float)be16(&d[2]) * 0.001f;
                    if (vmax > 0.1f && vmax > b->high_cell_V) b->high_cell_V = vmax;
                    if (vmin > 0.1f && (b->low_cell_V == 0.0f || vmin < b->low_cell_V)) b->low_cell_V = vmin;
                }
                return 1;

            case 0x180C0800u: /* TEMPERATURE_SUMMARY: max/min temps (signed 8-bit) */
                if (dlc >= 4) {
                    b->sys_temp_high_C = (float)((int8_t)d[0]);
                    b->sys_temp_low_C  = (float)((int8_t)d[1]);
                }
                return 1;

            case 0x18000800u: /* CELL_VOLTAGE_A */
            case 0x18010800u: /* CELL_VOLTAGE_B */
            {
                if (dlc >= 2) {
                    for (int i = 0; i + 1 < dlc; i += 2) {
                        float v = (float)be16(&d[i]) * 0.001f;
                        if (v > 0.1f) {
                            if (v > b->high_cell_V) b->high_cell_V = v;
                            if (b->low_cell_V == 0.0f || v < b->low_cell_V) b->low_cell_V = v;
                        }
                    }
                }
                return 1;
            }

            case 0x18040A00u: /* PARAMETERS_4: UID, FW, Battery Type */
                if (dlc >= 2) {
                    uint16_t type_be = (dlc >= 8) ? be16(&d[6]) : be16(&d[dlc - 2]);
                    if (type_be == 0x0020u) {
                        if (b->battery_type_code != 0x0402) {
                            b->battery_type_code = 0x0402;   /* Steatite */
                            bms_maybe_log_type(b->battery_type_code);
                        }
                    } else {
                        /* keep current subtype */
                    }
                }
                return 1;

            default: ;
        }
    }

    /* ===================== 400s legacy fallbacks ===================== */
    if ( (id == 0x18070800u) || (id == 0x18060800u) || (id == 0x180C0800u)
      || ((id & 0xFFFFF0FFu) == 0x18004000u) ) {
        if (b->battery_type_code == 0) {
            b->battery_type_code = 0x0400;
            bms_maybe_log_type(b->battery_type_code);
        }

        switch (id) {
            case 0x18070800u:
                if (dlc >= 7) {
                    uint8_t faultBit = (d[2] >> 6) & 0x01u;
                    b->bms_fault  = faultBit ? 1U : 0U;
                    b->bms_fault_raw = 0U;   // <--- keep 0, no raw byte defined for this family here
                    b->bms_state  = d[3];
                    b->soc_percent = d[6] ? d[6] : d[5];
                    printf("BMS: state=%s fault=%s\r\n",
                           BMS_state_to_text(b->battery_type_code, b->bms_state),
                           b->bms_fault ? "YES":"NO");
                }
                return 1;

            case 0x18060800u:
                if (dlc >= 4) {
                    b->sys_temp_high_C = (float)((int8_t)d[2]);
                    b->sys_temp_low_C  = (float)((int8_t)d[3]);
                }
                return 1;

            case 0x180C0800u:
                if (dlc >= 3) {
                    b->last_error_class = d[1];
                    b->last_error_code  = d[2];
                    if (b->last_error_class || b->last_error_code) b->bms_fault = 1U;
                }
                return 1;

            default:
                if ((id & 0xFFFFF0FFu) == 0x18004000u && dlc >= 6) {
                    float c1 = (float)be16(&d[0]) * 0.001f;
                    float c2 = (float)be16(&d[2]) * 0.001f;
                    float c3 = (float)be16(&d[4]) * 0.001f;
                    if (c1 > 0.1f) { if (b->high_cell_V < c1) b->high_cell_V = c1; if (b->low_cell_V == 0 || b->low_cell_V > c1) b->low_cell_V = c1; }
                    if (c2 > 0.1f) { if (b->high_cell_V < c2) b->high_cell_V = c2; if (b->low_cell_V == 0 || b->low_cell_V > c2) b->low_cell_V = c2; }
                    if (c3 > 0.1f) { if (b->high_cell_V < c3) b->high_cell_V = c3; if (b->low_cell_V == 0 || b->low_cell_V > c3) b->low_cell_V = c3; }
                    return 1;
                }
                break;
        }
    }

    return 0;
}

void BmsAO_ctor(void) {
    QActive_ctor(&l_bms.super, Q_STATE_CAST(&Bms_initial));
    QTimeEvt_ctorX(&l_bms.tick, &l_bms.super, BMS_TICK_SIG, 0U);
}

static QState Bms_initial(BmsAO * const me, void const * const par) {
    (void)par;
    memset(&me->snap, 0, sizeof(me->snap));
    me->have_any_data = 0U;
    me->tick10        = 0U;
    me->last_rx_ticks = 0U;
    me->pub_div       = 0U;

    QActive_subscribe(&me->super, CAN_RX_SIG);

    /* 10Hz internal tick */
    printf("BMS: initial, arming tick\r\n");
    QTimeEvt_armX(&me->tick, BSP_TICKS_PER_SEC/10U, BSP_TICKS_PER_SEC/10U);
    printf("BMS: initial, armed tick\r\n");
    return Q_TRAN(&Bms_active);
}

static QState Bms_active(BmsAO * const me, QEvt const * const e) {
    switch (e->sig) {
    case CAN_RX_SIG: {
        CanFrameEvt const *ce = Q_EVT_CAST(CanFrameEvt);
        if (BMS_ParseFrame(ce, &me->snap)) {
            printf("BMS: frame parsed (id=0x%lX, ext=%u, dlc=%u)\r\n",
                   (unsigned long)ce->id, ce->isExt, ce->dlc);
            me->have_any_data = 1U;
            me->last_rx_ticks = me->tick10;
        }
        return Q_HANDLED();
    }

    case BMS_TICK_SIG: {
        me->tick10++;

        /* ===== Publish cadence: 2 Hz (every 0.5 s) =====
           10 Hz tick -> publish every 5 ticks */
        if (++me->pub_div >= 5U) {
            me->pub_div = 0U;

            if (me->have_any_data) {
                BmsTelemetryEvt *be = Q_NEW(BmsTelemetryEvt, BMS_UPDATED_SIG);
                be->data = me->snap;
                /* point-to-point to Controller */
                if (!QACTIVE_POST_X(AO_Controller, &be->super, 1U, &me->super)) {
                    QF_gc(&be->super);
                }
            } else {
                (void)QACTIVE_POST_X(AO_Controller,
                                     Q_NEW(QEvt, BMS_NO_BATTERY_SIG), 1U, &me->super);
            }
        }
        return Q_HANDLED();
    }

    default: ;
    }
    return Q_SUPER(&QHsm_top);
}

void BMS_GetSnapshot(BmsTelemetry *dst) {
    QF_CRIT_STAT;
    QF_CRIT_ENTRY();
    *dst = l_bms.snap;
    QF_CRIT_EXIT();
}

/* ===================== Mapping helpers (text + classification) ===================== */

const char *BMS_state_to_text(uint16_t batt_type, uint8_t raw_state) {
    (void)batt_type; /* If families differ, split switch by batt_type later */
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

BmsStatus BMS_classify_fault(uint16_t batt_type, uint32_t rawFault, bool *recoverable_out) {
    uint32_t crit = critical_mask_for(batt_type);
    bool critical    = (rawFault & crit) != 0u;
    bool recoverable = (rawFault & ~crit) != 0u;

    if (recoverable_out) *recoverable_out = recoverable && !critical;

    if (critical)    return BMS_STATUS_FAULT;
    if (recoverable) return BMS_STATUS_WARN;
    return BMS_STATUS_OK;
}






// //
// // BMS Active Object (AO_Bms) -- multi-pack (400s/500s/600s)
// //
// #include "bms_app.h"
// #include "qpc_cfg.h"
// #include <ao_controller.h>
// #include "bsp.h"
// #include "qpc.h"
// #include <string.h>
// #include <stdio.h>
//
// Q_DEFINE_THIS_FILE
//
// #ifndef BSP_TICKS_PER_SEC
// #define BSP_TICKS_PER_SEC 100U
// #endif
//
// #ifndef BMS_ENDIAN_HELPERS
// #define BMS_ENDIAN_HELPERS
// static inline uint16_t be16(const uint8_t *d) { return (uint16_t)((((uint16_t)d[0])<<8) | d[1]); }
// static inline uint32_t be32(const uint8_t *d) { return  ((uint32_t)d[0]<<24) | ((uint32_t)d[1]<<16) | ((uint32_t)d[2]<<8) | d[3]; }
// static inline int16_t  be16s(const uint8_t *d) { return (int16_t)((d[0] << 8) | d[1]); }
// static inline  int16_t  s16(uint16_t u)       { return (int16_t)u; }
// #endif
//
// extern volatile uint16_t g_lastSig;
// extern volatile uint8_t  g_lastTag;
//
// /* ================= AO definition ================= */
//
// typedef struct {
//     QActive  super;
//     QTimeEvt tick;          /* 10Hz internal tick */
//     BmsTelemetry snap;
//     uint8_t  have_any_data;
//     uint32_t tick10;        /* 10Hz ticks */
//     uint32_t last_rx_ticks; /* age tracking */
//     uint16_t pub_div;       /* downcounter for publish cadence */
// } BmsAO;
//
// static BmsAO l_bms;
// QActive *AO_Bms = &l_bms.super;
//
// /* ----- forward decls ----- */
// static QState Bms_initial(BmsAO *me, void const *par);
// static QState Bms_active (BmsAO *me, QEvt const *e);
//
// /* Public helper: called by simulator to publish a complete snapshot */
// void BMS_publish_telemetry(BmsTelemetry const *t) {
//     /* keep the AO snapshot current (used by GetSnapshot, etc.) */
//     l_bms.snap = *t;
//     l_bms.have_any_data = 1U;
//
//     BmsTelemetryEvt *be = Q_NEW(BmsTelemetryEvt, BMS_UPDATED_SIG);
//     be->data = *t;  /* copy */
//
//     /* post to Controller with margin=1; GC if queue is temporarily full */
//     if (!QACTIVE_POST_X(AO_Controller, &be->super, 1U, 0U)) {
//         QF_gc(&be->super);
//     }
// }
//
// static char const *bms_type_str(uint16_t code) {
//     switch (code) {
//         case 0x0600: return "600s";
//         case 0x0500: return "500s Hyperdrive";
//         case 0x0501: return "500s BMZ";
//         case 0x0400: return "400s Hyperdrive";
//         case 0x0401: return "400s Dual-Zone";
//         case 0x0402: return "400s Steatite";
//         default:     return "Unknown";
//     }
// }
//
// static void bms_maybe_log_type(uint16_t new_code) {
//     static uint16_t s_last_logged_code = 0;
//     if (new_code != 0 && new_code != s_last_logged_code) {
//         s_last_logged_code = new_code;
//         printf("BMS: detected battery type: %s (0x%04X)\r\n",
//                bms_type_str(new_code), new_code);
//     }
// }
//
//
// /* Return 1 if frame used/parsed (battery data seen), 0 otherwise. */
// int BMS_ParseFrame(CanFrameEvt const *f, BmsTelemetry *b)
// {
//     uint32_t id  = f->id;
//     uint8_t  dlc = f->dlc;
//     uint8_t const *d = f->data;
//
//     /* ===================== 600s family (Ocado 0x100000xx, extended) ===================== */
//     /* Keep existing mapping; if we see 0x10000010 later we will classify as BMZ 500s. */
//     if ((id & 0xFFFF0000u) == 0x10000000u) {
//         /* If we haven't chosen a 500s-BMZ subtype yet, default this extended block to 600s. */
//         if (b->battery_type_code == 0) {
//             /* Will be overridden to 0x0501 if we see 0x10000010 (BMZ) */
//             b->battery_type_code = 0x0600;
//             bms_maybe_log_type(b->battery_type_code);
//         }
//
//         switch (id) {
//             case 0x10000090u: /* Pack ID & FW (Serial + FW) */
//                 if (dlc >= 8) {
//                     b->serial_number    = be32(&d[0]);
//                     b->firmware_version = be32(&d[4]);
//                 }
//                 return 1;
//
//             case 0x10000110u: /* Temperatures (0.1°C) */
//                 if (dlc >= 4) {
//                     int16_t th = (int16_t)be16(&d[0]);
//                     int16_t tl = (int16_t)be16(&d[2]);
//                     b->sys_temp_high_C = (float)th * 0.1f;
//                     b->sys_temp_low_C  = (float)tl * 0.1f;
//                 }
//                 return 1;
//
//             case 0x10000020u: /* Charging params / SOC (%) */
//                 if (dlc >= 4) {
//                     b->soc_percent = d[3]; /* 0..100 or 0xFF unknown */
//                 }
//                 return 1;
//
//             default: ; /* fall-through to allow BMZ handling below */
//         }
//     }
//
//     /* ===================== 500s BMZ (extended 0x100000xx) ===================== */
//     /* Presence of 0x10000010 indicates the BMZ flavour of the extended scheme. */
//     if (id == 0x10000010u) { /* Pack status & power + battery type */
//         if (b->battery_type_code == 0 || b->battery_type_code == 0x0600) {
//             b->battery_type_code = 0x0501; /* 500s BMZ sub-type */
//             bms_maybe_log_type(b->battery_type_code);
//         }
//         if (dlc >= 8) {
//             /* V, I, state, fault, type (per doc) */
//             uint16_t v10 = be16(&d[0]); /* 0.1 V/bit */
//             int16_t  i10 = (int16_t)be16(&d[2]); /* 0.1 A/bit, sign: + = charging */
//             (void)i10; /* not currently exposed in BmsTelemetry; keep for future */
//
//             b->array_voltage_V = (float)v10 * 0.1f;
//             b->bms_state       = d[4];                 /* enum */
//             b->bms_fault       = (d[5] != 0U) ? 1U : 0U; /* any bit -> fault present */
//
//             /* d[6] is "Battery Type" enumeration in BMZ doc; optional to store */
//         }
//         return 1;
//     }
//     if ((id & 0xFFFF0000u) == 0x10000000u && b->battery_type_code == 0x0501) {
//         switch (id) {
//             case 0x10000020u: /* SOC etc. */
//                 if (dlc >= 4) b->soc_percent = d[3];
//                 return 1;
//
//             case 0x10000100u: /* Highest/Lowest cell mV */
//                 if (dlc >= 4) {
//                     b->high_cell_V = (float)be16(&d[0]) * 0.001f;
//                     b->low_cell_V  = (float)be16(&d[2]) * 0.001f;
//                 }
//                 return 1;
//
//             case 0x10000110u: /* Temps 0.1°C */
//                 if (dlc >= 4) {
//                     int16_t th = (int16_t)be16(&d[0]);
//                     int16_t tl = (int16_t)be16(&d[2]);
//                     b->sys_temp_high_C = (float)th * 0.1f;
//                     b->sys_temp_low_C  = (float)tl * 0.1f;
//                 }
//                 return 1;
//
//             case 0x10000090u: /* Serial + FW */
//                 if (dlc >= 8) {
//                     b->serial_number    = be32(&d[0]);
//                     b->firmware_version = be32(&d[4]);
//                 }
//                 return 1;
//
//             default: ;
//         }
//     }
//
//     /* ===================== 500s Hyperdrive (J1939 0x18FFxx00) ===================== */
//     if ((id & 0xFFFF00FFu) == 0x18FF0000u) {
//         if (b->battery_type_code == 0) {
//             b->battery_type_code = 0x0500;
//             bms_maybe_log_type(b->battery_type_code);
//         }
//
//         switch (id) {
//             case 0x18FF4000u: /* Serial + FW (32/32-bit) */
//                 if (dlc >= 8) {
//                     b->serial_number    = be32(&d[0]);
//                     b->firmware_version = be32(&d[4]);
//                 }
//                 return 1;
//
//             case 0x18FF0300u: /* State/fault */
//                 if (dlc >= 2) {
//                     b->bms_fault = d[0];
//                     b->bms_state = d[1];
//                 }
//                 return 1;
//
//             case 0x18FF0600u: /* High/Low cell mV */
//                 if (dlc >= 4) {
//                     b->high_cell_V = (float)be16(&d[0]) * 0.001f;
//                     b->low_cell_V  = (float)be16(&d[2]) * 0.001f;
//                 }
//                 return 1;
//
//             case 0x18FF0700u: /* Array/pack voltage 0.1 V/LSB */
//                 if (dlc >= 2) b->array_voltage_V = (float)be16(&d[0]) * 0.1f;
//                 return 1;
//
//             case 0x18FFE000u: /* SOC % */
//                 if (dlc >= 1) b->soc_percent = d[0];
//                 return 1;
//
//             case 0x18FF0800u: /* System temps 0.1°C/LSB */
//                 if (dlc >= 4) {
//                     b->sys_temp_high_C = (float)((int16_t)be16(&d[0])) * 0.1f;
//                     b->sys_temp_low_C  = (float)((int16_t)be16(&d[2])) * 0.1f;
//                 }
//                 return 1;
//
//             case 0x18FF5000u: /* Fan RPM */
//                 if (dlc >= 2) b->fan_rpm = be16(&d[0]);
//                 return 1;
//
//             case 0x18FF0E00u: /* Last error class/code */
//                 if (dlc >= 2) {
//                     b->last_error_class = d[0];
//                     b->last_error_code  = d[1];
//                 }
//                 return 1;
//
//             default: ;
//         }
//     }
//
//     /* ===================== 400s family (Custom Power: Dual-Zone / Steatite) ===================== */
//     /* External frames: MAIN/MASTER ...00 and SECONDARY/SLAVE ...01 variants */
//     if ( (id == 0x18070800u) || (id == 0x18060800u) || (id == 0x180C0800u)
//       || (id == 0x18000800u) || (id == 0x18010800u)
//       || (id == 0x18040A00u)
//       || ((id & 0xFFFFFF00u) == 0x18000800u) /* allow mask for ...00/01 sets if needed */
//       || ((id & 0xFFFFFF00u) == 0x18010800u) ) {
//
//         /* Base family code if unknown */
//         if (b->battery_type_code == 0) {
//             b->battery_type_code = 0x0400;
//             bms_maybe_log_type(b->battery_type_code);
//         }
//
//         /* If we ever see a SECONDARY/SLAVE frame (…01), mark as Dual-Zone sub-type (tentative). */
//         if ((id & 0x000000FFu) == 0x01u) {
//             if ((b->battery_type_code & 0xFF00u) == 0x0400u && b->battery_type_code != 0x0401) {
//                 /* 0x0401 -> Dual-Zone (tentative; may be overridden to 0x0402 by type field) */
//                     b->battery_type_code = 0x0401;
//                     bms_maybe_log_type(b->battery_type_code);
//             }
//         }
//
//
//         switch (id) {
//             case 0x18070800u: /* STATUS_1: total voltage + SOC (per docs) */
//                 if (dlc >= 7) {
//                     /* Your original mapping kept SOC in d[6] or d[5]. Keep behaviour. */
//                     b->soc_percent = d[6] ? d[6] : d[5];
//                 }
//                 return 1;
//
//             case 0x18060800u: /* STATUS_0: min/max cell V (overall) */
//                 if (dlc >= 4) {
//                     float vmax = (float)be16(&d[0]) * 0.001f;
//                     float vmin = (float)be16(&d[2]) * 0.001f;
//                     if (vmax > 0.1f && vmax > b->high_cell_V) b->high_cell_V = vmax;
//                     if (vmin > 0.1f && (b->low_cell_V == 0.0f || vmin < b->low_cell_V)) b->low_cell_V = vmin;
//                 }
//                 return 1;
//
//             case 0x180C0800u: /* TEMPERATURE_SUMMARY: max/min temps */
//                 if (dlc >= 4) {
//                     b->sys_temp_high_C = (float)((int8_t)d[0]);
//                     b->sys_temp_low_C  = (float)((int8_t)d[1]);
//                 }
//                 return 1;
//
//             case 0x18000800u: /* CELL_VOLTAGE_A: cells 1..4 */
//             case 0x18010800u: /* CELL_VOLTAGE_B: cells 5..6 + max/min maybe */
//             {
//                 /* Robust: accumulate hi/lo from available 16-bit mV fields */
//                 if (dlc >= 2) {
//                     int i;
//                     for (i = 0; i + 1 < dlc; i += 2) {
//                         float v = (float)be16(&d[i]) * 0.001f;
//                         if (v > 0.1f) {
//                             if (v > b->high_cell_V) b->high_cell_V = v;
//                             if (b->low_cell_V == 0.0f || v < b->low_cell_V) b->low_cell_V = v;
//                         }
//                     }
//                 }
//                 return 1;
//             }
//
//             case 0x18040A00u: /* PARAMETERS_4: UID, FW, Battery Type */
//                 /* We only need the Battery Type to differentiate Steatite vs others.
//                    Spec lists “Battery Type” in this frame; treat last two bytes as BE type. */
//                 if (dlc >= 2) {
//                     uint16_t type_be = (dlc >= 8) ? be16(&d[6]) : be16(&d[dlc - 2]);
//                     /* Steatite value appears as 0x0020 in the doc set; treat that as Steatite. */
//                     if (type_be == 0x0020u) {
//                         if (b->battery_type_code != 0x0402) {
//                             b->battery_type_code = 0x0402;   /* 400s Steatite */
//                             bms_maybe_log_type(b->battery_type_code);       // <--- add this
//                         }
//                     } else {
//                         if ((b->battery_type_code & 0xFF00u) != 0x0400u) {
//                             /* keep as-is (e.g., already 0x0401) */
//                         } else {
//                             if (b->battery_type_code != 0x0400) {
//                                 b->battery_type_code = 0x0400; /* default 400s Hyperdrive/Dual-Zone main */
//                                 bms_maybe_log_type(b->battery_type_code);   // <--- optional, if you want logs on revert
//                             }
//                         }
//                     }
//                 }
//                 return 1;
//
//             default: ;
//         }
//     }
//
//     /* ===================== 400s legacy (your original fallbacks) ===================== */
//     if ( (id == 0x18070800u) || (id == 0x18060800u) || (id == 0x180C0800u)
//       || ((id & 0xFFFFF0FFu) == 0x18004000u) ) {
//         if (b->battery_type_code == 0) {
//             b->battery_type_code = 0x0400;
//             bms_maybe_log_type(b->battery_type_code);
//         }
//
//         switch (id) {
//             case 0x18070800u:
//                 if (dlc >= 7) {
//                     uint8_t faultBit = (d[2] >> 6) & 0x01u;
//                     b->bms_fault  = faultBit ? 1U : 0U;
//                     b->bms_state  = d[3];
//                     b->soc_percent = d[6] ? d[6] : d[5];
//                 }
//                 return 1;
//
//             case 0x18060800u:
//                 if (dlc >= 4) {
//                     b->sys_temp_high_C = (float)((int8_t)d[2]);
//                     b->sys_temp_low_C  = (float)((int8_t)d[3]);
//                 }
//                 return 1;
//
//             case 0x180C0800u:
//                 if (dlc >= 3) {
//                     b->last_error_class = d[1];
//                     b->last_error_code  = d[2];
//                     if (b->last_error_class || b->last_error_code) b->bms_fault = 1U;
//                 }
//                 return 1;
//
//             default:
//                 if ((id & 0xFFFFF0FFu) == 0x18004000u && dlc >= 6) {
//                     float c1 = (float)be16(&d[0]) * 0.001f;
//                     float c2 = (float)be16(&d[2]) * 0.001f;
//                     float c3 = (float)be16(&d[4]) * 0.001f;
//                     if (c1 > 0.1f) { if (b->high_cell_V < c1) b->high_cell_V = c1; if (b->low_cell_V == 0 || b->low_cell_V > c1) b->low_cell_V = c1; }
//                     if (c2 > 0.1f) { if (b->high_cell_V < c2) b->high_cell_V = c2; if (b->low_cell_V == 0 || b->low_cell_V > c2) b->low_cell_V = c2; }
//                     if (c3 > 0.1f) { if (b->high_cell_V < c3) b->high_cell_V = c3; if (b->low_cell_V == 0 || b->low_cell_V > c3) b->low_cell_V = c3; }
//                     return 1;
//                 }
//                 break;
//         }
//     }
//
//     return 0;
// }
//
// // int BMS_ParseFrame(CanFrameEvt const *f, BmsTelemetry *b)
// // {
// //     uint32_t id = f->id;
// //     uint8_t  dlc = f->dlc;
// //     uint8_t const *d = f->data;
// //
// //     /* -------- 600s family (Ocado “0x100000xx” range, extended) -------- */
// //     if ((id & 0xFFFF0000u) == 0x10000000u) {
// //         if (b->battery_type_code == 0) b->battery_type_code = 0x0600;
// //
// //         switch (id) {
// //             case 0x10000090: /* Pack ID & FW (SN + FW) */
// //                 if (dlc >= 6) {
// //                     b->serial_number    = be32(&d[0]);
// //                     b->firmware_version = be16(&d[4]);
// //                 }
// //                 return 1;
// //
// //             case 0x10000091: /* Battery part/definition (type) */
// //                 return 1;
// //
// //             case 0x10000110: /* Temperatures (high/low) */
// //                 if (dlc >= 2) {
// //                     b->sys_temp_high_C = (float)((int8_t)d[0]);
// //                     b->sys_temp_low_C  = (dlc >= 2) ? (float)((int8_t)d[1]) : b->sys_temp_low_C;
// //                 }
// //                 return 1;
// //
// //             case 0x10000020: /* Charging params / App SOC, etc. */
// //                 if (dlc >= 4) {
// //                     b->soc_percent = d[3]; /* 0..100 */
// //                 }
// //                 return 1;
// //             default: ;
// //         }
// //     }
// //
// //     /* -------- 500s family (0x18FFxx00 J1939-style PGNs) -------- */
// //     if ((id & 0xFFFF00FFu) == 0x18FF0000u) {
// //         if (b->battery_type_code == 0) b->battery_type_code = 0x0500;
// //
// //         switch (id) {
// //             case 0x18FF4000: /* Serial + FW */
// //                 if (dlc >= 8) {
// //                     b->serial_number    = be32(&d[0]);
// //                     b->firmware_version = be32(&d[4]);
// //                 }
// //                 return 1;
// //
// //             case 0x18FF0300: /* State/fault */
// //                 if (dlc >= 2) {
// //                     b->bms_fault = d[0];
// //                     b->bms_state = d[1];
// //                 }
// //                 return 1;
// //
// //             case 0x18FF0600: /* High/Low cell mV */
// //                 if (dlc >= 4) {
// //                     b->high_cell_V = (float)be16(&d[0]) * 0.001f;
// //                     b->low_cell_V  = (float)be16(&d[2]) * 0.001f;
// //                 }
// //                 return 1;
// //
// //             case 0x18FF0700: /* Array/pack voltage 0.1 V/LSB */
// //                 if (dlc >= 2) b->array_voltage_V = (float)be16(&d[0]) * 0.1f;
// //                 return 1;
// //
// //             case 0x18FFE000: /* SOC % */
// //                 if (dlc >= 1) b->soc_percent = d[0];
// //                 return 1;
// //
// //             case 0x18FF0800: /* System temps 0.1°C/LSB */
// //                 if (dlc >= 4) {
// //                     b->sys_temp_high_C = (float)((int16_t)be16(&d[0])) * 0.1f;
// //                     b->sys_temp_low_C  = (float)((int16_t)be16(&d[2])) * 0.1f;
// //                 }
// //                 return 1;
// //
// //             case 0x18FF5000: /* Fan RPM (BE) */
// //                 if (dlc >= 2) b->fan_rpm = be16(&d[0]);
// //                 return 1;
// //
// //             case 0x18FF0E00: /* Last error class/code */
// //                 if (dlc >= 2) {
// //                     b->last_error_class = d[0];
// //                     b->last_error_code  = d[1];
// //                 }
// //                 return 1;
// //             default: ;
// //         }
// //     }
// //
// //     /* -------- 400s family -------- */
// //     if ( (id == 0x18070800u) || (id == 0x18060800u) || (id == 0x180C0800u)
// //       || ((id & 0xFFFFF0FFu) == 0x18004000u) ) { /* 0x18040Axx */
// //         if (b->battery_type_code == 0) b->battery_type_code = 0x0400;
// //
// //         switch (id) {
// //             case 0x18070800: /* Status / Fault / SOC */
// //                 if (dlc >= 7) {
// //                     uint8_t faultBit = (d[2] >> 6) & 0x01u;
// //                     b->bms_fault = faultBit ? 1U : 0U;
// //                     b->bms_state = d[3];
// //                     b->soc_percent = d[6] ? d[6] : d[5];
// //                 }
// //                 return 1;
// //
// //             case 0x18060800: /* Highest/Lowest temperature */
// //                 if (dlc >= 4) {
// //                     b->sys_temp_high_C = (float)((int8_t)d[2]);
// //                     b->sys_temp_low_C  = (float)((int8_t)d[3]);
// //                 }
// //                 return 1;
// //
// //             case 0x180C0800: /* Last error class/code */
// //                 if (dlc >= 3) {
// //                     b->last_error_class = d[1];
// //                     b->last_error_code  = d[2];
// //                     if (b->last_error_class || b->last_error_code) b->bms_fault = 1U;
// //                 }
// //                 return 1;
// //
// //             default:
// //                 if ((id & 0xFFFFF0FFu) == 0x18004000u && dlc >= 6) {
// //                     float c1 = (float)be16(&d[0]) * 0.001f;
// //                     float c2 = (float)be16(&d[2]) * 0.001f;
// //                     float c3 = (float)be16(&d[4]) * 0.001f;
// //                     if (c1 > 0.1f) { if (b->high_cell_V < c1) b->high_cell_V = c1; if (b->low_cell_V == 0 || b->low_cell_V > c1) b->low_cell_V = c1; }
// //                     if (c2 > 0.1f) { if (b->high_cell_V < c2) b->high_cell_V = c2; if (b->low_cell_V == 0 || b->low_cell_V > c2) b->low_cell_V = c2; }
// //                     if (c3 > 0.1f) { if (b->high_cell_V < c3) b->high_cell_V = c3; if (b->low_cell_V == 0 || b->low_cell_V > c3) b->low_cell_V = c3; }
// //                     return 1;
// //                 }
// //                 break;
// //         }
// //     }
// //
// //     return 0;
// // }
//
// void BmsAO_ctor(void) {
//     QActive_ctor(&l_bms.super, Q_STATE_CAST(&Bms_initial));
//     QTimeEvt_ctorX(&l_bms.tick, &l_bms.super, BMS_TICK_SIG, 0U);
// }
//
// static QState Bms_initial(BmsAO * const me, void const * const par) {
//     (void)par;
//     memset(&me->snap, 0, sizeof(me->snap));
//     me->have_any_data = 0U;
//     me->tick10        = 0U;
//     me->last_rx_ticks = 0U;
//     me->pub_div       = 0U;
//
//     QActive_subscribe(&me->super, CAN_RX_SIG);
//
//     /* 10Hz internal tick */
//     printf("BMS: initial, arming tick\r\n");
//     QTimeEvt_armX(&me->tick, BSP_TICKS_PER_SEC/10U, BSP_TICKS_PER_SEC/10U);
//     printf("BMS: initial, armed tick\r\n");
//     return Q_TRAN(&Bms_active);
// }
//
// static QState Bms_active(BmsAO * const me, QEvt const * const e) {
//     switch (e->sig) {
//     case CAN_RX_SIG: {
//         CanFrameEvt const *ce = Q_EVT_CAST(CanFrameEvt);
//         if (BMS_ParseFrame(ce, &me->snap)) {
//             printf("BMS: frame parsed (id=0x%lX, ext=%u, dlc=%u)\r\n",
//                    (unsigned long)ce->id, ce->isExt, ce->dlc);
//             me->have_any_data = 1U;
//             me->last_rx_ticks = me->tick10;
//         }
//         return Q_HANDLED();
//     }
//
//     case BMS_TICK_SIG: {
//         me->tick10++;
//
//         /* ===== Publish cadence: 2 Hz (every 0.5 s) =====
//            10 Hz tick -> publish every 5 ticks */
//         if (++me->pub_div >= 5U) {
//             me->pub_div = 0U;
//
//             if (me->have_any_data) {
//                 BmsTelemetryEvt *be = Q_NEW(BmsTelemetryEvt, BMS_UPDATED_SIG);
//                 be->data = me->snap;
//                 /* point-to-point to Controller */
//                 if (!QACTIVE_POST_X(AO_Controller, &be->super, 1U, &me->super)) {
//                     QF_gc(&be->super);
//                 }
//             } else {
//                 (void)QACTIVE_POST_X(AO_Controller,
//                                      Q_NEW(QEvt, BMS_NO_BATTERY_SIG), 1U, &me->super);
//             }
//         }
//         return Q_HANDLED();
//     }
//
//     default: ;
//     }
//     return Q_SUPER(&QHsm_top);
// }
//
// void BMS_GetSnapshot(BmsTelemetry *dst) {
//     QF_CRIT_STAT;
//     QF_CRIT_ENTRY();
//     *dst = l_bms.snap;
//     QF_CRIT_EXIT();
// }
