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

Q_DEFINE_THIS_FILE

#ifndef BSP_TICKS_PER_SEC
#define BSP_TICKS_PER_SEC 100U
#endif
#define TICKS_2S (2U*BSP_TICKS_PER_SEC)

#ifndef BMS_ENDIAN_HELPERS
#define BMS_ENDIAN_HELPERS
static inline uint16_t be16(const uint8_t *d) { return (uint16_t)((((uint16_t)d[0])<<8) | d[1]); }
static inline uint32_t be32(const uint8_t *d) { return  ((uint32_t)d[0]<<24) | ((uint32_t)d[1]<<16) | ((uint32_t)d[2]<<8) | d[3]; }
static inline  int16_t  s16(uint16_t u)       { return (int16_t)u; }
#endif

extern volatile uint16_t g_lastSig;
extern volatile uint8_t  g_lastTag;

typedef struct {
    QActive  super;
    QTimeEvt tick;          /* 10Hz internal tick */
    BmsTelemetry snap;
    uint8_t  have_any_data;
    uint32_t tick10;
    uint32_t last_rx_ticks; /* age tracking */
    uint16_t pub2s_div;     /* count 10Hz ticks to 2s */
} BmsAO;

static BmsAO l_bms;
QActive *AO_Bms = &l_bms.super;

/* forward decl */
static QState Bms_initial(BmsAO *me, void const *par);
static QState Bms_active (BmsAO *me, QEvt const *e);

/* Return 1 if frame used/parsed (battery data seen), 0 otherwise. */
int BMS_ParseFrame(CanFrameEvt const *f, BmsTelemetry *b)
{
    uint32_t id = f->id;
    uint8_t  dlc = f->dlc;
    uint8_t const *d = f->data;

    /* -------- 600s family (Ocado “0x100000xx” range, extended) -------- */
    if ((id & 0xFFFF0000u) == 0x10000000u) {
        if (b->battery_type_code == 0) b->battery_type_code = 0x0600;

        switch (id) {
            case 0x10000090: /* Pack ID & FW (SN + FW) */
                if (dlc >= 6) {
                    b->serial_number    = be32(&d[0]);
                    b->firmware_version = be16(&d[4]);
                }
                return 1;

            case 0x10000091: /* Battery part/definition (type) */
                /* nothing numeric to store in our normalized view;
                   the mere presence confirms type */
                return 1;

            case 0x10000110: /* Temperatures (high/low) */
                if (dlc >= 2) {
                    /* docs show degree-C integer bytes; clamp to sane range */
                    b->sys_temp_high_C = (float)((int8_t)d[0]);
                    b->sys_temp_low_C  = (dlc >= 2) ? (float)((int8_t)d[1]) : b->sys_temp_low_C;
                }
                return 1;

            case 0x10000020: /* Charging params / App SOC, etc. */
                if (dlc >= 4) {
                    /* app SOC is byte[3] per pack sheet; scale 0..100 */
                    b->soc_percent = d[3];
                }
                /* some implementations also place current here (not standardized);
                   leave current/voltage untouched unless clearly documented. */
                return 1;
            default: ;

                /* Optional: if you later confirm pack voltage/current IDs for 600s,
               add them here and set b->array_voltage_V / b->current_dA */
        }
    }

    /* -------- 500s family (0x18FFxx00 J1939-style PGNs) -------- */
    if ((id & 0xFFFF00FFu) == 0x18FF0000u) {
        if (b->battery_type_code == 0) b->battery_type_code = 0x0500;

        switch (id) {
            case 0x18FF4000: /* Serial + FW */
                if (dlc >= 8) {
                    b->serial_number    = be32(&d[0]);
                    b->firmware_version = be32(&d[4]); /* treat as 32-bit build id if 2 bytes aren’t documented */
                }
                return 1;

            case 0x18FF0300: /* State/fault */
                if (dlc >= 2) {
                    b->bms_fault = d[0];   /* bitfield (pack document) */
                    b->bms_state = d[1];   /* 0 sleep, 1 standby, 2 run, 4 fault, etc. */
                }
                return 1;

            case 0x18FF0600: /* High/Low cell mV */
                if (dlc >= 4) {
                    b->high_cell_V = (float)be16(&d[0]) * 0.001f;
                    b->low_cell_V  = (float)be16(&d[2]) * 0.001f;
                }
                return 1;

            case 0x18FF0700: /* Array/pack voltage 0.1 V/LSB */
                if (dlc >= 2) {
                    b->array_voltage_V = (float)be16(&d[0]) * 0.1f;
                }
                return 1;

            case 0x18FFE000: /* SOC % */
                if (dlc >= 1) {
                    b->soc_percent = d[0];
                }
                return 1;

            case 0x18FF0800: /* System temps 0.1°C/LSB */
                if (dlc >= 4) {
                    b->sys_temp_high_C = (float)((int16_t)be16(&d[0])) * 0.1f;
                    b->sys_temp_low_C  = (float)((int16_t)be16(&d[2])) * 0.1f;
                }
                return 1;

            case 0x18FF5000: /* Fan RPM (BE) */
                if (dlc >= 2) {
                    b->fan_rpm = be16(&d[0]);
                }
                return 1;

            case 0x18FF0E00: /* Last error class/code */
                if (dlc >= 2) {
                    b->last_error_class = d[0];
                    b->last_error_code  = d[1];
                }
                return 1;
            default: ;

                /* If your 500s current PGN is known, set b->current_dA here */
        }
    }

    /* -------- 400s family (0x1807/0x1806/0x1804/0x180C) -------- */
    /* 0x18070800 : status/SoC/fault; 0x18060800 : temps; 0x18040Axx : cell Vs; 0x180C0800 : error Cx/code */
    if ( (id == 0x18070800u) || (id == 0x18060800u) || (id == 0x180C0800u)
      || ((id & 0xFFFFF0FFu) == 0x18004000u) ) { /* 0x18040Axx (A=0..6) -> 0x18004?00 with mask */
        if (b->battery_type_code == 0) b->battery_type_code = 0x0400;

        switch (id) {
            case 0x18070800: /* Status / Fault / SOC */
                /* From doc: B2 bit6=Pack Fault, B3=State, B5..B6=SOC (doc shows two bytes; many packs use 1 byte %) */
                if (dlc >= 7) {
                    uint8_t faultBit = (d[2] >> 6) & 0x01u;
                    b->bms_fault = faultBit ? 1U : 0U;
                    b->bms_state = d[3];                 /* meaning per table */
                    /* prefer byte 6 if it holds %; fall back to 5 */
                    b->soc_percent = d[6] ? d[6] : d[5];
                }
                return 1;

            case 0x18060800: /* Highest/Lowest temperature */
                if (dlc >= 4) {
                    /* Doc shows “Highest Temperature” and “Lowest Temperature” (integer °C or 0.1°C in some revs).
                       Most 400s I’ve seen report whole °C here. */
                    b->sys_temp_high_C = (float)((int8_t)d[2]);
                    b->sys_temp_low_C  = (float)((int8_t)d[3]);
                }
                return 1;

            case 0x180C0800: /* Last error class/code */
                if (dlc >= 3) {
                    b->last_error_class = d[1];  /* C1/C2/C3 */
                    b->last_error_code  = d[2];
                    if (b->last_error_class || b->last_error_code) b->bms_fault = 1U;
                }
                return 1;

            default:
                /* 0x18040Axx : cell voltages in mV, 3 cells/frame @ 16-bit each (big-endian) */
                if ((id & 0xFFFFF0FFu) == 0x18004000u && dlc >= 6) {
                    float c1 = (float)be16(&d[0]) * 0.001f;
                    float c2 = (float)be16(&d[2]) * 0.001f;
                    float c3 = (float)be16(&d[4]) * 0.001f;
                    /* track hi/lo from reported cells */
                    if (c1 > 0.1f) { if (b->high_cell_V < c1) b->high_cell_V = c1; if (b->low_cell_V == 0 || b->low_cell_V > c1) b->low_cell_V = c1; }
                    if (c2 > 0.1f) { if (b->high_cell_V < c2) b->high_cell_V = c2; if (b->low_cell_V == 0 || b->low_cell_V > c2) b->low_cell_V = c2; }
                    if (c3 > 0.1f) { if (b->high_cell_V < c3) b->high_cell_V = c3; if (b->low_cell_V == 0 || b->low_cell_V > c3) b->low_cell_V = c3; }
                    return 1;
                }
                break;
        }
    }

    /* Unrecognized frame */
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
    me->pub2s_div     = 0U;

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
            me->last_rx_ticks = me->tick10; /* coarse tick timebase is fine */
        }
        return Q_HANDLED();
    }

        case BMS_TICK_SIG: {
        me->tick10++;

        // 2 s cadence
        if (++me->pub2s_div >= 20U) {
            me->pub2s_div = 0U;

            if (me->have_any_data) {
                BmsTelemetryEvt *be = Q_NEW(BmsTelemetryEvt, BMS_UPDATED_SIG);
                be->data = me->snap;
                printf("BMS: publish update (V=%.1f)\r\n", me->snap.array_voltage_V);

                // point-to-point to Controller (not QF_PUBLISH)
                if (!QACTIVE_POST_X(AO_Controller, &be->super, 0U, &me->super)) {
                    QF_gc(&be->super);       // drop cleanly if queue is momentarily full
                }
            } else {
                (void)QACTIVE_POST_X(AO_Controller,
                                     Q_NEW(QEvt, BMS_NO_BATTERY_SIG), 0U, &me->super);
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
