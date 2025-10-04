#include "qpc_cfg.h"
#include "qpc.h"
#include "ao_controller.h"
#include "ao_nextion.h"
#include "ao_cotek.h"
#include "bms_app.h"
#include "app_signals.h"
#include "bsp.h"
#include <stdio.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include <math.h>

static uint32_t s_last_sum_ms;
static uint32_t s_last_det_ms;
static uint32_t s_last_sum_hash, s_last_det_hash;

Q_DEFINE_THIS_FILE
typedef enum
{
    CTL_STATE_WAIT=0,
    CTL_STATE_DETECT,
    CTL_STATE_CHARGE
} ctl_state_t;


typedef struct {
    QActive  super;
    QTimeEvt ui2s;      /* periodic UI refresh (2s) */
    QTimeEvt tCharge;   /* charging timeout (60s) */
    QTimeEvt tPsuOff;    // short watchdog while waiting for OFF confirm
    uint8_t page;
    BmsTelemetry last;
    uint8_t      haveData;
#ifdef ENABLE_BMS_SIM
    QTimeEvt simTick;
#endif
    ctl_state_t state;
} ControllerAO;

static void post_page_ex(ControllerAO *me, uint8_t page);
static void make_summary(NextionSummaryEvt *se, const BmsTelemetry *t);
extern volatile uint16_t g_lastSig;
extern volatile uint8_t  g_lastTag;
//static uint32_t s_last_ui_ms;


static QState Ctl_initial (ControllerAO *me, void const *e);
static QState Ctl_run     (ControllerAO *me, QEvt const *e);
static QState Ctl_wait    (ControllerAO *me, QEvt const *e);
static QState Ctl_detect  (ControllerAO *me, QEvt const *e);
static QState Ctl_charge  (ControllerAO *me, QEvt const *e);
static QState Ctl_poweringDown(ControllerAO * me, QEvt const * e);
static ControllerAO l_ctl;
QActive *AO_Controller = &l_ctl.super;

// quantizers (avoid UI spam from tiny jitter)
static inline int qV005(float v) {     // 0.05 V steps
    return (int)lrintf(v * 20.0f);
}
static inline int qT1(float t) {       // 1 °C steps
    return (int)lrintf(t);
}
static inline int qA01_dA(int16_t dA) {// 0.1 A steps (your current is deci-amps)
    return (int)(dA / 1);              // already deci-amps; keep as-is
}
static inline uint32_t rotl32(uint32_t x, int r){ return (x<<r)|(x>>(32-r)); }

static uint32_t hash_summary(const BmsTelemetry *t, bool charging, char const *reason) {
    uint32_t h = 0x9E3779B9u;
    h ^= (uint32_t)t->battery_type_code; h = rotl32(h, 7);
    h ^= (uint32_t)qV005(t->array_voltage_V); h = rotl32(h, 7);
    h ^= (uint32_t)t->bms_state;         h = rotl32(h, 7);
    h ^= (uint32_t)t->bms_fault;         h = rotl32(h, 7);
    h ^= (uint32_t)t->soc_percent;       h = rotl32(h, 7);
    h ^= (uint32_t)(charging ? 1u : 0u); h = rotl32(h, 7);
    if (reason && reason[0]) {
        for (char const *p = reason; *p; ++p) { h ^= (uint8_t)*p; h = rotl32(h, 5); }
    }
    return h;
}

static uint32_t hash_details(const BmsTelemetry *t) {
    uint32_t h = 0x85EBCA6Bu;
    h ^= (uint32_t)qV005(t->array_voltage_V); h = rotl32(h, 7);
    h ^= (uint32_t)qV005(t->high_cell_V);     h = rotl32(h, 7);
    h ^= (uint32_t)qV005(t->low_cell_V);      h = rotl32(h, 7);
    h ^= (uint32_t)qT1(t->sys_temp_high_C);   h = rotl32(h, 7);
    h ^= (uint32_t)qT1(t->sys_temp_low_C);    h = rotl32(h, 7);
    h ^= (uint32_t)t->fan_rpm;                h = rotl32(h, 7);
    h ^= (uint32_t)t->soc_percent;            h = rotl32(h, 7);
    h ^= (uint32_t)t->bms_state;              h = rotl32(h, 7);
    h ^= (uint32_t)t->bms_fault;              h = rotl32(h, 7);
    h ^= (uint32_t)t->last_error_class;       h = rotl32(h, 7);
    h ^= (uint32_t)t->last_error_code;        h = rotl32(h, 7);
    h ^= (uint32_t)t->battery_type_code;      h = rotl32(h, 7);
    h ^= (uint32_t)qA01_dA(t->current_dA);    h = rotl32(h, 7);
    h ^= (uint32_t)(t->serial_number ^ t->firmware_version);
    return h;
}

static inline bool ui_ok_now_sum(void) {
    uint32_t now = HAL_GetTick();
    if ((now - s_last_sum_ms) < 120U) return false;  // ~8 Hz max
    s_last_sum_ms = now; return true;
}
static inline bool ui_ok_now_det(void) {
    uint32_t now = HAL_GetTick();
    if ((now - s_last_det_ms) < 250U) return false;  // ~4 Hz max
    s_last_det_ms = now; return true;
}
static char const *bms_state_str(uint16_t st) {
    switch (st) {
    case 0:  return "Idle";
    case 1:  return "Charge";
    case 2:  return "Discharge";
        // ... add your real states here ...
    default: return "Unknown";
    }
}
static void make_summary(NextionSummaryEvt *se, const BmsTelemetry *t) {
    // pack voltage
    se->packV = t->array_voltage_V;

    // battery type string + color (your mapping)
    switch (t->battery_type_code) {
    case 0x0400: strcpy(se->battTypeStr, "400s");   se->typeColor565 = 2016;  break;   // green
    case 0x0500: strcpy(se->battTypeStr, "500s");   se->typeColor565 = 65504; break;   // yellow
    case 0x0600: strcpy(se->battTypeStr, "600s");   se->typeColor565 = 1023;  break;   // blue
    default:     strcpy(se->battTypeStr, "Unknown");se->typeColor565 = 63488; break;   // red
    }

    // status text
    strcpy(se->statusStr, bms_state_str(t->bms_state));

    // errors/warnings (you only have bms_fault bitfield right now)
    se->errors[0]   = '\0';
    se->warnIcon    = (t->bms_fault != 0U) ? 1U : 0U;
    se->recoverable = (t->bms_fault == 0U) ? 1U : 0U;         // you don't have a recoverable bit yet
    se->charging    = 0U;         // caller sets this if needed
    se->statusColor565 = 0U;      // leave 0 if you don’t tint the status label
    se->reason[0] = '\0';
}

static void make_details(NextionDetailsEvt *de, const BmsTelemetry *t) {
    // Voltages
    de->high_voltage_V = t->high_cell_V;
    de->low_voltage_V  = t->low_cell_V;
    // For "avg", we don't have per-cell average; use array voltage as a coarse overall indicator
    de->avg_voltage_V  = t->array_voltage_V;

    // Temps
    de->high_temp_C      = t->sys_temp_high_C;
    de->low_temp_C       = t->sys_temp_low_C;
    de->pack_high_temp_C = t->sys_temp_high_C; // you don't have separate pack temps
    de->pack_low_temp_C  = t->sys_temp_low_C;

    // Serial, FW (you only have a 32-bit firmware_version, not major/minor/patch)
    snprintf(de->serial_number, sizeof(de->serial_number), "%lu", (unsigned long)t->serial_number);
    snprintf(de->firmware, sizeof(de->firmware), "%lu", (unsigned long)t->firmware_version);

    // Fan + SoC
    de->fan_speed_rpm = t->fan_rpm;
    de->soc_percent   = t->soc_percent;
    de->soc2_percent  = t->soc_percent; // you don't have a second SoC; mirror main SoC

    // State + Fault text
    strcpy(de->bms_state_str, bms_state_str(t->bms_state));
    if (t->bms_fault == 0U) {
        strcpy(de->bms_fault_str, "None");
    } else {
        snprintf(de->bms_fault_str, sizeof(de->bms_fault_str), "0x%02X", t->bms_fault);
    }
    printf("CTL: posting details to HMI\n");
}

/* Build & send compact summary only if it changed  */
static void post_summary(ControllerAO *me, bool charging, char const *reason) {
    if (!ui_ok_now_sum()) return;

    uint32_t h = hash_summary(&me->last, charging, reason);
    if (h == s_last_sum_hash) return;
    s_last_sum_hash = h;

    NextionSummaryEvt *se = Q_NEW(NextionSummaryEvt, NEX_REQ_UPDATE_SUMMARY_SIG);
    make_summary(se, &me->last);
    se->charging = charging ? 1U : 0U;
    if (reason && reason[0]) {
        strncpy(se->reason, reason, sizeof(se->reason)-1);
        se->reason[sizeof(se->reason)-1] = '\0';
    } else {
        se->reason[0] = '\0';
    }
    // UI is non-critical → use margin=1 and GC if it can’t be posted right now
    if (!QACTIVE_POST_X(AO_Nextion, &se->super, QF_NO_MARGIN, &me->super)) {
        QF_gc(&se->super);
    }
}

static void post_details(ControllerAO *me) {
    if (!ui_ok_now_det()) return;

    uint32_t h = hash_details(&me->last);
    if (h == s_last_det_hash) return;
    s_last_det_hash = h;

    NextionDetailsEvt *de = Q_NEW(NextionDetailsEvt, NEX_REQ_UPDATE_DETAILS_SIG);
    make_details(de, &me->last);
    if (!QACTIVE_POST_X(AO_Nextion, &de->super, QF_NO_MARGIN, &me->super)) {
        QF_gc(&de->super);
    }
}

// --- HMI: PSU widget helper (same style as post_summary/post_details) ---
static void post_psu_to_hmi(uint8_t present, uint8_t output_on,
                            float v_out, float i_out, float temp_C) {
    NextionPsuEvt *pe = Q_NEW(NextionPsuEvt, NEX_REQ_UPDATE_PSU_SIG);
    pe->present   = present;
    pe->output_on = output_on;   // matches NextionPsuEvt field name
    pe->v_out     = v_out;
    pe->i_out     = i_out;
    pe->temp_C    = temp_C;

    if (!QACTIVE_POST_X(AO_Nextion, &pe->super, QF_NO_MARGIN, 0U)) {
        QF_gc(&pe->super);
    }
}

static bool in_charge;

static void post_page_ex(ControllerAO *me, uint8_t page) {
    // me->page = page;                    // keep in sync
    // NextionPageEvt *pg = Q_NEW(NextionPageEvt, NEX_REQ_SHOW_PAGE_SIG);
    // pg->page = page;
    // if (!QACTIVE_POST_X(AO_Nextion, &pg->super, QF_NO_MARGIN, 0U)) {
    //     QF_gc(&pg->super);
    // }
    // s_last_sum_hash = 0U;               // force repaint
    // s_last_det_hash = 0U;
    // keep our own notion of the current page in sync
    me->page = page;

    // tell Nextion to change page
    NextionPageEvt *pg = Q_NEW(NextionPageEvt, NEX_REQ_SHOW_PAGE_SIG);
    pg->page = page;
    if (!QACTIVE_POST_X(AO_Nextion, &pg->super, QF_NO_MARGIN, 0U)) {
        QF_gc(&pg->super);
    }

    // force next UI publish to repaint (reset de-dupe hashes)
    s_last_sum_hash = 0U;
    s_last_det_hash = 0U;

    // repaint immediately if we already have data
    if (me->haveData) {
        if (page == 2U) {               // pMain
            post_summary(me, (me->state == CTL_STATE_CHARGE || me->state==CTL_STATE_DETECT), NULL);
        } else if (page == 3U) {        // pDetails
            post_details(me);
        }
    }
}

/* ctor */
void ControllerAO_ctor(void) {
#ifdef ENABLE_BMS_SIM
    QTimeEvt_ctorX(&l_ctl.simTick, &l_ctl.super, SIM_TICK_SIG, 0U);
#endif
    QActive_ctor(&l_ctl.super, Q_STATE_CAST(&Ctl_initial));
    QTimeEvt_ctorX(&l_ctl.ui2s,   &l_ctl.super, TIMEOUT_SIG, 0U);
    QTimeEvt_ctorX(&l_ctl.tCharge, &l_ctl.super, CHARGE_TIMEOUT_SIG, 0U);
    QTimeEvt_ctorX(&l_ctl.tPsuOff, &l_ctl.super, PSU_OFF_WAIT_TO_SIG, 0U);
}

/* states */
static QState Ctl_initial(ControllerAO * const me, void const *const e) {
    (void)e;
    me->page     = 1U;   // start at pWait after splash
    me->haveData = 0U;
    memset(&me->last, 0, sizeof(me->last));

    /* subscribe AFTER we’re started */
    QActive_subscribe(&me->super, BMS_UPDATED_SIG);
    QActive_subscribe(&me->super, BMS_NO_BATTERY_SIG);
    QActive_subscribe(&me->super, BMS_CONN_LOST_SIG);

    /* small splash delay then go RUN */
    //QTimeEvt_armX(&me->ui2s, BSP_TICKS_PER_SEC/2U, 0U);
#ifdef ENABLE_BMS_SIM
    // every 500 ms (adjust as you like)
    QTimeEvt_armX(&me->simTick, BSP_TICKS_PER_SEC/2, BSP_TICKS_PER_SEC/2);
#endif
    return Q_TRAN(&Ctl_run);
}

static QState Ctl_run(ControllerAO * const me, QEvt const * const e) {
    switch (e->sig) {
#ifdef ENABLE_BMS_SIM
    case SIM_TICK_SIG: {
            BmsSim_tick();        // generates & posts a BMS_UPDATED_SIG with plausible data
            return Q_HANDLED();
    }
#endif
    case Q_INIT_SIG: {
        /* show wait screen to start with */
        //post_page(1); /* pWait */
        return Q_TRAN(&Ctl_wait);
    }
    case Q_ENTRY_SIG: {
            // Do entry-only side effects here (no transitions!)
            // e.g. show a page if you want, but DON'T return Q_TRAN from ENTRY
            // post_page(1);  // optional
            return Q_HANDLED();
    }
    case NEX_READY_SIG: {
        // Nextion finished its own init/splash; decide first page:
        if (me->haveData) {
            post_page_ex(me, 2);                     // pMain
            post_summary(me, false, "ready to charge");
            post_details(me);
            printf("CTL: NEX_READY\r\n");
            return Q_TRAN(&Ctl_detect);
        } else {
            post_page_ex(me, 1);                     // pWait
            return Q_TRAN(&Ctl_wait);
        }
    }

    case BMS_UPDATED_SIG: {
            BmsTelemetryEvt const *be = Q_EVT_CAST(BmsTelemetryEvt);
            me->haveData = 1U;
            me->last     = be->data;

            // page transition like you have
            if (me->page == 1U) { // pWait -> pMain
                me->page = 2U;
                post_page_ex(me, 2U);
            }

            // Always refresh pMain summary when on pMain
            if (me->page == 2U) {
                post_summary(me, /*charging?*/ false, "BMS updated");
                post_details(me);
            }

            return Q_HANDLED();
    }
    case TIMEOUT_SIG: {
            if (me->page == 3U) {      // pDetails
                    post_details(me);
            } else if (me->page == 2U) {
                    post_summary(me, false, 0);
            }
            //QTimeEvt_armX(&me->ui2s, BSP_TICKS_PER_SEC/2U, 0U);
            return Q_HANDLED();
    }
    case BUTTON_PRESSED_SIG: {
        /* Only act if we have a battery detected */
        if (me->haveData) {
            return Q_TRAN(&Ctl_charge);   /* logic lives in the charge state's entry */
        }
        return Q_HANDLED();
        }
        case NEX_REQ_SHOW_PAGE_SIG: {  // coming FROM Nextion via Nextion_OnRx()
        NextionPageEvt const *pe = (NextionPageEvt const*)e;
        me->page = pe->page;
        s_last_sum_hash = 0U; s_last_det_hash = 0U;   // force repaint
        if (me->haveData) {
            if (me->page == 2U) post_summary(me, (me->state==CTL_STATE_CHARGE || me->state == CTL_STATE_DETECT), "");
            if (me->page == 3U) post_details(me);
        }
        return Q_HANDLED();
        }
    default: {
            // crude signal tracer to prove path; remove after debugging
            printf("CTL(run): sig=%u (page=%u, haveData=%u, state=%s)\r\n",
                   (unsigned)e->sig, (unsigned)me->page, (unsigned)me->haveData,
                   (me->state == CTL_STATE_DETECT) ? "detect" :
                   (me->state == CTL_STATE_CHARGE) ? "charge" : "wait");
            break;
    }
    }
    return Q_SUPER(&QHsm_top);
}

/* -------- WAIT FOR BATTERY -------- */
static QState Ctl_wait(ControllerAO * const me, QEvt const * const e) {
    switch (e->sig) {
    // case Q_ENTRY_SIG:
    //     {
    //         //printf("Ctl_wait: entry\r\n");
    //         //me->state = CTL_STATE_WAIT;
    //     }
    case BMS_UPDATED_SIG: {
            BmsTelemetryEvt const *be = Q_EVT_CAST(BmsTelemetryEvt);
            me->haveData = 1U;
            me->last     = be->data;

            // page transition like you have
            if (me->page == 1U) { // pWait -> pMain
                me->page = 2U;
                post_page_ex(me, 2U);
            }

            // Always refresh pMain summary when on pMain
            if (me->page == 2U) {
                post_summary(me, /*charging?*/ false, "BMS updated");
                post_details(me);
            }

            return Q_HANDLED();
    }
    case BMS_NO_BATTERY_SIG:
    case BMS_CONN_LOST_SIG: {
        /* keep pWait; nothing else */
        return Q_HANDLED();
    }
        default: ;
    }
    return Q_SUPER(&Ctl_run);
}

/* -------- BATTERY DETECTED (IDLE) -------- */
static QState Ctl_detect(ControllerAO * const me, QEvt const * const e) {
    switch (e->sig) {
    case Q_ENTRY_SIG: {
        me->state = CTL_STATE_DETECT;
        printf("Ctl_detect -> ENTRY");
        /* 2s UI refresh, in case we want periodic updates anyway */
        QTimeEvt_armX(&me->ui2s, BSP_TICKS_PER_SEC*2U, BSP_TICKS_PER_SEC*2U);
        return Q_HANDLED();
    }
    case Q_EXIT_SIG: {
        QTimeEvt_disarm(&me->ui2s);
        return Q_HANDLED();
    }
    case TIMEOUT_SIG: { /* periodic UI refresh */
            post_summary(me, false, "ready to charge");
            post_details(me);

            printf("pMain: V=%.2fV type=0x%04X state=%u soc=%u recoverable=%u reason=\"%s\"\r\n",
                   (double)me->last.array_voltage_V,
                   (unsigned)me->last.battery_type_code,
                   (unsigned)me->last.bms_state,
                   (unsigned)me->last.soc_percent,
                   (unsigned)(me->last.bms_fault==0U),
                   "ready to charge");
            return Q_HANDLED();
    }
    case BMS_UPDATED_SIG: {
        BmsTelemetryEvt const *be = Q_EVT_CAST(BmsTelemetryEvt);
        me->last = be->data; me->haveData = 1U;
        post_summary(me, false, "ready to charge");
        post_details(me);
        return Q_HANDLED();
    }
    case BMS_CONN_LOST_SIG: {
        me->haveData = 0U;
        post_page_ex(me, 1); /* pWait */
        return Q_TRAN(&Ctl_wait);
    }

    case BUTTON_PRESSED_SIG: {
            printf("Ctl_detect-BTN: PC13 pressed\r\n");  // visibility
            if (!Cotek_isPresent()) {
                post_summary(me, false, "PSU not present/error");
                return Q_HANDLED();
            }
            /* allow charge only if we have fresh BMS data AND it’s recoverable */
            if (me->haveData
                && (me->last.bms_fault == 0U)
                && (me->last.last_error_class == 0U)
                ) {
                printf("Ctl_detect transition to Ctl_charge\r\n");
                return Q_TRAN(&Ctl_charge);
                } else {
                    /* stay idle and update UI with a short reason */
                    post_summary(me, false, "Not recoverable (fault or error)");
                    return Q_HANDLED();
                }
    }

    default: break;
    }
    return Q_SUPER(&Ctl_run);
}

/* -------- CHARGING -------- */
static QState Ctl_charge(ControllerAO * const me, QEvt const * const e) {
    switch (e->sig) {
    case Q_ENTRY_SIG: {
        me->state = CTL_STATE_CHARGE;
        in_charge = true;
        printf("Ctl_charge: entry\r\n");
        printf("CTL: start charging V=12.0 I=1.0 (30s)\r\n");
        /* command PSU: 12V, 1A */
        PsuSetEvt *se = Q_NEW(PsuSetEvt, PSU_REQ_SETPOINT_SIG);
        se->voltSet = 12.0f;
        se->currSet = 1.0f;
            if (!QACTIVE_POST_X(AO_Cotek, &se->super, QF_NO_MARGIN, 0U)) {
                QF_gc(&se->super);
            }
        post_summary(me, true, "charging");

        /* stop after 60s */
        QTimeEvt_armX(&me->tCharge, 30U*BSP_TICKS_PER_SEC, 0U);
        return Q_HANDLED();
    }
    case Q_EXIT_SIG: {
        in_charge = false;
        printf("Ctl_charge: exit\r\n");
        QTimeEvt_disarm(&me->tCharge);
        return Q_HANDLED();
    }
    case BMS_UPDATED_SIG: {
        BmsTelemetryEvt const *be = Q_EVT_CAST(BmsTelemetryEvt);
        me->last = be->data; me->haveData = 1U;

        /* guard: temp < 35C and no new errors */
        if (me->last.sys_temp_high_C > 35.0f || me->last.last_error_class) {
            QEvt *off = Q_NEW(QEvt, PSU_REQ_OFF_SIG);
            // UI/PSU requests are “best effort”: use margin 0U and GC if it can’t be queued right now
            if (!QACTIVE_POST_X(AO_Cotek, off, QF_NO_MARGIN, 0U)) {
                QF_gc(off);
            }
            post_summary(me, false,
                (me->last.sys_temp_high_C > 35.0f) ? "Stopped: temp > 35C"
                                                   : "Stopped: new error");
            printf("Ctl_charge: BMS_UPDATE_SIG - High temp or error detected\r\n");
            return Q_TRAN(&Ctl_detect);
        }
        /* refresh UI while charging */
        post_summary(me, true, "charging");
        // printf("pMain: CHG V=%.2fV type=0x%04X T=%.1fC soc=%u reason=\"charging\"\r\n",
        //        (double)me->last.array_voltage_V,
        //        (unsigned)me->last.battery_type_code,
        //        (double)me->last.sys_temp_high_C,
        //        (unsigned)me->last.soc_percent);

        return Q_HANDLED();
    }
    case BMS_CONN_LOST_SIG: {

            post_summary(me, false, "Stopped: BMS lost");
            /* 1) ask PSU to turn OFF */
            QEvt *off = Q_NEW(QEvt, PSU_REQ_OFF_SIG);
            // UI/PSU requests are “best effort”: use margin 0U and GC if it can’t be queued right now
            if (!QACTIVE_POST_X(AO_Cotek, off, QF_NO_MARGIN, 0U)) {
                QF_gc(off);
            }
            /* 2) start short timeout (e.g., 500 ms) as a guard */
            QTimeEvt_armX(&me->tPsuOff, 50U, 0U);   /* assuming your tick is 10ms */
            /* 3) go wait for confirmation */
            return Q_TRAN(&Ctl_poweringDown);
    }
    case CHARGE_TIMEOUT_SIG: {
            printf("Ctl_charge: Charge_timeout_sig\r\n");
            // Ask PSU to turn OFF, then wait for confirmation in the substate
            QEvt *off = Q_NEW(QEvt, PSU_REQ_OFF_SIG);
            if (!QACTIVE_POST_X(AO_Cotek, off, QF_NO_MARGIN, 0U)) {
                QF_gc(off);
            }
            post_summary(me, false, "Stopped: 30s timeout");
            return Q_TRAN(&Ctl_poweringDown);
    }
    case BUTTON_PRESSED_SIG:     // or BUTTON_RELEASED_SIG if you prefer

        post_summary(me, false, "Stopped: user");
        /* 1) ask PSU to turn OFF */
        QEvt *off = Q_NEW(QEvt, PSU_REQ_OFF_SIG);
        // UI/PSU requests are “best effort”: use margin 0U and GC if it can’t be queued right now
        if (!QACTIVE_POST_X(AO_Cotek, off, QF_NO_MARGIN, 0U)) {
            QF_gc(off);
        }
        /* 3) go wait for confirmation */
        return Q_TRAN(&Ctl_poweringDown);

        default: break;
    }
    return Q_SUPER(&Ctl_run);
}

/* -------- PSU OUTPUT OFF -------- */
static QState Ctl_poweringDown(ControllerAO * const me, QEvt const * const e) {
    switch (e->sig) {
    case Q_ENTRY_SIG: {
            // Ask PSU to turn OFF
            QEvt *off = Q_NEW(QEvt, PSU_REQ_OFF_SIG);
            // UI/PSU requests are “best effort”: use margin 0U and GC if it can’t be queued right now
            if (!QACTIVE_POST_X(AO_Cotek, off, 1U, 0U)) {
                QF_gc(off);
            }

            // Start a short watchdog while waiting for confirmation.
            // 200ms is typical; tune as you like.
            QTimeEvt_armX(&me->tPsuOff, BSP_TICKS_PER_SEC / 5U, 0U);

            // Optional: tell UI we’re stopping (don’t say OFF yet)
            post_summary(me, false, "stopping...");
            return Q_HANDLED();
    }

    case PSU_RSP_STATUS_SIG: {
            // Check the status “output disabled?”
            CotekStatusEvt const *se = (CotekStatusEvt const *)e;
            bool output_is_on = false;

            output_is_on = (se->out_on != 0U);

            if (!output_is_on) {
                // OFF confirmed → now safe to say OFF and leave the substate
                /* cancel the wait timer */
                post_psu_to_hmi(/*present=*/1U, /*output_on=*/0U,
                se->v_out, se->i_out, se->t_out);
                QTimeEvt_disarm(&me->tPsuOff);
                post_summary(me, false, "power off confirmed");
                return Q_TRAN(&Ctl_detect);
            }

            // Still ON; keep waiting.
            return Q_HANDLED();
    }

    case PSU_OFF_WAIT_TO_SIG: {
            // Didn’t see OFF yet; re-issue OFF and keep waiting.
            (void)QACTIVE_POST_X(AO_Cotek, Q_NEW(QEvt, PSU_REQ_OFF_SIG), 1U, 0U);
            QTimeEvt_rearm(&me->tPsuOff, BSP_TICKS_PER_SEC / 5U);
            return Q_HANDLED();
    }

    case Q_EXIT_SIG: {
            // Stop the watchdog timer cleanly
            QTimeEvt_disarm(&me->tPsuOff);
            return Q_HANDLED();
    }
    }
    return Q_SUPER(&Ctl_run);  // or your actual superstate
}
