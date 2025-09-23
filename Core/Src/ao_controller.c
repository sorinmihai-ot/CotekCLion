#include "qpc_cfg.h"
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

typedef struct {
    QActive  super;
    QTimeEvt ui2s;      /* periodic UI refresh (2s) */
    QTimeEvt tCharge;   /* charging timeout (60s) */
    uint8_t page;
    BmsTelemetry last;
    uint8_t      haveData;
#ifdef ENABLE_BMS_SIM
    QTimeEvt simTick;
#endif
} ControllerAO;

static void post_page(uint8_t page);
static void make_summary(NextionSummaryEvt *se, const BmsTelemetry *t);
extern volatile uint16_t g_lastSig;
extern volatile uint8_t  g_lastTag;
static uint32_t s_last_ui_ms;


static QState Ctl_initial (ControllerAO *me, void const *e);
static QState Ctl_run     (ControllerAO *me, QEvt const *e);
static QState Ctl_wait    (ControllerAO *me, QEvt const *e);
static QState Ctl_detect  (ControllerAO *me, QEvt const *e);
static QState Ctl_charge  (ControllerAO *me, QEvt const *e);

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
    se->recoverable = 0U;         // you don't have a recoverable bit yet
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

// Build & send compact summary only if it changed
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
    if (!QACTIVE_POST_X(AO_Nextion, &se->super, 1U, &me->super)) {
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
    if (!QACTIVE_POST_X(AO_Nextion, &de->super, 1U, &me->super)) {
        QF_gc(&de->super);
    }
}
static void post_page(uint8_t page) {
    NextionPageEvt *pg = Q_NEW(NextionPageEvt, NEX_REQ_SHOW_PAGE_SIG);
    pg->page = page;
    if (!QACTIVE_POST_X(AO_Nextion, &pg->super, 1U, 0U)) {
        QF_gc(&pg->super);
    }
}


/* ctor */
void ControllerAO_ctor(void) {
#ifdef ENABLE_BMS_SIM
    QTimeEvt_ctorX(&l_ctl.simTick, &l_ctl.super, SIM_TICK_SIG, 0U);
#endif
    QActive_ctor(&l_ctl.super, Q_STATE_CAST(&Ctl_initial));
    QTimeEvt_ctorX(&l_ctl.ui2s,   &l_ctl.super, TIMEOUT_SIG, 0U);
    QTimeEvt_ctorX(&l_ctl.tCharge,&l_ctl.super, PSU_RSP_STATUS_SIG/*reuse*/, 0U);
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
    QTimeEvt_armX(&me->ui2s, BSP_TICKS_PER_SEC/2U, 0U);
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
            post_page(2);                     // pMain
            post_summary(me, false, "ready to charge");
            post_details(me);
            printf("CTL: NEX_READY\r\n");
            return Q_TRAN(&Ctl_detect);
        } else {
            post_page(1);                     // pWait
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
                post_page(2U);
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
            QTimeEvt_armX(&me->ui2s, BSP_TICKS_PER_SEC/2U, 0U);
            return Q_HANDLED();
    }
    case BUTTON_PRESSED_SIG: {
        /* Only act if we have a battery detected */
        if (me->haveData) {
            return Q_TRAN(&Ctl_charge);   /* logic lives in the charge state's entry */
        }
        return Q_HANDLED();
        }
        default: break;
    }
    return Q_SUPER(&QHsm_top);
}

/* -------- WAIT FOR BATTERY -------- */
static QState Ctl_wait(ControllerAO * const me, QEvt const * const e) {
    switch (e->sig) {
    case BMS_UPDATED_SIG: {
            BmsTelemetryEvt const *be = Q_EVT_CAST(BmsTelemetryEvt);
            me->haveData = 1U;
            me->last     = be->data;

            // page transition like you have
            if (me->page == 1U) { // pWait -> pMain
                me->page = 2U;
                post_page(2U);
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
        /* 2s UI refresh, in case we want periodic updates anyway */
        QTimeEvt_armX(&me->ui2s, BSP_TICKS_PER_SEC/2, BSP_TICKS_PER_SEC/2);
        return Q_HANDLED();
    }
    case Q_EXIT_SIG: {
        QTimeEvt_disarm(&me->ui2s);
        return Q_HANDLED();
    }
    case TIMEOUT_SIG: { /* periodic UI refresh */
        post_summary(me, false, "ready to charge");
        post_details(me);
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
        post_page(1); /* pWait */
        return Q_TRAN(&Ctl_wait);
    }

    case BUTTON_PRESSED_SIG: {
        return Q_TRAN(&Ctl_charge);
    }

    default: break;
    }
    return Q_SUPER(&Ctl_run);
}

/* -------- CHARGING -------- */
static QState Ctl_charge(ControllerAO * const me, QEvt const * const e) {
    switch (e->sig) {
    case Q_ENTRY_SIG: {
        /* command PSU: 12V, 1A */
        PsuSetEvt *se = Q_NEW(PsuSetEvt, PSU_REQ_SETPOINT_SIG);
        se->voltSet = 12.0f;
        se->currSet = 1.0f;
            if (!QACTIVE_POST_X(AO_Cotek, &se->super, 1U, 0U)) {
                QF_gc(&se->super);
            }
        post_summary(me, true, "charging");

        /* stop after 60s */
        QTimeEvt_armX(&me->tCharge, 60U*BSP_TICKS_PER_SEC, 0U);
        return Q_HANDLED();
    }
    case Q_EXIT_SIG: {
        QTimeEvt_disarm(&me->tCharge);
        return Q_HANDLED();
    }
    case PSU_RSP_STATUS_SIG: {
        /* we reused this SIG for timeout above, so treat as timeout here */
        /* stop PSU and go back to DETECTED */
        (void)QACTIVE_POST_X(AO_Cotek, Q_NEW(QEvt, PSU_REQ_OFF_SIG), 1U, 0U);
        post_summary(me, false, "charge complete");
        return Q_TRAN(&Ctl_detect);
    }
    case BMS_UPDATED_SIG: {
        BmsTelemetryEvt const *be = Q_EVT_CAST(BmsTelemetryEvt);
        me->last = be->data; me->haveData = 1U;

        /* guard: temp < 35C and no new errors */
        if (me->last.sys_temp_high_C > 35.0f || me->last.last_error_class) {
            (void)QACTIVE_POST_X(AO_Cotek, Q_NEW(QEvt, PSU_REQ_OFF_SIG), 1U, 0U);
            post_summary(me, false,
                (me->last.sys_temp_high_C > 35.0f) ? "Stopped: temp > 35C"
                                                   : "Stopped: new error");
            return Q_TRAN(&Ctl_detect);
        }
        /* refresh UI while charging */
        post_summary(me, true, "charging");
        return Q_HANDLED();
    }
    case BMS_CONN_LOST_SIG: {
        (void)QACTIVE_POST_X(AO_Cotek, Q_NEW(QEvt, PSU_REQ_OFF_SIG), 1U, 0U);
        post_summary(me, false, "Stopped: BMS lost");
        return Q_TRAN(&Ctl_wait);
    }
        case BUTTON_PRESSED_SIG:     // or BUTTON_RELEASED_SIG if you prefer
            (void)QACTIVE_POST_X(AO_Cotek, Q_NEW(QEvt, PSU_REQ_OFF_SIG), 1U, 0U);
            post_summary(me, false, "Stopped: user");
            return Q_TRAN(&Ctl_detect);

        default: break;
    }
    return Q_SUPER(&Ctl_run);
}
