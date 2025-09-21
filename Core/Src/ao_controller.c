#include "qpc_cfg.h"
#include "ao_controller.h"
#include "ao_nextion.h"
#include "app_signals.h"
#include "bsp.h"
#include <stdio.h>
#include <string.h>

Q_DEFINE_THIS_FILE

typedef struct {
    QActive  super;
    QTimeEvt ui2s;      /* periodic UI refresh (2s) */
    QTimeEvt tCharge;   /* charging timeout (60s) */
    uint8_t page;
    BmsTelemetry last;
    uint8_t      haveData;
} ControllerAO;

static void post_page(uint8_t page);
static void make_summary(NextionSummaryEvt *se, const BmsTelemetry *t);
extern volatile uint16_t g_lastSig;
extern volatile uint8_t  g_lastTag;


static void make_summary(NextionSummaryEvt *se, const BmsTelemetry *t) {
    // Fill only what your Nextion uses right now; keep it simple.
    // Adjust field names if your BmsTelemetry differs.
    se->packV = t->array_voltage_V;

    // Type + color (placeholder until you add real typing)
    snprintf(se->battTypeStr, sizeof(se->battTypeStr), "Generic");
    se->typeColor565 = 2016; // light green as you used earlier

    // Status / errors (placeholder strings for now)
    snprintf(se->statusStr, sizeof(se->statusStr), "OK");
    se->errors[0] = '\0';

    // Verdict
    se->recoverable = 1U;
    se->reason[0] = '\0';
}

static QState Ctl_initial (ControllerAO *me, void const *e);
static QState Ctl_run     (ControllerAO *me, QEvt const *e);
static QState Ctl_wait    (ControllerAO *me, QEvt const *e);
static QState Ctl_detect  (ControllerAO *me, QEvt const *e);
static QState Ctl_charge  (ControllerAO *me, QEvt const *e);

static ControllerAO l_ctl;
QActive *AO_Controller = &l_ctl.super;

static void post_page(uint8_t page) {
    NextionPageEvt *pg = Q_NEW(NextionPageEvt, NEX_REQ_SHOW_PAGE_SIG);
    pg->page = page;
    if (!QACTIVE_POST_X(AO_Nextion, &pg->super, 0U, 0U)) {
        QF_gc(&pg->super);
    }
}

static void post_summary(const ControllerAO *me, bool charging, const char *reasonOpt) {
    NextionSummaryEvt *se = Q_NEW(NextionSummaryEvt, NEX_REQ_UPDATE_SUMMARY_SIG);
    memset(se, 0, sizeof(*se));
    /* banner */
    switch (me->last.battery_type_code) {
        case 0x0400: strcpy(se->battTypeStr, "400s");        se->typeColor565 = 2016; break;
        case 0x0500: strcpy(se->battTypeStr, "500s");        se->typeColor565 = 65504; break;
        case 0x0600: strcpy(se->battTypeStr, "600s");        se->typeColor565 = 1023; break;
        default:     strcpy(se->battTypeStr, "Unknown");     se->typeColor565 = 63488; break;
    }
    /* mid */
    se->packV = me->last.array_voltage_V;
    strcpy(se->statusStr, charging ? "CHARGING" : "IDLE");
    se->statusColor565 = charging ? 2016 : 65535;
    if (me->last.last_error_class) {
        snprintf(se->errors, sizeof(se->errors),
                 "Err C%u/%u", me->last.last_error_class, me->last.last_error_code);
        se->warnIcon = true;
    } else {
        strcpy(se->errors, "None");
        se->warnIcon = false;
    }
    /* bottom verdict */
    se->recoverable = true;  /* placeholder (all recoverable) */
    if (reasonOpt && reasonOpt[0]) {
        strncpy(se->reason, reasonOpt, sizeof(se->reason)-1);
    } else {
        strncpy(se->reason,
            charging ? "charging" : "ready to charge", sizeof(se->reason)-1);
    }
    se->charging = charging;
    if (!QACTIVE_POST_X(AO_Nextion, &se->super, 1U, 0U)) {
        QF_gc(&se->super);
    }
}

/* ctor */
void ControllerAO_ctor(void) {
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
    return Q_TRAN(&Ctl_run);
}

static QState Ctl_run(ControllerAO * const me, QEvt const * const e) {
    switch (e->sig) {
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
        me->last = be->data;
        printf("CTL: BMS_UPDATED -> pMain\r\n");
        // ensure we’re on pMain and refresh contents
        if (me->page == 1U) {         // from pWait
            me->page = 2U;
            post_page(2U);            // tell Nextion to show pMain
        }
        if (me->page == 2U) {         // whenever we're on pMain, refresh summary
            NextionSummaryEvt *se = Q_NEW(NextionSummaryEvt, NEX_REQ_UPDATE_SUMMARY_SIG);
            make_summary(se, &me->last);
            if (!QACTIVE_POST_X(AO_Nextion, &se->super, 1U, &me->super)) {
                QF_gc(&se->super);
            }
        }
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
        me->last = be->data;
        // DEBUG breadcrumb so we *see* the transition reason
        printf("CTL: BMS_UPDATED -> switching to pMain\n");

        if (me->page == 1U) {         // from pWait
            me->page = 2U;
            post_page(2U);            // tell Nextion to show pMain
        }
        if (me->page == 2U) {         // whenever we're on pMain, refresh summary
            NextionSummaryEvt *se = Q_NEW(NextionSummaryEvt, NEX_REQ_UPDATE_SUMMARY_SIG);
            make_summary(se, &me->last);
            if (!QACTIVE_POST_X(AO_Nextion, &se->super, 1U, &me->super)) {
                QF_gc(&se->super);
            }
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
        QTimeEvt_armX(&me->ui2s, 2U*BSP_TICKS_PER_SEC, 2U*BSP_TICKS_PER_SEC);
        return Q_HANDLED();
    }
    case Q_EXIT_SIG: {
        QTimeEvt_disarm(&me->ui2s);
        return Q_HANDLED();
    }
    case TIMEOUT_SIG: { /* periodic UI refresh */
        post_summary(me, false, "ready to charge");
        return Q_HANDLED();
    }
    case BMS_UPDATED_SIG: {
        BmsTelemetryEvt const *be = Q_EVT_CAST(BmsTelemetryEvt);
        me->last = be->data; me->haveData = 1U;
        post_summary(me, false, "ready to charge");
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
