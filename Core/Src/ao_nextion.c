#include "ao_nextion.h"
#include "app_signals.h"

#include "qpc.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

Q_DEFINE_THIS_FILE

/* ====== externs provided elsewhere ====== */
extern UART_HandleTypeDef huart3;          /* Nextion on USART3 */
extern QActive * const AO_Controller;      /* posted to from here */

/* ====== AO type ====== */
typedef struct {
    QActive super;
    /* (no timers needed for “Option B”, runtime only) */
} NextionAO;

/* ====== forward state prototypes ====== */
static QState Nex_initial(NextionAO * const me, QEvt const * const e);
static QState Nex_active (NextionAO * const me, QEvt const * const e);

/* ====== singleton instance & handle ====== */
static NextionAO l_nex;                    /* storage */
QActive * const AO_Nextion = &l_nex.super; /* public handle */

/* ====== small helpers (short, blocking writes) ====== */
static void nex_send_raw(uint8_t const *buf, uint16_t len) {
    /* short, bounded writes; 20 ms guard to avoid long stalls */
    (void)HAL_UART_Transmit(&huart3, (uint8_t*)buf, len, 20);
}

static void nex_send3(char const *s) {
    /* cmd + 0xFF 0xFF 0xFF */
    uint8_t end[3] = { 0xFF, 0xFF, 0xFF };
    nex_send_raw((uint8_t const*)s, (uint16_t)strlen(s));
    nex_send_raw(end, 3);
}

static void nex_sendf(char const *fmt, ...) {
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if ((size_t)n >= sizeof(buf)) n = (int)(sizeof(buf) - 1);
    nex_send3(buf);
}

/* ====== ctor ====== */
void NextionAO_ctor(void) {
    QActive_ctor(&l_nex.super, Q_STATE_CAST(&Nex_initial));
}

/* ====== states ====== */
static QState Nex_initial(NextionAO * const me, QEvt const * const e) {
    (void)e;
    /* Nothing heavy here (no reset/FW writes). Runtime only. */
    return Q_TRAN(&Nex_active);
}

static QState Nex_active(NextionAO * const me, QEvt const * const e) {
    switch (e->sig) {

    case Q_ENTRY_SIG: {
        /* tell Controller we are alive */
        QACTIVE_POST(AO_Controller, Q_NEW(QEvt, NEX_READY_SIG), &me->super);
        return Q_HANDLED();
    }

    case NEX_REQ_SHOW_PAGE_SIG: {
        /* 0=splash,1=wait,2=main,3=details */
        NextionPageEvt const *pe = (NextionPageEvt const*)e;
        switch (pe->page) {
            case 0: nex_send3("page pSplash");  break;
            case 1: nex_send3("page pWait");    break;
            case 2: nex_send3("page pMain");    break;
            case 3: nex_send3("page pDetails"); break;
            default: /* ignore unknown */       break;
        }
        return Q_HANDLED();
    }

    case NEX_REQ_UPDATE_SUMMARY_SIG: {
        /* Minimal runtime updates for pMain; adjust names to your HMI */
        NextionSummaryEvt const *se = (NextionSummaryEvt const*)e;

        /* Example widgets (rename if your HMI differs):
           pMain.tType, pMain.tPackV, pMain.tStatus, pMain.tErr
           pMain.tReason, pMain.warnIcon (vis), pMain.recIcon (vis)
        */
        /* battery type string */
        if (se->battTypeStr[0] != '\0') {
            nex_sendf("pMain.tType.txt=\"%s\"", se->battTypeStr);
        }
        /* pack voltage */
        nex_sendf("pMain.tPackV.txt=\"%.1fV\"", se->packV);

        /* status string + color */
        if (se->statusStr[0] != '\0') {
            nex_sendf("pMain.tStatus.txt=\"%s\"", se->statusStr);
            nex_sendf("pMain.tStatus.pco=%u", (unsigned)se->statusColor565);
        }

        /* errors list */
        if (se->errors[0] != '\0') {
            nex_sendf("pMain.tErr.txt=\"%s\"", se->errors);
        } else {
            nex_send3("pMain.tErr.txt=\"\"");
        }

        /* reason (optional) */
        if (se->reason[0] != '\0') {
            nex_sendf("pMain.tReason.txt=\"%s\"", se->reason);
        } else {
            nex_send3("pMain.tReason.txt=\"\"");
        }

        /* icons */
        nex_sendf("vis pMain.warnIcon,%d", se->warnIcon ? 1 : 0);
        nex_sendf("vis pMain.recIcon,%d",  se->recoverable ? 1 : 0);

        /* optional charging banner */
        nex_sendf("vis pMain.grpCharging,%d", se->charging ? 1 : 0);

        /* battery “type” color */
        nex_sendf("pMain.tType.pco=%u", (unsigned)se->typeColor565);

        return Q_HANDLED();
    }

    }
    return Q_SUPER(&QHsm_top);
}
