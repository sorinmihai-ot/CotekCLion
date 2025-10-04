#include "ao_nextion.h"
#include "app_signals.h"
#include "qpc_cfg.h"
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
    printf("NEX<< %s\r\n", s);
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
void Nextion_OnRx(uint8_t const *buf, uint16_t len) {
    // Minimal stub that compiles. Replace with event-posting if you want parsing in the AO.
    // (void)buf;
    // (void)len;
    if (len >= 2 && buf[0] == 0x66) {          // page id report
        uint8_t pid = buf[1];                  // your IDs: 1=pWait,2=pMain,3=pDetails
        NextionPageEvt *pg = Q_NEW(NextionPageEvt, NEX_REQ_SHOW_PAGE_SIG);
        pg->page = pid;
        (void)QACTIVE_POST_X(AO_Controller, &pg->super, 1U, 0U);
    }
    /*
    NextionRxEvt *e = Q_NEW(NextionRxEvt, NEX_RX_SIG);
    size_t n = len;
    if (n > sizeof(e->data)) n = sizeof(e->data);
    memcpy(e->data, buf, n);
    e->len = (uint16_t)n;
    (void)QACTIVE_POST_X(AO_Nextion, &e->super, 1U, 0U);
    */
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
    switch (e->sig)
    {
    case Q_ENTRY_SIG: {
            /* tell Controller we are alive */
            if (!QACTIVE_POST_X(AO_Controller, Q_NEW(QEvt, NEX_READY_SIG), 1U, 0U)) {
                /* nothing to GC because we used Q_NEW inline; this will almost never drop */
            }
            return Q_HANDLED();
    }

    case NEX_REQ_SHOW_PAGE_SIG: {
            /* 0=splash,1=wait,2=main,3=details */
            NextionPageEvt const *pe = (NextionPageEvt const*)e;
            switch (pe->page) {
            case 0: nex_send3("page pSplash");  printf("NEX: page pSplash\n");  break;
            case 1: nex_send3("page pWait");    printf("NEX: page pWait\n");    break;
            case 2: nex_send3("page pMain");    printf("NEX: page pMain\n");    break;
            case 3: nex_send3("page pDetails"); printf("NEX: page pDetails\n"); break;
            default: break;
            }
            return Q_HANDLED();
    }

    case NEX_REQ_UPDATE_SUMMARY_SIG:
        {
            NextionSummaryEvt const *se = Q_EVT_CAST(NextionSummaryEvt);

            // Battery type text and color bar
            if (se->battTypeStr[0] != '\0') {
                nex_sendf("pMain.tBattType.txt=\"Battery: %s\"", se->battTypeStr);
                nex_sendf("pMain.rTypeBar.bco=%u", (unsigned)se->typeColor565);
            }

            // Pack voltage
            nex_sendf("pMain.tVolt.txt=\"%.2f V\"", se->packV);

            // Status text (no color usage -> you can leave statusColor565 unused)
            if (se->statusStr[0] != '\0') {
                nex_sendf("pMain.tStatus.txt=\"%s\"", se->statusStr);
                // If you later choose to color the status text or bar:
                // nex_sendf("pMain.rStBar.bco=%u", (unsigned)se->statusColor565);
            }

            // Errors line (or "None")
            if (se->errors[0] != '\0') {
                nex_sendf("pMain.tErrors.txt=\"%s\"", se->errors);
            } else {
                nex_sendf("pMain.tErrors.txt=\"None\"");
            }

            // Warning icon group visibility
            nex_sendf("vis pMain.pWarn,%d", se->warnIcon ? 1 : 0);
#ifdef ENABLE_BMS_SIM
            // Set tRecReason text
            char cmd[128];
            snprintf(cmd, sizeof(cmd), "pMain.tRecReason.txt=\"%s\"", (se->reason[0] ? se->reason : ""));
            nex_send3(cmd);

            // Set background color: green=2016 when charging, else a neutral default (e.g., 63488=red? 50712=grey?)
            if (se->charging) {
                nex_send3("pMain.tRecReason.bco=2016");
            } else {
                nex_send3("pMain.tRecReason.bco=50712"); // pick your normal background; adjust if you use another
            }

            // ensure Nextion refreshes the component
            nex_send3("ref pMain.tRecReason");
            return Q_HANDLED();

#endif
#ifndef ENABLE_BMS_SIM
        // Recoverable widgets (use your names; here I assume text labels)
        if (se->recoverable) {
            nex_sendf("pMain.tRecHead.txt=\"Recoverable: YES\"");
            if (se->reason[0] != '\0') {
                nex_sendf("pMain.tRecReason.txt=\"Reason: %s\"", se->reason);
            } else {
                nex_sendf("pMain.tRecReason.txt=\"Reason: --\"");
            }
        } else {
            nex_sendf("pMain.tRecHead.txt=\"Recoverable: --\"");
            nex_sendf("pMain.tRecReason.txt=\"Reason: --\"");
        }

        // Charging group (only if you have a group)
        // nex_sendf("vis pMain.grpCharging,%d", se->charging ? 1 : 0);

        return Q_HANDLED();

#endif
        }
    case NEX_REQ_UPDATE_PSU_SIG: {
            NextionPsuEvt const *pe = Q_EVT_CAST(NextionPsuEvt);

            nex_sendf("pMain.tPsu.txt=\"PSU: %s\"", pe->present ? "Detected" : "Missing");
            nex_sendf("pMain.tOutState.txt=\"Output: %s\"", pe->output_on ? "ON" : "OFF");

            if (pe->v_out >= 0.0f)  nex_sendf("pMain.tOutV.txt=\"Vout: %.1f V\"", pe->v_out);
            if (pe->i_out >= 0.0f)  nex_sendf("pMain.tOutI.txt=\"Iout: %.1f A\"", pe->i_out);
            if (pe->temp_C > -90.0f && pe->temp_C < 200.0f)
                nex_sendf("pMain.tOutT.txt=\"Temp: %.0f °C\"", pe->temp_C);

            // background colors (red/green)
            nex_sendf("pMain.tPsu.bco=%u",      pe->present   ? 2016U : 63488U);
            nex_sendf("pMain.tOutState.bco=%u", pe->output_on ? 2016U : 63488U);

            return Q_HANDLED();
    }
    case NEX_REQ_UPDATE_DETAILS_SIG: {
            NextionDetailsEvt const *de = Q_EVT_CAST(NextionDetailsEvt);

            // voltage
            nex_sendf("pDetails.tHVolt.txt=\"%.2f\"", de->high_voltage_V);
            nex_sendf("pDetails.tLVolt.txt=\"%.2f\"", de->low_voltage_V);
            nex_sendf("pDetails.tAVolt.txt=\"%.2f\"", de->avg_voltage_V);

            // temps
            nex_sendf("pDetails.tHTemp.txt=\"%.1f\"", de->high_temp_C);
            nex_sendf("pDetails.tLTemp.txt=\"%.1f\"", de->low_temp_C);
            nex_sendf("pDetails.tPackHTemp.txt=\"%.1f\"", de->pack_high_temp_C);
            nex_sendf("pDetails.tPackLTemp.txt=\"%.1f\"", de->pack_low_temp_C);

            // serial, FW
            nex_sendf("pDetails.tSerialN.txt=\"%s\"", de->serial_number);
            nex_sendf("pDetails.tFW.txt=\"%s\"",      de->firmware);

            // fan / soc
            nex_sendf("pDetails.tFanSpeed.txt=\"%u\"", (unsigned)de->fan_speed_rpm);
            nex_sendf("pDetails.tSoC.txt=\"%u%%\"",    (unsigned)de->soc_percent);
            nex_sendf("pDetails.tSoC2.txt=\"%u%%\"",   (unsigned)de->soc2_percent);

            // state / fault
            nex_sendf("pDetails.tBmsState.txt=\"%s\"", de->bms_state_str);
            nex_sendf("pDetails.tBmsFault.txt=\"BMS_fault: %s\"", de->bms_fault_str);

            return Q_HANDLED();
    }

    }
    return Q_SUPER(&QHsm_top);
}
