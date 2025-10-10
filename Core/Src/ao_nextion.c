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
    if ((size_t)n >= sizeof(buf)) n = (int)sizeof(buf) - 1;
    buf[n] = '\0';
    va_end(ap);
    if (n < 0) return;
    if ((size_t)n >= sizeof(buf)) n = (int)(sizeof(buf) - 1);
    nex_send3(buf);
}

// Replace control/non-ASCII bytes with '?'. Prevents stray 0xFF (Nextion terminator) in payload.
static void ascii_sanitize(char *s) {
    for (; *s; ++s) {
        unsigned char c = (unsigned char)*s;
        if (c < 0x20 || c >= 0x7F) *s = '?';
    }
}

// Build text, sanitize, then send with terminator bytes.
// Use this for any command that writes to .txt fields.
static void nex_send_textf(const char *fmt, ...) {
    char buf[128];
    memset(buf, 0, sizeof(buf));
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    if ((size_t)n >= sizeof(buf)) n = (int)sizeof(buf) - 1;
    buf[n] = '\0';
    va_end(ap);
    if (n < 0) return;
    if ((size_t)n >= sizeof(buf)) buf[sizeof(buf)-1] = '\0';
    ascii_sanitize(buf);
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
        return;
    }
    if (len >= 5 && buf[0] == 0x71) {               // numeric return from "get"
        // value is little-endian 32-bit
        uint32_t val = (uint32_t)buf[1]
                     | ((uint32_t)buf[2] << 8)
                     | ((uint32_t)buf[3] << 16)
                     | ((uint32_t)buf[4] << 24);
        printf("NEX>> numeric: %lu\r\n", (unsigned long)val);
        return;
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

// Map battery_type_code -> label for HMI
static char const *nex_batt_name(uint16_t code) {
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
// RGB565 helpers (for reference)
// #define RGB565(r,g,b)  ( ((r)&0xF8)<<8 | ((g)&0xFC)<<3 | ((b)>>3) )

static uint16_t nex_batt_color(uint16_t code) {
    switch (code) {
        case 0x0600: return 0xFD20; // 600s → Orange
        case 0x0500: return 0xFFE0; // 500s Hyperdrive → Yellow (matches your prior 65504)
        case 0x0501: return 0xFD20; // 500s BMZ → Orange (same family color as 600s or choose distinct if you prefer)
        case 0x0400: return 0x07E0; // 400s Hyperdrive → Green (2016)
        case 0x0401: return 0x07FF; // 400s Dual-Zone → Teal/Cyan
        case 0x0402: return 0x001F; // 400s Steatite → Blue
        default:     return 0xC618; // Unknown → Neutral grey (50712)
    }
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
                case 2: nex_send3("page pMain");
                    printf("NEX: page pMain\n");    // ensure warning icon starts hidden each time we arrive on pMain
                    nex_send3("vis pMain.pWarn,0");
                    nex_send3("ref pMain.pWarn");
                    break;
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
                nex_send_textf("pMain.tBattType.txt=\"Battery: %s\"", se->battTypeStr);
                nex_sendf      ("pMain.rTypeBar.bco=%u", (unsigned)se->typeColor565);
            }
        // show classification label/color (on real batteries it will be Operational/Recoverable/Not Recoverable;
        // on SIM builds it will show "SIM" in grey)
        nex_send_textf("pMain.tRecHead.txt=\"%s\"", se->classStr);
        nex_sendf      ("pMain.tRecHead.pco=%u", (unsigned)se->classColor565);
        nex_send3("ref pMain.tRecHead");

            // Pack voltage
            nex_send_textf("pMain.tVolt.txt=\"%.2f V\"", se->packV);

            // Status text (no color usage -> you can leave statusColor565 unused)
            if (se->statusStr[0] != '\0') {
                nex_send_textf("pMain.tStatus.txt=\"%s\"", se->statusStr);
                // If you later choose to color the status text or bar:
                // nex_sendf("pMain.rStBar.bco=%u", (unsigned)se->statusColor565);
            }

            // Errors line (or "None")
            if (se->errors[0] != '\0') {
                nex_send_textf("pMain.tErrors.txt=\"%s\"", se->errors);
            } else {
                nex_send_textf("pMain.tErrors.txt=\"None\"");
            }

            // Warning icon group visibility

        const uint8_t want = se->warnIcon ? 1U : 0U;
        nex_send_textf("vis pMain.pWarn,%u", want);
        nex_send3("ref pMain.pWarn");

        // if (want) {
        //     nex_send3("pMain.pWarn.pic=3");   // your real icon ID
        //     nex_send3("vis pMain.pWarn,1");
        // } else {
        //     nex_send3("pMain.pWarn.pic=0");   // empty / nothing
        //     nex_send3("vis pMain.pWarn,0");
        // }
        // nex_send3("ref pMain.pWarn");
        // nex_send3("get pMain.pWarn.pic");     // optional: should be 0 or 2
        // nex_send3("get pMain.pWarn.vis");     // optional: should be 0 or 1

        // debug trace
        printf("NEX: pWarn vis=%u (charging=%u, warnIcon=%u)\r\n",
               (unsigned)want, (unsigned)se->charging, (unsigned)se->warnIcon);

#ifdef ENABLE_BMS_SIM
            // Set tRecReason text
            char cmd[128];
            snprintf(cmd, sizeof(cmd), "pMain.tAppStatus.txt=\"%s\"", (se->reason[0] ? se->reason : ""));
            nex_send_textf("%s", cmd);

            // Set background color: green=2016 when charging, else a neutral default (e.g., 63488=red? 50712=grey?)
            if (se->charging) {
                nex_send3("pMain.tAppStatus.bco=2016");
            } else {
                nex_send3("pMain.tAppStatus.bco=50712"); // pick your normal background; adjust if you use another
            }

            // ensure Nextion refreshes the component
            nex_send3("ref pMain.tAppStatus");
            return Q_HANDLED();

#endif
        if (se->reason[0] != '\0') {
            nex_send_textf("pMain.tRecReason.txt=\"%s\"", se->reason);
        } else {
            nex_send_textf("pMain.tRecReason.txt=\"\"");
        }
        return Q_HANDLED();
        }

    case NEX_REQ_UPDATE_PSU_SIG: {
        NextionPsuEvt const *pe = Q_EVT_CAST(NextionPsuEvt);

        nex_send_textf("pMain.tPsu.txt=\"PSU: %s\"", pe->present ? "Detected" : "Missing");
        nex_send_textf("pMain.tOutState.txt=\"Output: %s\"", pe->output_on ? "ON" : "OFF");
        if (pe->v_out >= 0.0f)  nex_send_textf("pMain.tOutV.txt=\"Vout: %.1f V\"", pe->v_out);
        if (pe->i_out >= 0.0f)  nex_send_textf("pMain.tOutI.txt=\"Iout: %.1f A\"", pe->i_out);
        if (pe->temp_C > -90.0f && pe->temp_C < 200.0f)
            nex_send_textf("pMain.tOutT.txt=\"Temp: %.0f C\"", pe->temp_C);  // ASCII only
        // colors can stay with nex_sendf
        nex_sendf("pMain.tPsu.bco=%u",      pe->present   ? 2016U : 63488U);
        nex_sendf("pMain.tOutState.bco=%u", pe->output_on ? 2016U : 63488U);

        return Q_HANDLED();
    }

    case NEX_REQ_UPDATE_DETAILS_SIG: {
            NextionDetailsEvt const *de = Q_EVT_CAST(NextionDetailsEvt);

            // voltage
            nex_send_textf("pDetails.tHVolt.txt=\"%.2f\"", de->high_voltage_V);
            nex_send_textf("pDetails.tLVolt.txt=\"%.2f\"", de->low_voltage_V);
            nex_send_textf("pDetails.tAVolt.txt=\"%.2f\"", de->avg_voltage_V);

            // temps
            nex_send_textf("pDetails.tHTemp.txt=\"%.1f\"", de->high_temp_C);
            nex_send_textf("pDetails.tLTemp.txt=\"%.1f\"", de->low_temp_C);
            nex_send_textf("pDetails.tPackHTemp.txt=\"%.1f\"", de->pack_high_temp_C);
            nex_send_textf("pDetails.tPackLTemp.txt=\"%.1f\"", de->pack_low_temp_C);

            // serial, FW
            nex_send_textf("pDetails.tSerialN.txt=\"%s\"", de->serial_number);
            nex_send_textf("pDetails.tFW.txt=\"%s\"",      de->firmware);

            // fan / soc
            nex_sendf("pDetails.tFanSpeed.txt=\"%u\"", (unsigned)de->fan_speed_rpm);
            nex_sendf("pDetails.tSoC.txt=\"%u%%\"",    (unsigned)de->soc_percent);
            nex_sendf("pDetails.tSoC2.txt=\"%u%%\"",   (unsigned)de->soc2_percent);

            // state / fault
            nex_send_textf("pDetails.tBmsState.txt=\"%s\"", de->bms_state_str);
            nex_send_textf("pDetails.tBmsFault.txt=\"BMS_fault: %s\"", de->bms_fault_str);

            return Q_HANDLED();
    }

    }
    return Q_SUPER(&QHsm_top);
}
