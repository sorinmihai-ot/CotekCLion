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

extern UART_HandleTypeDef huart3;
extern QActive * const AO_Controller;

typedef struct {
    QActive super;
} NextionAO;

static QState Nex_initial(NextionAO * const me, QEvt const * const e);
static QState Nex_active (NextionAO * const me, QEvt const * const e);

static NextionAO l_nex;
QActive * const AO_Nextion = &l_nex.super;

/* ========= UART helpers ========= */
static void nex_send_raw(uint8_t const *buf, uint16_t len) {
    (void)HAL_UART_Transmit(&huart3, (uint8_t*)buf, len, 20);
}
static void nex_send3(char const *s) {
    uint8_t end[3] = { 0xFF, 0xFF, 0xFF };
    printf("NEX<< %s\r\n", s);
    nex_send_raw((uint8_t const*)s, (uint16_t)strlen(s));
    nex_send_raw(end, 3);
}
static void nex_sendf(char const *fmt, ...) {
    char buf[128];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if ((size_t)n >= sizeof(buf)) { n = (int)sizeof(buf)-1; buf[n] = '\0'; }
    nex_send3(buf);
}
static void ascii_sanitize(char *s) {
    for (; *s; ++s) {
        unsigned char c = (unsigned char)*s;
        if (c < 0x20 || c >= 0x7F) *s = '?';
        if (c == 0xFF) *s = '?';
    }
}
static void nex_send_textf(const char *fmt, ...) {
    char buf[128] = {0};
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if ((size_t)n >= sizeof(buf)) buf[sizeof(buf)-1] = '\0';
    ascii_sanitize(buf);
    nex_send3(buf);
}

/* ========= API ========= */
void Nextion_OnRx(uint8_t const *buf, uint16_t len) {
    if (len >= 2 && buf[0] == 0x66) {
        uint8_t pid = buf[1];
        NextionPageEvt *pg = Q_NEW(NextionPageEvt, NEX_REQ_SHOW_PAGE_SIG);
        pg->page = pid;
        (void)QACTIVE_POST_X(AO_Controller, &pg->super, 1U, 0U);
        return;
    }
    if (len >= 5 && buf[0] == 0x71) {
        uint32_t val = (uint32_t)buf[1] | ((uint32_t)buf[2]<<8) |
                       ((uint32_t)buf[3]<<16) | ((uint32_t)buf[4]<<24);
        printf("NEX>> numeric: %lu\r\n", (unsigned long)val);
        return;
    }
}

/* Labels/colors */
static const char *nex_batt_name(uint16_t code) {
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
static uint16_t nex_batt_color(uint16_t code) {
    switch (code) {
        case 0x0600: return 0xFD20;
        case 0x0500: return 0xFFE0;
        case 0x0501: return 0xFD20;
        case 0x0400: return 0x07E0;
        case 0x0401: return 0x07FF;
        case 0x0402: return 0x001F;
        default:     return 0xC618;
    }
}

/* ========= ctor/state ========= */
void NextionAO_ctor(void) {
    QActive_ctor(&l_nex.super, Q_STATE_CAST(&Nex_initial));
}
static QState Nex_initial(NextionAO * const me, QEvt const * const e) {
    (void)me; (void)e;
    return Q_TRAN(&Nex_active);
}
static QState Nex_active(NextionAO * const me, QEvt const * const e) {
    (void)me;
    switch (e->sig) {
    case Q_ENTRY_SIG: {
        if (!QACTIVE_POST_X(AO_Controller, Q_NEW(QEvt, NEX_READY_SIG), 1U, 0U)) { }
        return Q_HANDLED();
    }
    case NEX_REQ_SHOW_PAGE_SIG: {
        NextionPageEvt const *pe = (NextionPageEvt const*)e;
        switch (pe->page) {
            case 0: nex_send3("page pSplash"); break;
            case 1: nex_send3("page pWait");   break;
            case 2:
                nex_send3("page pMain");
                nex_send3("vis pMain.pWarn,0");
                nex_send3("ref pMain.pWarn");
                break;
            case 3: nex_send3("page pDetails"); break;
            default: break;
        }
        return Q_HANDLED();
    }
    case NEX_REQ_UPDATE_SUMMARY_SIG: {
        NextionSummaryEvt const *se = Q_EVT_CAST(NextionSummaryEvt);

        if (se->battTypeStr[0]) {
            nex_send_textf("pMain.tBattType.txt=\"Battery: %s\"", se->battTypeStr);
            nex_sendf("pMain.rTypeBar.bco=%u", (unsigned)se->typeColor565);
        }

        nex_send_textf("pMain.tRecHead.txt=\"%s\"", se->classStr);
        nex_sendf("pMain.tRecHead.pco=%u", (unsigned)se->classColor565);
        nex_send3("ref pMain.tRecHead");

        nex_send_textf("pMain.tVolt.txt=\"%.2f V\"", se->packV);

        if (se->statusStr[0]) {
            nex_send_textf("pMain.tStatus.txt=\"%s\"", se->statusStr);
        }

        if (se->errors[0]) {
            nex_send_textf("pMain.tErrors.txt=\"%s\"", se->errors);
        } else {
            nex_send_textf("pMain.tErrors.txt=\"None\"");
        }

        const uint8_t want = se->warnIcon ? 1U : 0U;
        nex_send_textf("vis pMain.pWarn,%u", want);
        nex_send3("ref pMain.pWarn");

#ifdef ENABLE_BMS_SIM
        nex_send_textf("pMain.tAppStatus.txt=\"%s\"", (se->reason[0] ? se->reason : ""));
        nex_send3(se->charging ? "pMain.tAppStatus.bco=2016" : "pMain.tAppStatus.bco=50712");
        nex_send3("ref pMain.tAppStatus");
#else
        if (se->reason[0]) nex_send_textf("pMain.tRecReason.txt=\"%s\"", se->reason);
        else               nex_send_textf("pMain.tRecReason.txt=\"\"");
#endif
        return Q_HANDLED();
    }

    case NEX_REQ_UPDATE_PSU_SIG: {
        NextionPsuEvt const *pe = Q_EVT_CAST(NextionPsuEvt);
        nex_send_textf("pMain.tPsu.txt=\"PSU: %s\"", pe->present ? "Detected" : "Missing");
        nex_send_textf("pMain.tOutState.txt=\"Output: %s\"", pe->output_on ? "ON" : "OFF");
        if (pe->v_out >= 0.0f) nex_send_textf("pMain.tOutV.txt=\"Vout: %.1f V\"", pe->v_out);
        if (pe->i_out >= 0.0f) nex_send_textf("pMain.tOutI.txt=\"Iout: %.1f A\"", pe->i_out);
        if (pe->temp_C > -90.0f && pe->temp_C < 200.0f)
            nex_send_textf("pMain.tOutT.txt=\"Temp: %.0f C\"", pe->temp_C);
        nex_sendf("pMain.tPsu.bco=%u",      pe->present   ? 2016U : 63488U);
        nex_sendf("pMain.tOutState.bco=%u", pe->output_on ? 2016U : 63488U);
        return Q_HANDLED();
    }

    case NEX_REQ_UPDATE_DETAILS_SIG: {
        NextionDetailsEvt const *de = Q_EVT_CAST(NextionDetailsEvt);

        nex_send_textf("pDetails.tHVolt.txt=\"%.2f\"", de->high_voltage_V);
        nex_send_textf("pDetails.tLVolt.txt=\"%.2f\"", de->low_voltage_V);
        nex_send_textf("pDetails.tAVolt.txt=\"%.2f\"", de->avg_voltage_V);

        nex_send_textf("pDetails.tHTemp.txt=\"%.1f\"", de->high_temp_C);
        nex_send_textf("pDetails.tLTemp.txt=\"%.1f\"", de->low_temp_C);
        nex_send_textf("pDetails.tPackHTemp.txt=\"%.1f\"", de->pack_high_temp_C);
        nex_send_textf("pDetails.tPackLTemp.txt=\"%.1f\"", de->pack_low_temp_C);

        nex_send_textf("pDetails.tSerialN.txt=\"%s\"", de->serial_number);
        nex_send_textf("pDetails.tFW.txt=\"%s\"",      de->firmware);

        nex_sendf("pDetails.tFanSpeed.txt=\"%u\"", (unsigned)de->fan_speed_rpm);
        nex_sendf("pDetails.tSoC.txt=\"%u%%\"",    (unsigned)de->soc_percent);
        nex_sendf("pDetails.tSoC2.txt=\"%u%%\"",   (unsigned)de->soc2_percent);

        nex_send_textf("pDetails.tBmsState.txt=\"%s\"", de->bms_state_str);
        nex_send_textf("pDetails.tBmsFault.txt=\"BMS_fault: %s\"", de->bms_fault_str);
        return Q_HANDLED();
    }

    default: break;
    }
    return Q_SUPER(&QHsm_top);
}
