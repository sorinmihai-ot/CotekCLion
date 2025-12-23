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
#include "batt_classify.h"
#include "bms_fault_decode.h"
#include "bms_debug.h"
//#include "events.h"

static uint32_t s_last_sum_ms;
static uint32_t s_last_det_ms;
static uint32_t s_last_sum_hash, s_last_det_hash;
/* Monotonic tick accessor (HAL_GetTick or BSP tick) */
uint32_t tick_ms(void);
/* Local mirrors to detect transitions and de-spam logs */
static uint8_t  s_prev_fresh = 255U;    /* 255 = unknown first run */
static uint32_t s_prev_age_bucket = 999999U;
// Use the mapper from bms_app.c
extern const char *BMS_state_to_text(uint16_t batt_type, uint8_t raw_state);
#define LOW_CELL_STOP_V  2.80f   // TODO: set per battery family if needed
#define PACK_GT_PSU_MARGIN_V  1.0f
#define PACK_PSU_GRACE_MS     3000U
#define DEFAULT_CHARGE_TIME_S  (30U)

#define PSU_ON_WAIT_MS         20000U   // how long we wait for PSU output ON
#define PSU_VOUT_OK_MARGIN_V   1.0f    // acceptable difference between setpoint & measured vout

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
    QTimeEvt tLostHold;  // 10 s “stay on pMain” after comms lost
    uint8_t page;
    BmsTelemetry last;
    uint8_t      haveData;
#ifdef ENABLE_BMS_SIM
    QTimeEvt simTick;
#endif
    ctl_state_t state;
    uint8_t psu_present, psu_out_on;
    uint32_t charge_start_ms;
    uint32_t charge_total_s;
    float   psu_v_out, psu_i_out, psu_temp;
    QTimeEvt tChargeMon;          // fast monitor tick while charging
    uint8_t  latch_prev;          // previous latch state (0/1)
    uint8_t  stop_issued;         // prevent duplicate stop handling
    // charging stop logic
    uint32_t sw_total_s;            // software charge window (seconds)
    uint32_t hw_extra_s;            // extra seconds allowed before HW timer expiry classification (e.g. 60)
    uint32_t hw_deadline_ms;        // charge_start_ms + (sw_total_s + hw_extra_s)*1000
    ChargeStopReason stop_req_reason;
    ChargeStopReason last_stop_reason;
    char     last_stop_text[48];
    char     stop_req_text[48];
    QTimeEvt tPsuOnWait;      // NEW: watchdog while waiting for PSU output to turn ON
    uint8_t  waiting_psu_on;  // NEW: 1 while waiting for output ON
    float    cmd_vset;        // NEW: last commanded setpoints
    float    cmd_iset;
    uint8_t waiting_psu_ready;   // NEW: waiting for PSU_RSP_READY_SIG
    uint8_t ui_div;   // divider counter for charge UI refresh
    QTimeEvt tPsuReady;   // 5s watchdog waiting for PSU_READY_SIG

} ControllerAO;

/* ===== START/STOP buttons + Relay(2/3/4) helper block =====
 * HW:
 *  - StartButton: PC0 (pressed = voltage present)
 *  - StopButton : PC1 (pressed = grounded, per your wiring)
 *  - Relay2 PB13 (active-low): "Charge allowed"
 *  - Relay3 PB14 (active-low): Blue LED
 *  - Relay4 PB15 (active-low): Green LED
 *
 * BSP functions expected (you already planned these):
 *  - BSP_isStartPressed()
 *  - BSP_isStopPressed()
 *  - BSP_isInterlockOK()
 *  - BSP_relay2_set(on)
 *  - BSP_relay3_set(on)
 *  - BSP_relay4_set(on)
 */

static void ctl_outputs_all_off(void) {
    BSP_relay2_set(false);
    BSP_relay3_set(false);
    BSP_relay4_set(false);
}

static void ctl_outputs_ready_blue(void) {
    BSP_relay2_set(true);   // allow charge
    BSP_relay3_set(true);   // blue ON
    BSP_relay4_set(false);  // green OFF
}

static void ctl_outputs_charging_green(void) {
    BSP_relay2_set(true);   // keep allow-charge ON during charge
    BSP_relay3_set(false);  // blue OFF
    BSP_relay4_set(true);   // green ON
}

/* Returns true if battery class + interlock allow charging */
static bool ctl_charge_allowed(ControllerAO *me) {
#if defined(ENABLE_BMS_SIM)
    /* SIM: allow if we have data */
    return (me->haveData != 0U);
#else
    if (!me->haveData) return false;
    if (!BSP_isInterlockOK()) return false;

    BattClassResult cr = batt_classify(&me->last, /*bms_sim_active=*/false);
    return (cr.cls == BATT_CLASS_RECOVERABLE) || (cr.cls == BATT_CLASS_OPERATIONAL);
#endif
}

/* Apply correct outputs for current situation/state */
static void ctl_update_outputs(ControllerAO *me) {
    /* If not allowed, always OFF */
    if (!ctl_charge_allowed(me)) {
        ctl_outputs_all_off();
        return;
    }

    /* Allowed: choose blue vs green depending on controller state */
    if (me->state == CTL_STATE_CHARGE) {
        ctl_outputs_charging_green();
    } else {
        ctl_outputs_ready_blue();
    }
}

static void post_page_ex(ControllerAO *me, uint8_t page);
static void make_summary(NextionSummaryEvt *se, const BmsTelemetry *t);
extern volatile uint16_t g_lastSig;
extern volatile uint8_t  g_lastTag;
//static uint32_t s_last_ui_ms;
static BmsBatteryFamily fam_from_code(uint16_t code) {
    switch (code) {
        case 0x0500: return BMS_FAM_HYP500;         // 500s Hyperdrive (J1939)
        case 0x0501: return BMS_FAM_BMZ500;         // 500s BMZ (extended)
        case 0x0600: return BMS_FAM_CP600;          // 600s CP (extended)
        case 0x0401: return BMS_FAM_CP400_DUAL;     // 400s Dual-Zone
        case 0x0402: return BMS_FAM_CP400_CHILL;    // 400s Steatite (treat as chill-only)
        case 0x0400: /* fall-through */
        default:     return BMS_FAM_CP400_CHILL;    // generic 400s
    }
}

static void decode_faults_for_ui(const BmsTelemetry *t,
                                 char *out_text, size_t out_len,
                                 BmsSeverity *out_sev) {
    if (!t || !out_text || out_len == 0) return;
    out_text[0] = '\0';
    if (out_sev) *out_sev = BMS_SEV_NONE;

    BmsSeverity    sev = BMS_SEV_NONE;
    BmsDomainMask  dom = BMS_DOM_NONE;

    switch (t->battery_type_code) {
        case 0x0500: { // 500s Hyperdrive (J1939)
            bms_decode_hyp500(
                /* hw_fault      */ t->bms_fault_raw,      // your parse fills this from 0x18FF0300 byte0
                /* error_sev_raw */ t->last_error_class,   // from 0x18FF0E00
                /* error_code    */ t->last_error_code,    // from 0x18FF0E00
                out_text, out_len, &sev, &dom);
        } break;

        case 0x0501:   // 500s BMZ (extended)
        case 0x0600: { // 600s CP (extended)
            bms_decode_bmz500_cp600(
                /* pack_fault */ t->bms_fault_raw,     // from 0x10000010 byte5 in your parse
                out_text, out_len, &sev, &dom);
        } break;

        case 0x0400:   // 400s Hyperdrive (base)
        case 0x0401:   // 400s Dual-Zone
        case 0x0402: { // 400s Steatite
            /* We don’t have all the detailed boolean flags yet; use what we have.
               Map last_error_class into a coarse “master fault code”, and mark
               the usual suspects as false for now. You can refine these as you
               parse more 400s bits. */
            BmsCp400Input in = {0};
            /* Coarse mapping:
               0=Normal, 1=Warning, 2=Fault, 3=Permanent, 4=HW Fault (per header comment) */
            if (t->last_error_class) {
                in.master_fault_code = t->last_error_class;
            } else if (t->bms_fault) {
                in.master_fault_code = 2; // any fault present -> “Fault”
            } else {
                in.master_fault_code = 0; // Normal
            }
            in.uv = in.ov = in.ot = in.ut = false;
            in.dchg_oc = in.chg_oc = false;
            in.therm_warning = false;
            in.imbalance = false;

            bms_decode_cp400(&in, out_text, out_len, &sev, &dom);
        } break;

        default: {
            /* Generic fallback through the unified entry point. Build a best-effort input. */
            BmsDecodeInput in = {0};
            in.family = fam_from_code(t->battery_type_code);
            switch (in.family) {
                case BMS_FAM_HYP500:
                    in.u.hyp500.hw_fault       = t->bms_fault_raw;
                    in.u.hyp500.error_severity = t->last_error_class;
                    in.u.hyp500.error_code     = t->last_error_code;
                    break;
                case BMS_FAM_BMZ500:
                case BMS_FAM_CP600:
                    in.u.bmz_cp600.pack_fault  = t->bms_fault_raw;
                    break;
                case BMS_FAM_CP400_CHILL:
                case BMS_FAM_CP400_DUAL: {
                    BmsCp400Input c = {0};
                    if (t->last_error_class) c.master_fault_code = t->last_error_class;
                    else if (t->bms_fault)   c.master_fault_code = 2;
                    else                      c.master_fault_code = 0;
                    in.u.cp400 = c;
                    break;
                }
                default:
                    break;
            }
            bms_decode_any(&in, out_text, out_len, &sev, &dom);
        } break;
    }

    if (out_sev) *out_sev = sev;

    if (out_text[0] == '\0') {
        strncpy(out_text, "None", out_len - 1);
        out_text[out_len - 1] = '\0';
    }
}

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
static void make_summary(NextionSummaryEvt *se, const BmsTelemetry *t) {
    // pack voltage
    se->packV = t->array_voltage_V;

    // pass through numeric code for HMI color mapping
    se->battery_type_code = t->battery_type_code ? t->battery_type_code : 0x0000;

    // friendly name + a fallback color (RGB565) in case you still use typeColor565
    switch (t->battery_type_code) {
        case 0x0600: // 600s family (Ocado extended)
            strcpy(se->battTypeStr, "600s");
            se->typeColor565 = 0xFD20; // orange
            break;

        case 0x0500: // 500s Hyperdrive (J1939)
            strcpy(se->battTypeStr, "500s Hyperdrive");
            se->typeColor565 = 0xFFE0; // yellow
            break;

        case 0x0501: // 500s BMZ (extended)
            strcpy(se->battTypeStr, "500s BMZ");
            se->typeColor565 = 0xFD20; // orange (or pick a distinct amber if you prefer)
            break;

        case 0x0401: // 400s Dual-Zone
            strcpy(se->battTypeStr, "400s Dual-Zone");
            se->typeColor565 = 0x07FF; // teal/cyan
            break;

        case 0x0402: // 400s Steatite
            strcpy(se->battTypeStr, "400s Steatite");
            se->typeColor565 = 0x001F; // blue
            break;

        case 0x0400: // 400s Hyperdrive (base)
            strcpy(se->battTypeStr, "400s Hyperdrive");
            se->typeColor565 = 0x07E0; // green
            break;

        default:
            strcpy(se->battTypeStr, "Unknown");
            se->typeColor565 = 0xC618; // neutral grey
            break;
    }

    // status text
    strncpy(se->statusStr,
        BMS_state_to_text(t->battery_type_code, t->bms_state),
        sizeof(se->statusStr)-1);
    se->statusStr[sizeof(se->statusStr)-1] = '\0';

#if !defined(ENABLE_BMS_SIM)
    {
        BattClassResult cr = batt_classify(t, /*bms_sim_active=*/false);
        strncpy(se->classStr, cr.label, sizeof(se->classStr) - 1);
        se->classStr[sizeof(se->classStr) - 1] = '\0';
        se->classColor565 = cr.color565;
        /* RGB565: 0xFC00 ≈ (255,128,0) — vivid orange */
        if ((cr.cls == BATT_CLASS_RECOVERABLE) ||
            (strcmp(cr.label, "Recoverable") == 0)) {
            se->classColor565 = 0xFC00;
            }
    }

#else
{
    // SIM build – don’t classify
    strcpy(se->classStr, "SIMULATOR");
    se->classColor565 = 0xC618; // grey
}
#endif

    // errors/warnings (you only have bms_fault bitfield right now)
    char faults[128];
    BmsSeverity sev;
    decode_faults_for_ui(t, faults, sizeof(faults), &sev);

    // Put the text into the one-line errors field for pMain.
    // Keep it short if you want: you can clip or pick the first item.
    if (strcmp(faults, "None") == 0) {
        se->errors[0] = '\0';              // Nextion code prints "None" itself
        se->warnIcon  = 0U;
    } else {
        strncpy(se->errors, faults, sizeof(se->errors)-1);
        se->errors[sizeof(se->errors)-1] = '\0';
        se->warnIcon = (sev >= BMS_SEV_WARNING) ? 1U : 0U;
    }
    se->recoverable = (t->bms_fault == 0U) ? 1U : 0U;         // you don't have a recoverable bit yet
    se->charging    = 0U;         // caller sets this if needed
    se->statusColor565 = 0U;      // leave 0 if you don’t tint the status label
    se->reason[0] = '\0';
#if defined(ENABLE_BMS_SIM)
    se->interlock_ok = 1U;   // SIM: treat as OK (or mirror a GPIO if you want)
#else
    se->interlock_ok = BSP_isInterlockOK() ? 1U : 0U;  // HIGH = Closed/OK
#endif
}

static void make_details(NextionDetailsEvt *de, const BmsTelemetry *t) {
    // Voltages
    if (t->high_cell_V >= 2.0f && t->high_cell_V <= 4.6f)
        de->high_voltage_V = t->high_cell_V;
    if (t->low_cell_V >= 2.0f && t->low_cell_V <= 4.6f)
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
    strncpy(de->bms_state_str,
        BMS_state_to_text(t->battery_type_code, t->bms_state),
        sizeof(de->bms_state_str)-1);
    de->bms_state_str[sizeof(de->bms_state_str)-1] = '\0';

    if (t->bms_fault == 0U) {
        strcpy(de->bms_fault_str, "None");
    } else {
        char reasons[128];
        BmsSeverity sev;
        decode_faults_for_ui(t, reasons, sizeof(reasons), &sev);
        snprintf(de->bms_fault_str, sizeof(de->bms_fault_str),
                 "%s (0x%02X)", reasons, t->bms_fault);
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

// --- FORCE versions: ignore rate limits & de-dupe hashes ---
static void post_summary_force(ControllerAO *me, bool charging, char const *reason) {
    // build (no ui_ok_now_sum, no hash compare)
    NextionSummaryEvt *se = Q_NEW(NextionSummaryEvt, NEX_REQ_UPDATE_SUMMARY_SIG);
    make_summary(se, &me->last);
    se->charging = charging ? 1U : 0U;
    if (reason && reason[0]) {
        strncpy(se->reason, reason, sizeof(se->reason)-1);
        se->reason[sizeof(se->reason)-1] = '\0';
    } else {
        se->reason[0] = '\0';
    }
    if (!QACTIVE_POST_X(AO_Nextion, &se->super, QF_NO_MARGIN, &me->super)) {
        QF_gc(&se->super);
    }
}

static void post_details_force(ControllerAO *me) {
    NextionDetailsEvt *de = Q_NEW(NextionDetailsEvt, NEX_REQ_UPDATE_DETAILS_SIG);
    make_details(de, &me->last);
    if (!QACTIVE_POST_X(AO_Nextion, &de->super, QF_NO_MARGIN, &me->super)) {
        QF_gc(&de->super);
    }
}
static void post_charging_page(ControllerAO *me, uint8_t force_page) {
    NextionChargeEvt *ce = Q_NEW(NextionChargeEvt, NEX_REQ_UPDATE_CHARGE_SIG);

    // decide recovery vs charge (REAL only; SIM pick one)
#if !defined(ENABLE_BMS_SIM)
    BattClassResult cr = batt_classify(&me->last, /*bms_sim_active=*/false);
    ce->is_recovery = (cr.cls == BATT_CLASS_RECOVERABLE) ? 1U : 0U;
#else
    ce->is_recovery = 0U;
#endif

    ce->show_page = force_page ? 1U : 0U;

    // timers
    uint32_t now = HAL_GetTick();
    uint32_t elapsed_s = (now - me->charge_start_ms) / 1000U;
    uint32_t left_s = (elapsed_s >= me->charge_total_s) ? 0U : (me->charge_total_s - elapsed_s);
    ce->elapsed_s   = (uint16_t)elapsed_s;
    ce->time_left_s = (uint16_t)left_s;

    // battery telemetry (use what you already compute for details)
    ce->pack_v   = me->last.array_voltage_V;
    ce->h_v      = me->last.high_cell_V;
    ce->l_v      = me->last.low_cell_V;
    ce->a_v      = me->last.array_voltage_V; // you said avg uses array as coarse
    ce->h_t      = me->last.sys_temp_high_C;
    ce->l_t      = me->last.sys_temp_low_C;
    ce->pack_h_t = me->last.sys_temp_high_C;
    ce->pack_l_t = me->last.sys_temp_low_C;
    ce->soc      = me->last.soc_percent;

    // bms state text
    strncpy(ce->bms_state,
            BMS_state_to_text(me->last.battery_type_code, me->last.bms_state),
            sizeof(ce->bms_state)-1);
    ce->bms_state[sizeof(ce->bms_state)-1] = '\0';

    // errors text (reuse your existing decoder)
    {
        char faults[128];
        BmsSeverity sev;
        decode_faults_for_ui(&me->last, faults, sizeof(faults), &sev);
        strncpy(ce->errors, faults, sizeof(ce->errors)-1);
        ce->errors[sizeof(ce->errors)-1] = '\0';
    }

    // PSU snapshot (you already cache these)
    ce->psu_present = me->psu_present;
    ce->psu_out_on  = me->psu_out_on;
    ce->psu_v_out   = me->psu_v_out;
    ce->psu_i_out   = me->psu_i_out;
    ce->psu_temp    = me->psu_temp;

    if (!QACTIVE_POST_X(AO_Nextion, &ce->super, QF_NO_MARGIN, &me->super)) {
        QF_gc(&ce->super);
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
// stop charging helper
static void ctl_request_stop(ControllerAO *me, ChargeStopReason r, const char *txt) {
    if (me->stop_issued) {
        return; // already stopping/stopped
    }
    me->stop_issued = 1U;

    // remember what SW decided as the stop cause
    me->stop_req_reason = r;
    if (txt && txt[0]) {
        strncpy(me->stop_req_text, txt, sizeof(me->stop_req_text)-1);
        me->stop_req_text[sizeof(me->stop_req_text)-1] = '\0';
    } else {
        me->stop_req_text[0] = '\0';
    }

    // also keep your "last stop" fields (used by poweringDown UI)
    me->last_stop_reason = r;
    strncpy(me->last_stop_text, me->stop_req_text, sizeof(me->last_stop_text)-1);
    me->last_stop_text[sizeof(me->last_stop_text)-1] = '\0';

    ChargingStoppedEvt *ev = Q_NEW(ChargingStoppedEvt, CHARGING_STOPPED_SIG);
    ev->reason  = r;
    ev->when_ms = HAL_GetTick();
    strncpy(ev->text, me->stop_req_text, sizeof(ev->text)-1);
    ev->text[sizeof(ev->text)-1] = '\0';

    if (!QACTIVE_POST_X(&me->super, &ev->super, QF_NO_MARGIN, &me->super)) {
        QF_gc(&ev->super);
    }
}

static void post_stop_charging_page(ControllerAO *me) {
    // Show page
    post_page_ex(me, 5U);  // page id for pStopCharging

    // Build stop snapshot
    NextionStopEvt *st = Q_NEW(NextionStopEvt, NEX_REQ_UPDATE_STOP_SIG);

    // reason
    if (me->last_stop_text[0]) {
        strncpy(st->reason, me->last_stop_text, sizeof(st->reason)-1);
        st->reason[sizeof(st->reason)-1] = '\0';
    } else {
        st->reason[0] = '\0';
    }

    // voltages
    st->packV    = me->last.array_voltage_V;
    st->lowCellV = me->last.low_cell_V;

    // bms state
    strncpy(st->bms_state,
            BMS_state_to_text(me->last.battery_type_code, me->last.bms_state),
            sizeof(st->bms_state)-1);
    st->bms_state[sizeof(st->bms_state)-1] = '\0';

    // interlock
#if defined(ENABLE_BMS_SIM)
    st->interlock_ok = 1U;
#else
    st->interlock_ok = BSP_isInterlockOK() ? 1U : 0U;
#endif

    // errors
    {
        char faults[128];
        BmsSeverity sev;
        decode_faults_for_ui(&me->last, faults, sizeof(faults), &sev);
        if (faults[0]) {
            strncpy(st->errors, faults, sizeof(st->errors)-1);
            st->errors[sizeof(st->errors)-1] = '\0';
        } else {
            st->errors[0] = '\0';
        }
    }

    // charge length (seconds)
    uint32_t now_ms = HAL_GetTick();
    st->charge_len_s = (now_ms >= me->charge_start_ms)
                     ? ((now_ms - me->charge_start_ms) / 1000U)
                     : 0U;

    // Send to HMI
    if (!QACTIVE_POST_X(AO_Nextion, &st->super, QF_NO_MARGIN, &me->super)) {
        QF_gc(&st->super);
    }
}

extern float Cotek_getVout_V(void);

static float ctl_get_psu_vout_V(const ControllerAO *me) {
    // Prefer direct Cotek reading if available/valid
    float v = Cotek_getVout_V();      // <-- if you don't have this, comment it and use fallback only
    if (v > 1.0f && v < 70.0f) {
        return v;
    }

    // Fallback to cached status from PSU_RSP_STATUS_SIG
    if (me && me->psu_v_out > 1.0f && me->psu_v_out < 70.0f) {
        return me->psu_v_out;
    }

    return 0.0f; // unknown / not available
}

static void post_page_ex(ControllerAO *me, uint8_t page) {
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
        if (page == 2U) {    // pMain
            post_summary_force(me,
                (me->state == CTL_STATE_CHARGE || me->state == CTL_STATE_DETECT),
                NULL);
            // also push last-known PSU snapshot right away
            post_psu_to_hmi(me->psu_present, me->psu_out_on,
                            me->psu_v_out, me->psu_i_out, me->psu_temp);
        } else if (page == 3U) {   // pDetails
            post_details_force(me);
        }
    }
}

static void post_comms_lost(const ControllerAO *me) {
    // ensure next summary pushes through no matter what
    s_last_sum_hash = 0U;
    s_last_det_hash = 0U;

    NextionSummaryEvt *se = Q_NEW(NextionSummaryEvt, NEX_REQ_UPDATE_SUMMARY_SIG);
    make_summary(se, &me->last);

    strncpy(se->classStr, "Comms Lost!", sizeof(se->classStr)-1);
    se->classStr[sizeof(se->classStr)-1] = '\0';

    strncpy(se->reason, "Check the battery connection", sizeof(se->reason)-1);
    se->reason[sizeof(se->reason)-1] = '\0';

    se->warnIcon = 1U;
    se->charging = 0U;
    se->recoverable = 0U; // show as not recoverable while we’re blind

    if (!QACTIVE_POST_X(AO_Nextion, &se->super, QF_NO_MARGIN, &me->super)) {
        QF_gc(&se->super);
    }
}

static inline uint32_t bms_age_ms(void) {
    return tick_ms() - last_bms_ms;
}
/* coarsen an age to 100 ms buckets so we don’t spam */
static inline uint32_t age_bucket_100ms(uint32_t age_ms){
    return age_ms / 100U;
}
/* Example freshness checker used by UI + controller */
bool bms_is_fresh(void){
    uint32_t age = bms_age_ms();
    bool fresh = (age < BMS_WATCH_MS);  /* or whatever threshold you use for 'fresh' */

#if BMS_DEBUG
    if (s_prev_fresh == 255U) {
        BMS_DBG("BMSDBG: init freshness fresh=%u age=%lu ms (th=%lu)\r\n",
                (unsigned)fresh, (unsigned long)age, (unsigned long)BMS_WATCH_MS);
    } else if ((bool)s_prev_fresh != fresh) {
        BMS_DBG("BMSDBG: freshness transition %s → %s at age=%lu ms\r\n",
                s_prev_fresh ? "FRESH" : "STALE",
                fresh ? "FRESH" : "STALE",
                (unsigned long)age);
    }
    s_prev_fresh = (uint8_t)fresh;
#endif
    return fresh;
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
    QTimeEvt_ctorX(&l_ctl.tLostHold, &l_ctl.super, LOST_HOLD_TO_SIG, 0U);
    QTimeEvt_ctorX(&l_ctl.tChargeMon, &l_ctl.super, CHARGE_MON_TICK_SIG, 0U);
    QTimeEvt_ctorX(&l_ctl.tPsuReady, &l_ctl.super, PSU_READY_TIMEOUT_SIG, 0U);
    QTimeEvt_ctorX(&l_ctl.tPsuOnWait, &l_ctl.super, PSU_ON_WAIT_TO_SIG,    0U);
}

/* states */
static QState Ctl_initial(ControllerAO * const me, void const *const e) {
    (void)e;
    me->page     = 1U;   // start at pWait after splash
    me->haveData = 0U;
    memset(&me->last, 0, sizeof(me->last));
    me->psu_present = 0U;
    me->psu_out_on  = 0U;
    me->psu_v_out   = 0.0f;
    me->psu_i_out   = 0.0f;
    me->psu_temp    = 0.0f;
    me->waiting_psu_ready = 0U;
    //QTimeEvt_disarm(&me->tBmsWatch);
    /* subscribe AFTER we’re started */
    QActive_subscribe(&me->super, BMS_UPDATED_SIG);
    QActive_subscribe(&me->super, BMS_NO_BATTERY_SIG);
    QActive_subscribe(&me->super, BMS_CONN_LOST_SIG);
    QActive_subscribe(&me->super, LATCH_TURNED_ON_SIG);
    QActive_subscribe(&me->super, LATCH_TURNED_OFF_SIG);
    QActive_subscribe(&me->super, STOPBUTTON_PRESSED_SIG);
    QActive_subscribe(&me->super, PSU_READY_SIG);


#ifdef ENABLE_BMS_SIM
    printf("SIM: ENABLE_BMS_SIM is ON\r\n");
#else
    printf("SIM: ENABLE_BMS_SIM is OFF\r\n");
#endif

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
    case PSU_RSP_STATUS_SIG: {
        CotekStatusEvt const *se = (CotekStatusEvt const *)e;

        // cache
        me->psu_present = se->present;
        me->psu_out_on  = se->out_on ? 1U : 0U;
        me->psu_v_out   = se->v_out;
        me->psu_i_out   = se->i_out;
        me->psu_temp    = se->t_out;

        // if we're on pMain, repaint immediately
        if (me->page == 2U) {
            post_psu_to_hmi(me->psu_present, me->psu_out_on,
                            me->psu_v_out, me->psu_i_out, me->psu_temp);
        }
        if (me->state == CTL_STATE_CHARGE) {
            post_charging_page(me, 0U);
        }
        return Q_HANDLED();
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
        uint32_t age = bms_age_ms();
        uint32_t bucket = age_bucket_100ms(age);
        if (bucket != s_prev_age_bucket) {
            s_prev_age_bucket = bucket;
            BMS_DBG("BMSDBG: HB age=%lu ms fresh=%u haveData=%u state=%u page=%u\r\n",
                    (unsigned long)age, (unsigned)bms_is_fresh(),
                    (unsigned)me->haveData, (unsigned)me->state,
                    (unsigned)me->page);
        }
        if (!me->haveData) { return Q_HANDLED(); }  // nothing fresh → don’t overwrite banner

        if (me->page == 3U) {
            post_details(me);
        } else if (me->page == 2U) {
            post_summary(me, false, 0);
        }
        return Q_HANDLED();
    }
    case NEX_REQ_SHOW_PAGE_SIG: {  // coming FROM Nextion via Nextion_OnRx()
        NextionPageEvt const *pe = (NextionPageEvt const*)e;
        me->page = pe->page;
        s_last_sum_hash = 0U; s_last_det_hash = 0U;   // force repaint
        if (me->haveData) {
            if (me->page == 2U) {
                post_summary_force(me,
                    (me->state==CTL_STATE_CHARGE || me->state==CTL_STATE_DETECT), "");
                post_psu_to_hmi(me->psu_present, me->psu_out_on,
                                me->psu_v_out, me->psu_i_out, me->psu_temp);
            } else if (me->page == 3U) {
                post_details_force(me);
            }
        }
        return Q_HANDLED();
        }
    case BMS_CONN_LOST_SIG: {
        me->haveData = 0U;
        /* NEW: wipe last-known telemetry so UI can’t reuse stale numbers */
        memset(&me->last, 0, sizeof(me->last));

        // If user is on pDetails, switch to pMain
        if (me->page == 3U) {
            post_page_ex(me, 2U);   // pMain
        }

        // If user is on pMain, or we just switched to it, post comms-lost banner
        if (me->page == 2U) {
            post_comms_lost(me);
            QTimeEvt_armX(&me->tLostHold, 10U * BSP_TICKS_PER_SEC, 0U);
        }

        /* 1) ask PSU to turn OFF */
        QEvt *off = Q_NEW(QEvt, PSU_REQ_OFF_SIG);
        // UI/PSU requests are “best effort”: use margin 0U and GC if it can’t be queued right now
        if (!QACTIVE_POST_X(AO_Cotek, off, QF_NO_MARGIN, 0U)) {
            QF_gc(off);
        }
        // /* 2) start short timeout (e.g., 500 ms) as a guard */
        // QTimeEvt_armX(&me->tPsuOff, 50U, 0U);   /* assuming your tick is 10ms */
        /* 3) go wait for OFF confirmation, timer will be armed in the entry case */
        return Q_TRAN(&Ctl_poweringDown);
        }
    case LOST_HOLD_TO_SIG: {
        // After 10s on pMain with comms lost, show pWait and go to WAIT
        post_page_ex(me, 1U);   // pWait
    return Q_TRAN(&Ctl_wait);
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
    case Q_ENTRY_SIG: {

            printf("Ctl_wait: entry\r\n");
                //me->state = CTL_STATE_WAIT;
            return Q_HANDLED();
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

        //leave WAIT once we have data, so button logic / PSU checks live in Ctl_detect
        return Q_TRAN(&Ctl_detect);
    }
    case BMS_NO_BATTERY_SIG: {
        return Q_HANDLED();
    }
    case BMS_CONN_LOST_SIG: {
        // QTimeEvt_disarm(&me->tBmsWatch);
        // printf("CTL: BMS watchdog disarm\n");
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
        printf("Ctl_detect -> ENTRY\r\n");
        ctl_update_outputs(me);
        /* 2s UI refresh, in case we want periodic updates anyway */
        QTimeEvt_armX(&me->ui2s, BSP_TICKS_PER_SEC*2U, BSP_TICKS_PER_SEC*2U);
        /* ensure watchdog is not running */
        QTimeEvt_disarm(&me->tPsuReady);
        return Q_HANDLED();
    }
    case Q_EXIT_SIG: {
        QTimeEvt_disarm(&me->ui2s);
        /* ensure watchdog is not running */
        QTimeEvt_disarm(&me->tPsuReady);
        return Q_HANDLED();
    }
    case TIMEOUT_SIG: { /* periodic UI refresh */
        if (!me->haveData) { return Q_HANDLED(); }  // nothing fresh → don’t overwrite banner

        if (me->page == 3U) {
            post_details(me);
        } else if (me->page == 2U) {
            if (!bms_is_fresh()) {
                char why[64];
                // show age with one decimal (e.g. "No fresh BMS for 1.7 s")
                float age_s = bms_age_ms() / 1000.0f;
                snprintf(why, sizeof(why), "No fresh BMS for %.1f s", (double)age_s);
                post_summary(me, false, why);
            } else {
                post_summary(me, false, "ready to charge");
            }
        }
        printf("pMain: V=%.2fV type=0x%04X state=%u soc=%u recoverable=%u reason=\"%s\"\r\n",
                   (double)me->last.array_voltage_V,
                   (unsigned)me->last.battery_type_code,
                   (unsigned)me->last.bms_state,
                   (unsigned)me->last.soc_percent,
                   (unsigned)(me->last.bms_fault==0U),
                   "ready to charge");
        return Q_HANDLED();
    }
    case LATCH_TURNED_ON_SIG: {
#if !defined(ENABLE_BMS_SIM)
        if (!me->haveData || !bms_is_fresh()) {
            post_summary(me, false, "Start ignored: no recent BMS data");
            return Q_HANDLED();
        }
        if (!ctl_charge_allowed(me)) {
            post_summary(me, false, "Start blocked: not allowed");
            return Q_HANDLED();
        }
        if (!BSP_isInterlockOK()) {
            post_summary(me, false, "Start blocked: interlock open");
            return Q_HANDLED();
        }
#endif

        /* Show waiting banner on pMain */
        if (me->page != 2U) {
            post_page_ex(me, 2U); /* ensure pMain */
        }
        post_summary_force(me, false, "waiting the PSU to power up");

        /* Arm 5s watchdog waiting for PSU_READY_SIG */
        QTimeEvt_disarm(&me->tPsuReady);
        QTimeEvt_armX(&me->tPsuReady, 5U * BSP_TICKS_PER_SEC, 0U);

        printf("CTL: LATCH_TURNED_ON -> waiting PSU_READY (5s)\r\n");
        return Q_HANDLED();
    }
    case PSU_READY_SIG: {
        /* CotekAO confirmed PSU comms up */
        QTimeEvt_disarm(&me->tPsuReady);

        printf("CTL: PSU_READY -> transition to Ctl_charge\r\n");
        return Q_TRAN(&Ctl_charge);
        }
    case PSU_READY_TIMEOUT_SIG: {
        /* Didn’t get PSU_READY in 5s => power down flow */
        QTimeEvt_disarm(&me->tPsuReady);

        me->last_stop_reason = CHG_STOP_ELECTRICAL;
        strncpy(me->last_stop_text,
                "PSU did not become ready in 5s",
                sizeof(me->last_stop_text)-1);
        me->last_stop_text[sizeof(me->last_stop_text)-1] = '\0';

        printf("CTL: PSU_READY timeout -> powering down\r\n");
        return Q_TRAN(&Ctl_poweringDown);
        }
    case STOPBUTTON_PRESSED_SIG: {
        /* Not charging yet: just force “ready/blue” outputs if you want */
        ctl_outputs_ready_blue();
        post_summary(me, false, "Stopped (idle)");
        return Q_HANDLED();
    }
    case BMS_UPDATED_SIG: {
        BmsTelemetryEvt const *be = Q_EVT_CAST(BmsTelemetryEvt);
        me->last = be->data; me->haveData = 1U;
        ctl_update_outputs(me);
        if (me->page == 2) {
            post_summary(me, false, "ready to charge");
            post_details(me);
        }
        return Q_HANDLED();
    }
    case BMS_CONN_LOST_SIG: {
        me->haveData = 0U;
        /* NEW: wipe last-known telemetry so UI can’t reuse stale numbers */
        memset(&me->last, 0, sizeof(me->last));
        QTimeEvt_disarm(&me->tPsuReady);
        // If user is on pDetails, switch to pMain
        if (me->page == 3U) {
            post_page_ex(me, 2U);   // pMain
        }

        // If user is on pMain, or we just switched to it, post comms-lost banner
        if (me->page == 2U) {
            post_comms_lost(me);
            QTimeEvt_armX(&me->tLostHold, 10U * BSP_TICKS_PER_SEC, 0U);
        }
        return Q_HANDLED();
    }
    default: break;
    }
    return Q_SUPER(&Ctl_run);
}

/* -------- CHARGING -------- */
static QState Ctl_charge(ControllerAO * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            me->ui_div = 0U;
#ifdef ENABLE_BMS_SIM
            BmsSim_setCharging(1U);
        #endif

            me->stop_issued = 0U;
            me->stop_req_reason = CHG_STOP_NONE;
            me->stop_req_text[0] = '\0';

            me->state = CTL_STATE_CHARGE;
            me->charge_start_ms = HAL_GetTick();

            me->sw_total_s = DEFAULT_CHARGE_TIME_S;
            me->charge_total_s = me->sw_total_s;

            me->hw_extra_s = 60U;
            me->hw_deadline_ms = me->charge_start_ms
                               + (uint32_t)(me->sw_total_s + me->hw_extra_s) * 1000U;

            in_charge = true;
            ctl_update_outputs(me);

            QTimeEvt_armX(&me->tChargeMon, BSP_TICKS_PER_SEC/50U, BSP_TICKS_PER_SEC/50U);

            printf("Ctl_charge: entry\r\n");

            float v_set = 48.0f;
            float i_set = 1.0f;

        #if !defined(ENABLE_BMS_SIM)
            BattClassResult cr = batt_classify(&me->last, /*bms_sim_active=*/false);

            if (cr.cls == BATT_CLASS_NOT_RECOVERABLE) {
                post_summary(me, false, "Blocked: Not Recoverable");
                return Q_TRAN(&Ctl_detect);
            } else if (cr.cls == BATT_CLASS_RECOVERABLE) {
                i_set = 1.0f;
            } else if (cr.cls == BATT_CLASS_OPERATIONAL) {
                i_set = 3.0f;
            } else {
                post_summary(me, false, "Unknown class – cannot charge");
                return Q_TRAN(&Ctl_detect);
            }
        #else
            // SIM build default
            v_set = 48.0f;
            i_set = 1.0f;
        #endif

            me->cmd_vset = v_set;
            me->cmd_iset = i_set;
            // In ao_controller.c (or wherever you post to Cotek)
            printf("CTL: Charge start -> posting PSU setpoint V=%.2f I=%.2f\r\n",
                (double)v_set, (double)i_set);
            /* Tell PSU to apply setpoint ONCE */
            PsuSetEvt *se = Q_NEW(PsuSetEvt, PSU_REQ_SETPOINT_SIG);
            se->voltSet = me->cmd_vset;
            se->currSet = me->cmd_iset;
            // In ao_controller.c (or wherever you post to Cotek)
            printf("CTL: Charge start -> posting PSU setpoint V=%.2f I=%.2f\r\n",
                (double)v_set, (double)i_set);
            if (!QACTIVE_POST_X(AO_Cotek, &se->super, QF_NO_MARGIN, 0U)) {
                printf("CTL: FAILED to post PSU_REQ_SETPOINT (queue full)\r\n");
                QF_gc(&se->super);
            }
            me->waiting_psu_on = 1U;
            QTimeEvt_disarm(&me->tPsuOnWait);
            QTimeEvt_armX(&me->tPsuOnWait,
                          (PSU_ON_WAIT_MS * BSP_TICKS_PER_SEC) / 1000U,
                          0U);
            /* Start SW charge timer immediately (simple mode) */
            QTimeEvt_disarm(&me->tCharge);
            QTimeEvt_armX(&me->tCharge, me->sw_total_s * BSP_TICKS_PER_SEC, 0U);

            /* Ensure we’re on charge page if you want */
            post_charging_page(me, 1U);
            return Q_HANDLED();
        }
        case Q_EXIT_SIG: {
            in_charge = false;
            printf("Ctl_charge: exit\r\n");
            me->state = CTL_STATE_DETECT;
            ctl_outputs_all_off();     // RM2, RM3, RM4 are off
            QTimeEvt_disarm(&me->tCharge);
            QTimeEvt_disarm(&me->tChargeMon);
#ifdef ENABLE_BMS_SIM
            BmsSim_setCharging(0U);
#endif
            return Q_HANDLED();
        }
        case PSU_RSP_STATUS_SIG: {
            CotekStatusEvt const *se = (CotekStatusEvt const *)e;
            printf("CTL: PSU_RSP_STATUS_SIG entry\r\n");
            // cache
            me->psu_present = se->present;
            me->psu_out_on  = se->out_on ? 1U : 0U;
            me->psu_v_out   = se->v_out;
            me->psu_i_out   = se->i_out;
            me->psu_temp    = se->t_out;
            if (me->waiting_psu_on) {
                float dv = fabsf(me->psu_v_out - me->cmd_vset);
                if (me->psu_out_on && me->psu_v_out > 1.0f && dv <= PSU_VOUT_OK_MARGIN_V) {
                    me->waiting_psu_on = 0U;
                    QTimeEvt_disarm(&me->tPsuOnWait);
                    printf("CTL: PSU reached setpoint (vout=%.2f cmd=%.2f)\r\n",
                           (double)me->psu_v_out, (double)me->cmd_vset);
                }
            }
            post_charging_page(me, 0U);
            printf("CTL: PSU status present=%u out_on=%u v=%.2f cmd=%.2f wait=%u\r\n",
                me->psu_present, me->psu_out_on,
                (double)me->psu_v_out, (double)me->cmd_vset,
                me->waiting_psu_on);
            return Q_HANDLED();
        }
        case PSU_ON_WAIT_TO_SIG: {
            me->waiting_psu_on = 0U;
            ctl_request_stop(me, CHG_STOP_ELECTRICAL, "PSU Vout did not reach setpoint");
            return Q_HANDLED();
        }
        case CHARGE_MON_TICK_SIG: {
            if (++me->ui_div >= 10U) {
                me->ui_div = 0U;
                post_charging_page(me, 0U);
            }
            return Q_HANDLED();
        }
        case BMS_UPDATED_SIG: {
            BmsTelemetryEvt const *be = Q_EVT_CAST(BmsTelemetryEvt);
            me->last = be->data; me->haveData = 1U;

            /* guard: temp < 35C and no new errors */
            if (me->last.sys_temp_high_C > 40.0f || me->last.last_error_class) {
                ctl_request_stop(me, CHG_STOP_BMS_CRITICAL,
                                  "Temp > 40C");
                return Q_HANDLED();
            }
            if (me->last.low_cell_V > 0.1f && me->last.low_cell_V < LOW_CELL_STOP_V) {
                ctl_request_stop(me, CHG_STOP_CELL_UV, "Stopped: cell undervoltage");
                return Q_HANDLED();
            }
            if (me->last.last_error_class) {
                ctl_request_stop(me, CHG_STOP_BMS_CRITICAL, "BMS error");
                return Q_HANDLED();
            }
            // uint32_t now_ms = HAL_GetTick();
            // if ((now_ms - me->charge_start_ms) > PACK_PSU_GRACE_MS) {
            //     if (me->psu_out_on && me->psu_v_out > 1.0f) {
            //         if (me->last.array_voltage_V > (me->psu_v_out + PACK_GT_PSU_MARGIN_V)) {
            //             ctl_request_stop(me, CHG_STOP_PACK_GT_PSU, "Stopped: Pack Total V > PSU Vout");
            //             return Q_HANDLED();
            //         }
            //     }
            // }
            return Q_HANDLED();
        }
        case BMS_CONN_LOST_SIG: {
            ctl_request_stop(me, CHG_STOP_LOST_COMMS_BMS, "Stopped: BMS comms lost");
            return Q_HANDLED();
            // /* NEW: wipe last-known telemetry so UI can’t reuse stale numbers */
            // memset(&me->last, 0, sizeof(me->last));
            //
            // post_summary(me, false, "Stopped: BMS lost");
            // // ensure page and comms-lost banner + warn icon
            // if (me->page == 3U) { post_page_ex(me, 2U); }
            // post_comms_lost(me);
            // QTimeEvt_armX(&me->tLostHold, 10U * BSP_TICKS_PER_SEC, 0U);
            // /* 1) ask PSU to turn OFF */
            // QEvt *off = Q_NEW(QEvt, PSU_REQ_OFF_SIG);
            // // UI/PSU requests are “best effort”: use margin 0U and GC if it can’t be queued right now
            // if (!QACTIVE_POST_X(AO_Cotek, off, QF_NO_MARGIN, 0U)) {
            //     QF_gc(off);
            // }
            // // /* 2) start short timeout (e.g., 500 ms) as a guard */
            // // QTimeEvt_armX(&me->tPsuOff, 50U, 0U);   /* assuming your tick is 10ms */
            // /* 3) go wait for OFF confirmation */
            // return Q_TRAN(&Ctl_poweringDown);
        }
        case CHARGE_TIMEOUT_SIG: {
            ctl_request_stop(me, CHG_STOP_TIMEOUT_SW, " Software Timeout");
            return Q_HANDLED();
//             printf("Ctl_charge: Charge_timeout_sig\r\n");
//             // Ask PSU to turn OFF, then wait for confirmation in the substate
//             QEvt *off = Q_NEW(QEvt, PSU_REQ_OFF_SIG);
// #if         defined(ENABLE_BMS_SIM)
//             // SIM: no PSU handshake, go straight back to DETECT
//             return Q_TRAN(&Ctl_detect);
// #else       if (!QACTIVE_POST_X(AO_Cotek, off, QF_NO_MARGIN, 0U)) {
//                 QF_gc(off);
//             }
//             post_summary(me, false, "Stopped: 30s timeout");
//             return Q_TRAN(&Ctl_poweringDown);
// #endif

        }
        case STOPBUTTON_PRESSED_SIG: {
            // We *expect* latch to open; still log “user stop” as the reason
            ctl_request_stop(me, CHG_STOP_USER, "Stopped by user");
            return Q_HANDLED();
        }
        // case LATCH_TURNED_OFF_SIG: {
        //     // Latch opened => circuit dropped => stop charging
        //     if (me->stop_req_reason == CHG_STOP_NONE) {
        //         ctl_request_stop(me, CHG_STOP_ELECTRICAL, "Stopped: latch opened");
        //     }
        //     return Q_HANDLED();
        // }
        case CHARGING_STOPPED_SIG: {
            ctl_outputs_all_off();   // RM2, RM3, RM4 are off

            QEvt *off = Q_NEW(QEvt, PSU_REQ_OFF_SIG);
            if (!QACTIVE_POST_X(AO_Cotek, off, QF_NO_MARGIN, 0U)) {
                QF_gc(off);
            }
            return Q_TRAN(&Ctl_poweringDown);
        }
        default: break;
            }
    return Q_SUPER(&Ctl_run);
    }

/* -------- PSU OUTPUT OFF -------- */
static QState Ctl_poweringDown(ControllerAO * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            // Show StopCharging page + populate fields
            post_stop_charging_page(me);

            // we are no longer charging logically
            me->state = CTL_STATE_DETECT;
            ctl_outputs_all_off();  // RM2, RM3, RM4 are off

            // Ask PSU to turn OFF (idempotent)
            QEvt *off = Q_NEW(QEvt, PSU_REQ_OFF_SIG);
            if (!QACTIVE_POST_X(AO_Cotek, off, 1U, 0U)) {
                QF_gc(off);
            }
            // watchdog while waiting for OFF
            QTimeEvt_disarm(&me->tPsuOff);
            QTimeEvt_armX(&me->tPsuOff, BSP_TICKS_PER_SEC / 5U, 0U);
            return Q_HANDLED();
        }
        case PSU_RSP_STATUS_SIG: {
            // Check the status “output disabled?”
            CotekStatusEvt const *se = (CotekStatusEvt const *)e;

            /* update PSU widget */
            me->psu_present = se->present;
            me->psu_out_on  = se->out_on ? 1U : 0U;
            me->psu_v_out   = se->v_out;
            me->psu_i_out   = se->i_out;
            me->psu_temp    = se->t_out;

            if (!me->psu_out_on) {
                /* OFF confirmed; stop retry timer */
                QTimeEvt_disarm(&me->tPsuOff);
            }
            return Q_HANDLED();
        }
        case PSU_OFF_WAIT_TO_SIG: {
            /* Retry OFF occasionally */
            QEvt *off = Q_NEW(QEvt, PSU_REQ_OFF_SIG);
            if (!QACTIVE_POST_X(AO_Cotek, off, QF_NO_MARGIN, 0U)) {
                QF_gc(off);
            }
            QTimeEvt_rearm(&me->tPsuOff, BSP_TICKS_PER_SEC / 5U);
            return Q_HANDLED();
        }
        case NEX_BACK_MAIN_SIG: {
            QTimeEvt_disarm(&me->tPsuOff);
            // user pressed bBackMain on pStopCharging
            post_page_ex(me, 1U);     // pWait (or 2U if you prefer pMain)
            return Q_TRAN(&Ctl_wait);
        }
        case Q_EXIT_SIG: {
            // Stop the watchdog timer cleanly
            QTimeEvt_disarm(&me->tPsuOff);
            return Q_HANDLED();
        }
        default: break;
    }
    return Q_SUPER(&Ctl_run);  // or your actual superstate
}
