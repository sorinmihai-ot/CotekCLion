//
// 4-AO project: application signals & event payloads
//
#ifndef APP_SIGNALS_H
#define APP_SIGNALS_H

#include <stdint.h>
#include <stdbool.h>
#include "qpc.h"


/* Signals (publishable first; order matters for MAX_PUB_SIG) */
enum AppSignals {
    /* ====== PUBLISHED signals (must be < MAX_PUB_SIG) ====== */
    CAN_RX_SIG = Q_USER_SIG,   /* from CAN ISR -> publish; BMS subscribes   */
    BMS_UPDATED_SIG,           /* BMS publishes telemetry snapshots         */
    BMS_NO_BATTERY_SIG,        /* BMS publishes: no traffic yet             */
    BMS_CONN_LOST_SIG,         /* BMS publishes: data stale                 */



    /* ====== NOT published (direct posts / time events) ====== */
    TIMEOUT_SIG , /* used by QTimeEvt; not published           */
    CHARGE_TIMEOUT_SIG,      // <-- new: 30s charge window elapsed
    PSU_OFF_WAIT_TO_SIG,     // internal: watchdog while we wait for OFF confirmation
    BMS_TICK_SIG,              /* internal periodic tick for BMS            */
#ifdef ENABLE_BMS_SIM
    SIM_TICK_SIG,          /* private periodic tick for the in-firmware BMS simulator */
#endif
    MAX_PUB_SIG,               // sentinel for QF_psInit only
    /* HMI <-> Controller (direct posts) */
    BOOT_SIG = MAX_PUB_SIG + 1,
    NEX_READY_SIG,             /* Nextion AO -> Controller                   */
    NEX_REQ_SHOW_PAGE_SIG,     /* Controller -> Nextion                      */
    NEX_REQ_UPDATE_SUMMARY_SIG,/* Controller -> Nextion*/
    NEX_REQ_UPDATE_LIVE_SIG,
    NEX_REQ_UPDATE_DETAILS_SIG,
    NEX_REQ_UPDATE_PSU_SIG,

    /* PSU control/status (direct posts) */
    PSU_REQ_SETPOINT_SIG,      /* Controller -> Cotek                        */
    PSU_REQ_OFF_SIG,           /* Controller -> Cotek                        */
    PSU_RSP_STATUS_SIG,        /* Cotek -> Controller  */
    // ---- Cotek status broadcast (AO_Cotek -> AO_Controller) ----
    COTEK_STATUS_SIG,     // carries PSU presence, out state, and latest readings
    COTEK_TICK_SIG,

    /* Board button (direct posts) */
    BUTTON_PRESSED_SIG,
    BUTTON_RELEASED_SIG,
};


/* CAN frame event posted from HAL CAN ISR */
typedef struct {
    QEvt    super;      /* MUST be 1st */
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
    uint8_t  isExt;     /* 0=std,1=ext */
} CanFrameEvt;

/* BMS telemetry (unified) */
typedef struct {
    uint32_t serial_number;
    uint32_t firmware_version;

    uint8_t  bms_state;
    uint8_t  bms_fault;

    float    array_voltage_V;
    float    high_cell_V;
    float    low_cell_V;

    uint8_t  soc_percent;

    float    sys_temp_high_C;
    float    sys_temp_low_C;

    uint16_t fan_rpm;

    uint8_t  last_error_class;
    uint8_t  last_error_code;

    uint16_t battery_type_code;  /* 0x400=400s, 0x500=500s, 0x600=600s */
    int16_t  current_dA;         /* deci-amps (+ charge, - discharge) */
} BmsTelemetry;

/* Published telemetry event */
typedef struct {
    QEvt        super;
    BmsTelemetry data;
    uint8_t     _pad[8];   /* keep larger than CanFrameEvt */
} BmsTelemetryEvt;

/* PSU setpoint request */
typedef struct {
    QEvt super;
    float voltSet;   /* V */
    float currSet;   /* A */
} PsuSetEvt;

/* PSU status response (stub for now) */
typedef struct {
    QEvt    super;
    bool    powerOn;
    float   outV;
    float   outI;
    uint16_t statusWord;
    uint16_t faultWord;
} PsuStatusEvt;
// posted by AO_Cotek whenever its view of the PSU changes or on periodic refresh
typedef struct {
    QEvt super;
    uint8_t present;     // 1 = PSU detected, 0 = missing
    uint8_t out_on;      // 1 = output ON, 0 = OFF/unknown
    float   v_out;       // PSU output voltage (V), if available
    float   i_out;       // PSU output current (A), if available
    float   t_out;       // PSU temperature (°C), if available
} CotekStatusEvt;
typedef struct {
    QEvt super;
    uint8_t present;     /* 1/0 */
    uint8_t output_on;   /* 1/0 */
    float   v_out;       /* V */
    float   i_out;       /* A */
    float   temp_C;      /* degC (if you have it; else fill NAN) */
} NextionPsuEvt;

/* Nextion: change page */
typedef struct {
    QEvt super;
    uint8_t page;  /* 0=splash,1=wait,2=main,3=details */
} NextionPageEvt;

/* Nextion: summary payload for pMain */
typedef struct {
    QEvt super;
    // Summary values for pMain
    char      battTypeStr[12];     // "400s","500s","600s","Unknown"
    uint16_t  typeColor565;        // 2016 / 65504 / 1023 / 63488
    float     packV;               // from BmsTelemetry.array_voltage_V

    char      statusStr[24];       // from bms_state_str(bms_state)
    uint16_t  statusColor565;      // optional (set 0 if you don’t use a color on HMI)

    char      errors[48];          // short combined error summary or "" if none
    uint8_t   warnIcon;            // show “warning” icon? 0/1
    uint8_t   recoverable;         // show “recoverable” icon? 0/1
    uint8_t   charging;            // show “charging” group? 0/1

    // ---- PSU (Cotek) summary for pMain ----
    uint8_t psu_present;   // 1/0
    uint8_t psu_out_on;    // 1/0
    float   psu_v;         // volts
    float   psu_i;         // amps
    float   psu_t;         // degC

    // Optional “reason” breadcrumb you print to UART only
    char      reason[96];
} NextionSummaryEvt;
// ----- details shown on pDetails -----
typedef struct {
    QEvt super;

    // Details page (pDetails) – names mirror your labels
    float high_voltage_V;
    float low_voltage_V;
    float avg_voltage_V;

    float high_temp_C;
    float low_temp_C;
    float pack_high_temp_C;
    float pack_low_temp_C;

    char  serial_number[24];
    char  firmware[16];

    uint16_t fan_speed_rpm;

    uint8_t soc_percent;
    uint8_t soc2_percent;

    char  bms_state_str[24];   // text for current state
    char  bms_fault_str[48];   // combined faults text
} NextionDetailsEvt;

typedef struct {
    QEvt super;
    // BMS
    float packV;          // V
    float packA;          // A (positive=discharge, negative=charge) or as you prefer
    uint8_t soc;          // %
    int16_t tempC;        // main temp (or average)
    uint8_t bms_present;  // 1=present, 0=lost/stale
    // --- BMS details (pDetails) ---
    float   vMinCell;       // V
    float   vMaxCell;       // V
    float   vDeltaCell;     // V (max-min)
    int16_t tMinC;          // °C
    int16_t tMaxC;          // °C
    uint16_t cycles;        // charge cycles
    uint32_t faultsMask;    // your bitfield, if you have one
    // COTEK
    uint8_t cotek_present;  // 1=psu detected
    uint8_t cotek_out_on;   // 1=output enabled
    float cotek_V;          // V (readback)
    float cotek_I;          // A (readback)
    int16_t cotek_T;        // C (optional)
} NextionLiveEvt;

/* compile-time sanity */
Q_ASSERT_COMPILE(sizeof(CanFrameEvt)     >= sizeof(QEvt));
Q_ASSERT_COMPILE(sizeof(BmsTelemetryEvt) >= sizeof(QEvt));

#endif /* APP_SIGNALS_H */
