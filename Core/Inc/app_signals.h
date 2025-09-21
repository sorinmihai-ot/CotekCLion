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
    BMS_TICK_SIG,              /* internal periodic tick for BMS            */

    MAX_PUB_SIG,               // sentinel for QF_psInit only
    /* HMI <-> Controller (direct posts) */
    BOOT_SIG = MAX_PUB_SIG + 1,
    NEX_READY_SIG,             /* Nextion AO -> Controller                   */
    NEX_REQ_SHOW_PAGE_SIG,     /* Controller -> Nextion                      */
    NEX_REQ_UPDATE_SUMMARY_SIG,/* Controller -> Nextion                      */

    /* PSU control/status (direct posts) */
    PSU_REQ_SETPOINT_SIG,      /* Controller -> Cotek                        */
    PSU_REQ_OFF_SIG,           /* Controller -> Cotek                        */
    PSU_RSP_STATUS_SIG,        /* Cotek -> Controller                        */

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

/* Nextion: change page */
typedef struct {
    QEvt super;
    uint8_t page;  /* 0=splash,1=wait,2=main,3=details */
} NextionPageEvt;

/* Nextion: summary payload for pMain */
typedef struct {
    QEvt super;
    char      battTypeStr[24];
    uint16_t  typeColor565;
    float     packV;
    char      statusStr[20];
    uint16_t  statusColor565;
    char      errors[64];
    bool      warnIcon;
    bool      recoverable;
    char      reason[96];
    bool      charging;       /* NEW: show “charging” banner */
} NextionSummaryEvt;

/* compile-time sanity */
Q_ASSERT_COMPILE(sizeof(CanFrameEvt)     >= sizeof(QEvt));
Q_ASSERT_COMPILE(sizeof(BmsTelemetryEvt) >= sizeof(QEvt));

#endif /* APP_SIGNALS_H */
