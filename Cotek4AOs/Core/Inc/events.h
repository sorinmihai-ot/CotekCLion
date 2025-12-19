//
// Created by sorin.mihai on 17/09/2025.
//

#ifndef EVENTS_H
#define EVENTS_H

#include "qpc.h"
#include <stdint.h>
#include <stdbool.h>

/* Raw CAN frame from ISR */
typedef struct {
    QEvt super;
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
    bool     isExt;
} CanFrameEvt;

/* Telemetry snapshot published by BMS_AO */
typedef struct {
    QEvt super;
    float packV;
    float current;
    int   soc;         /* 0..100 */
    int   soh;         /* 0..100 */
    float tMinC;
    float tMaxC;
    float cellMinV;
    float cellMaxV;
    float cellDeltaV;
    bool  fault;
    char  errors[64];  /* short CSV reason(s) */
    uint8_t battType;  /* 0:unknown, 1:400s, 2:500s Hyperdrive, 3:500s BMZ, 4:600s */
} BmsSnapshotEvt;

/* Controller→Cotek setpoints */
typedef struct {
    QEvt super;
    float voltSet;     /* V */
    float currSet;     /* A */
} PsuSetEvt;

/* Cotek→Controller status/fault */
typedef struct {
    QEvt super;
    bool  powerOn;
    float outV;
    float outI;
    uint16_t statusWord;
    uint16_t faultWord;
} PsuStatusEvt;

/* Controller→Nextion page change */
typedef struct {
    QEvt super;
    uint8_t page;      /* 0=splash,1=wait,2=main,3=details */
} NextionPageEvt;

/* Controller→Nextion summary update (pMain) */
typedef struct {
    QEvt super;
    char  battTypeStr[24];
    uint16_t typeColor565;
    float packV;
    char  statusStr[16];
    uint16_t statusColor565;
    char  errors[64];
    bool  warnIcon;
    bool  recoverable;
    char  reason[96];
    uint8_t interlock_ok;   // 1 = Closed/OK, 0 = Open
} NextionSummaryEvt;

typedef struct {
    QEvt super;

    // page control
    uint8_t show_page;       // 1 = force pCharging, 0 = don't force

    // mode/status
    uint8_t is_recovery;     // 1=recovery, 0=charging
    char    bms_state[24];   // text shown in tBmsState
    char    errors[96];      // text shown in tErrors (readable)

    // timers (seconds)
    uint16_t time_left_s;    // for tTimeLeft
    uint16_t elapsed_s;      // for tPsuOnline

    // Battery telemetry (strings or numbers)
    float pack_v;            // tPackV
    float h_v;               // tHVolt
    float l_v;               // tLVolt
    float a_v;               // tAVolt
    float h_t;               // tHTemp
    float l_t;               // tLTemp
    float pack_h_t;          // tPackHTemp
    float pack_l_t;          // tPackLTemp
    uint8_t soc;             // tSoC

    // PSU status
    uint8_t psu_present;     // tPsu + color
    uint8_t psu_out_on;      // tOutState + color
    float   psu_v_out;       // tOutV
    float   psu_i_out;       // tOutI
    float   psu_temp;        // tPsuTemp
} NextionChargeEvt;

/* Nextion touch/button */
typedef struct {
    QEvt super;
    uint16_t code;  /* e.g., BTN_DETAILS, BTN_BACK, etc. */
} NextionTouchEvt;

#endif //EVENTS_H