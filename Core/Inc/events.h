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
} NextionSummaryEvt;

/* Nextion touch/button */
typedef struct {
    QEvt super;
    uint16_t code;  /* e.g., BTN_DETAILS, BTN_BACK, etc. */
} NextionTouchEvt;

#endif //EVENTS_H