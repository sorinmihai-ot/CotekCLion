#ifndef CAN_TX_H
#define CAN_TX_H

#include "main.h"
#include "can.h"
#include <stdint.h>
#include <stdbool.h>

// ===================== Battery families we simulate =====================
// These map to the type codes your Cotek side infers:
typedef enum {
    BATT_600S = 0,
    BATT_500S_HYPER,
    BATT_500S_BMZ,
    BATT_400S_HYPER,
    BATT_400S_DUALZ,
    BATT_400S_STEAT,
    BATT_COUNT
} BattType_e;

typedef enum {
    MODE_OPERATIONAL = 0,   // "normal / active"
    MODE_REC,               // Recovery / maintenance
    MODE_NONREC,            // Non-recoverable fault / locked out
    MODE_COUNT
} Mode_e;

typedef struct {
    float   highCell_V;      /* highest cell voltage in volts */
    float   lowCell_V;       /* lowest cell voltage in volts */
    float   pack_V;          /* pack / system voltage in volts */
    uint8_t soc_pct;         /* state of charge, % */
    int16_t tempHigh_0p1C;   /* hottest temp, 0.1°C units, signed */
    int16_t tempLow_0p1C;    /* coolest temp, 0.1°C units, signed */
    uint8_t last_error_code; /* for NR / fault mapping */
} TelemetryOut;

// public API
void CAN_TX_Init(void);

void CAN_TX_SetBatteryType(uint8_t idx); // 0..BATT_COUNT-1
void CAN_TX_SetMode(uint8_t idx);        // 0..MODE_COUNT-1

// Call from main loop ~every 10ms
void CAN_TX_PeriodicTask(bool simActive);

// Trigger from NON-CRIT btn
void CAN_TX_SendNonCritical(void);

// Trigger from CRIT btn
void CAN_TX_SendCritical(void);

#endif // CAN_TX_H
