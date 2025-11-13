//
// Created by sorin.mihai on 26/10/2025.
//
#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdint.h>
#include <stdbool.h>

// ----- battery types -----
typedef enum {
    BATT_B600 = 0,
    BATT_H500,
    BATT_B500,
    BATT_H400,
    BATT_DZB400,
    BATT_STEATITE400,
    BATT_TYPE_COUNT
} BatteryType_t;

// ----- operating modes -----
typedef enum {
    MODE_OPERATIONAL = 0,
    MODE_REC,
    MODE_NON_REC,
    MODE_COUNT
} Mode_t;

// ----- high-level UI steps -----
typedef enum {
    UI_BOOT = 0,          // "Ocado Technology Battery Simulator" splash
    UI_SELECT_BATT,       // choose battery type
    UI_SELECT_MODE,       // choose mode
    UI_RUNNING,           // sending CAN frames + can inject faults
} UiScreen_t;

// app data we’ll keep globally (but not expose directly)
typedef struct {
    UiScreen_t   screen;
    BatteryType_t selectedBatt;
    Mode_t        selectedMode;
    bool          can_active;        // true once we start sending frames
    bool          inject_noncrit;    // user toggled non-critical faults
    bool          inject_crit;       // user toggled critical faults
    uint32_t      ms_since_boot;     // simple tick counter for splash timeout
    bool          dirty;             // set when LCD should be refreshed
} AppState_t;

// --- lifecycle ---
void App_Init(void);             // call once at startup
void App_TickMs(uint32_t ms);    // call every ms (or ~periodically) to advance timers

// --- button-driven actions ---
// advance battery selection (only valid in UI_SELECT_BATT)
void App_NextBattery(void);

// user confirmed battery selection, go to mode select
void App_SelectBattery(void);

// advance mode selection (only valid in UI_SELECT_MODE)
void App_NextMode(void);

// confirm mode -> go RUNNING (starts CAN)
void App_SelectMode(void);

// toggle faults while running
void App_ToggleNonCritical(void);
void App_ToggleCritical(void);

// go back to battery menu, stop CAN
void App_BackToMain(void);

// --- getters we’ll use to print to LCD / drive CAN ---
UiScreen_t   App_GetScreen(void);
BatteryType_t App_GetBattery(void);
Mode_t        App_GetMode(void);
bool          App_CanActive(void);
bool          App_NonCritActive(void);
bool          App_CritActive(void);

// did something change that needs LCD redraw?
bool App_IsDirty(void);
void App_ClearDirty(void);

#endif // APP_STATE_H
