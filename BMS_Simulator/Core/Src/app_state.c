//
// Created by sorin.mihai on 26/10/2025.
//
#include "app_state.h"

static AppState_t g;

// nice string tables for LCD later
static const char* battNames[BATT_TYPE_COUNT] = {
    "B600",
    "H500",
    "B500",
    "H400",
    "DZB400",
    "STEATITE400"
};

static const char* modeNames[MODE_COUNT] = {
    "OPERATIONAL",
    "REC",
    "NON-REC"
};

void App_Init(void)
{
    g.screen        = UI_BOOT;
    g.selectedBatt  = BATT_B600;
    g.selectedMode  = MODE_OPERATIONAL;
    g.can_active    = false;
    g.inject_noncrit= false;
    g.inject_crit   = false;
    g.ms_since_boot = 0;
    g.dirty         = true; // first draw
}

void App_TickMs(uint32_t ms)
{
    g.ms_since_boot += ms;

    if (g.screen == UI_BOOT) {
        // after 5 seconds -> go to battery select
        if (g.ms_since_boot >= 5000) {
            g.screen = UI_SELECT_BATT;
            g.dirty  = true;
        }
    }
}

// ----------- user actions --------------

void App_NextBattery(void)
{
    if (g.screen == UI_SELECT_BATT) {
        g.selectedBatt = (BatteryType_t)((g.selectedBatt + 1) % BATT_TYPE_COUNT);
        g.dirty = true;
    }
}

void App_SelectBattery(void)
{
    if (g.screen == UI_SELECT_BATT) {
        g.screen       = UI_SELECT_MODE;
        g.selectedMode = MODE_OPERATIONAL;
        g.dirty        = true;
    }
}

void App_NextMode(void)
{
    if (g.screen == UI_SELECT_MODE) {
        g.selectedMode = (Mode_t)((g.selectedMode + 1) % MODE_COUNT);
        g.dirty = true;
    }
}

void App_SelectMode(void)
{
    if (g.screen == UI_SELECT_MODE) {
        g.screen     = UI_RUNNING;
        g.can_active = true;
        g.dirty      = true;
        // later: start CAN periodic transmit
    }
}

void App_ToggleNonCritical(void)
{
    if (g.screen == UI_RUNNING) {
        g.inject_noncrit = !g.inject_noncrit;
        g.dirty = true;
    }
}

void App_ToggleCritical(void)
{
    if (g.screen == UI_RUNNING) {
        g.inject_crit = !g.inject_crit;
        g.dirty = true;
    }
}

void App_BackToMain(void)
{
    // always allowed: kills CAN, back to batt select
    g.screen        = UI_SELECT_BATT;
    g.can_active    = false;
    g.inject_noncrit= false;
    g.inject_crit   = false;
    g.dirty         = true;
}

// ----------- getters --------------

UiScreen_t App_GetScreen(void)        { return g.screen; }
BatteryType_t App_GetBattery(void)    { return g.selectedBatt; }
Mode_t App_GetMode(void)              { return g.selectedMode; }
bool App_CanActive(void)              { return g.can_active; }
bool App_NonCritActive(void)          { return g.inject_noncrit; }
bool App_CritActive(void)             { return g.inject_crit; }

bool App_IsDirty(void)                { return g.dirty; }
void App_ClearDirty(void)             { g.dirty = false; }

// (Optional helpers for later render/can code)
const char* App_GetBatteryName(void)
{
    return battNames[g.selectedBatt];
}

const char* App_GetModeName(void)
{
    return modeNames[g.selectedMode];
}
