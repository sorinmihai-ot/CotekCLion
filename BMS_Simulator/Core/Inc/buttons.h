//
// Created by sorin.mihai on 26/10/2025.
//
#ifndef BUTTONS_H
#define BUTTONS_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// Logical IDs for the 6 buttons
typedef enum {
    BTN_BATT = 0,     // PC0
    BTN_MODE,         // PC1
    BTN_NONCRIT,      // PC2
    BTN_CRIT,         // PC3
    BTN_SELECT,       // PC4
    BTN_BACK,         // PC5
    BTN_COUNT
} ButtonId;

// Call once at startup
void Buttons_Init(void);

// Call periodically (e.g. every 10ms in main loop)
// This updates internal debounce state.
void Buttons_Update(void);

// Edge query: returns true ONCE when a button transitions
// from released -> pressed (i.e. when user just pressed it).
bool Button_WasPressed(ButtonId id);

#endif // BUTTONS_H
