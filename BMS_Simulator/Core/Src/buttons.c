//
// Created by sorin.mihai on 26/10/2025.
//
#include "buttons.h"
#include "stm32l4xx_hal.h"

// map ButtonId -> actual GPIO port/pin
typedef struct {
    GPIO_TypeDef* port;
    uint16_t      pin;
} ButtonHW;

static const ButtonHW hwMap[BTN_COUNT] = {
    [BTN_BATT]    = {GPIOC, GPIO_PIN_0},
    [BTN_MODE]    = {GPIOC, GPIO_PIN_1},
    [BTN_NONCRIT] = {GPIOC, GPIO_PIN_2},
    [BTN_CRIT]    = {GPIOC, GPIO_PIN_3},
    [BTN_SELECT]  = {GPIOC, GPIO_PIN_4},
    [BTN_BACK]    = {GPIOC, GPIO_PIN_5},
};

// simple debounce state
// We assume active-low buttons (pressed = 0)
static uint8_t stableState[BTN_COUNT];    // 0=released,1=pressed
static uint8_t lastStableState[BTN_COUNT];
static uint8_t counter[BTN_COUNT];        // counts consecutive samples
static uint8_t pressedEdge[BTN_COUNT];    // latched edge flag

#define DEBOUNCE_THRESHOLD 3  // how many consecutive identical reads (3 * 10ms = ~30ms)

void Buttons_Init(void)
{
    for (int i = 0; i < BTN_COUNT; i++) {
        uint32_t raw = HAL_GPIO_ReadPin(hwMap[i].port, hwMap[i].pin);
        uint8_t logicalPressed = (raw == GPIO_PIN_RESET) ? 1 : 0;

        stableState[i]     = logicalPressed;
        lastStableState[i] = logicalPressed;
        counter[i]         = 0;
        pressedEdge[i]     = 0;
    }
}

void Buttons_Update(void)
{
    for (int i = 0; i < BTN_COUNT; i++) {
        uint32_t raw = HAL_GPIO_ReadPin(hwMap[i].port, hwMap[i].pin);
        uint8_t logicalPressed = (raw == GPIO_PIN_RESET) ? 1 : 0;

        if (logicalPressed == stableState[i]) {
            // no change, reset counter
            counter[i] = 0;
        } else {
            // possible change
            counter[i]++;
            if (counter[i] >= DEBOUNCE_THRESHOLD) {
                // accept new state
                lastStableState[i] = stableState[i];
                stableState[i]     = logicalPressed;
                counter[i]         = 0;

                // latch edge if we just went released -> pressed
                if (stableState[i] == 1 && lastStableState[i] == 0) {
                    pressedEdge[i] = 1;
                }
            }
        }
    }
}

bool Button_WasPressed(ButtonId id)
{
    if (id >= BTN_COUNT) return false;
    if (pressedEdge[id]) {
        pressedEdge[id] = 0;
        return true;
    }
    return false;
}
