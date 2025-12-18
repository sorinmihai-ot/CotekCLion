/*****************************************************************************
* BSP for TimeBomb example with QP/C framework
*****************************************************************************/
#ifndef BSP_H
#define BSP_H
#include "qpc.h"
#include "stm32f1xx_hal.h"
#include "main.h"
#define BSP_TICKS_PER_SEC 100

bool POSTX_TRACE_TAG();

void BSP_init(void);
void BSP_start(void);
bool BSP_qfStarted(void);
/* Weak hook: where you can add prints on boot, etc. */
void BSP_print_banner(void);
void BSP_markUart2Ready(void);
void BSP_breadcrumb(uint8_t tag);     // non-blocking single marker
void BSP_die(uint8_t code);           // blink forever with code
void BSP_dumpIRQs(void);
void BSP_ledOn(void);
void BSP_ledOff(void);
void BSP_delay(uint32_t ms);

// Returns true if the dedicated 600s sense pin is high (600s connected),
// false if low (BMZ or nothing).
bool BSP_is600s_gpio_high(void);

// ---- Buttons ----
bool BSP_isStartPressed(void);   // PC0
bool BSP_isStopPressed(void);    // PC1

// ---- Relays (active-low) ----
void BSP_relay2_set(bool on);    // PB3
void BSP_relay3_set(bool on);    // PB4
void BSP_relay4_set(bool on);    // PB5

// ---- Interlock ----
bool BSP_isInterlockOK(void);    // PB12

/* Active objects... */
extern QActive *AO_Cotek;

#endif /* BSP_H */
