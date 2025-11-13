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

/* Active objects... */
extern QActive *AO_Cotek;

#endif /* BSP_H */
