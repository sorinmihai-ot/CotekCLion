#ifndef AO_NEXTION_H
#define AO_NEXTION_H

#include "qpc.h"
#include "app_signals.h"

#ifdef __cplusplus
extern "C" {
#endif

extern QActive *const AO_Nextion;
void NextionAO_ctor(void);

// Nextion RX hook called from HAL_UARTEx_RxEventCallback()
void Nextion_OnRx(uint8_t const *buf, uint16_t len);

#ifdef __cplusplus
}
#endif
#endif
