#ifndef AO_NEXTION_H
#define AO_NEXTION_H

#include "qpc.h"
#include "app_signals.h"

#ifdef __cplusplus
extern "C" {
#endif

    extern QActive *const AO_Nextion;
    void NextionAO_ctor(void);

    /* Call this from HAL_UARTEx_RxEventCallback(huart2) */
    void Nextion_OnRx(const uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif
#endif
