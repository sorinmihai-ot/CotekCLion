//
// Created by sorin.mihai on 28/08/2025.
//

#ifndef CAN_APP_H
#define CAN_APP_H

#include "stm32f1xx_hal.h"
#include "app_signals.h"
#include "bms_app.h"   /* for AO_Bms and CanFrameEvt */
#include "qpc.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    extern CAN_HandleTypeDef hcan;

    /* Bring up CAN, configure an "accept-all" filter into FIFO0,
     * start the peripheral and enable FIFO0-msg-pending interrupt. */
    void CANAPP_InitAll(void);

#ifdef __cplusplus
}
#endif
#endif /* CAN_APP_H */