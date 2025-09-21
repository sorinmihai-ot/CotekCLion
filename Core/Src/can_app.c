//
// Created by sorin.mihai on 28/08/2025.
//
// can_app.c
#include "qpc_cfg.h"
#include "can_app.h"
#include "bms_app.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "bsp.h"


Q_DEFINE_THIS_FILE

extern QActive *AO_Bms;
extern volatile uint16_t g_lastSig;
extern volatile uint8_t  g_lastTag;

static void print_hal(const char *tag, HAL_StatusTypeDef st) {
    printf("%s: %s\r\n", tag,
           st==HAL_OK?"OK":st==HAL_ERROR?"ERR":st==HAL_BUSY?"BUSY":"TIMEOUT");
}

void CANAPP_InitAll(void) {
    // Assume MX_CAN_Init() already set timing. Just do filter + start + IRQs here.

    // --- Accept-all filter into FIFO0 (standard + extended) ---
    CAN_FilterTypeDef f = {0};
    f.FilterBank           = 0;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;
    f.FilterMaskIdLow      = 0x0000;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14; // not used on F1 single CAN, but set anyway

    print_hal("CAN ConfigFilter", HAL_CAN_ConfigFilter(&hcan, &f));

    // --- Start peripheral ---
    print_hal("CAN Start", HAL_CAN_Start(&hcan));

    // --- Enable RX FIFO0 pending + basic error notifs ---
    uint32_t it = CAN_IT_RX_FIFO0_MSG_PENDING
                | CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE
                | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR;
    print_hal("CAN ActivateNotif", HAL_CAN_ActivateNotification(&hcan, it));

    printf("CAN ready (FIFO0 IRQ)\r\n");
}

// RX callback from HAL (called in IRQ context)
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hh) {
    CAN_RxHeaderTypeDef rxh;
    uint8_t data[8];
    if (HAL_CAN_GetRxMessage(hh, CAN_RX_FIFO0, &rxh, data) != HAL_OK) {
        printf("CAN RX: HAL_GetRxMessage ERR\r\n");
        return;
    }
    printf("CAN RX: id=0x%08lX IDE=%lu DLC=%lu\r\n",
           (unsigned long)rxh.StdId | ((unsigned long)rxh.ExtId<<3), // quick print
           (unsigned long)rxh.IDE, (unsigned long)rxh.DLC);

    // build event and POST to BMS AO (margin=1 from ISR!)
    CanFrameEvt *e = Q_NEW(CanFrameEvt, CAN_RX_SIG);
    e->id     = (rxh.IDE==CAN_ID_STD) ? rxh.StdId : rxh.ExtId;
    e->isExt    = (rxh.IDE==CAN_ID_EXT) ? 1U : 0U;
    e->dlc    = (uint8_t)rxh.DLC;
    memset(e->data, 0, sizeof(e->data));
    if (e->dlc > 8) e->dlc = 8;
    memcpy(e->data, data, e->dlc);
    g_lastSig = CAN_RX_SIG; g_lastTag = 10;  // tag 10 = CAN RX
    if (!QACTIVE_POST_X(AO_Bms, &e->super, 3U, 0U)) { // 1-slot margin
        QF_gc(&e->super);
    }
}

// Optional: error callback for visibility
void HAL_CAN_ErrorCallback(const CAN_HandleTypeDef *hh) {
    uint32_t e = HAL_CAN_GetError(hh);
    printf("CAN ERR: 0x%08lX\r\n", (unsigned long)e);
}
