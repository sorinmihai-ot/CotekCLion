// can_app.c
#include "qpc_cfg.h"
#include "can_app.h"
#include "bms_app.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "bsp.h"
#include <math.h>
#include <stdbool.h>
#include "bms_debug.h"

Q_DEFINE_THIS_FILE

extern CAN_HandleTypeDef hcan;
extern volatile uint16_t g_lastSig;
extern volatile uint8_t  g_lastTag;
static volatile uint8_t s_rxEnabled = 0u;

/* ---------- helpers ---------- */
static void print_hal(const char *tag, HAL_StatusTypeDef st) {
    printf("%s: %s\r\n", tag,
           st==HAL_OK?"OK":st==HAL_ERROR?"ERR":st==HAL_BUSY?"BUSY":"TIMEOUT");
}
static void pack_ext_filter(uint32_t id, uint32_t mask,
                            uint32_t *idh, uint32_t *idl,
                            uint32_t *mh,  uint32_t *ml)
{
    uint32_t fid  = ((id   & 0x1FFFFFFFu) << 3) | (1u << 2);
    uint32_t fmsk = ((mask & 0x1FFFFFFFu) << 3) | (1u << 2);
    *idh = (fid  >> 16) & 0xFFFFu;
    *idl = (fid         & 0xFFFFu);
    *mh  = (fmsk >> 16) & 0xFFFFu;
    *ml  = (fmsk        & 0xFFFFu);
}
static inline bool can_id_is_expected(uint32_t id, uint8_t isExt) {
    if (!isExt) return false;
    if ((id & 0xFFFF0000u) == 0x18FF0000u) return true; /* 500 HYP */
    if ((id & 0xFFFF0000u) == 0x10000000u) return true; /* 600s / 500 BMZ */
    if (id == 0x18070800u || id == 0x18060800u || id == 0x180C0800u) return true; /* 400s */
    if ((id & 0xFFFFFF00u) == 0x18000800u) return true; /* 400s cell pages A master/slave */
    if ((id & 0xFFFFFF00u) == 0x18010800u) return true; /* 400s cell pages B master/slave */
    if (id == 0x18040A00u) return true; /* 400s params/type */
    return false;
}

/* ---------- init ---------- */
void CANAPP_InitAll(void) {
    /* Filters (EXT only) into FIFO0 */
    {
        CAN_FilterTypeDef f = {0};
        f.FilterBank = 0; f.FilterMode = CAN_FILTERMODE_IDMASK; f.FilterScale = CAN_FILTERSCALE_32BIT;
        pack_ext_filter(0x18FF0000u, 0xFFFF0000u, &f.FilterIdHigh, &f.FilterIdLow, &f.FilterMaskIdHigh, &f.FilterMaskIdLow);
        f.FilterFIFOAssignment = CAN_RX_FIFO0; f.FilterActivation = ENABLE; f.SlaveStartFilterBank = 14;
        print_hal("CAN ConfigFilter (0x18FFxxxx)", HAL_CAN_ConfigFilter(&hcan, &f));
    }
    {
        CAN_FilterTypeDef f = {0};
        f.FilterBank = 1; f.FilterMode = CAN_FILTERMODE_IDMASK; f.FilterScale = CAN_FILTERSCALE_32BIT;
        pack_ext_filter(0x10000000u, 0xFFFF0000u, &f.FilterIdHigh, &f.FilterIdLow, &f.FilterMaskIdHigh, &f.FilterMaskIdLow);
        f.FilterFIFOAssignment = CAN_RX_FIFO0; f.FilterActivation = ENABLE; f.SlaveStartFilterBank = 14;
        print_hal("CAN ConfigFilter (0x100000xx)", HAL_CAN_ConfigFilter(&hcan, &f));
    }
    {
        CAN_FilterTypeDef f = {0};
        f.FilterBank = 2; f.FilterMode = CAN_FILTERMODE_IDMASK; f.FilterScale = CAN_FILTERSCALE_32BIT;
        pack_ext_filter(0x18070800u, 0x1FFFFFFFu, &f.FilterIdHigh, &f.FilterIdLow, &f.FilterMaskIdHigh, &f.FilterMaskIdLow);
        f.FilterFIFOAssignment = CAN_RX_FIFO0; f.FilterActivation = ENABLE; f.SlaveStartFilterBank = 14;
        print_hal("CAN ConfigFilter (0x18070800)", HAL_CAN_ConfigFilter(&hcan, &f));
    }
    {
        CAN_FilterTypeDef f = {0};
        f.FilterBank = 3; f.FilterMode = CAN_FILTERMODE_IDMASK; f.FilterScale = CAN_FILTERSCALE_32BIT;
        pack_ext_filter(0x18060800u, 0x1FFFFFFFu, &f.FilterIdHigh, &f.FilterIdLow, &f.FilterMaskIdHigh, &f.FilterMaskIdLow);
        f.FilterFIFOAssignment = CAN_RX_FIFO0; f.FilterActivation = ENABLE; f.SlaveStartFilterBank = 14;
        print_hal("CAN ConfigFilter (0x18060800)", HAL_CAN_ConfigFilter(&hcan, &f));
    }
    {
        CAN_FilterTypeDef f = {0};
        f.FilterBank = 4; f.FilterMode = CAN_FILTERMODE_IDMASK; f.FilterScale = CAN_FILTERSCALE_32BIT;
        pack_ext_filter(0x180C0800u, 0x1FFFFFFFu, &f.FilterIdHigh, &f.FilterIdLow, &f.FilterMaskIdHigh, &f.FilterMaskIdLow);
        f.FilterFIFOAssignment = CAN_RX_FIFO0; f.FilterActivation = ENABLE; f.SlaveStartFilterBank = 14;
        print_hal("CAN ConfigFilter (0x180C0800)", HAL_CAN_ConfigFilter(&hcan, &f));
    }
    {
        CAN_FilterTypeDef f = {0};
        f.FilterBank = 5; f.FilterMode = CAN_FILTERMODE_IDMASK; f.FilterScale = CAN_FILTERSCALE_32BIT;
        pack_ext_filter(0x18000800u, 0xFFFFFF00u, &f.FilterIdHigh, &f.FilterIdLow, &f.FilterMaskIdHigh, &f.FilterMaskIdLow);
        f.FilterFIFOAssignment = CAN_RX_FIFO0; f.FilterActivation = ENABLE; f.SlaveStartFilterBank = 14;
        print_hal("CAN ConfigFilter (0x18000800/..01)", HAL_CAN_ConfigFilter(&hcan, &f));
    }
    {
        CAN_FilterTypeDef f = {0};
        f.FilterBank = 6; f.FilterMode = CAN_FILTERMODE_IDMASK; f.FilterScale = CAN_FILTERSCALE_32BIT;
        pack_ext_filter(0x18010800u, 0xFFFFFF00u, &f.FilterIdHigh, &f.FilterIdLow, &f.FilterMaskIdHigh, &f.FilterMaskIdLow);
        f.FilterFIFOAssignment = CAN_RX_FIFO0; f.FilterActivation = ENABLE; f.SlaveStartFilterBank = 14;
        print_hal("CAN ConfigFilter (0x18010800/..01)", HAL_CAN_ConfigFilter(&hcan, &f));
    }
    {
        CAN_FilterTypeDef f = {0};
        f.FilterBank = 7; f.FilterMode = CAN_FILTERMODE_IDMASK; f.FilterScale = CAN_FILTERSCALE_32BIT;
        pack_ext_filter(0x18040A00u, 0x1FFFFFFFu, &f.FilterIdHigh, &f.FilterIdLow, &f.FilterMaskIdHigh, &f.FilterMaskIdLow);
        f.FilterFIFOAssignment = CAN_RX_FIFO0; f.FilterActivation = ENABLE; f.SlaveStartFilterBank = 14;
        print_hal("CAN ConfigFilter (0x18040A00)", HAL_CAN_ConfigFilter(&hcan, &f));
    }

    print_hal("CAN Start", HAL_CAN_Start(&hcan));

    /* DO NOT enable notifications here anymore. We’ll enable later. */
    s_rxEnabled = 0u;
    // uint32_t it = CAN_IT_RX_FIFO0_MSG_PENDING
    //             | CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE
    //             | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR;
    // print_hal("CAN ActivateNotif", HAL_CAN_ActivateNotification(&hcan, it));
    printf("CAN ready (FIFO0 IRQ)\r\n");
}
void CANAPP_FlushRx(void) {
    CAN_RxHeaderTypeDef rxh;
    uint8_t data[8];
    int drained = 0;
    while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxh, data) != HAL_OK) break;
        drained++;
    }
    if (drained) {
        printf("CAN: drained %d queued frames before enabling RX\r\n", drained);
    }
}
void CANAPP_EnableRx(bool enable) {
    if (enable && !s_rxEnabled) {
        CANAPP_FlushRx();  // drop backlog so we don’t start with a flood
        uint32_t it = CAN_IT_RX_FIFO0_MSG_PENDING
                    | CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE
                    | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR;
        print_hal("CAN ActivateNotif", HAL_CAN_ActivateNotification(&hcan, it));
        s_rxEnabled = 1u;
        printf("CAN RX enabled\r\n");
    } else if (!enable && s_rxEnabled) {
        print_hal("CAN DeactivateNotif",
                  HAL_CAN_DeactivateNotification(&hcan,
                      CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR));
        s_rxEnabled = 0u;
        printf("CAN RX disabled\r\n");
    }
}

/* ---------- RX ISR ---------- */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hh) {
    if (!s_rxEnabled) {
        /* If we ever get here due to race, drain one and bail. */
        CAN_RxHeaderTypeDef rxh; uint8_t dummy[8];
        (void)HAL_CAN_GetRxMessage(hh, CAN_RX_FIFO0, &rxh, dummy);
        return;
    }
    CAN_RxHeaderTypeDef rxh;
    uint8_t data[8];
    if (HAL_CAN_GetRxMessage(hh, CAN_RX_FIFO0, &rxh, data) != HAL_OK) {
        printf("CAN RX: HAL_GetRxMessage ERR\r\n");
        return;
    }
    const uint32_t id    = (rxh.IDE == CAN_ID_STD) ? rxh.StdId : rxh.ExtId;
    const uint8_t  isExt = (rxh.IDE == CAN_ID_EXT) ? 1U : 0U;

    if (!can_id_is_expected(id, isExt)) return;

    CanFrameEvt *e = Q_NEW_X(CanFrameEvt, 0U, CAN_RX_SIG);
    if (!e) return;

    e->id    = id;
    e->isExt = isExt;
    e->dlc   = (uint8_t)(rxh.DLC > 8 ? 8 : rxh.DLC);
    memset(e->data, 0, sizeof(e->data));
    memcpy(e->data, data, e->dlc);

    g_lastSig = CAN_RX_SIG; g_lastTag = 10;
    if (!QACTIVE_POST_X(AO_Bms, &e->super, 1U, 0U)) {
        QF_gc(&e->super);
    }
}

/* ---------- error cb ---------- */
void HAL_CAN_ErrorCallback(const CAN_HandleTypeDef *hh) {
    uint32_t e = HAL_CAN_GetError(hh);
    printf("CAN ERR: 0x%08lX\r\n", (unsigned long)e);
}
