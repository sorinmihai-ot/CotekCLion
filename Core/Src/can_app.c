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
#include <math.h>      // for sinf (you already link with -lm)
#include <stdbool.h>   // for bool

//////--------Testing helpers for CAn when a battery is not connected
static void CAN_SendSimFrame(uint32_t id, const uint8_t *data, uint8_t dlc, bool ext) {
    CAN_TxHeaderTypeDef tx;
    tx.StdId = (ext ? 0 : (id & 0x7FF));
    tx.ExtId = (ext ? (id & 0x1FFFFFFF) : 0);
    tx.IDE   = ext ? CAN_ID_EXT : CAN_ID_STD;
    tx.RTR   = CAN_RTR_DATA;
    tx.DLC   = dlc;

    uint32_t mb;
    (void)HAL_CAN_AddTxMessage(&hcan, &tx, (uint8_t*)data, &mb);
}

// Example simulator for a subset of frames your parser already handles
static void CAN_PushBmsSimFrames(void) {
    // 0x10000091 – battery type (you already look at this to color/label)
    {   // Example: 0x0500 = “500s”
        uint8_t d[8] = { 0x00, 0x05, 0,0,0,0,0,0 }; // adjust endianness to your parser
        CAN_SendSimFrame(0x10000091, d, 8, true);
    }

    // 0x10000110 – temperatures: H/L/A/Pack (match your parser’s layout)
    {
        // Put sensible test values; use the same scaling as your decode
        // Example (shorts in 0.1°C): H=315 (=31.5°C), L=278, A=300, PackH=320
        uint8_t d[8] = { 0x3B,0x01, 0x16,0x01, 0x2C,0x01, 0x40,0x01 };
        CAN_SendSimFrame(0x10000110, d, 8, true);
    }

    // Add a few more frames that your bms_app.c expects (pack voltage, SOC, faults)
    // Use the exact IDs you parse; if you have the IDs handy, add them here.
}

////// ---------- end of helpers

#ifdef ENABLE_BMS_SIM
#include <math.h>
#include "app_signals.h"
#include "ao_controller.h"  // for extern QActive * const AO_Controller

// Simple “alive” waveforms around plausible values
static uint32_t s_sim_step;

void BmsSim_tick(void) {
    static float phase = 0.0f;

    // Prepare a synthetic telemetry packet
    BmsTelemetry t = (BmsTelemetry){0};
    // --- identity / type ---
    t.battery_type_code = 0x0500u;          // 500s
    // Voltage: 51.5 .. 52.5 V slow sine
    t.array_voltage_V = 52.0f + 0.5f * sinf(phase);

    // --- electricals ---
    t.array_voltage_V = 41.0f;              // Pack V shown on pMain
    t.high_cell_V     = 4.10f;              // pDetails High
    t.low_cell_V      = 3.95f;              // pDetails Low
    t.current_dA      = 0;                  // 0.0 A (deci-amps)
    // --- temperatures ---
    t.sys_temp_high_C = 32.0f;
    t.sys_temp_low_C  = 32.0f;
    // --- status / health ---
    t.bms_state = 0u;                        // 0 = Idle (see ao_controller mapping)
    t.bms_fault = 0u;                        // no faults
    t.soc_percent = 80u;                     // any steady SoC
    t.fan_rpm = 0u;                          // fan stopped
    t.last_error_class = 0u;
    t.last_error_code  = 0u;

    // --- identifiers (shown in pDetails) ---
    t.serial_number     = 12345678u;         // any fixed number
    t.firmware_version  = 401u;              // matches your "0.4.1" feel, but numeric

    // Publish just like a real CAN decode would
    BMS_publish_telemetry(&t);

    // advance phase
    phase += 0.10f;
    if (phase > 6.283185f) {
        phase -= 6.283185f;
    }
}
#endif  // ENABLE_BMS_SIM


Q_DEFINE_THIS_FILE

extern volatile uint16_t g_lastSig;
extern volatile uint8_t  g_lastTag;

/* ----------------- local helpers ----------------- */
static void print_hal(const char *tag, HAL_StatusTypeDef st) {
    printf("%s: %s\r\n", tag,
           st==HAL_OK?"OK":st==HAL_ERROR?"ERR":st==HAL_BUSY?"BUSY":"TIMEOUT");
}

/* Helper to pack extended filter fields (IDE must be 1 in the filter)
 * NOTE: your HAL uses uint32_t fields in CAN_FilterTypeDef, so pointers are uint32_t*.
 */
static void pack_ext_filter(uint32_t id, uint32_t mask,
                            uint32_t *idh, uint32_t *idl,
                            uint32_t *mh,  uint32_t *ml)
{
    uint32_t fid  = ((id   & 0x1FFFFFFFu) << 3) | (1u << 2);   // set IDE bit
    uint32_t fmsk = ((mask & 0x1FFFFFFFu) << 3) | (1u << 2);   // match IDE bit too
    *idh = (fid  >> 16) & 0xFFFFu;
    *idl = (fid         & 0xFFFFu);
    *mh  = (fmsk >> 16) & 0xFFFFu;
    *ml  = (fmsk        & 0xFFFFu);
}

/* Software pre-filter to reject frames your parser won't use */
static inline bool can_id_is_expected(uint32_t id, uint8_t isExt) {
    if (!isExt) {
        // We don’t parse any 11-bit IDs in your decoder. Drop all STD frames.
        return false;
    }
    // 500s Hyperdrive (J1939-style 0x18FFxx00, any SA): check top 16 bits
    if ((id & 0xFFFF0000u) == 0x18FF0000u) return true;

    // 600s / 500s BMZ (extended 0x100000xx range)
    if ((id & 0xFFFF0000u) == 0x10000000u) return true;

    // 400s family fixed/known extended IDs and patterns
    if (id == 0x18070800u || id == 0x18060800u || id == 0x180C0800u) return true;
    if ((id & 0xFFFFF0FFu) == 0x18004000u) return true;  // cell pages A/B/C
    if (id == 0x18040A00u) return true;                  // parameters/type

    return false;
}

/* ----------------- init (filters + start + IRQs) ----------------- */
void CANAPP_InitAll(void) {
    // Assume MX_CAN_Init() already set timing. Just do filters + start + IRQs here.

    // --- Configure several narrow filters (EXT only) into FIFO0 ---
    // Bank 0: 0x18FFxxxx (J1939 PGNs for 500s Hyperdrive)
    {
        CAN_FilterTypeDef f = {0};
        f.FilterBank           = 0;
        f.FilterMode           = CAN_FILTERMODE_IDMASK;
        f.FilterScale          = CAN_FILTERSCALE_32BIT;
        pack_ext_filter(0x18FF0000u, 0xFFFF0000u,
                        &f.FilterIdHigh, &f.FilterIdLow,
                        &f.FilterMaskIdHigh, &f.FilterMaskIdLow);
        f.FilterFIFOAssignment = CAN_RX_FIFO0;
        f.FilterActivation     = ENABLE;
        f.SlaveStartFilterBank = 14; // single-CAN on F1, but keep set
        print_hal("CAN ConfigFilter (0x18FFxxxx)", HAL_CAN_ConfigFilter(&hcan, &f));
    }

    // Bank 1: 0x100000xx (600s / 500s BMZ extended range)
    {
        CAN_FilterTypeDef f = {0};
        f.FilterBank           = 1;
        f.FilterMode           = CAN_FILTERMODE_IDMASK;
        f.FilterScale          = CAN_FILTERSCALE_32BIT;
        pack_ext_filter(0x10000000u, 0xFFFF0000u,
                        &f.FilterIdHigh, &f.FilterIdLow,
                        &f.FilterMaskIdHigh, &f.FilterMaskIdLow);
        f.FilterFIFOAssignment = CAN_RX_FIFO0;
        f.FilterActivation     = ENABLE;
        f.SlaveStartFilterBank = 14;
        print_hal("CAN ConfigFilter (0x100000xx)", HAL_CAN_ConfigFilter(&hcan, &f));
    }

    // Bank 2: 0x18070800 (exact)
    {
        CAN_FilterTypeDef f = {0};
        f.FilterBank           = 2;
        f.FilterMode           = CAN_FILTERMODE_IDMASK;
        f.FilterScale          = CAN_FILTERSCALE_32BIT;
        pack_ext_filter(0x18070800u, 0x1FFFFFFFu,
                        &f.FilterIdHigh, &f.FilterIdLow,
                        &f.FilterMaskIdHigh, &f.FilterMaskIdLow);
        f.FilterFIFOAssignment = CAN_RX_FIFO0;
        f.FilterActivation     = ENABLE;
        f.SlaveStartFilterBank = 14;
        print_hal("CAN ConfigFilter (0x18070800)", HAL_CAN_ConfigFilter(&hcan, &f));
    }

    // Bank 3: 0x18060800 (exact)
    {
        CAN_FilterTypeDef f = {0};
        f.FilterBank           = 3;
        f.FilterMode           = CAN_FILTERMODE_IDMASK;
        f.FilterScale          = CAN_FILTERSCALE_32BIT;
        pack_ext_filter(0x18060800u, 0x1FFFFFFFu,
                        &f.FilterIdHigh, &f.FilterIdLow,
                        &f.FilterMaskIdHigh, &f.FilterMaskIdLow);
        f.FilterFIFOAssignment = CAN_RX_FIFO0;
        f.FilterActivation     = ENABLE;
        f.SlaveStartFilterBank = 14;
        print_hal("CAN ConfigFilter (0x18060800)", HAL_CAN_ConfigFilter(&hcan, &f));
    }

    // Bank 4: 0x180C0800 (exact)
    {
        CAN_FilterTypeDef f = {0};
        f.FilterBank           = 4;
        f.FilterMode           = CAN_FILTERMODE_IDMASK;
        f.FilterScale          = CAN_FILTERSCALE_32BIT;
        pack_ext_filter(0x180C0800u, 0x1FFFFFFFu,
                        &f.FilterIdHigh, &f.FilterIdLow,
                        &f.FilterMaskIdHigh, &f.FilterMaskIdLow);
        f.FilterFIFOAssignment = CAN_RX_FIFO0;
        f.FilterActivation     = ENABLE;
        f.SlaveStartFilterBank = 14;
        print_hal("CAN ConfigFilter (0x180C0800)", HAL_CAN_ConfigFilter(&hcan, &f));
    }

    // Bank 5: 0x18004000/10/20 pattern (mask nibble for cell pages, fix rest)
    {
        CAN_FilterTypeDef f = {0};
        f.FilterBank           = 5;
        f.FilterMode           = CAN_FILTERMODE_IDMASK;
        f.FilterScale          = CAN_FILTERSCALE_32BIT;
        pack_ext_filter(0x18004000u, 0xFFFFF0FFu,
                        &f.FilterIdHigh, &f.FilterIdLow,
                        &f.FilterMaskIdHigh, &f.FilterMaskIdLow);
        f.FilterFIFOAssignment = CAN_RX_FIFO0;
        f.FilterActivation     = ENABLE;
        f.SlaveStartFilterBank = 14;
        print_hal("CAN ConfigFilter (0x18004000/10/20 pattern)", HAL_CAN_ConfigFilter(&hcan, &f));
    }

    // Bank 6: 0x18040A00 (exact) – parameters/type
    {
        CAN_FilterTypeDef f = {0};
        f.FilterBank           = 6;
        f.FilterMode           = CAN_FILTERMODE_IDMASK;
        f.FilterScale          = CAN_FILTERSCALE_32BIT;
        pack_ext_filter(0x18040A00u, 0x1FFFFFFFu,
                        &f.FilterIdHigh, &f.FilterIdLow,
                        &f.FilterMaskIdHigh, &f.FilterMaskIdLow);
        f.FilterFIFOAssignment = CAN_RX_FIFO0;
        f.FilterActivation     = ENABLE;
        f.SlaveStartFilterBank = 14;
        print_hal("CAN ConfigFilter (0x18040A00)", HAL_CAN_ConfigFilter(&hcan, &f));
    }

    // --- Start peripheral ---
    print_hal("CAN Start", HAL_CAN_Start(&hcan));

    // --- Enable RX FIFO0 pending + basic error notifs ---
    uint32_t it = CAN_IT_RX_FIFO0_MSG_PENDING
                | CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE
                | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR;
    print_hal("CAN ActivateNotif", HAL_CAN_ActivateNotification(&hcan, it));

    printf("CAN ready (FIFO0 IRQ)\r\n");
}

/* ----------------- RX callback (ISR) ----------------- */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hh) {
    CAN_RxHeaderTypeDef rxh;
    uint8_t data[8];
    if (HAL_CAN_GetRxMessage(hh, CAN_RX_FIFO0, &rxh, data) != HAL_OK) {
        printf("CAN RX: HAL_GetRxMessage ERR\r\n");
        return;
    }

    // Extract ID and type first
    const uint32_t id    = (rxh.IDE == CAN_ID_STD) ? rxh.StdId : rxh.ExtId;
    const uint8_t  isExt = (rxh.IDE == CAN_ID_EXT) ? 1U : 0U;

    // Software pre-filter (final guard to save the event pool)
    if (!can_id_is_expected(id, isExt)) {
        // Optional: debug prints if needed:
        // printf("CAN DROP: %s id=0x%08lX\n", isExt?"EXT":"STD", (unsigned long)id);
        return;
    }

    // margin-aware allocation in ISR: returns NULL instead of asserting
    CanFrameEvt *e = Q_NEW_X(CanFrameEvt, 0U, CAN_RX_SIG);  // margin=0U
    if (!e) {
        // Pool temporarily empty -> drop safely (no assert)
        return;
    }

    e->id    = id;
    e->isExt = isExt;
    e->dlc   = (uint8_t)rxh.DLC;
    if (e->dlc > 8) e->dlc = 8;
    memset(e->data, 0, sizeof(e->data));
    memcpy(e->data, data, e->dlc);

    g_lastSig = CAN_RX_SIG; g_lastTag = 10;  // tag 10 = CAN RX
    if (!QACTIVE_POST_X(AO_Bms, &e->super, 1U, 0U)) { // 1-slot margin to AO_Bms
        QF_gc(&e->super);
    }
}

/* Optional: error callback for visibility */
void HAL_CAN_ErrorCallback(const CAN_HandleTypeDef *hh) {
    uint32_t e = HAL_CAN_GetError(hh);
    printf("CAN ERR: 0x%08lX\r\n", (unsigned long)e);
}





// //
// // Created by sorin.mihai on 28/08/2025.
// //
// // can_app.c
// #include "qpc_cfg.h"
// #include "can_app.h"
// #include "bms_app.h"
// #include "main.h"
// #include <stdio.h>
// #include <string.h>
// #include "bsp.h"
// #include <math.h>      // for sinf (you already link with -lm)
//
//
// //////--------Testing helpers for CAn when a battery is not connected
// static void CAN_SendSimFrame(uint32_t id, const uint8_t *data, uint8_t dlc, bool ext) {
//     CAN_TxHeaderTypeDef tx;
//     tx.StdId = (ext ? 0 : (id & 0x7FF));
//     tx.ExtId = (ext ? (id & 0x1FFFFFFF) : 0);
//     tx.IDE   = ext ? CAN_ID_EXT : CAN_ID_STD;
//     tx.RTR   = CAN_RTR_DATA;
//     tx.DLC   = dlc;
//
//     uint32_t mb;
//     (void)HAL_CAN_AddTxMessage(&hcan, &tx, (uint8_t*)data, &mb);
// }
//
// // Example simulator for a subset of frames your parser already handles
// static void CAN_PushBmsSimFrames(void) {
//     // 0x10000091 – battery type (you already look at this to color/label)
//     {   // Example: 0x0500 = “500s”
//         uint8_t d[8] = { 0x00, 0x05, 0,0,0,0,0,0 }; // adjust endianness to your parser
//         CAN_SendSimFrame(0x10000091, d, 8, true);
//     }
//
//     // 0x10000110 – temperatures: H/L/A/Pack (match your parser’s layout)
//     {
//         // Put sensible test values; use the same scaling as your decode
//         // Example (shorts in 0.1°C): H=315 (=31.5°C), L=278, A=300, PackH=320
//         uint8_t d[8] = { 0x3B,0x01, 0x16,0x01, 0x2C,0x01, 0x40,0x01 };
//         CAN_SendSimFrame(0x10000110, d, 8, true);
//     }
//
//     // Add a few more frames that your bms_app.c expects (pack voltage, SOC, faults)
//     // Use the exact IDs you parse; if you have the IDs handy, add them here.
// }
//
// ////// ---------- end of helpers
//
// #ifdef ENABLE_BMS_SIM
// #include <math.h>
// #include "app_signals.h"
// #include "ao_controller.h"  // for extern QActive * const AO_Controller
//
// // Simple “alive” waveforms around plausible values
// static uint32_t s_sim_step;
//
// void BmsSim_tick(void) {
//     static float phase = 0.0f;
//
//     // Prepare a synthetic telemetry packet
//     BmsTelemetry t = {0};
//     // --- identity / type ---
//     t.battery_type_code = 0x0500u;          // 500s
//     // Voltage: 51.5 .. 52.5 V slow sine
//     t.array_voltage_V = 52.0f + 0.5f * sinf(phase);
//
//     // --- electricals ---
//     t.array_voltage_V = 41.0f;              // Pack V shown on pMain
//     t.high_cell_V     = 4.10f;              // pDetails High
//     t.low_cell_V      = 3.95f;              // pDetails Low
//     t.current_dA      = 0;                  // 0.0 A (deci-amps)
//     // --- temperatures ---
//     t.sys_temp_high_C = 32.0f;
//     t.sys_temp_low_C  = 32.0f;
//     // --- status / health ---
//     t.bms_state = 0u;                        // 0 = Idle (see ao_controller mapping)
//     t.bms_fault = 0u;                        // no faults
//     t.soc_percent = 80u;                     // any steady SoC
//     t.fan_rpm = 0u;                          // fan stopped
//     t.last_error_class = 0u;
//     t.last_error_code  = 0u;
//
//     // --- identifiers (shown in pDetails) ---
//     t.serial_number     = 12345678u;         // any fixed number
//     t.firmware_version  = 401u;              // matches your "0.4.1" feel, but numeric
//
//     // Publish just like a real CAN decode would
//     BMS_publish_telemetry(&t);
//
//     // advance phase
//     phase += 0.10f;
//     if (phase > 6.283185f) {
//         phase -= 6.283185f;
//     }
// }
// #endif  // ENABLE_BMS_SIM
//
//
// Q_DEFINE_THIS_FILE
//
// extern QActive *AO_Bms;
// extern volatile uint16_t g_lastSig;
// extern volatile uint8_t  g_lastTag;
//
// static void print_hal(const char *tag, HAL_StatusTypeDef st) {
//     printf("%s: %s\r\n", tag,
//            st==HAL_OK?"OK":st==HAL_ERROR?"ERR":st==HAL_BUSY?"BUSY":"TIMEOUT");
// }
// // Helper to pack extended filter fields
// static void pack_ext_filter(uint32_t id, uint32_t mask,
//                             uint16_t *idh, uint16_t *idl,
//                             uint16_t *mh,  uint16_t *ml)
// {
//     uint32_t fid  = (id   << 3) | (1u << 2);   // set IDE bit
//     uint32_t fmsk = (mask << 3) | (1u << 2);   // match IDE bit too
//     *idh = (uint16_t)(fid  >> 16);
//     *idl = (uint16_t)(fid  & 0xFFFFu);
//     *mh  = (uint16_t)(fmsk >> 16);
//     *ml  = (uint16_t)(fmsk & 0xFFFFu);
// }
// void CANAPP_InitAll(void) {
//     // Assume MX_CAN_Init() already set timing. Just do filter + start + IRQs here.
//
//     // --- Accept-all filter into FIFO0 (standard + extended) ---
//     CAN_FilterTypeDef f = {0};
//     f.FilterBank           = 0;
//     f.FilterMode           = CAN_FILTERMODE_IDMASK;
//     f.FilterScale          = CAN_FILTERSCALE_32BIT;
//     f.FilterIdHigh         = 0x0000;
//     f.FilterIdLow          = 0x0000;
//     f.FilterMaskIdHigh     = 0x0000;
//     f.FilterMaskIdLow      = 0x0000;
//     f.FilterFIFOAssignment = CAN_RX_FIFO0;
//     f.FilterActivation     = ENABLE;
//     f.SlaveStartFilterBank = 14; // not used on F1 single CAN, but set anyway
//
//     print_hal("CAN ConfigFilter", HAL_CAN_ConfigFilter(&hcan, &f));
//
//     // --- Start peripheral ---
//     print_hal("CAN Start", HAL_CAN_Start(&hcan));
//
//     // --- Enable RX FIFO0 pending + basic error notifs ---
//     uint32_t it = CAN_IT_RX_FIFO0_MSG_PENDING
//                 | CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE
//                 | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR;
//     print_hal("CAN ActivateNotif", HAL_CAN_ActivateNotification(&hcan, it));
//
//     printf("CAN ready (FIFO0 IRQ)\r\n");
// }
//
// static inline bool can_id_is_expected(uint32_t id, uint8_t isExt) {
//     if (!isExt) {
//         // We don’t parse any 11-bit IDs in your decoder. Drop all STD frames.
//         return false;
//     }
//     // 500s Hyperdrive (J1939-style 0x18FFxx00, any SA): mask top 17 bits of PGN
//     if ((id & 0xFFFF0000u) == 0x18FF0000u) return true;
//
//     // 600s / 500s BMZ (extended 0x100000xx range from your parser)
//     if ((id & 0xFFFF0000u) == 0x10000000u) return true;
//
//     // 400s family uses specific extended IDs
//     if (id == 0x18070800u || id == 0x18060800u || id == 0x180C0800u) return true;
//     if ((id & 0xFFFFF0FFu) == 0x18004000u) return true;  // cell pages A/B/C
//     if (id == 0x18040A00u) return true;                  // params (type)
//
//     return false;
// }
// // RX callback from HAL (called in IRQ context)
// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hh) {
//     CAN_RxHeaderTypeDef rxh;
//     uint8_t data[8];
//     if (HAL_CAN_GetRxMessage(hh, CAN_RX_FIFO0, &rxh, data) != HAL_OK) {
//         printf("CAN RX: HAL_GetRxMessage ERR\r\n");
//         return;
//     }
//     // printf("CAN RX: id=0x%08lX IDE=%lu DLC=%lu\r\n",
//     //        (unsigned long)rxh.StdId | ((unsigned long)rxh.ExtId<<3), // quick print
//     //        (unsigned long)rxh.IDE, (unsigned long)rxh.DLC);
//
//     // build event and POST to BMS AO (margin=1 from ISR!)
//     // margin-aware allocation in ISR: return NULL instead of asserting
//     CanFrameEvt *e = Q_NEW_X(CanFrameEvt, 0U, CAN_RX_SIG);  // margin=0U, returns NULL if pool empty
//     if (!e) {
//         // drop when pools are temporarily exhausted
//         return;
//     }    e->id     = (rxh.IDE==CAN_ID_STD) ? rxh.StdId : rxh.ExtId;
//     e->isExt    = (rxh.IDE==CAN_ID_EXT) ? 1U : 0U;
//     e->dlc    = (uint8_t)rxh.DLC;
//     memset(e->data, 0, sizeof(e->data));
//     if (e->dlc > 8) e->dlc = 8;
//     memcpy(e->data, data, e->dlc);
//     g_lastSig = CAN_RX_SIG; g_lastTag = 10;  // tag 10 = CAN RX
//     if (!QACTIVE_POST_X(AO_Bms, &e->super, 1U, 0U)) { // 1-slot margin
//         QF_gc(&e->super);
//     }
// }
//
// // Optional: error callback for visibility
// void HAL_CAN_ErrorCallback(const CAN_HandleTypeDef *hh) {
//     uint32_t e = HAL_CAN_GetError(hh);
//     printf("CAN ERR: 0x%08lX\r\n", (unsigned long)e);
// }
