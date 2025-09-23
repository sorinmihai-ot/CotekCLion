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
    BmsTelemetry t = {0};

    // Voltage: 51.5 .. 52.5 V slow sine
    t.array_voltage_V = 52.0f + 0.5f * sinf(phase);

    // SoC: bounce between ~55..85%
    uint8_t soc = (uint8_t)(70.0f + 15.0f * sinf(phase * 0.35f));
    if (soc > 100U) soc = 100U;
    t.soc_percent = soc;

    // Battery type cycling through your known codes (400s/500s/600s)
    // Pick one and keep it stable; or rotate every few seconds:
    static uint8_t sel = 0U;
    if (((int)(phase * 10.0f)) % 50 == 0) { // very slow rotate
        sel = (uint8_t)((sel + 1U) % 3U);
    }
    t.battery_type_code = (sel == 0U) ? 0x0400U
                        : (sel == 1U) ? 0x0500U
                                      : 0x0600U;

    // BMS state and faults: normal / no faults
    t.bms_state = 2U;        // pick a value that your UI maps to "OK/Run"
    t.bms_fault = 1U;

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
    if (!QACTIVE_POST_X(AO_Bms, &e->super, 1U, 0U)) { // 1-slot margin
        QF_gc(&e->super);
    }
}

// Optional: error callback for visibility
void HAL_CAN_ErrorCallback(const CAN_HandleTypeDef *hh) {
    uint32_t e = HAL_CAN_GetError(hh);
    printf("CAN ERR: 0x%08lX\r\n", (unsigned long)e);
}
