// can_500HYP.c — MASTER-only refactor to match real 500s-Hyperdrive log
// 1 Hz: 0x18FF0600, 0x18FF0700, 0x18FF0800, 0x18FF1900
// 0x18FF0E00 only when fault needs surfacing.

#include "../Inc/can_500HYP.h"
#include "can.h"
#include <string.h>
#include <stdint.h>

extern CAN_HandleTypeDef hcan1;

/* IDs */
#define HYP_ID_ARRAY_STATUS_0   0x18FF0600u   /* 1 Hz */
#define HYP_ID_ARRAY_STATUS_1   0x18FF0700u   /* 1 Hz */
#define HYP_ID_ARRAY_STATUS_2   0x18FF0800u   /* 1 Hz */
#define HYP_ID_ARRAY_STATUS_3   0x18FF1900u   /* 1 Hz */
#define HYP_ID_ERROR_MASTER     0x18FF0E00u   /* on fault */

/* timing */
#define HYP_PERIOD_1S_MS        1000u
#define HYP_PERIOD_5S_MS        5000u

static uint32_t s_last_1s_ms  = 0u;
static uint32_t s_last_5s_ms  = 0u;

/* send helper (29-bit) */
static void hyp_send_ext(uint32_t canId, const uint8_t *data, uint8_t dlc) {
    CAN_TxHeaderTypeDef hdr;
    uint32_t mbox;
    memset(&hdr, 0, sizeof(hdr));
    hdr.ExtId = canId & 0x1FFFFFFFu;
    hdr.IDE   = CAN_ID_EXT;
    hdr.RTR   = CAN_RTR_DATA;
    hdr.DLC   = dlc;
    (void)HAL_CAN_AddTxMessage(&hcan1, &hdr, (uint8_t*)data, &mbox);
}

/* little-endian pack helpers for 16-bit quantities */
static inline void put_le16(uint8_t *d, uint16_t v) {
    d[0] = (uint8_t)(v & 0xFFu);
    d[1] = (uint8_t)(v >> 8);
}

/* 0x18FF0600 — Array Status 0
 * B4..B5 = Highest mV (LE); B6..B7 = Lowest mV (LE)
 */
static void send_array_status_0(const TelemetryOut *t) {
    uint8_t d[8] = {0};
    d[0] = 0x00; d[1] = 0x00; d[2] = 0x00; d[3] = 0x00;
    uint16_t hi_mV = (uint16_t)(t->highCell_V * 1000.0f);
    uint16_t lo_mV = (uint16_t)(t->lowCell_V  * 1000.0f);
    put_le16(&d[4], hi_mV);
    put_le16(&d[6], lo_mV);
    hyp_send_ext(HYP_ID_ARRAY_STATUS_0, d, 8);
}

/* 0x18FF0700 — Array Status 1
 * B0..B1: Pack Voltage 0.1V LE
 * B2    : SoC %
 * B3..B7: benign
 */
static void send_array_status_1(const TelemetryOut *t) {
    uint8_t d[8] = {0};
    uint16_t v_0p1 = (uint16_t)(t->pack_V * 10.0f);
    put_le16(&d[0], v_0p1);
    d[2] = t->soc_pct;
    d[3] = 0xFF; d[4] = 0x00; d[5] = 0xFF; d[6] = 0xFF; d[7] = 0xFF;
    hyp_send_ext(HYP_ID_ARRAY_STATUS_1, d, 8);
}

/* 0x18FF0800 — Array Status 2
 * B0..B1: High temp 0.1C LE
 * B2..B3: Low  temp 0.1C LE
 * B4..B5: Minutes to empty LE (coarse)
 * B6    : charge request bit1 set if no error
 */
static void send_array_status_2(const TelemetryOut *t) {
    uint8_t d[8] = {0};
    put_le16(&d[0], (uint16_t)t->tempHigh_0p1C);
    put_le16(&d[2], (uint16_t)t->tempLow_0p1C);
    put_le16(&d[4], 240u);
    d[6] = (t->last_error_code == 0u) ? (1u << 1) : 0x00u;
    d[7] = 0x00;
    hyp_send_ext(HYP_ID_ARRAY_STATUS_2, d, 8);
}

/* 0x18FF1900 — Array Status 3 */
static void send_array_status_3(void) {
    uint8_t d[8] = {0};
    put_le16(&d[4], 1000u); /* 100.0A in 0.1A LE */
    hyp_send_ext(HYP_ID_ARRAY_STATUS_3, d, 8);
}

/* 0x18FF0E00 — Master Error (only when present) */
static void send_master_error(const TelemetryOut *t) {
    if (t->last_error_code == 0u) return;
    uint8_t d[8] = {0};
    d[0] = 0xC2; /* severity class used commonly */
    d[2] = t->last_error_code;
    hyp_send_ext(HYP_ID_ERROR_MASTER, d, 8);
}

/* Public API */
void CAN_500HYP_Init(uint8_t node_id) {
    (void)node_id;
    s_last_1s_ms = 0u;
    s_last_5s_ms = 0u;
}
void CAN_500HYP_Tick(const TelemetryOut *t, uint32_t nowMs, bool isSlave) {
    (void)isSlave;
    if ((nowMs - s_last_1s_ms) >= HYP_PERIOD_1S_MS) {
        send_array_status_0(t);
        send_array_status_1(t);
        send_array_status_2(t);
        send_array_status_3();
        s_last_1s_ms = nowMs;
    }
    if ((nowMs - s_last_5s_ms) >= HYP_PERIOD_5S_MS) {
        send_master_error(t);
        s_last_5s_ms = nowMs;
    }
}
