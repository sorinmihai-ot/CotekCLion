// can_500BMZ.c — BMZ 500s simulator (IDs & cadence from your logs)
// Fast pair (~180 ms): 0x10000010 (DLC=8) + 0x10000011 (DLC=8)
// Tail bytes: B6=0x00, B7 toggles 0x01/0x05 occasionally (as seen)

#include "../Inc/can_500BMZ.h"
#include "can.h"
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

/* IDs */
#define BMZ_CAN_ID_PACK_STATUS     0x10000010u
#define BMZ_CAN_ID_POWER           0x10000011u
#define BMZ_CAN_ID_CHG_PARAMS      0x10000020u
#define BMZ_CAN_ID_CELL_VOLT       0x10000100u
#define BMZ_CAN_ID_TEMP_MEAS       0x10000110u
#define BMZ_CAN_ID_FAN_SPEED       0x10000080u
#define BMZ_CAN_ID_COUNTERS        0x10000050u
#define BMZ_CAN_ID_ERROR           0x10000000u
#define BMZ_CAN_ID_SERIAL_FW       0x10000090u
#define BMZ_CAN_ID_SERIAL_FW_2     0x10000091u
#define BMZ_CAN_ID_VENDOR_A0       0x100000A0u

/* Cadence */
#define BMZ_PERIOD_FAST_MS         180u
#define BMZ_PERIOD_ERROR_MS        1458u
#define BMZ_PERIOD_12M_MS          743900u
#define BMZ_PERIOD_21M_MS         1266580u
#define BMZ_PERIOD_31M_MS         1874660u
#define BMZ_PERIOD_38M_MS         2308390u
#define BMZ_PERIOD_47M_MS         2825200u
#define BMZ_PERIOD_80M_MS         4826950u
#define BMZ_PERIOD_VSLOW_MS        600000u

extern CAN_HandleTypeDef hcan1;

static uint8_t  s_node_id       = 0u;
static uint32_t s_last_fast_ms  = 0u;
static uint32_t s_last_err_ms   = 0u;
static uint32_t s_last_12m_ms   = 0u;
static uint32_t s_last_21m_ms   = 0u;
static uint32_t s_last_31m_ms   = 0u;
static uint32_t s_last_38m_ms   = 0u;
static uint32_t s_last_47m_ms   = 0u;
static uint32_t s_last_80m_ms   = 0u;
static uint32_t s_last_vslow_ms = 0u;

/* Helpers */
static inline void put_be16(uint8_t *d, uint16_t v) {
    d[0] = (uint8_t)(v >> 8);
    d[1] = (uint8_t)(v & 0xFFu);
}
static inline void put_be_s32(uint8_t *d, int32_t v) {
    d[0] = (uint8_t)((uint32_t)v >> 24);
    d[1] = (uint8_t)((uint32_t)v >> 16);
    d[2] = (uint8_t)((uint32_t)v >>  8);
    d[3] = (uint8_t)((uint32_t)v      );
}
static void bmz_sendExt(uint32_t canId, const uint8_t *data, uint8_t dlc) {
    CAN_TxHeaderTypeDef hdr;
    uint32_t mailbox;
    memset(&hdr, 0, sizeof(hdr));
    hdr.ExtId = canId & 0x1FFFFFFFu;
    hdr.IDE   = CAN_ID_EXT;
    hdr.RTR   = CAN_RTR_DATA;
    hdr.DLC   = dlc;
    (void)HAL_CAN_AddTxMessage(&hcan1, &hdr, (uint8_t*)data, &mailbox);
}

/* --------- Fast pair --------- */
/* 0x10000010 — Pack Status & Flags (BE) */
static void bmz_send_0x10000010(const TelemetryOut *t, uint32_t nowMs) {
    static uint8_t tail_flip = 0; /* flip B7 between 0x01 and 0x05 */

    uint8_t d[8] = {0};
    /* Pack voltage 0.1V BE — logs commonly ~0x01 0xC0 (44.8V) */
    uint16_t v10 = (uint16_t)lrintf(fmaxf(0.f, t->pack_V) * 10.0f);
    put_be16(&d[0], v10);

    /* Current 0.1A BE (signed) — tiny ripple */
    float iA = 0.1f * sinf((float)(nowMs % 1500) * (2.f * 3.1415926f / 1500.f));
    int16_t i10 = (int16_t)lrintf(iA * 10.0f);
    put_be16(&d[2], (uint16_t)i10);

    /* State / fault */
    d[4] = (t->last_error_code != 0u) ? 0x04u : 0x02u;    /* 2=run, 4=fault */
    d[5] = (t->last_error_code != 0u) ? (1u << 6) : 0x00; /* bit6=general fault */

    d[6] = 0x00;                                          /* observed constant */
    /* B7: flip 0x01/0x05 like in captures */
    if ((nowMs / 10000u) & 1u) tail_flip = 0x05u; else tail_flip = 0x01u;
    d[7] = tail_flip;

    bmz_sendExt(BMZ_CAN_ID_PACK_STATUS, d, 8);
}

/* 0x10000011 — signed 32-bit BE Power + 4x00 */
static void bmz_send_0x10000011(const TelemetryOut *t, uint32_t nowMs) {
    uint8_t d[8] = {0};
    /* coarse power model */
    float iA = (t->last_error_code == 0u) ? (0.5f * sinf((float)(nowMs % 2000) * (2.f * 3.1415926f / 2000.f))) : 0.0f;
    float pW = t->pack_V * iA;
    int32_t raw_mW = (int32_t)lrintf(pW * 1000.0f);
    put_be_s32(&d[0], raw_mW);  /* d[4..7] left 0x00 */
    bmz_sendExt(BMZ_CAN_ID_POWER, d, 8);
}

/* --------- Slow/very-slow groups --------- */
static void bmz_send_0x10000000(const TelemetryOut *t) {
    uint8_t d[8] = {0};
    d[0] = (t->last_error_code == 0u) ? 0u : 3u;  /* severity */
    d[2] = t->last_error_code;
    bmz_sendExt(BMZ_CAN_ID_ERROR, d, 8);
}
static void bmz_send_0x10000020(const TelemetryOut *t) {
    uint8_t d[8] = {0};
    d[0] = (t->last_error_code == 0u) ? 0x01u : 0x00u; /* receiving allowed */
    d[3] = t->soc_pct;
    d[4] = 100u;
    bmz_sendExt(BMZ_CAN_ID_CHG_PARAMS, d, 8);
}
static void bmz_send_0x10000100(const TelemetryOut *t) {
    uint8_t d[8] = {0};
    uint16_t hi_mV = (uint16_t)lrintf(t->highCell_V * 1000.0f);
    uint16_t lo_mV = (uint16_t)lrintf(t->lowCell_V  * 1000.0f);
    put_be16(&d[0], hi_mV);
    put_be16(&d[2], lo_mV);
    d[4] = 0x01; d[5] = 0x02; /* ids */
    bmz_sendExt(BMZ_CAN_ID_CELL_VOLT, d, 8);
}
static void bmz_send_0x10000110(const TelemetryOut *t) {
    uint8_t d[8] = {0};
    put_be16(&d[0], (uint16_t)t->tempHigh_0p1C);
    put_be16(&d[2], (uint16_t)t->tempLow_0p1C);
    d[4] = 0x01; d[5] = 0x02;
    bmz_sendExt(BMZ_CAN_ID_TEMP_MEAS, d, 8);
}
static void bmz_send_0x10000080(void) {
    uint8_t d[8] = {0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF};
    bmz_sendExt(BMZ_CAN_ID_FAN_SPEED, d, 8);
}
static void bmz_send_0x10000050(uint32_t nowMs) {
    uint8_t d[8] = {0};
    uint32_t hours = nowMs / 3600000UL; if (hours > 0xFFFFFFu) hours = 0xFFFFFFu;
    d[4] = (uint8_t)((hours >> 16) & 0xFFu);
    d[5] = (uint8_t)((hours >>  8) & 0xFFu);
    d[6] = (uint8_t)( hours        & 0xFFu);
    bmz_sendExt(BMZ_CAN_ID_COUNTERS, d, 8);
}
static void bmz_send_0x10000090(const TelemetryOut *t) {
    (void)t;
    uint8_t d[8] = {0};
    uint32_t serial = 0x500B0000UL + (uint32_t)s_node_id;
    uint32_t fwver  = 0x00050001UL;
    d[0] = (uint8_t)(serial >> 24);
    d[1] = (uint8_t)(serial >> 16);
    d[2] = (uint8_t)(serial >>  8);
    d[3] = (uint8_t)(serial      );
    d[4] = (uint8_t)(fwver  >> 24);
    d[5] = (uint8_t)(fwver  >> 16);
    d[6] = (uint8_t)(fwver  >>  8);
    d[7] = (uint8_t)(fwver       );
    bmz_sendExt(BMZ_CAN_ID_SERIAL_FW, d, 8);
}
static void bmz_send_0x10000091(void) {
    uint8_t d[8] = {0};
    bmz_sendExt(BMZ_CAN_ID_SERIAL_FW_2, d, 8);
}
static void bmz_send_0x100000A0(void) {
    uint8_t d[8] = {0};
    bmz_sendExt(BMZ_CAN_ID_VENDOR_A0, d, 8);
}

/* Public API */
void CAN_500BMZ_Init(uint8_t node_id)
{
    s_node_id       = node_id;
    s_last_fast_ms  = 0u;
    s_last_err_ms   = 0u;
    s_last_12m_ms   = 0u;
    s_last_21m_ms   = 0u;
    s_last_31m_ms   = 0u;
    s_last_38m_ms   = 0u;
    s_last_47m_ms   = 0u;
    s_last_80m_ms   = 0u;
    s_last_vslow_ms = 0u;
}

void CAN_500BMZ_Tick(const TelemetryOut *t, uint32_t nowMs, bool isSlave)
{
    (void)isSlave;

    /* Fast group — ~180 ms */
    if ((nowMs - s_last_fast_ms) >= BMZ_PERIOD_FAST_MS) {
        bmz_send_0x10000010(t, nowMs);
        bmz_send_0x10000011(t, nowMs);
        s_last_fast_ms = nowMs;
    }

    /* Error group — ~1.5 s */
    if ((nowMs - s_last_err_ms) >= BMZ_PERIOD_ERROR_MS) {
        bmz_send_0x10000000(t);
        s_last_err_ms = nowMs;
    }

    /* Very-slow groups (minutes) */
    if ((nowMs - s_last_12m_ms) >= BMZ_PERIOD_12M_MS) { bmz_send_0x10000020(t); s_last_12m_ms = nowMs; }
    if ((nowMs - s_last_21m_ms) >= BMZ_PERIOD_21M_MS) { bmz_send_0x100000A0(); s_last_21m_ms = nowMs; }
    if ((nowMs - s_last_31m_ms) >= BMZ_PERIOD_31M_MS) { bmz_send_0x10000080(); s_last_31m_ms = nowMs; }
    if ((nowMs - s_last_38m_ms) >= BMZ_PERIOD_38M_MS) { bmz_send_0x10000110(t); s_last_38m_ms = nowMs; }
    if ((nowMs - s_last_47m_ms) >= BMZ_PERIOD_47M_MS) { bmz_send_0x10000091(); s_last_47m_ms = nowMs; }
    if ((nowMs - s_last_80m_ms) >= BMZ_PERIOD_80M_MS) { bmz_send_0x10000100(t); s_last_80m_ms = nowMs; }
    if ((nowMs - s_last_vslow_ms) >= BMZ_PERIOD_VSLOW_MS) { bmz_send_0x10000090(t); s_last_vslow_ms = nowMs; }
}
