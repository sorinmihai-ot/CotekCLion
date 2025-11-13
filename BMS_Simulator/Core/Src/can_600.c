// can_600s.c — 600s simulator (wire-identical to your capture)
// Fast pair: 0x10000010 (DLC=8) + 0x10000011 (DLC=8) every ~500 ms
// Other frames: sent periodically (seconds-plus), payloads shaped like logs.

#include "../Inc/can_600.h"
#include "can.h"
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

extern CAN_HandleTypeDef hcan1;

/* ---------------- Periods (ms) ---------------- */
#define C600_FAST_MS        500u   /* 0x10000010 + 0x10000011 pair cadence */
#define C600_SLOW_MS       1000u   /* 1s group: cells, temps, params, info */
#define C600_INFO_MS      60000u   /* stamp-like error/info */

/* ---------------- Local state ----------------- */
static uint8_t  s_node_id        = 0u;
static uint32_t s_last_fast_ms   = 0u;
static uint32_t s_last_slow_ms   = 0u;
static uint32_t s_last_info_ms   = 0u;

/* ---------------- Helpers -------------------- */
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
static void send_ext(uint32_t id, const uint8_t *data, uint8_t dlc) {
    CAN_TxHeaderTypeDef hdr;
    uint32_t mailbox;
    memset(&hdr, 0, sizeof(hdr));
    hdr.ExtId = id & 0x1FFFFFFFu;
    hdr.IDE   = CAN_ID_EXT;
    hdr.RTR   = CAN_RTR_DATA;
    hdr.DLC   = dlc;
    (void)HAL_CAN_AddTxMessage(&hcan1, &hdr, (uint8_t*)data, &mailbox);
}

/* ---------------- Fast pair ------------------ */
/* 0x10000010 — DLC=8 (matches log shape)
 * B0..B1: pack voltage 0.1V BE (e.g., 0x02 0x25 ≈ 54.9V)
 * B2..B3: current 0.1A BE (signed) — we keep small ripple around 0
 * B4    : mode/state byte (logs: 0x02)
 * B5    : flags/faults (logs usually 0x00)
 * B6    : 0xFF (constant)
 * B7    : 0x05 (constant)
 */
static void c600_send_pack_status_8B(const TelemetryOut *t, uint32_t now_ms) {
    uint8_t d[8] = {0};

    /* Voltage 0.1V BE */
    float v = fmaxf(0.0f, t->pack_V);
    uint16_t v10 = (uint16_t)lrintf(v * 10.0f);
    put_be16(&d[0], v10);

    /* Current 0.1A BE (signed) — tiny oscillation so lower byte changes */
    float iA = 0.1f * sinf((float)(now_ms % 2000) * (2.f * 3.1415926f / 2000.f));
    int16_t i10 = (int16_t)lrintf(iA * 10.0f);
    put_be16(&d[2], (uint16_t)i10);

    d[4] = 0x02;                  /* state/mode */
    d[5] = (t->last_error_code ? 0x01 : 0x00);
    d[6] = 0xFF;
    d[7] = 0x05;

    send_ext(0x10000010u, d, 8);
}

/* 0x10000011 — signed 32-bit BE “power” + 0x00 0x00 0x00 0x00 */
static void c600_send_pack_power_8B(const TelemetryOut *t, uint32_t now_ms) {
    uint8_t d[8] = {0};
    /* simple signed power model: discharge negative, small ripple */
    float iA = 0.0f;
    if (t->last_error_code == 0u) {
        iA = 0.5f + 0.5f * sinf((float)(now_ms % 3000) * (2.f * 3.1415926f / 3000.f));
    }
    float pW = t->pack_V * iA;                     /* positive when charging */
    int32_t raw_mW = (int32_t)lrintf(pW * 1000.0f);
    put_be_s32(&d[0], raw_mW);
    /* d[4..7] remain 0x00 */
    send_ext(0x10000011u, d, 8);
}

/* ---------------- Slow group (≈1 s) --------- */
static void c600_send_cell_voltage_8B(const TelemetryOut *t) {
    uint8_t d[8] = {0};
    uint16_t hi_mV = (uint16_t)lrintf(t->highCell_V * 1000.0f);
    uint16_t lo_mV = (uint16_t)lrintf(t->lowCell_V  * 1000.0f);
    put_be16(&d[0], hi_mV);
    put_be16(&d[2], lo_mV);
    d[4] = 0x0D;  /* hi-id (example from log) */
    d[5] = 0x01;  /* lo-id */
    d[6] = 0xFF;
    d[7] = 0xFF;
    send_ext(0x10000100u, d, 8);
}
static void c600_send_temps_8B(const TelemetryOut *t) {
    uint8_t d[8] = {0};
    /* temps in 0.1C BE */
    put_be16(&d[0], (uint16_t)t->tempHigh_0p1C);
    put_be16(&d[2], (uint16_t)t->tempLow_0p1C);
    d[4] = 0x01;  /* high temp id (stub) */
    d[5] = 0x02;  /* low  temp id (stub) */
    send_ext(0x10000110u, d, 8);
}
static void c600_send_charge_params(const TelemetryOut *t) {
    uint8_t d[8] = {0};
    d[0] = (t->last_error_code == 0u) ? 0x01u : 0x00u; /* allow receive */
    /* leave others zero like many rows in the log */
    send_ext(0x10000020u, d, 8);
}
/* Info / error stamp (periodic) */
static void c600_send_error_like_log(uint32_t now_ms) {
    uint8_t d[8] = {0};
    d[0] = (uint8_t)((now_ms / 1000u) & 0xFFu); /* coarse tick */
    d[1] = 0x00;
    d[2] = 0x00;  /* code placeholder */
    d[3] = 0x00;
    d[4] = 0x00;
    d[5] = 0xB0;  /* matches style seen in logs */
    d[6] = 0xBD;
    d[7] = 0x0A;
    send_ext(0x10000000u, d, 8);
}
/* Optional: identity bursts occasionally (payload not relied upon) */
static void c600_send_identity_bursts(void) {
    uint8_t z[8] = {0};
    send_ext(0x10000090u, z, 8);
    send_ext(0x10000091u, z, 8);
}

/* ---------------- Public API ----------------- */
void CAN_600_Init(uint8_t node_id)
{
    s_node_id       = node_id;
    s_last_fast_ms  = 0u;
    s_last_slow_ms  = 0u;
    s_last_info_ms  = 0u;
    (void)s_node_id;
}

void CAN_600_Tick(const TelemetryOut *t, uint32_t now_ms)
{
    /* FAST pair */
    if ((now_ms - s_last_fast_ms) >= C600_FAST_MS) {
        c600_send_pack_status_8B(t, now_ms);
        c600_send_pack_power_8B (t, now_ms);
        s_last_fast_ms = now_ms;
    }

    /* SLOW (≈1 s) */
    if ((now_ms - s_last_slow_ms) >= C600_SLOW_MS) {
        c600_send_cell_voltage_8B(t);
        c600_send_temps_8B       (t);
        c600_send_charge_params  (t);
        c600_send_identity_bursts();
        s_last_slow_ms = now_ms;
    }

    /* INFO (≈60 s) */
    if ((now_ms - s_last_info_ms) >= C600_INFO_MS) {
        c600_send_error_like_log(now_ms);
        s_last_info_ms = now_ms;
    }
}
