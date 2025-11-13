#include "can_400ST.h"
#include "can.h"        // HAL_CAN_xx, hcan1
#include <string.h>     // memset
#include <math.h>       // lrintf

/* --------------------------------------------------------------------------
 * 400s Steatite (0x0402) — MASTER ONLY
 * IDs/cadence modelled from filtered_400s_Steatite_master_only.log
 * We reuse the same field conventions we aligned for DZB so your receiver
 * classifies this pack correctly.
 * -------------------------------------------------------------------------- */

extern CAN_HandleTypeDef hcan1;

/* =============================== CADENCE =============================== */
/* Status loop in the capture repeats ~350–400 ms, slow blocks ~1 s.      */
#define ST_STATUS_PERIOD_MS     350u    /* 18060800 / 18070800 / 18080800 */
#define ST_SLOW_PERIOD_MS      1000u    /* 1800x0800 cells/temps + 1803/4/5/9 */
#define ST_OA_PERIOD_MS        1000u    /* 18010A00 .. 180A0A00, 18080A00 etc */

/* =============================== HELPERS =============================== */
static inline void be16(uint8_t *d, uint16_t v){ d[0]=(uint8_t)(v>>8); d[1]=(uint8_t)v; }
static inline void be32(uint8_t *d, uint32_t v){ d[0]=(uint8_t)(v>>24); d[1]=(uint8_t)(v>>16); d[2]=(uint8_t)(v>>8); d[3]=(uint8_t)v; }

/* scale helpers consistent with the other packs you’ve done */
static inline uint16_t v_to_1p5mV(float v){ float raw=v*1000.0f/1.5f; if(raw<0)raw=0; if(raw>65535)raw=65535; return (uint16_t)lrintf(raw); }
static inline uint16_t v_to_12mV (float v){ float raw=v*1000.0f/12.0f; if(raw<0)raw=0; if(raw>65535)raw=65535; return (uint16_t)lrintf(raw); }
/* temps: TelemetryOut gives 0.1°C. For status/summary we follow prior packs. */
static inline uint16_t tC0p1_to_0p1K(uint16_t t01C){ /* 0.1°C → 0.1K */
    int32_t k = (int32_t)t01C + 2731; if(k<0)k=0; if(k>65535)k=65535; return (uint16_t)k;
}

/* HAL extended sender */
static void st_send_ext(uint32_t can_id, const uint8_t *data, uint8_t dlc){
    CAN_TxHeaderTypeDef hdr; uint32_t mbox;
    memset(&hdr, 0, sizeof(hdr));
    hdr.ExtId = can_id & 0x1FFFFFFFu;
    hdr.IDE   = CAN_ID_EXT;
    hdr.RTR   = CAN_RTR_DATA;
    hdr.DLC   = dlc;
    HAL_CAN_AddTxMessage(&hcan1, &hdr, (uint8_t*)data, &mbox);
}

/* =============================== STATE ================================ */
static uint32_t s_last_status_ms = 0u;
static uint32_t s_last_slow_ms   = 0u;
static uint32_t s_last_oa_ms     = 0u;

/* =============================== BUILDERS ============================= */
/* ---- Cells A/B & misc blocks (…0800 family) — seen each ~1 s --------- */
/* 18000800: cells1..4 (1.5 mV/bit). We only have high/low → fill 2, pad. */
static void st_send_18000800(const TelemetryOut *t){
    uint8_t d[8]={0};
    uint16_t cHi=v_to_1p5mV(t->highCell_V);
    uint16_t cLo=v_to_1p5mV(t->lowCell_V);
    be16(&d[0], cHi);
    be16(&d[2], cLo);
    /* d[4..7] left as in capture (often small deltas/zeros). */
    st_send_ext(0x18000800u, d, 8);
}

/* 18010800: cells5..6 + max/min (1.5 mV/bit) */
static void st_send_18010800(const TelemetryOut *t){
    uint8_t d[8]={0};
    uint16_t cHi=v_to_1p5mV(t->highCell_V);
    uint16_t cLo=v_to_1p5mV(t->lowCell_V);
    be16(&d[0], cHi);   /* treat as “cell5” */
    be16(&d[2], cLo);   /* treat as “cell6” */
    be16(&d[4], cHi);   /* max */
    be16(&d[6], cLo);   /* min */
    st_send_ext(0x18010800u, d, 8);
}

/* 18020800: present with all zeros in your log */
static void st_send_18020800(void){
    uint8_t d[8]={0};
    st_send_ext(0x18020800u, d, 8);
}

/* 180C0800: temperature summary (two values observed) */
static void st_send_180C0800(const TelemetryOut *t){
    uint8_t d[8]={0};
    /* keep same interpretation as other packs: put 0.1K temps in first two pairs,
       leave the extra two pairs as small placeholders */
    uint16_t tMaxK=tC0p1_to_0p1K(t->tempHigh_0p1C);
    uint16_t tMinK=tC0p1_to_0p1K(t->tempLow_0p1C);
    be16(&d[0], tMaxK);
    be16(&d[2], tMinK);
    be16(&d[4], 0x001E);   /* mild constants like capture: 00 1E / 00 1C */
    be16(&d[6], 0x001C);
    st_send_ext(0x180C0800u, d, 8);
}

/* 180A0800 / 180B0800: vendor/counters — keep shape from capture */
static void st_send_180A0800(void){
    uint8_t d[8]={0x51,0x90,0x00,0x00,0x84,0x5F,0x00,0x1F};
    st_send_ext(0x180A0800u, d, 8);
}
static void st_send_180B0800(void){
    uint8_t d[6]={0xFF,0xFF,0xFF,0xFF,0x00,0x27};
    st_send_ext(0x180B0800u, d, 6);
}

/* 18030800: pack voltage (12 mV/bit) + a couple of flavour bytes */
static void st_send_18030800(const TelemetryOut *t){
    uint8_t d[8]={0};
    be16(&d[0], v_to_12mV(t->pack_V));
    d[2]=0x00; d[3]=0x01;     /* small nonzero like the log’s pattern */
    /* bytes 4..7 often carry echoed cell bytes in capture; use hi/lo hints */
    be16(&d[4], v_to_1p5mV(t->highCell_V));
    be16(&d[6], v_to_1p5mV(t->lowCell_V));
    st_send_ext(0x18030800u, d, 8);
}

/* 18040800: uptime-ish stamp & flags (shape like capture) */
static void st_send_18040800(uint32_t now_ms){
    uint8_t d[8]={0};
    uint32_t sec = now_ms/1000u;
    d[0]=(uint8_t)((sec/86400u) & 0xFF);   /* days */
    d[1]=(uint8_t)((sec/3600u)%24u);
    d[2]=(uint8_t)((sec/60u)%60u);
    d[3]=(uint8_t)(sec%60u);
    /* d[4..7]: keep small activity like logs (00 00 05 8C / 00 08 05 8C) */
    d[4]=0x00; d[5]=(uint8_t)((sec>>8)&0x0F); d[6]=0x05; d[7]=0x8C;
    st_send_ext(0x18040800u, d, 8);
}

/* 18050800: faults page — logs often zeros with small tail 05 88 */
static void st_send_18050800(const TelemetryOut *t){
    uint8_t d[8]={0};
    if(t->last_error_code){ d[0]=0x00; d[1]=0x00; d[2]=0x00; d[3]=0x00; d[4]=0x00; d[5]=0x01; }
    d[6]=0x05; d[7]=0x88;
    st_send_ext(0x18050800u, d, 8);
}

/* 18090800: short 6-byte status — keep shape/timing */
static void st_send_18090800(void){
    uint8_t d[6]={0x0F,0xFE,0x0A,0xD0,0x0B,0x09}; /* values vary per tick; fixed is fine */
    st_send_ext(0x18090800u, d, 6);
}

/* ---- Fast “status” trio every ~350 ms (…608/708/808) ------------------ */
static void st_send_18060800(const TelemetryOut *t){
    uint8_t d[8]={0};
    d[0]=0x13; d[1]=0x00; d[2]=0x00; d[3]=0x00;
    be16(&d[4], v_to_1p5mV(t->highCell_V));   /* bytes 4..5 ≈ “0A 23” in cap */
    be16(&d[6], v_to_1p5mV(t->lowCell_V));    /* bytes 6..7 ≈ “0A 1C/1D…”    */
    st_send_ext(0x18060800u, d, 8);
}
static void st_send_18070800(const TelemetryOut *t){
    uint8_t d[8]={0};
    be16(&d[0], v_to_12mV(t->pack_V));        /* first two match log (0F 2A / 0F 29 ..) */
    /* rest zero in capture */
    st_send_ext(0x18070800u, d, 8);
}
static void st_send_18080800(const TelemetryOut *t){
    uint8_t d[8]={0};
    /* byte1 in logs toggles small values 06/09/0B… use tiny counter feel: */
    d[1]=(t->last_error_code? 0x0B: 0x06);
    d[2]=0x01; /* present */
    st_send_ext(0x18080800u, d, 8);
}

/* ---- “0A” diagnostic/status group every ~1 s (sizes per capture) ------- */
static void st_send_OA_group(const TelemetryOut *t){
    uint8_t d[8];

    /* 18010A00 — 5B */
    uint8_t a10[5]={0x01,0x04,0x06,0x02,0x00};
    st_send_ext(0x18010A00u, a10, 5);

    /* 18020A00 — 6B */
    uint8_t a20[6]={0x0B,0x11,0x07,0x8D,0x0A,0x28};
    st_send_ext(0x18020A00u, a20, 6);

    /* 18030A00 — 6B */
    uint8_t a30[6]={0x01,0xF4,0x0A,0xF0,0x07,0xD0};
    st_send_ext(0x18030A00u, a30, 6);

    /* 18040A00 — 6B */
    uint8_t a40[6]={0x00,0x00,0x3F,0x51,0x04,0xD7};
    st_send_ext(0x18040A00u, a40, 6);

    /* 18050A00 — 5B (semi-static counters) */
    uint8_t a50[5]={0x02,0x35,0x01,0x8A,0xC2};
    st_send_ext(0x18050A00u, a50, 5);

    /* 18060A00 — 6B */
    uint8_t a60[6]={0x00,0x08,0x03,0x00,0x00,0x02};
    st_send_ext(0x18060A00u, a60, 6);

    /* 18070A00 — 7B */
    uint8_t a70[7]={0x00,0x05,0x69,0x28,0x00,0xD0,0x91};
    st_send_ext(0x18070A00u, a70, 7);

    /* 18080A00 — 5B */
    uint8_t a80[5]={0x00,0x5E,0x01,0xF8,0x04};
    st_send_ext(0x18080A00u, a80, 5);

    /* 18090A00 — 7B */
    uint8_t a90[7]={0x0A,0x28,0x00,0x4B,0x00,0xD7,0x91};
    st_send_ext(0x18090A00u, a90, 7);

    /* 180A0A00 — 7B */
    uint8_t aa0[7]={0x01,0x01,0x01,0x00,0x00,0x25,0x00};
    st_send_ext(0x180A0A00u, aa0, 7);
}

/* =============================== PUBLIC API ============================== */
static inline int elapsed(uint32_t now, uint32_t last, uint32_t period){
    return (uint32_t)(now - last) >= period;
}

void CAN_400ST_Init(uint8_t node_id){
    (void)node_id; /* master-only: no …01 frames in this ref */
    s_last_status_ms = 0u;
    s_last_slow_ms   = 0u;
    s_last_oa_ms     = 0u;
}

void CAN_400ST_Tick(const TelemetryOut *t, uint32_t now_ms)
{
    /* ~350 ms fast status trio */
    if (elapsed(now_ms, s_last_status_ms, ST_STATUS_PERIOD_MS)){
        st_send_18060800(t);
        st_send_18070800(t);
        st_send_18080800(t);
        s_last_status_ms = now_ms;
    }

    /* ~1 s slow set: cells/temps + info/faults short blocks */
    if (elapsed(now_ms, s_last_slow_ms, ST_SLOW_PERIOD_MS)){
        st_send_18000800(t);
        st_send_18010800(t);
        st_send_18020800();
        st_send_180C0800(t);
        st_send_180A0800();
        st_send_180B0800();
        st_send_18030800(t);
        st_send_18040800(now_ms);
        st_send_18050800(t);
        st_send_18090800();
        s_last_slow_ms = now_ms;
    }

    /* ~1 s diagnostic “0A” group (present in Steatite capture) */
    if (elapsed(now_ms, s_last_oa_ms, ST_OA_PERIOD_MS)){
        st_send_OA_group(t);
        s_last_oa_ms = now_ms;
    }
}
