// can_400HYP.c — refined to mimic real 400s Hyperdrive traffic
// - Fast frames (100 ms): hi/lo + type (0x180608NN) and pack + SoC (0x180708NN)
//   with a rolling byte to look “alive” on the bus.
// - Slow frames (1 s): temp summary (0x180C08NN) and ID block (0x18040ANN).
// - Fault announce (5 s when active): 0x18FF0EXX with severity+code.
//
// Notes:
// * NN (low 8 bits) is the node id. Keep node_id==0 for “master” feel.
// * last_error_code==0 => normal; !=0 => raise general-fault feel + error frame.
// * All byte choices are conservative and receiver-friendly.

#include "can_400HYP.h"
#include "can.h"        // HAL CAN hcan1
#include <string.h>
#include <math.h>
#include <stdbool.h>

/* ---- external CAN handle (CubeMX) --------------------------------------- */
extern CAN_HandleTypeDef hcan1;

/* ---- periods ------------------------------------------------------------- */
#define C400HYP_FAST_MS     100u   /* hi/lo + pack/soc        (0x180608.., 0x180708..) */
#define C400HYP_SLOW_MS    1000u   /* temps + identity        (0x180C08.., 0x18040A..) */
#define C400HYP_FAULT_MS   5000u   /* error announce cadence  (0x18FF0E..) when active */

/* ---- internal state ------------------------------------------------------ */
static uint32_t s_last_fast_ms  = 0u;  /* last sent fast frames time */
static uint32_t s_last_slow_ms  = 0u;  /* last sent slow frames time */
static uint32_t s_last_fault_ms = 0u;  /* last sent error frame time */

static uint8_t  s_node_id   = 0u;      /* 0 = master-like, 1..N = node id low-byte */
static bool     s_is_master = true;     /* kept for future branching if needed */

/* identity for 0x18040A.. */
static uint32_t s_serial     = 0x40000001u;  /* bytes 0..3 */
static uint32_t s_fw_version = 0x00040100u;  /* bytes 4..5 used (major.minor), rest by type */
static uint16_t s_batt_type  = 0x0400u;      /* 400s Hyper (bytes 6..7) */

/* a small rolling counter to make fast frames’ tail move like real packs */
static uint8_t  s_roll_fast  = 0u;

/* ------------------------------------------------------------------------- */
/* helper: send ext ID frame                                                 */
/* ------------------------------------------------------------------------- */
static void c400hyp_send_ext(uint32_t eid, const uint8_t *data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef hdr;
    uint32_t mailbox;

    memset(&hdr, 0, sizeof(hdr));
    hdr.ExtId = eid & 0x1FFFFFFFu;   /* 29-bit ID (J1939 style) */
    hdr.IDE   = CAN_ID_EXT;          /* extended ID */
    hdr.RTR   = CAN_RTR_DATA;        /* data frame */
    hdr.DLC   = dlc;                  /* 0..8 */

    (void)HAL_CAN_AddTxMessage(&hcan1, &hdr, (uint8_t*)data, &mailbox);
}

/* ------------------------------------------------------------------------- */
/* 0x180608NN – "hi/lo + error + type" (100 ms)                              */
/* Layout:
 *   B0..B1 : Highest cell (mV) big-endian
 *   B2..B3 : Lowest  cell (mV) big-endian
 *   B4     : Error code (0 = OK, else family code you set in profile)
 *   B5     : Reserved (0x00) — used by some fleets for state; keep 0
 *   B6..B7 : Battery type tag (0x04,0x00 for 400-HYP)  [helps your receiver]
 */
static void c400hyp_send_hilo(const TelemetryOut *t)
{
    uint8_t  d[8] = {0};
    uint16_t hi_mV = (uint16_t)lrintf(t->highCell_V * 1000.0f);  /* e.g., 4100 mV */
    uint16_t lo_mV = (uint16_t)lrintf(t->lowCell_V  * 1000.0f);  /* e.g., 4050 mV */
    const uint32_t can_id = 0x18060800u | (uint32_t)s_node_id;   /* low byte = node */

    d[0] = (uint8_t)(hi_mV >> 8);    /* hi mV MSB */
    d[1] = (uint8_t)(hi_mV & 0xFF);  /* hi mV LSB */
    d[2] = (uint8_t)(lo_mV >> 8);    /* lo mV MSB */
    d[3] = (uint8_t)(lo_mV & 0xFF);  /* lo mV LSB */

    d[4] = t->last_error_code;       /* 0 = OK, non-zero → receiver can mark fault/nonrec */
    d[5] = 0x00;                     /* reserved/compat */

    d[6] = (uint8_t)(s_batt_type >> 8);   /* 0x04 */
    d[7] = (uint8_t)(s_batt_type & 0xFF); /* 0x00 */

    c400hyp_send_ext(can_id, d, 8);
}

/* ------------------------------------------------------------------------- */
/* 0x180708NN – "pack + SoC + rolling" (100 ms)                              */
/* Layout:
 *   B0..B1 : Pack Voltage (0.1 V/bit), big-endian
 *   B2..B4 : Reserved (0x00) — leave clean
 *   B5..B6 : SoC % duplicated (seen on Hyperdrive families)
 *   B7     : Rolling byte (adds bus “liveness” like real packs)
 */
static void c400hyp_send_pack_soc(const TelemetryOut *t)
{
    uint8_t d[8] = {0};
    const uint32_t can_id = 0x18070800u | (uint32_t)s_node_id;

    uint16_t v_0p1 = (uint16_t)lrintf(t->pack_V * 10.0f); /* e.g., 48.3 V → 483 */
    d[0] = (uint8_t)(v_0p1 >> 8);    /* MSB */
    d[1] = (uint8_t)(v_0p1 & 0xFF);  /* LSB */

    d[5] = t->soc_pct;               /* SoC main */
    d[6] = t->soc_pct;               /* SoC mirror */

    d[7] = s_roll_fast++;            /* rolling tail byte */

    c400hyp_send_ext(can_id, d, 8);
}

/* ------------------------------------------------------------------------- */
/* 0x180C08NN – "temp summary" (1000 ms)                                     */
/* Layout:
 *   B0..B1 : Highest temp (0.1°C) big-endian (signed)
 *   B2..B3 : Lowest  temp (0.1°C) big-endian (signed)
 *   B4..B5 : Sensor IDs (fake 1,2 for now)
 *   B6..B7 : Mean temp (0.1°C) big-endian
 */
static void c400hyp_send_temp(const TelemetryOut *t)
{
    uint8_t d[8] = {0};
    const uint32_t can_id = 0x180C0800u | (uint32_t)s_node_id;

    int16_t hi = t->tempHigh_0p1C;        /* e.g., 300 → 30.0°C */
    int16_t lo = t->tempLow_0p1C;         /* e.g., 280 → 28.0°C */
    int16_t mn = (int16_t)((hi + lo) / 2);

    d[0] = (uint8_t)(hi >> 8);
    d[1] = (uint8_t)(hi & 0xFF);
    d[2] = (uint8_t)(lo >> 8);
    d[3] = (uint8_t)(lo & 0xFF);

    d[4] = 1u;                            /* highest sensor ID (placeholder) */
    d[5] = 2u;                            /* lowest  sensor ID (placeholder) */

    d[6] = (uint8_t)(mn >> 8);
    d[7] = (uint8_t)(mn & 0xFF);

    c400hyp_send_ext(can_id, d, 8);
}

/* ------------------------------------------------------------------------- */
/* 0x18040ANN – "serial / fw / type" (1000 ms)                               */
/* Layout:
 *   B0..B3 : Serial (big-endian)
 *   B4..B5 : FW (we export top two bytes; you can expand if you prefer)
 *   B6..B7 : Battery type tag (0x04,0x00)
 */
static void c400hyp_send_info(void)
{
    uint8_t d[8] = {0};
    const uint32_t can_id = 0x18040A00u | (uint32_t)s_node_id;

    d[0] = (uint8_t)(s_serial >> 24);
    d[1] = (uint8_t)(s_serial >> 16);
    d[2] = (uint8_t)(s_serial >>  8);
    d[3] = (uint8_t)(s_serial      );

    d[4] = (uint8_t)(s_fw_version >> 24);  /* major */
    d[5] = (uint8_t)(s_fw_version >> 16);  /* minor */

    d[6] = (uint8_t)(s_batt_type >> 8);    /* 0x04 */
    d[7] = (uint8_t)(s_batt_type & 0xFF);  /* 0x00 */

    c400hyp_send_ext(can_id, d, 8);
}

/* ------------------------------------------------------------------------- */
/* 0x18FF0EXX – "error announce" (every 5 s while error active)              */
/* Layout (simple, receiver-friendly):
 *   B0 : Severity (0xC2 = critical; 0x82 = error; or 0x00 = none)
 *   B1 : Reserved (0x00)
 *   B2 : Error code (your family NonRec / warn code)
 *   B3..B7 : 0x00 (you can add RTC later if you want)
 */
static void c400hyp_send_error(uint8_t severity, uint8_t err_code)
{
    uint8_t d[8] = {0};
    const uint32_t can_id = 0x18FF0E00u | (uint32_t)s_node_id;

    d[0] = severity;    /* 0x00 none, 0x82 error, 0xC2 critical — pick what you prefer */
    d[1] = 0x00;        /* reserved */
    d[2] = err_code;    /* your profile’s last_error_code */

    c400hyp_send_ext(can_id, d, 8);
}

/* ------------------------------------------------------------------------- */
/* PUBLIC API                                                                */
/* ------------------------------------------------------------------------- */
void CAN_400HYP_Init(uint8_t node_id)
{
    s_node_id     = node_id;           /* low byte goes into NN */
    s_is_master   = (node_id == 0u);   /* kept for future master/node split */
    s_last_fast_ms  = 0u;
    s_last_slow_ms  = 0u;
    s_last_fault_ms = 0u;
    s_roll_fast     = 0u;
}

void CAN_400HYP_SetIdentity(uint32_t serial,
                            uint32_t fw_ver,
                            uint16_t batt_type)
{
    s_serial     = serial;
    s_fw_version = fw_ver;
    s_batt_type  = batt_type;          /* e.g., 0x0400 for 400-HYP */
}

void CAN_400HYP_Tick(const TelemetryOut *t, uint32_t now_ms)
{
    /* derive “have error?” from profile’s last_error_code */
    const bool has_error = (t->last_error_code != 0u);

    /* 100 ms group: fast frames that your receiver watches for liveness */
    if ((now_ms - s_last_fast_ms) >= C400HYP_FAST_MS) {
        s_last_fast_ms = now_ms;
        c400hyp_send_hilo(t);          /* hi/lo + family tag */
        c400hyp_send_pack_soc(t);      /* pack + SoC + rolling tail */
    }

    /* 1000 ms group: temps + identity banner */
    if ((now_ms - s_last_slow_ms) >= C400HYP_SLOW_MS) {
        s_last_slow_ms = now_ms;
        c400hyp_send_temp(t);
        c400hyp_send_info();
    }

    /* 5000 ms group (only while fault is active): announce via 0x18FF0EXX */
    if (has_error && ((now_ms - s_last_fault_ms) >= C400HYP_FAULT_MS)) {
        s_last_fault_ms = now_ms;
        /* pick a severity style: 0x82 = “error”; 0xC2 = “critical” */
        c400hyp_send_error(0x82u, t->last_error_code);
    }
}
