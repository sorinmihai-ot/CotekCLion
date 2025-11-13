#include "can_400DZB.h"
#include <string.h>
#include "can_tx.h"

/* We reuse your HAL-based senders from can_tx.c */
extern void CAN_TX_SendStd(uint32_t canId, const uint8_t *data, uint8_t dlc);
extern void CAN_TX_SendExt(uint32_t canIdExt, const uint8_t *data, uint8_t dlc);

/* ========================= PERIODS (ms) =========================
 * Derived to “feel” like your capture:
 * - Internal std frames: 250 ms
 * - External slow block (cells/pack/summary): 1000 ms
 * - External status block (…608/708/808): 400 ms
 * - External “0A” diagnostic/status block: 1000 ms (new)
 */
#define DZB_PERIOD_INT_MAIN_MS         250u
#define DZB_PERIOD_INT_SEC_MS          250u
#define DZB_PERIOD_EXT_SLOW_MS        1000u
#define DZB_PERIOD_EXT_STATUS_MS       400u
#define DZB_PERIOD_EXT_SEC_STATUS_MS   400u
#define DZB_PERIOD_EXT_A_GROUP_MS     1000u   /* new: …0A… IDs */

/* =================== PER-CHANNEL LAST-TX TIMES ================== */
static uint32_t s_last_int_main_ms;
static uint32_t s_last_int_sec_ms;
static uint32_t s_last_ext_slow_ms;
static uint32_t s_last_ext_status_ms;
static uint32_t s_last_ext_sec_status_ms;
static uint32_t s_last_ext_A_group_ms;   /* new */

/* node ID (low byte used in many 0x18xx080x IDs) */
static uint8_t  s_node_id = 0u;

/* ------------------------- helpers ------------------------------ */
static inline void le16(uint8_t *d, uint16_t v) { d[0]=(uint8_t)v; d[1]=(uint8_t)(v>>8); }
static inline void le32(uint8_t *d, uint32_t v) { d[0]=(uint8_t)v; d[1]=(uint8_t)(v>>8); d[2]=(uint8_t)(v>>16); d[3]=(uint8_t)(v>>24); }
static inline void be16(uint8_t *d, uint16_t v) { d[0]=(uint8_t)(v>>8); d[1]=(uint8_t)v; }
static inline void be32(uint8_t *d, uint32_t v) { d[0]=(uint8_t)(v>>24); d[1]=(uint8_t)(v>>16); d[2]=(uint8_t)(v>>8); d[3]=(uint8_t)v; }

/* 1 mV/bit, clamp to 0..65.535 V */
static inline uint16_t volt_to_1mV(float v){ if(v<0)v=0; if(v>65.535f)v=65.535f; return (uint16_t)(v*1000.0f); }
/* 1.5 mV/bit */
static inline uint16_t volt_to_1p5mV(float v){ float raw=v*1000.0f/1.5f; if(raw<0)raw=0; if(raw>65535)raw=65535; return (uint16_t)raw; }
/* 12 mV/bit */
static inline uint16_t volt_to_12mV(float v){ float raw=v*1000.0f/12.0f; if(raw<0)raw=0; if(raw>65535)raw=65535; return (uint16_t)raw; }
/* 0.1 K/bit (input °C) */
static inline uint16_t tempC_to_0p1K(float c){ float k=(c+273.15f)*10.0f; if(k<0)k=0; if(k>65535)k=65535; return (uint16_t)k; }
/* abs current, 1 mA/bit */
static inline uint32_t amp_to_mA_abs(float a){ float x=(a<0.0f)?-a:a; return (uint32_t)(x*1000.0f); }

/* Make 6 pseudo-cells from TelemetryOut (keeps your previous layout) */
static void dzb_make_cells(const TelemetryOut *T, float outCells[6]){
    float hi=T->highCell_V, lo=T->lowCell_V;
    outCells[0]=hi; outCells[1]=hi-0.01f; outCells[2]=lo+0.01f;
    outCells[3]=lo; outCells[4]=lo; outCells[5]=lo;
}

/* ================= MAIN INTERNAL (0x064..0x067) ================= */
static void dzb_send_main_internal(const TelemetryOut *T){
    uint8_t d[8]; float cells[6]; dzb_make_cells(T,cells);

    le16(&d[0], volt_to_1mV(cells[0]));
    le16(&d[2], volt_to_1mV(cells[1]));
    le16(&d[4], volt_to_1mV(cells[2]));
    le16(&d[6], volt_to_1mV(cells[3]));
    CAN_TX_SendStd(0x064u, d, 8);

    le16(&d[0], volt_to_1mV(cells[4]));
    le16(&d[2], volt_to_1mV(cells[5]));
    le32(&d[4], amp_to_mA_abs(0.0f));
    CAN_TX_SendStd(0x065u, d, 8);

    float tHighC=(float)T->tempHigh_0p1C*0.1f;
    float tLowC =(float)T->tempLow_0p1C *0.1f;
    float tAvgC =0.5f*(tHighC+tLowC);

    le16(&d[0], tempC_to_0p1K(tHighC));
    le16(&d[2], tempC_to_0p1K(tLowC));
    le16(&d[4], tempC_to_0p1K(tAvgC));
    d[6]=T->soc_pct; d[7]=0x00;
    CAN_TX_SendStd(0x066u, d, 8);

    memset(d,0,8);
    le32(&d[0], 0x00000040u); /* pack uid */
    d[4]=0x02u;               /* SYS_STATE_IDLE */
    CAN_TX_SendStd(0x067u, d, 8);
}

/* ================= SECONDARY INTERNAL (0x074..0x077) ================= */
static void dzb_send_secondary_internal(const TelemetryOut *T){
    uint8_t d[8]; float cells[6]; dzb_make_cells(T,cells);

    le16(&d[0], volt_to_1mV(cells[0]));
    le16(&d[2], volt_to_1mV(cells[1]));
    le16(&d[4], volt_to_1mV(cells[2]));
    le16(&d[6], volt_to_1mV(cells[3]));
    CAN_TX_SendStd(0x074u, d, 8);

    le16(&d[0], volt_to_1mV(cells[4]));
    le16(&d[2], volt_to_1mV(cells[5]));
    le32(&d[4], amp_to_mA_abs(0.0f));
    CAN_TX_SendStd(0x075u, d, 8);

    float tHighC=(float)T->tempHigh_0p1C*0.1f;
    float tLowC =(float)T->tempLow_0p1C *0.1f;
    float tAvgC =0.5f*(tHighC+tLowC);

    le16(&d[0], tempC_to_0p1K(tHighC));
    le16(&d[2], tempC_to_0p1K(tLowC));
    le16(&d[4], tempC_to_0p1K(tAvgC));
    d[6]=T->soc_pct; d[7]=0x00;
    CAN_TX_SendStd(0x076u, d, 8);

    memset(d,0,8);
    le32(&d[0], 0x00000041u);  /* secondary uid = main+1 */
    d[4]=0x02u;                /* SYS_STATE_IDLE */
    d[5]=0x08u;                /* MAIN_BATTERY_PRESENT_SECONDARY = 1 */
    CAN_TX_SendStd(0x077u, d, 8);
}

/* ================= EXTERNAL SLOW (1 s) – main + secondary =================
 * Mirrors 0x180008xx / 0x180108xx / 0x180208xx / 0x180308xx / 0x180408xx / 0x180508xx
 * seen in the capture. We keep payload structure as in your previous version,
 * and add the missing 0x180208xx and 0x180508xx frames.
 */
static void dzb_send_main_external_slow(const TelemetryOut *T, bool haveSecondary)
{
    uint8_t d[8]; float cells[6]; dzb_make_cells(T,cells);

    float maxv=cells[0], minv=cells[0];
    for(int i=1;i<6;++i){ if(cells[i]>maxv)maxv=cells[i]; if(cells[i]<minv)minv=cells[i]; }

    uint32_t base0 = 0x18000800u | s_node_id; /* cells 0..3 */
    uint32_t base1 = 0x18010800u | s_node_id; /* cells 4..5, max, min */
    uint32_t base2 = 0x18020800u | s_node_id; /* new: matches log */
    uint32_t base3 = 0x18030800u | s_node_id; /* pack voltage (12 mV/bit) */
    uint32_t base4 = 0x18040800u | s_node_id; /* summary/temp */
    uint32_t base5 = 0x18050800u | s_node_id; /* new: present in log */

    /* 180008xx */
    be16(&d[0], volt_to_1p5mV(cells[0]));
    be16(&d[2], volt_to_1p5mV(cells[1]));
    be16(&d[4], volt_to_1p5mV(cells[2]));
    be16(&d[6], volt_to_1p5mV(cells[3]));
    CAN_TX_SendExt(base0, d, 8);

    /* 180108xx */
    be16(&d[0], volt_to_1p5mV(cells[4]));
    be16(&d[2], volt_to_1p5mV(cells[5]));
    be16(&d[4], volt_to_1p5mV(maxv));
    be16(&d[6], volt_to_1p5mV(minv));
    CAN_TX_SendExt(base1, d, 8);

    /* 180208xx (lightweight misc – seen low nonzero nibble) */
    memset(d,0,8);
    d[0]=0x00; d[1]=0x0A; /* minimal nonzero like capture */
    CAN_TX_SendExt(base2, d, 8);

    /* 180308xx (pack V @ 12 mV/bit) */
    memset(d,0,8);
    be16(&d[0], volt_to_12mV(T->pack_V));
    CAN_TX_SendExt(base3, d, 8);

    /* 180408xx (summary: uid/temps/etc.; keep your layout feel) */
    memset(d,0,8);
    be32(&d[0], 0x000C9090u);           /* rolling-ish summary stamp */
    d[6]=0x0B; d[7]=0x86;               /* sample tails similar to log */
    CAN_TX_SendExt(base4, d, 8);

    /* 180508xx (present in log; contents appear counters-ish) */
    memset(d,0,8);
    d[6]=0x0B; d[7]=0x8A;               /* keep low activity bytes */
    CAN_TX_SendExt(base5, d, 8);

    if(haveSecondary){
        CAN_TX_SendExt((base0|1u), d, 8);  /* ensure “xx…01” also shows up */
        /* (if you want full secondary mirrors, replicate base1..base5+1 similarly) */
    }
}

/* ================= EXTERNAL STATUS (400 ms) ====================
 * 180608xx / 180708xx / 180808xx (and a minimal secondary presence ping)
 */
static void dzb_send_main_external_status400(const TelemetryOut *T, bool haveSecondary)
{
    uint8_t d[8]; float cells[6]; dzb_make_cells(T,cells);

    float maxv=cells[0], minv=cells[0];
    for(int i=1;i<6;++i){ if(cells[i]>maxv)maxv=cells[i]; if(cells[i]<minv)minv=cells[i]; }

    uint32_t base0 = 0x18060800u | s_node_id;
    uint32_t base1 = 0x18070800u | s_node_id;
    uint32_t base2 = 0x18080800u | s_node_id;

    /* 180608xx – status A */
    memset(d,0,8);
    d[0]=0x40u;                              /* matches top nibble in capture */
    be16(&d[4], volt_to_1p5mV(maxv));
    be16(&d[6], volt_to_1p5mV(minv));
    CAN_TX_SendExt(base0, d, 8);

    /* 180708xx – status B */
    memset(d,0,8);
    float totalV = T->pack_V; /* single pack unless you want to double it */
    be16(&d[0], volt_to_12mV(totalV));
    d[2]=T->soc_pct;
    CAN_TX_SendExt(base1, d, 8);

    /* 180808xx – status C */
    memset(d,0,8);
    d[4]=0x00u;
    d[5]=(T->last_error_code ? 0x02u : 0x00u); /* error bit */
    be16(&d[6], 0x0000u);                       /* main-present flag pattern */
    CAN_TX_SendExt(base2, d, 8);

    if(haveSecondary){
        uint8_t z[8]={0};
        CAN_TX_SendExt(0x18080801u, z, 8);      /* secondary presence ping */
    }
}

/* ================= EXTERNAL “0A” DIAG/STATUS GROUP (1 s) =========
 * Adds the missing IDs that appear frequently in your DZB logs:
 * 18010Axx (5B), 18020Axx (6B), 18030Axx (6B), 18040Axx (8B),
 * 18050Axx (5B), 18060Axx (6B), 18070Axx (7B), 18090Axx (7B),
 * 180A0A00 (7B), plus 180B080x (6B) and 180C080x (4B).
 * Payloads are conservative and shaped to match the observed sizes and
 * byte flavours so your receiver’s heuristics line up with the capture.
 */
static void dzb_send_external_A_group(const TelemetryOut *T, bool haveSecondary)
{
    uint8_t d[8]={0};

    /* 18010A00 / 01 – 5 bytes */
    d[0]=0x01; d[1]=0x04; d[2]=0x06; d[3]=0x02; d[4]=0x00;
    CAN_TX_SendExt(0x18010A00u, d, 5);
    if(haveSecondary){ d[4]=0x01; CAN_TX_SendExt(0x18010A01u, d, 5); }

    /* 18020A00 / 01 – 6 bytes (0x0AF0 etc in log) */
    d[0]=0x0A; d[1]=0xF0; d[2]=0x06; d[3]=0x82; d[4]=0x00; d[5]=0x0D;
    CAN_TX_SendExt(0x18020A00u, d, 6);
    if(haveSecondary){ CAN_TX_SendExt(0x18020A01u, d, 6); }

    /* 18030A00 / 01 – 6 bytes (0x03E8 0x0AF0 …) */
    d[0]=0x03; d[1]=0xE8; d[2]=0x0A; d[3]=0xF0; d[4]=0x09; d[5]=0xA2;
    CAN_TX_SendExt(0x18030A00u, d, 6);
    if(haveSecondary){ CAN_TX_SendExt(0x18030A01u, d, 6); }

    /* 18040A00 / 01 – 8 bytes (… 00 CC CC) */
    d[0]=0x00; d[1]=0x1E; d[2]=0x86; d[3]=0x1B; d[4]=0x11; d[5]=0x00; d[6]=0xCC; d[7]=0xCC;
    CAN_TX_SendExt(0x18040A00u, d, 8);
    if(haveSecondary){ d[3]=0x1C; CAN_TX_SendExt(0x18040A01u, d, 8); }

    /* 18050A00 / 01 – 5 bytes (00 01 00 00 00) */
    d[0]=0x00; d[1]=0x01; d[2]=0x00; d[3]=0x00; d[4]=0x00;
    CAN_TX_SendExt(0x18050A00u, d, 5);
    if(haveSecondary){ CAN_TX_SendExt(0x18050A01u, d, 5); }

    /* 18060A00 / 01 – 6 bytes (0B 78 00 5A FA 00/0B) */
    d[0]=0x0B; d[1]=0x78; d[2]=0x00; d[3]=0x5A; d[4]=0xFA; d[5]=0x00;
    CAN_TX_SendExt(0x18060A00u, d, 6);
    d[5]=0x0B;
    if(haveSecondary){ CAN_TX_SendExt(0x18060A01u, d, 6); }

    /* 18070A00 / 01 – 7 bytes (FF FF FE 0C 01 5F 90) */
    d[0]=0xFF; d[1]=0xFF; d[2]=0xFE; d[3]=0x0C; d[4]=0x01; d[5]=0x5F; d[6]=0x90;
    CAN_TX_SendExt(0x18070A00u, d, 7);
    if(haveSecondary){ CAN_TX_SendExt(0x18070A01u, d, 7); }

    /* 18090A00 / 01 – 7 bytes (0A 28 00 5A 02 0C 5F) */
    d[0]=0x0A; d[1]=0x28; d[2]=0x00; d[3]=0x5A; d[4]=0x02; d[5]=0x0C; d[6]=0x5F;
    CAN_TX_SendExt(0x18090A00u, d, 7);
    if(haveSecondary){ CAN_TX_SendExt(0x18090A01u, d, 7); }

    /* 180A0A00 – 7 bytes (main only observed) */
    d[0]=0x00; d[1]=0x00; d[2]=0x00; d[3]=0x00; d[4]=0xFF; d[5]=0xFE; d[6]=0x00;
    CAN_TX_SendExt(0x180A0A00u, d, 7);

    /* 180B0800/01 – 6 bytes */
    d[0]=0x94; d[1]=0x70; d[2]=0x03; d[3]=0x2E; d[4]=0x00; d[5]=0x29;
    CAN_TX_SendExt(0x180B0800u, d, 6);
    if(haveSecondary){ d[0]=0x96; d[1]=0x64; d[2]=0x03; d[3]=0x39; CAN_TX_SendExt(0x180B0801u, d, 6); }

    /* 180C0800/01 – 4 bytes (two cell-ish bytes pairs) */
    d[0]=0x0B; d[1]=0x8A; d[2]=0x0B; d[3]=0x82;
    CAN_TX_SendExt(0x180C0800u, d, 4);
    if(haveSecondary){ d[1]=0x86; d[3]=0x7F; CAN_TX_SendExt(0x180C0801u, d, 4); }
}

/* ================= PUBLICS ================= */
void CAN_400DZB_Init(uint8_t node_id)
{
    s_node_id = node_id;
    s_last_int_main_ms       = 0u;
    s_last_int_sec_ms        = 0u;
    s_last_ext_slow_ms       = 0u;
    s_last_ext_status_ms     = 0u;
    s_last_ext_sec_status_ms = 0u;
    s_last_ext_A_group_ms    = 0u;  /* new */
}

void CAN_400DZB_Tick(const TelemetryOut *T,
                     uint32_t nowMs,
                     bool mirrorToSecondary)
{
    /* Internal std frames */
    if ((nowMs - s_last_int_main_ms) >= DZB_PERIOD_INT_MAIN_MS) {
        dzb_send_main_internal(T);
        s_last_int_main_ms = nowMs;
    }
    if (mirrorToSecondary &&
        (nowMs - s_last_int_sec_ms) >= DZB_PERIOD_INT_SEC_MS) {
        dzb_send_secondary_internal(T);
        s_last_int_sec_ms = nowMs;
    }

    /* External slow block (cells/pack/summary) */
    if ((nowMs - s_last_ext_slow_ms) >= DZB_PERIOD_EXT_SLOW_MS) {
        dzb_send_main_external_slow(T, mirrorToSecondary);
        s_last_ext_slow_ms = nowMs;
    }

    /* External status (…608/708/808) */
    if ((nowMs - s_last_ext_status_ms) >= DZB_PERIOD_EXT_STATUS_MS) {
        dzb_send_main_external_status400(T, mirrorToSecondary);
        s_last_ext_status_ms = nowMs;
    }

    /* Secondary presence ping at same cadence as status */
    if (mirrorToSecondary &&
        (nowMs - s_last_ext_sec_status_ms) >= DZB_PERIOD_EXT_SEC_STATUS_MS) {
        uint8_t z[8]={0};
        CAN_TX_SendExt(0x18080801u, z, 8);
        s_last_ext_sec_status_ms = nowMs;
    }

    /* NEW: External “0A” diag/status group */
    if ((nowMs - s_last_ext_A_group_ms) >= DZB_PERIOD_EXT_A_GROUP_MS) {
        dzb_send_external_A_group(T, mirrorToSecondary);
        s_last_ext_A_group_ms = nowMs;
    }
}
