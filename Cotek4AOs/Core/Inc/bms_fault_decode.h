
/**
 * bms_fault_decode.h
 *
 * Centralized decoding for BMS faults/errors across multiple battery families.
 *
 * Families covered (from your docs):
 *  - Hyperdrive 400 (HYP400)
 *  - Hyperdrive 500 (HYP500)
 *  - BMZ 500 (BMZ500)
 *  - Custom Power 600s (CP600)
 *  - Custom Power 400 (chill-only) (CP400_CHILL)
 *  - Custom Power 400 (dual-zone) (CP400_DUAL)
 *
 * This module converts raw bitfields/bytes into human-readable strings and a severity.
 * All functions are re-entrant and avoid static buffers; you provide the output buffer.
 */
#ifndef BMS_FAULT_DECODE_H
#define BMS_FAULT_DECODE_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*--------------------------- Types & enums ---------------------------*/

typedef enum {
    BMS_FAM_HYP400 = 0,
    BMS_FAM_HYP500,
    BMS_FAM_BMZ500,
    BMS_FAM_CP600,
    BMS_FAM_CP400_CHILL,
    BMS_FAM_CP400_DUAL
} BmsBatteryFamily;

typedef enum {
    BMS_SEV_NONE = 0,     /* No active issue */
    BMS_SEV_WARNING,      /* Warning / recoverable */
    BMS_SEV_FAULT,        /* Fault (recoverable by user/system) */
    BMS_SEV_PERMANENT,    /* Permanent fault (latched) */
    BMS_SEV_HW_FAULT,     /* Hardware fault */
    BMS_SEV_FATAL         /* Fatal / stop */
} BmsSeverity;

/* Output mask describing which decoded domains are present (optional) */
typedef enum {
    BMS_DOM_NONE   = 0x00,
    BMS_DOM_PACK   = 0x01,  /* pack-level fault bitfield (BMZ/CP600 style) */
    BMS_DOM_HWCOMM = 0x02,  /* comms HW fault (HYP500) */
    BMS_DOM_TEMP   = 0x04,
    BMS_DOM_VOLT   = 0x08,
    BMS_DOM_CURR   = 0x10,
    BMS_DOM_BAL    = 0x20,
    BMS_DOM_NODE   = 0x40,
    BMS_DOM_OTHER  = 0x80
} BmsDomainMask;

/*--------------------------- Family inputs ---------------------------*/

/* BMZ500 / CP600: 0x10000010 pack fault byte (bitfield) */
typedef struct {
    uint8_t pack_fault; /* bit0..7 per spec */
} BmsBmzCp600Input;

/* HYP500: combine HW comm fault byte and an error message frame (severity+code). */
typedef struct {
    uint8_t hw_fault;       /* 0: I2C1, 1: I2C2, 2: CAN, 3: SPI */
    uint8_t error_severity; /* from 0x18FF0EXX; mapping below */
    uint8_t error_code;     /* numeric code (table is external/not provided) */
} BmsHyp500Input;

/* HYP400: consolidate booleans gathered from 0x180508/0x180608/0x180808 */
typedef struct {
    bool bms_fault;     /* 0x180608XX byte3 -> any BMS fault observed */
    bool cell_uv;       /* any cell undervoltage asserted */
    bool cell_ov;       /* any cell overvoltage asserted */
    bool dchg_oc;       /* discharge OC */
    bool chg_oc;        /* charge OC */
    bool unbalanced;    /* unbalanced pack */
    bool node_missing;  /* node missing */
    bool hw_fault;      /* HW fault (from 0x180508XX byte4) */
} BmsHyp400Input;

/* CP400 chill-only / dual-zone: high-level master fault code + detailed flags */
typedef struct {
    uint8_t  master_fault_code; /* 0=Normal,1=Warning,2=Fault,3=Permanent,4=HW Fault */
    bool     uv; bool ov; bool ot; bool ut;
    bool     dchg_oc; bool chg_oc;
    bool     therm_warning; /* if present in your parse */
    bool     imbalance;
} BmsCp400Input;

/* Tagged union to use a single entry point if preferred */
typedef struct {
    BmsBatteryFamily family;
    union {
        BmsBmzCp600Input bmz_cp600;
        BmsHyp500Input   hyp500;
        BmsHyp400Input   hyp400;
        BmsCp400Input    cp400;
    } u;
} BmsDecodeInput;

/* Result container (optional, you can also just use the string + severity) */
typedef struct {
    BmsSeverity severity;
    BmsDomainMask domains;
    /* You provide this buffer to the API call */
} BmsDecodeResult;

/*--------------------------- API ---------------------------*/

/**
 * Decode BMZ500 / CP600 pack fault byte (0x10000010).
 */
void bms_decode_bmz500_cp600(uint8_t pack_fault,
                             char *out, size_t out_cap,
                             BmsSeverity *out_sev,
                             BmsDomainMask *out_domains);

/**
 * Decode HYP500 faults based on HW comm fault bits and message severity/code.
 * Note: error_code textual mapping is not available; we print "Code 0xNN".
 */
void bms_decode_hyp500(uint8_t hw_fault, uint8_t error_sev_raw, uint8_t error_code,
                       char *out, size_t out_cap,
                       BmsSeverity *out_sev,
                       BmsDomainMask *out_domains);

/**
 * Decode HYP400 from consolidated flags collected while parsing 0x1805/0x1806/0x1808 frames.
 */
void bms_decode_hyp400(const BmsHyp400Input *in,
                       char *out, size_t out_cap,
                       BmsSeverity *out_sev,
                       BmsDomainMask *out_domains);

/**
 * Decode CP400 (chill-only / dual-zone) using master fault code and detailed flags.
 */
void bms_decode_cp400(const BmsCp400Input *in,
                      char *out, size_t out_cap,
                      BmsSeverity *out_sev,
                      BmsDomainMask *out_domains);

/**
 * Single entry point if you prefer one call site.
 * Provide a BmsDecodeInput with 'family' set and relevant union field filled.
 */
void bms_decode_any(const BmsDecodeInput *in,
                    char *out, size_t out_cap,
                    BmsSeverity *out_sev,
                    BmsDomainMask *out_domains);

/* Optional helpers for UI */
const char* bms_severity_to_text(BmsSeverity sev);
const char* bms_family_to_text(BmsBatteryFamily fam);

#ifdef __cplusplus
}
#endif
#endif /* BMS_FAULT_DECODE_H */
