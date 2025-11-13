
# BMS Fault Decoder – Integration Guide

This module centralizes decoding of BMS faults/errors across your six battery families.
It converts raw bitfields/bytes into human text and a severity you can show on `pMain` and `pDetails`.

## Files
- `bms_fault_decode.h`
- `bms_fault_decode.c`

## 1) Add to your build
```
src/
  bms_fault_decode.c
inc/
  bms_fault_decode.h
```
Include `inc` in your include path and compile `bms_fault_decode.c` into your project.

## 2) Wire into your CAN parsers

### BMZ500 or CP600 (0x10000010 -> pack fault byte)
```c
#include "bms_fault_decode.h"

// After you parse the pack fault byte (example: data[5] or wherever you store it)
uint8_t fault = pack_fault_byte;
char reasons[128];
BmsSeverity sev; BmsDomainMask dom;

bms_decode_bmz500_cp600(fault, reasons, sizeof(reasons), &sev, &dom);

// Post to HMI
nextion_set_text("pDetails.tBmsFault", reasons);
nextion_set_text("pMain.tErrors", reasons);  // or a shortened roll-up
// Optionally color by severity:
ui_set_recoverable_color(sev);
```

### HYP500 (0x18FF05xx -> hw_fault; 0x18FF0Exx -> severity+code)
```c
#include "bms_fault_decode.h"

uint8_t hw_fault = hyp500_hw_fault_byte;      // from 0x18FF05xx byte4
uint8_t sev_raw  = hyp500_error_severity;     // from 0x18FF0Exx
uint8_t err_code = hyp500_error_code;         // from 0x18FF0Exx

char reasons[160];
BmsSeverity sev; BmsDomainMask dom;
bms_decode_hyp500(hw_fault, sev_raw, err_code, reasons, sizeof(reasons), &sev, &dom);

nextion_set_text("pDetails.tBmsFault", reasons);
nextion_set_text("pMain.tErrors", reasons);
ui_set_recoverable_color(sev);
```

> Note: Until the HYP500 Error Code Table is available, the decoder prints `Severity: <text> (0xNN), Code: 0xNN`.

### HYP400 (gather flags from 0x180508/0x180608/0x180808)
While parsing those frames, set a snapshot of booleans and pass it in:
```c
BmsHyp400Input h4 = {
  .bms_fault   = bool_from_0x180608,
  .cell_uv     = any_uv_from_0x180508,
  .cell_ov     = any_ov_from_0x180508,
  .dchg_oc     = oc_dchg_from_0x180808,
  .chg_oc      = oc_chg_from_0x180808,
  .unbalanced  = unbalanced_from_0x180808,
  .node_missing= node_miss_from_0x180808,
  .hw_fault    = hw_fault_from_0x180508
};
char reasons[160]; BmsSeverity sev; BmsDomainMask dom;
bms_decode_hyp400(&h4, reasons, sizeof(reasons), &sev, &dom);

nextion_set_text("pDetails.tBmsFault", reasons);
nextion_set_text("pMain.tErrors", reasons);
ui_set_recoverable_color(sev);
```

### CP400 (chill-only or dual-zone)
Use the master fault code and detailed flags you parse:
```c
BmsCp400Input cp = {
  .master_fault_code = master_code, // 0=Normal,1=Warning,2=Fault,3=Permanent,4=HW Fault
  .uv = uv, .ov = ov, .ot = ot, .ut = ut,
  .dchg_oc = d_oc, .chg_oc = c_oc,
  .therm_warning = therm_warn,
  .imbalance = imbalance
};
char reasons[160]; BmsSeverity sev; BmsDomainMask dom;
bms_decode_cp400(&cp, reasons, sizeof(reasons), &sev, &dom);

nextion_set_text("pDetails.tBmsFault", reasons);
nextion_set_text("pMain.tErrors", reasons);
ui_set_recoverable_color(sev);
```

## 3) One-call entry (optional)
If you prefer a single entry point across your parser, fill the tagged union:
```c
BmsDecodeInput in = { .family = BMS_FAM_BMZ500 };
in.u.bmz_cp600.pack_fault = fault_byte;

char reasons[160]; BmsSeverity sev; BmsDomainMask dom;
bms_decode_any(&in, reasons, sizeof(reasons), &sev, &dom);
```

## 4) Coloring / Recoverable logic
Map `BmsSeverity` to your HMI colors:
- `NONE` → normal
- `WARNING` → **solid orange** (your new clearer orange)
- `FAULT` / `PERMANENT` → red
- `HW_FAULT` / `FATAL` → deep red

Example:
```c
void ui_set_recoverable_color(BmsSeverity sev){
    uint16_t color;
    switch(sev){
        case BMS_SEV_WARNING:   color = 64512; break; /* solid orange */
        case BMS_SEV_FAULT:
        case BMS_SEV_PERMANENT:
        case BMS_SEV_HW_FAULT:
        case BMS_SEV_FATAL:     color = 63488; break; /* red */
        case BMS_SEV_NONE:
        default:                color = 2016;  break; /* green/normal */
    }
    nextion_set_bco("pMain.tRecHead", color);
}
```

## 5) Example with your `fault=0xAA`
```
uint8_t f = 0xAA;
char reasons[128];
BmsSeverity sev; BmsDomainMask dom;
bms_decode_bmz500_cp600(f, reasons, sizeof(reasons), &sev, &dom);
// reasons => "Charger current > demand, Discharge overcurrent, Under-voltage, Over-voltage, Over-temperature, Under-temperature, General BMS fault, Voltage imbalance"
// (depending on bits set; for 0xAA, "Discharge overcurrent, Over-voltage, Under-temperature, Voltage imbalance")
// sev => FAULT (since OV/UT/OC present)
```

## 6) Notes
- All functions are pure and safe for ISRs if your string ops are avoided there.
- Pre-size `reasons` to ~160 chars if you plan to include multiple domains.
- If you receive the HYP500 Error Code Table, add a mapping in `bms_decode_hyp500()`.
