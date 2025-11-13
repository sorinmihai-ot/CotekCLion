/**
 * bms_fault_decode.c
 */
#include "bms_fault_decode.h"
#include <string.h>
#include <stdio.h>

static inline bool bit(uint32_t v, unsigned i){ return (v >> i) & 1U; }

static void append_reason(char *dst, size_t cap, const char *txt) {
    if (!dst || cap == 0) return;
    size_t len = strlen(dst);
    if (len && len < cap-2) { dst[len++] = ','; dst[len++] = ' '; dst[len] = '\0'; }
    strncat(dst, txt, cap - strlen(dst) - 1);
}
static void set_if_empty(char *dst, size_t cap, const char *txt) {
    if (dst && dst[0] == '\0') {
        strncpy(dst, txt, cap);
        if (cap) dst[cap-1] = '\0';
    }
}

const char* bms_severity_to_text(BmsSeverity sev){
    switch(sev){
        case BMS_SEV_NONE:      return "None";
        case BMS_SEV_WARNING:   return "Warning";
        case BMS_SEV_FAULT:     return "Fault";
        case BMS_SEV_PERMANENT: return "Permanent Fault";
        case BMS_SEV_HW_FAULT:  return "Hardware Fault";
        case BMS_SEV_FATAL:     return "Fatal";
        default:                return "Unknown";
    }
}
const char* bms_family_to_text(BmsBatteryFamily fam){
    switch(fam){
        case BMS_FAM_HYP400:      return "HYP400";
        case BMS_FAM_HYP500:      return "HYP500";
        case BMS_FAM_BMZ500:      return "BMZ500";
        case BMS_FAM_CP600:       return "CP600";
        case BMS_FAM_CP400_CHILL: return "CP400(chill)";
        case BMS_FAM_CP400_DUAL:  return "CP400(dual)";
        default:                  return "Unknown";
    }
}

/* BMZ500 / CP600: 8-bit pack fault */
void bms_decode_bmz500_cp600(uint8_t f, char *out, size_t cap,
                             BmsSeverity *out_sev, BmsDomainMask *out_domains){
    if(out) out[0]='\0';
    if(out_sev) *out_sev = BMS_SEV_NONE;
    if(out_domains) *out_domains = BMS_DOM_NONE;

    if(bit(f,0)) { append_reason(out,cap,"Charger current > demand"); if(out_domains) *out_domains |= BMS_DOM_CURR; }
    if(bit(f,1)) { append_reason(out,cap,"Discharge overcurrent");    if(out_domains) *out_domains |= BMS_DOM_CURR; }
    if(bit(f,2)) { append_reason(out,cap,"Under-voltage");            if(out_domains) *out_domains |= BMS_DOM_VOLT; }
    if(bit(f,3)) { append_reason(out,cap,"Over-voltage");             if(out_domains) *out_domains |= BMS_DOM_VOLT; }
    if(bit(f,4)) { append_reason(out,cap,"Over-temperature");         if(out_domains) *out_domains |= BMS_DOM_TEMP; }
    if(bit(f,5)) { append_reason(out,cap,"Under-temperature");        if(out_domains) *out_domains |= BMS_DOM_TEMP; }
    if(bit(f,6)) { append_reason(out,cap,"General BMS fault");        if(out_domains) *out_domains |= BMS_DOM_OTHER; }
    if(bit(f,7)) { append_reason(out,cap,"Voltage imbalance");        if(out_domains) *out_domains |= BMS_DOM_BAL; }

    if(out_sev){
        if (f == 0) *out_sev = BMS_SEV_NONE;
        else if (bit(f,6)) *out_sev = BMS_SEV_HW_FAULT;
        else if (bit(f,3) || bit(f,2) || bit(f,1) || bit(f,4) || bit(f,5)) *out_sev = BMS_SEV_FAULT;
        else if (bit(f,7) || bit(f,0)) *out_sev = BMS_SEV_WARNING;
        else *out_sev = BMS_SEV_FAULT;
    }
    set_if_empty(out,cap,"None");
}

/* HYP500 */
static BmsSeverity map_hyp500_sev(uint8_t raw){
    switch(raw){
        case 0xC1: return BMS_SEV_WARNING;
        case 0xC2: return BMS_SEV_FAULT;
        case 0xC3: return BMS_SEV_FATAL;
        default:
            switch(raw){
                case 0x00: return BMS_SEV_NONE;
                case 0x01: return BMS_SEV_WARNING;
                case 0x02: return BMS_SEV_FAULT;
                case 0x03: return BMS_SEV_PERMANENT;
                case 0x04: return BMS_SEV_HW_FAULT;
                default:   return BMS_SEV_FAULT;
            }
    }
}
void bms_decode_hyp500(uint8_t hw_fault, uint8_t error_sev_raw, uint8_t error_code,
                       char *out, size_t cap,
                       BmsSeverity *out_sev,
                       BmsDomainMask *out_domains){
    if(out) out[0]='\0';
    if(out_domains) *out_domains = BMS_DOM_NONE;

    if(bit(hw_fault,0)) { append_reason(out,cap,"I2C ch1 error"); if(out_domains) *out_domains |= BMS_DOM_HWCOMM; }
    if(bit(hw_fault,1)) { append_reason(out,cap,"I2C ch2 error"); if(out_domains) *out_domains |= BMS_DOM_HWCOMM; }
    if(bit(hw_fault,2)) { append_reason(out,cap,"CAN bus error"); if(out_domains) *out_domains |= BMS_DOM_HWCOMM; }
    if(bit(hw_fault,3)) { append_reason(out,cap,"SPI error");     if(out_domains) *out_domains |= BMS_DOM_HWCOMM; }

    BmsSeverity sev = map_hyp500_sev(error_sev_raw);
    if(out_sev) *out_sev = sev;

    char tmp[64];
    snprintf(tmp, sizeof(tmp), "Severity: %s (0x%02X), Code: 0x%02X",
             bms_severity_to_text(sev), error_sev_raw, error_code);
    append_reason(out, cap, tmp);

    set_if_empty(out,cap,"None");
}

/* HYP400 */
void bms_decode_hyp400(const BmsHyp400Input *in,
                       char *out, size_t cap,
                       BmsSeverity *out_sev,
                       BmsDomainMask *out_domains){
    if(!in) return;
    if(out) out[0]='\0';
    if(out_domains) *out_domains = BMS_DOM_NONE;

    if(in->bms_fault)   { append_reason(out,cap,"BMS fault");            if(out_domains) *out_domains |= BMS_DOM_OTHER; }
    if(in->cell_uv)     { append_reason(out,cap,"Cell undervoltage");    if(out_domains) *out_domains |= BMS_DOM_VOLT; }
    if(in->cell_ov)     { append_reason(out,cap,"Cell overvoltage");     if(out_domains) *out_domains |= BMS_DOM_VOLT; }
    if(in->dchg_oc)     { append_reason(out,cap,"Discharge overcurrent");if(out_domains) *out_domains |= BMS_DOM_CURR; }
    if(in->chg_oc)      { append_reason(out,cap,"Charge overcurrent");   if(out_domains) *out_domains |= BMS_DOM_CURR; }
    if(in->unbalanced)  { append_reason(out,cap,"Pack unbalanced");      if(out_domains) *out_domains |= BMS_DOM_BAL; }
    if(in->node_missing){ append_reason(out,cap,"Node missing");         if(out_domains) *out_domains |= BMS_DOM_NODE; }
    if(in->hw_fault)    { append_reason(out,cap,"BMS hardware fault");   if(out_domains) *out_domains |= BMS_DOM_OTHER; }

    if(out_sev){
        if(in->hw_fault) *out_sev = BMS_SEV_HW_FAULT;
        else if (in->cell_uv || in->cell_ov || in->dchg_oc || in->chg_oc) *out_sev = BMS_SEV_FAULT;
        else if (in->unbalanced || in->node_missing || in->bms_fault) *out_sev = BMS_SEV_WARNING;
        else *out_sev = BMS_SEV_NONE;
    }
    set_if_empty(out,cap,"None");
}

/* CP400 */
static BmsSeverity map_cp400_master(uint8_t code){
    switch(code){
        case 0x00: return BMS_SEV_NONE;
        case 0x01: return BMS_SEV_WARNING;
        case 0x02: return BMS_SEV_FAULT;
        case 0x03: return BMS_SEV_PERMANENT;
        case 0x04: return BMS_SEV_HW_FAULT;
        default:   return BMS_SEV_FAULT;
    }
}
void bms_decode_cp400(const BmsCp400Input *in,
                      char *out, size_t cap,
                      BmsSeverity *out_sev,
                      BmsDomainMask *out_domains){
    if(!in) return;
    if(out) out[0]='\0';
    if(out_domains) *out_domains = BMS_DOM_NONE;

    BmsSeverity master = map_cp400_master(in->master_fault_code);
    char tmp[48];
    snprintf(tmp,sizeof(tmp),"State: %s (0x%02X)", bms_severity_to_text(master), in->master_fault_code);
    append_reason(out,cap,tmp);

    if(in->uv)           { append_reason(out,cap,"Under-voltage");        if(out_domains) *out_domains |= BMS_DOM_VOLT; }
    if(in->ov)           { append_reason(out,cap,"Over-voltage");         if(out_domains) *out_domains |= BMS_DOM_VOLT; }
    if(in->ot)           { append_reason(out,cap,"Over-temperature");     if(out_domains) *out_domains |= BMS_DOM_TEMP; }
    if(in->ut)           { append_reason(out,cap,"Under-temperature");    if(out_domains) *out_domains |= BMS_DOM_TEMP; }
    if(in->dchg_oc)      { append_reason(out,cap,"Discharge overcurrent");if(out_domains) *out_domains |= BMS_DOM_CURR; }
    if(in->chg_oc)       { append_reason(out,cap,"Charge overcurrent");   if(out_domains) *out_domains |= BMS_DOM_CURR; }
    if(in->therm_warning){ append_reason(out,cap,"Thermistor warning");   if(out_domains) *out_domains |= BMS_DOM_TEMP; }
    if(in->imbalance)    { append_reason(out,cap,"Voltage imbalance");    if(out_domains) *out_domains |= BMS_DOM_BAL; }

    if(out_sev) *out_sev = master;
    set_if_empty(out,cap,"None");
}

/* Single entry */
void bms_decode_any(const BmsDecodeInput *in,
                    char *out, size_t out_cap,
                    BmsSeverity *out_sev,
                    BmsDomainMask *out_domains){
    if(!in) return;
    switch(in->family){
        case BMS_FAM_BMZ500:
        case BMS_FAM_CP600:
            bms_decode_bmz500_cp600(in->u.bmz_cp600.pack_fault, out, out_cap, out_sev, out_domains);
            break;
        case BMS_FAM_HYP500:
            bms_decode_hyp500(in->u.hyp500.hw_fault, in->u.hyp500.error_severity, in->u.hyp500.error_code,
                              out, out_cap, out_sev, out_domains);
            break;
        case BMS_FAM_HYP400:
            bms_decode_hyp400(&in->u.hyp400, out, out_cap, out_sev, out_domains);
            break;
        case BMS_FAM_CP400_CHILL:
        case BMS_FAM_CP400_DUAL:
            bms_decode_cp400(&in->u.cp400, out, out_cap, out_sev, out_domains);
            break;
        default:
            if(out) { strncpy(out,"Unknown family", out_cap); if(out_cap) out[out_cap-1]='\0'; }
            if(out_sev) *out_sev = BMS_SEV_FAULT;
            if(out_domains) *out_domains = BMS_DOM_OTHER;
            break;
    }
}
