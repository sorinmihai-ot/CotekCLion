//
// BMS Active Object (AO_Bms) -- multi-pack (400s / 500s / 600s)
//
#ifndef BMS_APP_H
#define BMS_APP_H

#include <stdint.h>
#include "can_app.h"
#include "app_signals.h"
#include "qpc.h"

#ifdef __cplusplus
extern "C" {
#endif

    /* AO handle + ctor */
    extern QActive *AO_Bms;
    void BmsAO_ctor(void);

    /* helpers (also handy for tests) */
    int  BMS_ParseFrame(const CanFrameEvt *f, BmsTelemetry *bms);
    void BMS_GetSnapshot(BmsTelemetry *dst);

#ifdef __cplusplus
}
#endif

#endif /* BMS_APP_H */
