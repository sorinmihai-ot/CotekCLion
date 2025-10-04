#ifndef AO_COTEK_H
#define AO_COTEK_H

#include "qpc.h"
#include "app_signals.h"

#ifdef __cplusplus
extern "C" {
#endif

    extern QActive *AO_Cotek;
    void CotekAO_ctor(void);
    uint8_t Cotek_isPresent(void);

#ifdef __cplusplus
}
#endif
#endif
