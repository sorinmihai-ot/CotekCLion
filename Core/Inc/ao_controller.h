#ifndef AO_CONTROLLER_H
#define AO_CONTROLLER_H

#include "qpc.h"
#include "app_signals.h"
#include "bms_app.h"     // for BmsTelemetry

#ifdef __cplusplus
extern "C" {
#endif

    extern QActive *AO_Controller;
    void ControllerAO_ctor(void);

#ifdef __cplusplus
}
#endif
#endif
