#ifndef CAN_400HYP_H_
#define CAN_400HYP_H_

#include <stdint.h>
#include <stdbool.h>
#include "can_tx.h"    // for TelemetryOut

#ifdef __cplusplus
extern "C" {
#endif

    /* init with node id (0 = master, anything else = slave/secondary) */
    void CAN_400HYP_Init(uint8_t node_id);

    /* call every 10 ms from can_tx.c – it will self-throttle to 100 ms / 1 s */
    void CAN_400HYP_Tick(const TelemetryOut *t, uint32_t now_ms);

    /* optional: override serial / fw / type shown in 0x18040A.. */
    void CAN_400HYP_SetIdentity(uint32_t serial,
                                uint32_t fw_ver,
                                uint16_t batt_type);

#ifdef __cplusplus
}
#endif

#endif /* CAN_400HYP_H_ */
