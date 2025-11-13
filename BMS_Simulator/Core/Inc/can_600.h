//
// Created by sorin.mihai on 31/10/2025.
//
#ifndef CAN_600_H_
#define CAN_600_H_

#include <stdint.h>
#include <stdbool.h>
#include "can_tx.h"    // for TelemetryOut (hi_V, lo_V, pack_V, temps, soc, last_error_code)

#ifdef __cplusplus
extern "C" {
#endif

    /* initialise the 600s CAN sender (call once at startup) */
    void CAN_600_Init(uint8_t node_id);

    /* periodic tick – call from CAN_TX_PeriodicTask() every 10 ms with sim time */
    void CAN_600_Tick(const TelemetryOut *t, uint32_t now_ms);

    /* push a new highest-severity error (will be sent immediately + next 1s slot) */
    void CAN_600_PushError(uint8_t severity, uint16_t error_code);

    /* optional: override IDs / versions used in 0x10000090 and 0x10000091 */
    void CAN_600_SetIdentity(uint32_t serial,
                             uint32_t fw_ver,
                             uint32_t part_no,
                             uint8_t  part_rev,
                             uint8_t  proto_ver,
                             uint8_t  batt_type);

#ifdef __cplusplus
}
#endif

#endif /* CAN_600_H_ */
