//
// Created by sorin.mihai on 31/10/2025.
//
#ifndef CAN_500HYP_H
#define CAN_500HYP_H

#include <stdint.h>
#include <stdbool.h>
#include "can_tx.h"   // for TelemetryOut

#ifdef __cplusplus
extern "C" {
#endif

    /**
     * @brief Init the 500s Hyperdrive CAN sender.
     * @param node_id low 8 bits used to form 0x18FF00XX-style node frames
     */
    void CAN_400DZB_Init(uint8_t node_id);

    /**
     * @brief Periodic tick for 500s Hyperdrive sender.
     *
     * - call from CAN_TX_PeriodicTask()
     * - we internally handle 1s / 5s / 10s send intervals
     * - if isSlave == false → send “array/master” frames (…600, …700, …800, …1900, …1300, …0E00)
     * - if isSlave == true  → send “node/slave” frames (…03XX, …0CXX, …50XX, …E0XX, …F0XX, …0EXX, …40XX)
     */
    void CAN_500HYP_Tick(const TelemetryOut *t, uint32_t nowMs, bool isSlave);

#ifdef __cplusplus
}
#endif

#endif /* CAN_500HYP_H */
