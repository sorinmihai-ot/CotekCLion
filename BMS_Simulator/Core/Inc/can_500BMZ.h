//
// Created by sorin.mihai on 31/10/2025.
//
#ifndef CAN_500BMZ_H
#define CAN_500BMZ_H

#include <stdint.h>
#include <stdbool.h>
#include "can_tx.h"   // for TelemetryOut

#ifdef __cplusplus
extern "C" {
#endif

    /**
     * @brief Init the 500s BMZ CAN sender.
     * @param node_id  logical/node id (kept for future multi-pack support)
     */
    void CAN_500BMZ_Init(uint8_t node_id);

    /**
     * @brief Periodic tick for 500s BMZ sender.
     *
     * Follows the same pattern as CAN_400DZB_Tick / CAN_400ST_Tick:
     *  - call from CAN_TX_PeriodicTask()
     *  - we internally rate-limit frames (20 Hz / 1 Hz / 0.1 Hz)
     *
     * @param t       pointer to current telemetry set (hi/lo cell, pack V, temps, SoC, last_err)
     * @param nowMs   monotonic ms from the simulator (the same fakeMs you use elsewhere)
     * @param isSlave if true we could later adjust IDs for a secondary pack (ignored for now)
     */
    void CAN_500BMZ_Tick(const TelemetryOut *t, uint32_t nowMs, bool isSlave);

#ifdef __cplusplus
}
#endif

#endif /* CAN_500BMZ_H */
