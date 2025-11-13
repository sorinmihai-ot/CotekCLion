//
// Created by sorin.mihai on 31/10/2025.
//
#ifndef CAN_400DZB_H
#define CAN_400DZB_H

#include <stdint.h>
#include <stdbool.h>
#include "can_tx.h"

/*
 * 400DZB / 400s Dual-Zone BMS simulator
 *
 * Call CAN_400DZB_Tick() from your main periodic CAN task
 * (you already do this for CAN_400HYP_Tick).
 *
 * The tick self-throttles and sends the frames at:
 *  - 250 ms: 0x064..0x067 (main internal)
 *  - 250 ms: 0x074..0x077 (secondary internal, if mirror=true)
 *  - 1000 ms: external “slow” frames 0x1800.. for main (+secondary if mirror)
 *  - 400 ms: external status 0x18060800 / 0x18070800 / 0x18080800
 *  - 400 ms: secondary status 0x18080801 (if mirror=true)
 */

/**
     * @brief Init the 400s DZB CAN sender.
     * @param node_id low 8 bits used to form 0x18FF00XX-style node frames
     */
    void CAN_400DZB_Init(uint8_t node_id);
/**
 * Tick the 400DZB generator.
 *
 * @param t        pointer to global TelemetryOut (the only one we use!)
 * @param nowMs    system time in ms (the same fakeMs you use in can_tx.c)
 * @param mirrorToSecondary  true → also send the secondary pack (IDs ...01)
 */
void CAN_400DZB_Tick(const TelemetryOut *t,
                     uint32_t nowMs,
                     bool mirrorToSecondary);

#endif /* CAN_400DZB_H */
