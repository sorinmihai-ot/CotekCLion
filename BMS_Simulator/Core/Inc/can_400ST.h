#ifndef CAN_400ST_H_
#define CAN_400ST_H_

#include <stdint.h>
#include "can_tx.h"    // for TelemetryOut (you already have this type)

#ifdef __cplusplus
extern "C" {
#endif

 /* init – match the 400DZB / 400HYP style */
 void CAN_400ST_Init(uint8_t node_id);

 /* periodic tick – called from CAN_TX_PeriodicTask() */
 void CAN_400ST_Tick(const TelemetryOut *t, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* CAN_400ST_H_ */
