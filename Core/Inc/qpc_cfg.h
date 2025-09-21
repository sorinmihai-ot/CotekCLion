//
// Created by sorin.mihai on 20/09/2025.
//
#ifndef QPC_CFG_H_
#define QPC_CFG_H_

// Make the QP port unambiguously use "aware == 5" on STM32F1 (4 prio bits)
#undef  QF_BASEPRI
#define QF_BASEPRI  0x50        // 0x50 >> (8-4) = 5

#undef  QF_AWARE_ISR_CMSIS_PRI  // belt & braces so we *know* it’s 5
#define QF_AWARE_ISR_CMSIS_PRI  5

#endif
