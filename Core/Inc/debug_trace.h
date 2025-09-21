//
// Created by sorin.mihai on 18/09/2025.
//
#ifndef DEBUG_TRACE_H
#define DEBUG_TRACE_H

#include <stdint.h>
#include "qpc.h"

/* Global breadcrumbs so Q_onError() can print what was just posted */
extern volatile uint16_t g_lastSig;
extern volatile uint16_t g_lastPostPrio;

/* Call this right before QACTIVE_POST/_X when you want breadcrumbs */
#define DBG_POST(me_, e_) \
do { \
g_lastSig = (uint16_t)((e_)->sig); \
g_lastPostPrio = (uint16_t)((QActive *)(me_))->prio; \
} while (0)

#endif /* DEBUG_TRACE_H */
