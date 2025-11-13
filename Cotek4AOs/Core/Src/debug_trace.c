//
// Created by sorin.mihai on 18/09/2025.
//
#include "debug_trace.h"
#include "qpc_cfg.h"

/* Provide storage for the externs */
volatile uint16_t g_lastSig = 0U;
volatile uint16_t g_lastPostPrio = 0U;
volatile uint8_t  g_lastTag = 0U;
