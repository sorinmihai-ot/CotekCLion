// Core/Inc/bms_debug.h
#pragma once
#include <stdint.h>
#include <stdio.h>
#include "stm32f1xx_hal.h"   // for HAL_GetTick()

/* ---- debug toggle guarded to satisfy -Wundef ---- */
#if !defined(BMS_DEBUG)
#define BMS_DEBUG 0
#endif

#if BMS_DEBUG
  #define BMS_DBG(...)  printf(__VA_ARGS__)
#else
  #define BMS_DBG(...)
#endif

/* ---- time helpers / globals ---- */
static inline uint32_t tick_ms(void) { return HAL_GetTick(); }

/* Defined exactly once in bms_app.c, extern everywhere else */
extern volatile uint32_t last_bms_ms;

/* ---- common thresholds (used by controller/UI) ---- */
#ifndef BMS_WATCH_MS
#define BMS_WATCH_MS 1500U
#endif

#ifndef BMS_STALE_MS
#define BMS_STALE_MS 1400U
#endif
