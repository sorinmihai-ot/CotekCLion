//
// Created by sorin.mihai on 08/10/2025.
//
#include "qpc.h"  // if you want to use QP timebase; else use HAL_GetTick()
#include "can_proto.h"

typedef struct {
    bool     present;
    uint8_t  version;
    uint8_t  serial;
    uint32_t last_seen_ms;
} bsut_state_t;

static bsut_state_t g_bsut = {0};

// consider BSUT “gone” if we don’t hear from it for this long
#define BSUT_TIMEOUT_MS  3000U
