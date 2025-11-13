//
// Created by sorin.mihai on 08/10/2025.
//

#pragma once
#include <stdint.h>

#define CAN_ID_BSUT_TO_BSU   0x1000004U  // BSUT → BSU
#define CAN_ID_BSU_TO_BSUT   0x1000005U  // BSU  → BSUT (already filtered)

typedef struct __attribute__((packed)) {
    uint8_t test_num;       // 0 == handshake
    uint8_t voltage_value;  // 1..255 -> 35–70V map, 0 -> off
    uint8_t check_bms;      // 0/1
    uint8_t r3;
    uint8_t r4;
    uint8_t r5;
    uint8_t serial_number;  // unique per BSUT
    uint8_t version;        // must be 1 for v1 firmware
} bsut_msg_t;
