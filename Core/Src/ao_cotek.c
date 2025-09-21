#include "ao_cotek.h"
#include <stdio.h>
#include "qpc_cfg.h"
#include "qpc.h"
#include <string.h>
#include <stdio.h>
#include "bsp.h"
#include "can_app.h"
#include "app_signals.h"
#include "bms_app.h"
#include "ao_nextion.h"
#include "ao_controller.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_i2c.h"
#include "main.h"


Q_DEFINE_THIS_FILE

#define COTEK_I2C_ADDR ((0x50) << 1)  // STM32 expects 8-bit address (shifted left)
#define I2C_TIMEOUT_MS 100
static uint8_t tx_data[2];
static uint8_t rx_data[2];
extern volatile uint16_t g_lastSig;
extern volatile uint8_t  g_lastTag;

static void Scan_I2C_Bus(I2C_HandleTypeDef *hi2c);
/* local helper prototypes (file-local linkage) */
static void  cotek_set_remote_mode(void);
static void  cotek_set_output_voltage(float voltage);
static void  cotek_set_output_current(float current);
static void  cotek_commit_settings(void);
static void  cotek_power_on(void);
static void  cotek_power_off(void);
static float cotek_read_voltage(void);
static float cotek_read_current(void);
static float cotek_read_temperature(void);

typedef struct {
    QActive super;
    uint8_t on;
    float   vset, iset;
} CotekAO;

static QState Cotek_initial(CotekAO *me, void const *par);
static QState Cotek_active (CotekAO *me, QEvt const *e);

static CotekAO l_psu;
QActive *AO_Cotek = &l_psu.super;

void CotekAO_ctor(void) {
    QActive_ctor(&l_psu.super, Q_STATE_CAST(&Cotek_initial));
}

static QState Cotek_initial(CotekAO * const me, void const *par) {
    (void)par;
    cotek_set_remote_mode();
    cotek_power_off();
    me->on = 0U; me->vset = 0.f; me->iset = 0.f;
    return Q_TRAN(&Cotek_active);
}

static QState Cotek_active(CotekAO * const me, QEvt const * const e) {
    switch (e->sig) {
        case PSU_REQ_SETPOINT_SIG: {
            PsuSetEvt const *se = Q_EVT_CAST(PsuSetEvt);
            me->vset = se->voltSet; me->iset = se->currSet; me->on = 1U;
            printf("COTEK: ON V=%.2f I=%.2f\r\n", me->vset, me->iset);
            /* In a real driver you’d program the PSU and maybe start a poll timer.
               Here we just printf. */
            return Q_HANDLED();
        }
        case PSU_REQ_OFF_SIG: {
            me->on = 0U; printf("COTEK: OFF\r\n");
            return Q_HANDLED();
        }
    }
    return Q_SUPER(&QHsm_top);
}

void cotek_set_remote_mode(void) {
    // Write 0x80 to 0x7C (bit 7 = 1 ? Remote mode)
    uint8_t cmd[2] = {0x7C, 0x80};
    HAL_I2C_Master_Transmit(&hi2c1, COTEK_I2C_ADDR, cmd, 2, I2C_TIMEOUT_MS);
}

void cotek_set_output_voltage(float voltage) {
    // Voltage * 100 -> hex ? write to 0x70 (LSB), 0x71 (MSB)
    uint16_t val = (uint16_t)(voltage * 100); // e.g. 24.25 * 100 = 2425 = 0x979
    uint8_t cmd[3] = {0x70, val & 0xFF, (val >> 8)};
    HAL_I2C_Master_Transmit(&hi2c1, COTEK_I2C_ADDR, cmd, 3, I2C_TIMEOUT_MS);
}

void cotek_set_output_current(float current) {
    // Current * 100 -> hex ? write to 0x72 (LSB), 0x73 (MSB)
    uint16_t val = (uint16_t)(current * 100); // e.g. 45.75 * 100 = 4575 = 0x11DF
    uint8_t cmd[3] = {0x72, val & 0xFF, (val >> 8)};
    HAL_I2C_Master_Transmit(&hi2c1, COTEK_I2C_ADDR, cmd, 3, I2C_TIMEOUT_MS);
}

void cotek_commit_settings() {
    // Write 0x04 to 0x7C (bit 2 = 1 ? update settings)
    uint8_t cmd[2] = {0x7C, 0x84};  // Bit 7 still set for remote + bit 2 for update
    HAL_I2C_Master_Transmit(&hi2c1, COTEK_I2C_ADDR, cmd, 2, I2C_TIMEOUT_MS);
}

void cotek_power_on() {
    // Write 0x85 to 0x7C (bit 7 = 1 ? remote, bit 0 = 1 ? power ON)
    uint8_t cmd[2] = {0x7C, 0x85};  // Bit7 = Remote, Bit0 = Power ON
    HAL_I2C_Master_Transmit(&hi2c1, COTEK_I2C_ADDR, cmd, 2, I2C_TIMEOUT_MS);
}

void cotek_power_off(void) {
    // Remote mode bit set (bit7 = 1), Power bit cleared (bit0 = 0) -> 0x80
    uint8_t cmd[2] = {0x7C, 0x80};
    HAL_I2C_Master_Transmit(&hi2c1, COTEK_I2C_ADDR, cmd, 2, I2C_TIMEOUT_MS);
}

float cotek_read_voltage() {
    uint8_t reg = 0x60;
    HAL_I2C_Master_Transmit(&hi2c1, COTEK_I2C_ADDR, &reg, 1, I2C_TIMEOUT_MS);
    HAL_I2C_Master_Receive(&hi2c1, COTEK_I2C_ADDR, rx_data, 2, I2C_TIMEOUT_MS);
    uint16_t raw = rx_data[1] << 8 | rx_data[0];
    return raw / 100.0f;
}

float cotek_read_current() {
    uint8_t reg = 0x62;
    HAL_I2C_Master_Transmit(&hi2c1, COTEK_I2C_ADDR, &reg, 1, I2C_TIMEOUT_MS);
    HAL_I2C_Master_Receive(&hi2c1, COTEK_I2C_ADDR, rx_data, 2, I2C_TIMEOUT_MS);
    uint16_t raw = rx_data[1] << 8 | rx_data[0];
    return raw / 100.0f;
}

float cotek_read_temperature() {
    uint8_t reg = 0x68;
    HAL_I2C_Master_Transmit(&hi2c1, COTEK_I2C_ADDR, &reg, 1, I2C_TIMEOUT_MS);
    HAL_I2C_Master_Receive(&hi2c1, COTEK_I2C_ADDR, rx_data, 1, I2C_TIMEOUT_MS);
    return rx_data[0];
}
