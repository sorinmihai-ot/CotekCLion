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
#include <math.h>

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
    // --- presence monitor ---
    QTimeEvt tick;        // 200 ms tick
    uint32_t alive_ms;    // ms since last valid reply
    // last known status (what we publish)
    uint8_t present;
    uint8_t out_on;
    float   v_out, i_out, t_out;
    /* --- startup policy --- */
    uint8_t startup_sync;   /* 1 = we are confirming OFF at boot */
    uint8_t off_acks;       /* consecutive reads showing output OFF */
} CotekAO;

static void publish_status(CotekAO *me) {
    CotekStatusEvt *se = Q_NEW(CotekStatusEvt, PSU_RSP_STATUS_SIG);
    se->present = me->present;
    se->out_on  = me->out_on;   /* 1=ON, 0=OFF to match your struct */
    se->v_out   = me->v_out;
    se->i_out   = me->i_out;
    se->t_out  = me->t_out;
    if (!QACTIVE_POST_X(AO_Controller, &se->super, 0U, &me->super)) {
        QF_gc(&se->super);
    }
}

//helper to send the Cotek data to the HMI screen
static void post_psu(const CotekAO *me,
                     uint8_t present, uint8_t output_on,
                     float v_out, float i_out, float temp_C)
{
    (void)me;
    NextionPsuEvt *pe = Q_NEW(NextionPsuEvt, NEX_REQ_UPDATE_PSU_SIG);
    pe->present   = present;
    pe->output_on = output_on;
    pe->v_out     = v_out;
    pe->i_out     = i_out;
    pe->temp_C    = temp_C;
    if (!QACTIVE_POST_X(AO_Nextion, &pe->super, 0U, &me->super)) {
        QF_gc(&pe->super);
    }
}


static QState Cotek_initial(CotekAO *me, void const *par);
static QState Cotek_active (CotekAO *me, QEvt const *e);

static CotekAO l_psu;
QActive *AO_Cotek = &l_psu.super;

void CotekAO_ctor(void) {
    QActive_ctor(&l_psu.super, Q_STATE_CAST(&Cotek_initial));
}
static uint8_t cotek_read_control(uint8_t *ctrl) {
    uint8_t reg = 0x7C;
    if (HAL_I2C_Master_Transmit(&hi2c1, COTEK_I2C_ADDR, &reg, 1, I2C_TIMEOUT_MS) != HAL_OK) {
        return 0U;
    }
    if (HAL_I2C_Master_Receive(&hi2c1, COTEK_I2C_ADDR, rx_data, 1, I2C_TIMEOUT_MS) != HAL_OK) {
        return 0U;
    }
    *ctrl = rx_data[0];
    return 1U;
}
static uint8_t i2c_read_u16(uint8_t reg, uint16_t *out) {
    if (HAL_I2C_Master_Transmit(&hi2c1, COTEK_I2C_ADDR, &reg, 1, I2C_TIMEOUT_MS) != HAL_OK) {
        return 0U;
    }
    if (HAL_I2C_Master_Receive(&hi2c1, COTEK_I2C_ADDR, rx_data, 2, I2C_TIMEOUT_MS) != HAL_OK) {
        return 0U;
    }
    *out = (uint16_t)((rx_data[1] << 8) | rx_data[0]);
    return 1U;
}
static uint8_t i2c_read_u8(uint8_t reg, uint8_t *out) {
    if (HAL_I2C_Master_Transmit(&hi2c1, COTEK_I2C_ADDR, &reg, 1, I2C_TIMEOUT_MS) != HAL_OK) {
        return 0U;
    }
    if (HAL_I2C_Master_Receive(&hi2c1, COTEK_I2C_ADDR, rx_data, 1, I2C_TIMEOUT_MS) != HAL_OK) {
        return 0U;
    }
    *out = rx_data[0];
    return 1U;
}

static QState Cotek_initial(CotekAO * const me, void const *par) {
    (void)par;
    cotek_set_remote_mode();
    cotek_power_off();
    me->on = 0U; me->vset = 0.f; me->iset = 0.f;
    QTimeEvt_ctorX(&me->tick, &me->super, COTEK_TICK_SIG, 0U);
    // start periodic every 500ms
    QTimeEvt_armX(&me->tick, 50U, 50U); // assuming 1 tick = 10 ms (adjust to your BSP tick)
    me->alive_ms = 1000U;   // start as stale
    me->present = 0U;
    me->out_on  = 0U;
    me->v_out = me->i_out = me->t_out = 0.0f;
    /* start in "startup sync": confirm real OFF before we publish “OFF” */
    me->startup_sync = 1U;
    me->off_acks     = 0U;
    return Q_TRAN(&Cotek_active);
}

static QState Cotek_active(CotekAO * const me, QEvt const * const e)
{
    switch (e->sig)
    {
            case COTEK_TICK_SIG: {
                uint16_t rawV = 0, rawI = 0;   uint8_t rawT = 0, ctrl = 0;
                uint8_t okV = i2c_read_u16(0x60, &rawV);          // V*100
                uint8_t okI = i2c_read_u16(0x62, &rawI);          // A*100
                uint8_t okT = i2c_read_u8 (0x68, &rawT);          // °C
                uint8_t okC = cotek_read_control(&ctrl);          // bit0=ON

                uint8_t okAny = (okV || okI || okT || okC);
                if (okAny) {
                    me->alive_ms = 0U;
                    if (okV) me->v_out = (float)rawV / 100.0f;
                    if (okI) me->i_out = (float)rawI / 100.0f;
                    if (okT) me->t_out = (float)rawT;
                    if (okC) me->out_on = ((ctrl & 0x01U) != 0U);  // bit0 = output enable
                } else {
                    if (me->alive_ms < 5000U) { me->alive_ms += 200U; } // 200 ms tick
                }
                uint8_t new_present = (me->alive_ms <= 1000U) ? 1U : 0U;
                me->present = new_present;

                static uint8_t last_present = 0xFFU, last_out_on = 0xFFU;
                static float   last_v = -999.0f, last_i = -999.0f, last_t = -999.0f;

                if (   (new_present != last_present)
                    || (me->out_on   != last_out_on)
                    || (fabsf(last_v - me->v_out) > 0.05f)
                    || (fabsf(last_i - me->i_out) > 0.05f)
                    || (fabsf(last_t - me->t_out) > 0.5f)) {

                    last_present = new_present;
                    last_out_on  = me->out_on;
                    last_v       = me->v_out;
                    last_i       = me->i_out;
                    last_t       = me->t_out;

                    post_psu(me, new_present, me->out_on, me->v_out, me->i_out, me->t_out);
                    publish_status(me);
    }
                    return Q_HANDLED();
            }
            case PSU_REQ_SETPOINT_SIG: {
                    // refuse if not present (prevents programming into a bus error)
                    if (me->present == 0U) {
                        printf("COTEK: IGNORE setpoint (PSU not present)\r\n");
                        return Q_HANDLED();
                    }
                    PsuSetEvt const *se = Q_EVT_CAST(PsuSetEvt);
                    me->vset = se->voltSet;
                    me->iset = se->currSet;
                    me->on = 1U;
                    /* Program the supply over I2C */
                    cotek_set_remote_mode();
                    cotek_set_output_voltage(me->vset);
                    cotek_set_output_current(me->iset);
                    cotek_commit_settings();
                    cotek_power_on();
                    me->out_on = 1U;


            printf("COTEK: ON V=%.2f I=%.2f\r\n", me->vset, me->iset);
                    /* Push an immediate UI update so pMain shows PSU group “live” */
                    post_psu(me,
                             /*present=*/1U,
                             /*output_on=*/me->out_on,
                             /*v_out=*/me->vset,   /* show setpoints until readback arrives */
                             /*i_out=*/0.0f,
                             /*temp_C=*/NAN);

                    printf("COTEK: ON V=%.2f I=%.2f\r\n", me->vset, me->iset);
                    return Q_HANDLED();
            }
            case PSU_REQ_OFF_SIG: {
                    me->on = 0U;
                    cotek_set_remote_mode();
                    cotek_power_off();   /* actively command OFF */
                    printf("COTEK: OFF\r\n");
                    me->startup_sync = 1U;
                    me->off_acks = 0U;

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

// simple health accessor for the controller
uint8_t Cotek_isPresent(void) {
    return l_psu.present;    // alive in the last ~1s
}
