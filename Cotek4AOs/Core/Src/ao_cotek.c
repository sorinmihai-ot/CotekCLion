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

// We already use COTEK_TICK_SIG. Keep its period at 500ms (50 ticks @ 10ms tick).
#define COTEK_TICK_ARM_FIRST   50U
#define COTEK_TICK_ARM_PERIOD  50U

// ~3 seconds using 500ms ticks => 6 periods
#define COTEK_BOOT_TICKS_3S    6U

#define COTEK_LOG_EN 1

#if COTEK_LOG_EN
  #define COTEK_LOG(fmt, ...)  do { \
  printf("COTEK_DBG: " fmt "\r\n", ##__VA_ARGS__); \
  } while (0)
#else
  #define COTEK_LOG(fmt, ...)  ((void)0)
#endif

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
static uint8_t cotek_read_control(uint8_t *ctrl);
static uint8_t i2c_read_u16(uint8_t reg, uint16_t *out);
static uint8_t i2c_read_u8(uint8_t reg, uint8_t *out);
//  helpers for the breadcrumbs
static void cotek_dump_bytes(char const *tag, uint8_t const *p, uint16_t n) {
    printf("COTEK_%s [%u]: ", tag, (unsigned)n);
    for (uint16_t i = 0; i < n; i++) printf("%02X ", p[i]);
    printf("\r\n");
}
static void cotek_log_i2c_result(char const *what, HAL_StatusTypeDef st) {
    if (st == HAL_OK) {
        printf("COTEK_I2C OK: %s\r\n", what);
    } else {
        uint32_t err = HAL_I2C_GetError(&hi2c1);
        printf("COTEK_I2C FAIL: %s st=%d err=0x%08lX\r\n",
               what, (int)st, (unsigned long)err);
    }
}

typedef struct {
    QActive super;
    uint8_t on;
    float   vset, iset;
    // --- presence monitor ---
    QTimeEvt tick;        // 200 ms tick
    uint8_t force_pub_ticks;   // number of tick to publish status regardless if changed or not
    uint32_t alive_ms;    // ms since last valid reply
    // last known status (what we publish)
    uint8_t present;
    uint8_t out_on;
    float   v_out, i_out, t_out;
    /* --- startup policy --- */
    uint8_t startup_sync;   /* 1 = we are confirming OFF at boot */
    uint8_t off_acks;       /* consecutive reads showing output OFF */
    // --- NEW: boot gating / buffering ---
    uint8_t boot_ticks_left;     // used in Cotek_Power_up_delay
    uint8_t pending_setpoint;    // 1 if we received PSU_REQ_SETPOINT while not ready
    float   pend_vset, pend_iset;
    // --- NEW: handshake with Controller ---
    uint8_t sync_requested;     // 1 after PSU_REQ_SYNC_SIG received
    uint8_t ready_sent;         // 1 after PSU_RSP_READY_SIG published (once per sync)
    uint8_t last_present;       // for edge detect present 0->1
} CotekAO;

static CotekAO l_psu;
QActive *AO_Cotek = &l_psu.super;

/* ===== Forward declarations of states ===== */
static QState Cotek_qp_initial (CotekAO *me, void const *par);
static QState Cotek_off        (CotekAO *me, QEvt const *e);
static QState Cotek_Power_up_delay         (CotekAO *me, QEvt const *e);
static QState Cotek_initial(CotekAO *me, QEvt const *e);
static QState Cotek_active (CotekAO *me, QEvt const *e);

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

/* small helper so OFF state can “announce” PSU is gone */
static void psu_mark_offline(CotekAO *me, char const *why) {
    me->present = 0U;
    me->out_on  = 0U;
    me->v_out   = 0.0f;
    me->i_out   = 0.0f;
    me->t_out   = 0.0f;
    me->alive_ms = 5000U;

    if (why) {
        printf("COTEK: %s\r\n", why);
    }
    post_psu(me, 0U, 0U, 0.0f, 0.0f, NAN);
    publish_status(me);
}

void CotekAO_ctor(void) {
    QActive_ctor(&l_psu.super, Q_STATE_CAST(&Cotek_qp_initial));
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
    *out = 0;
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
static void force_publish_for(CotekAO *me, uint8_t ticks) {
    if (ticks > me->force_pub_ticks) {
        me->force_pub_ticks = ticks;
    }
}
static void apply_pending_setpoint(CotekAO *me) {
    if ((me->present != 0U) && (me->pending_setpoint != 0U)) {
        me->pending_setpoint = 0U;

        me->vset = me->pend_vset;
        me->iset = me->pend_iset;
        me->on   = 1U;

        cotek_set_remote_mode();
        cotek_set_output_voltage(me->vset);
        cotek_set_output_current(me->iset);
        cotek_commit_settings();
        cotek_power_on();
        me->out_on = 1U;

        printf("COTEK: APPLY PENDING -> ON V=%.2f I=%.2f\r\n",
               (double)me->vset, (double)me->iset);

        post_psu(me,
                 /*present=*/1U,
                 /*output_on=*/1U,
                 /*v_out=*/me->vset,
                 /*i_out=*/me->iset,
                 /*temp_C=*/NAN);
        publish_status(me);
    }
}
float Cotek_getVout_V(void) {
    return l_psu.v_out;
}

/* ===== QP initial pseudo-state ===== */
static QState Cotek_qp_initial(CotekAO * const me, void const *par) {
    (void)par;
    me->force_pub_ticks = 0U;

    me->on = 0U;
    me->vset = 0.f;
    me->iset = 0.f;

    me->alive_ms = 5000U;
    me->present = 0U;
    me->out_on  = 0U;
    me->v_out = me->i_out = me->t_out = 0.0f;

    me->startup_sync = 0U;
    me->off_acks = 0U;

    me->boot_ticks_left = 0U;
    me->pending_setpoint = 0U;
    me->pend_vset = 0.0f;
    me->pend_iset = 0.0f;

    me->sync_requested = 0U;
    me->ready_sent     = 0U;
    me->last_present   = 0U;


    QTimeEvt_ctorX(&me->tick, &me->super, COTEK_TICK_SIG, 0U);

    /* Cotek AO needs to see Start/Stop too (posted or published) */
    QActive_subscribe(&me->super, LATCH_TURNED_ON_SIG);
    QActive_subscribe(&me->super, LATCH_TURNED_OFF_SIG);
    QActive_subscribe(&me->super, STOPBUTTON_PRESSED_SIG);

    return Q_TRAN(&Cotek_off);
}

/* ===== COTEK_OFF: AC is OFF, do not touch I2C ===== */
static QState Cotek_off(CotekAO * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            /* stop any periodic activity */
            QTimeEvt_disarm(&me->tick);

            /* clear logical state */
            me->on = 0U;
            me->vset = 0.f;
            me->iset = 0.f;

            /* clear “pending setpoint” */
            me->pending_setpoint = 0U;
            me->boot_ticks_left = 0U;

            psu_mark_offline(me, "state=OFF (AC disabled)");
            return Q_HANDLED();
        }
        case PSU_REQ_SYNC_SIG:
        case STOPBUTTON_PRESSED_SIG: {
            /* already off */
            return Q_HANDLED();
        }
        case LATCH_TURNED_ON_SIG: {
            printf("COTEK: LATCH ON -> state=Power_up_delay\r\n");
            return Q_TRAN(&Cotek_Power_up_delay);
        }
        case PSU_REQ_SETPOINT_SIG: {
            /* Controller might send setpoint immediately; buffer it */
            PsuSetEvt const *se = Q_EVT_CAST(PsuSetEvt);
            me->pending_setpoint = 1U;
            me->pend_vset = se->voltSet;
            me->pend_iset = se->currSet;
            printf("COTEK: buffered setpoint V=%.2f I=%.2f (PSU OFF)\r\n",
                   (double)me->pend_vset, (double)me->pend_iset);
            return Q_HANDLED();
        }
        case PSU_REQ_OFF_SIG:
        case LATCH_TURNED_OFF_SIG: {
            return Q_HANDLED();
        }
        default: break;
    }
    return Q_SUPER(&QHsm_top);
}

/* ===== Cotek_Power_up_delay: wait ~3s for Cotek to boot after AC enabled ===== */
static QState Cotek_Power_up_delay(CotekAO * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            /* 3s boot timer using the 500ms tick */
            me->boot_ticks_left = COTEK_BOOT_TICKS_3S;

            /* ensure we publish “not present yet” during boot */
            psu_mark_offline(me, "state=UP delay(waiting for Cotek boot)");
            force_publish_for(me, COTEK_BOOT_TICKS_3S + 2U);
            /* start tick for countdown */
            QTimeEvt_armX(&me->tick, COTEK_TICK_ARM_FIRST, COTEK_TICK_ARM_PERIOD);
            return Q_HANDLED();
        }
        case STOPBUTTON_PRESSED_SIG: {
            printf("COTEK: Stop pressed -> state=OFF\r\n");
            return Q_TRAN(&Cotek_off);
        }
        case LATCH_TURNED_OFF_SIG: {
            printf("COTEK: Latch circuit open -> state=OFF\r\n");
            return Q_TRAN(&Cotek_off);
        }
        case PSU_REQ_SETPOINT_SIG: {
            /* buffer while booting */
            PsuSetEvt const *se = Q_EVT_CAST(PsuSetEvt);
            me->pending_setpoint = 1U;
            me->pend_vset = se->voltSet;
            me->pend_iset = se->currSet;
            printf("COTEK: buffered setpoint V=%.2f I=%.2f (booting)\r\n",
                   (double)me->pend_vset, (double)me->pend_iset);
            COTEK_LOG("RX PSU_REQ_SETPOINT_SIG: V=%.2f I=%.2f (present=%u pending=%u boot_left=%u)",
              (double)se->voltSet, (double)se->currSet,
              (unsigned)me->present, (unsigned)me->pending_setpoint,
              (unsigned)me->boot_ticks_left);
            return Q_HANDLED();
        }
        case COTEK_TICK_SIG: {
            if (me->boot_ticks_left > 0U) {
                printf("COTEK: tick received %d remaining\r\n", me->boot_ticks_left);
                --me->boot_ticks_left;
            }
            publish_status(me);
            if (me->boot_ticks_left == 0U) {
                printf("COTEK: boot wait done -> state=Initial\r\n");
                return Q_TRAN(&Cotek_initial);
            }
            publish_status(me);
            return Q_HANDLED();
        }
        default: break;
    }
    return Q_SUPER(&QHsm_top);
}

/* ===== COTEK_Initial: now it should be alive; configure + start polling ===== */
static QState Cotek_initial(CotekAO * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            printf("COTEK: state=Initial (config + start polling)\r\n");
            HAL_I2C_DeInit(&hi2c1);
            HAL_I2C_Init(&hi2c1);
            /* Now AC should be on: safe to touch I2C */
            cotek_set_remote_mode();


            /* Try a simple read to confirm comms */
            uint8_t ctrl = 0U;
            uint8_t okC  = cotek_read_control(&ctrl);

            if (okC) {
                me->present = 1U;
                me->out_on  = ((ctrl & 0x01U) != 0U);

                /* publish a status snapshot */
                force_publish_for(me, 6U);
                publish_status(me);

                /* Notify Controller that PSU is ready */
                static QEvt const psuReadyEvt = QEVT_INITIALIZER(PSU_READY_SIG);
                if (!QACTIVE_POST_X(AO_Controller, &psuReadyEvt, QF_NO_MARGIN, &me->super)) {
                    /* if post fails, we still continue; controller watchdog will handle it */
                }
                printf("COTEK: PSU_READY_SIG posted to Controller\r\n");

                /* Start active polling */
                QTimeEvt_disarm(&me->tick);
                QTimeEvt_armX(&me->tick, COTEK_TICK_ARM_FIRST, COTEK_TICK_ARM_PERIOD);

                return Q_TRAN(&Cotek_active);
            }
            /* Not responding yet: mark offline but keep polling a bit */
            psu_mark_offline(me, "COTEK: Initial - not responding yet");
            force_publish_for(me, 6U);

            QTimeEvt_disarm(&me->tick);
            QTimeEvt_armX(&me->tick, COTEK_TICK_ARM_FIRST, COTEK_TICK_ARM_PERIOD);

            /* Stay in initial; tick will retry */
            return Q_HANDLED();
        }
        case COTEK_TICK_SIG: {
            /* Retry comms until it responds */
            uint8_t ctrl = 0U;
            uint8_t okC  = cotek_read_control(&ctrl);

            if (okC) {
                me->present = 1U;
                me->out_on  = ((ctrl & 0x01U) != 0U);

                publish_status(me);

                static QEvt const psuReadyEvt = QEVT_INITIALIZER(PSU_READY_SIG);
                (void)QACTIVE_POST_X(AO_Controller, &psuReadyEvt, QF_NO_MARGIN, &me->super);

                printf("COTEK: PSU responded on retry -> PSU_READY sent -> Active\r\n");
                return Q_TRAN(&Cotek_active);
            }

            publish_status(me);
            return Q_HANDLED();
        }
        case LATCH_TURNED_OFF_SIG:
        case STOPBUTTON_PRESSED_SIG: {
            printf("COTEK: Initial aborted -> OFF\r\n");
            return Q_TRAN(&Cotek_off);
        }
        case PSU_REQ_SETPOINT_SIG: {
            PsuSetEvt const *se = Q_EVT_CAST(PsuSetEvt);
            me->pending_setpoint = 1U;
            me->pend_vset = se->voltSet;
            me->pend_iset = se->currSet;
            printf("COTEK: buffered setpoint V=%.2f I=%.2f (in Initial)\r\n",
                   (double)me->pend_vset, (double)me->pend_iset);
            return Q_HANDLED();
        }
        default: break;
    }
    return Q_SUPER(&QHsm_top);
}

/* ===== COTEK_Active: your existing behaviour + Stop->OFF ===== */
static QState Cotek_active(CotekAO * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            cotek_set_remote_mode();
            /* If controller already asked for a setpoint, apply immediately */
            apply_pending_setpoint(me);
            printf("Cotek: active, entry state, setting the output on\r\n");
            cotek_power_on();
            printf("Cotek: active, entry state,the output is on\r\n");
            return Q_HANDLED();
        }
        case STOPBUTTON_PRESSED_SIG: {
            /* Safety: request OFF then go to OFF (AC will drop externally) */
            printf("COTEK: Stop -> OFF\r\n");
            return Q_TRAN(&Cotek_off);
        }
        case LATCH_TURNED_OFF_SIG: {
            // AC/latch dropped -> don’t talk I2C anymore, go OFF
            printf("COTEK: Latch OFF -> state=OFF\r\n");
            return Q_TRAN(&Cotek_off);
        }
        case COTEK_TICK_SIG: {
            uint16_t rawV = 0, rawI = 0;
            uint8_t  rawT = 0, ctrl = 0;

            uint8_t okV = i2c_read_u16(0x60, &rawV);          // V*100
            uint8_t okI = i2c_read_u16(0x62, &rawI);          // A*100
            uint8_t okT = i2c_read_u8 (0x68, &rawT);          // °C
            uint8_t okC = cotek_read_control(&ctrl);          // bit0=ON

            uint8_t okAny = (okV || okI || okT || okC);
            printf("COTEK: tick okV=%u okI=%u okT=%u okC=%u rawV=%u rawI=%u ctrl=0x%02X\r\n",
                okV, okI, okT, okC, rawV, rawI, ctrl);
            if (okAny) {
                me->alive_ms = 0U;
                if (okV) me->v_out = (float)rawV / 100.0f;
                if (okI) me->i_out = (float)rawI / 100.0f;
                if (okT) me->t_out = (float)rawT;
                if (okC) me->out_on = ((ctrl & 0x01U) != 0U);
                // If we have any comms at all, try applying pending setpoint now.
                if (me->pending_setpoint) {
                    printf("COTEK: comms OK -> applying pending setpoint now\r\n");
                    apply_pending_setpoint(me);
                    force_publish_for(me, 6U);
                }
            } else {
                if (me->alive_ms < 5000U) { me->alive_ms += 500U; } // 500ms tick now
            }

            uint8_t new_present = (me->alive_ms <= 1000U) ? 1U : 0U;
            me->present = new_present;

            if ((me->last_present == 0U) && (new_present != 0U)) {
                printf("COTEK: present 0->1, applying pending if any\r\n");
                apply_pending_setpoint(me);
            }
            me->last_present = new_present;

            static uint8_t last_present = 0xFFU, last_out_on = 0xFFU;
            static float   last_v = -999.0f, last_i = -999.0f, last_t = -999.0f;

            bool changed =
                   (new_present != last_present)
                || (me->out_on   != last_out_on)
                || (fabsf(last_v - me->v_out) > 0.05f)
                || (fabsf(last_i - me->i_out) > 0.05f)
                || (fabsf(last_t - me->t_out) > 0.5f);

            if (changed || (me->force_pub_ticks > 0U)) {

                // update last_* only when changed (optional, but keeps your de-jitter logic meaningful)
                if (changed) {
                    last_present = new_present;
                    last_out_on  = me->out_on;
                    last_v       = me->v_out;
                    last_i       = me->i_out;
                    last_t       = me->t_out;
                }

                post_psu(me, new_present, me->out_on, me->v_out, me->i_out, me->t_out);
                publish_status(me);

                if (me->force_pub_ticks > 0U) {
                    --me->force_pub_ticks;
                }
            }
            return Q_HANDLED();
        }
        case PSU_REQ_SETPOINT_SIG: {
            if (me->present == 0U) {
                /* buffer if supply disappears */
                PsuSetEvt const *se = Q_EVT_CAST(PsuSetEvt);
                me->pending_setpoint = 1U;
                me->pend_vset = se->voltSet;
                me->pend_iset = se->currSet;
                printf("COTEK: buffered setpoint V=%.2f I=%.2f (not present)\r\n",
                       (double)me->pend_vset, (double)me->pend_iset);
                COTEK_LOG("RX PSU_REQ_SETPOINT_SIG: V=%.2f I=%.2f (present=%u pending=%u boot_left=%u)",
                  (double)se->voltSet, (double)se->currSet,
                  (unsigned)me->present, (unsigned)me->pending_setpoint,
                  (unsigned)me->boot_ticks_left);
                return Q_HANDLED();
            }

            PsuSetEvt const *se = Q_EVT_CAST(PsuSetEvt);
            me->vset = se->voltSet;
            me->iset = se->currSet;
            me->on   = 1U;

            cotek_set_remote_mode();
            cotek_set_output_voltage(me->vset);
            cotek_set_output_current(me->iset);
            cotek_commit_settings();
            cotek_power_on();

            /* optimistic UI immediately; polling will correct it */
            me->out_on = 1U;

            printf("COTEK: SETPOINT applied -> ON V=%.2f I=%.2f\r\n",
                   (double)me->vset, (double)me->iset);

            post_psu(me, 1U, 1U, me->vset, 0.0f, NAN);
            publish_status(me);
            force_publish_for(me, 6U);

            return Q_HANDLED();
        }
        case PSU_REQ_OFF_SIG: {
            me->on = 0U;
            cotek_set_remote_mode();
            cotek_power_off();
            printf("COTEK: OFF\r\n");
            force_publish_for(me, 10U); // publish for next 10 ticks (~5 seconds at 500ms)
            me->startup_sync = 1U;
            me->off_acks = 0U;

            /* Don’t force state OFF here; controller might want PSU comms alive but output off.
               If you DO want AC removed on OFF request, then return Q_TRAN(&Cotek_off); */
            return Q_HANDLED();
        }
        case Q_EXIT_SIG: {
            cotek_power_off();
            me->sync_requested = 0U;
            me->ready_sent     = 0U;
            me->last_present   = 0U;
            printf("COTEK: Cotek_active - exit\r\n");
            return Q_HANDLED();
        }
        default: break;
    }
    return Q_SUPER(&QHsm_top);
}

// static QState Cotek_initial(CotekAO * const me, void const *par) {
//     (void)par;
//     cotek_set_remote_mode();
//     cotek_power_off();
//     me->on = 0U; me->vset = 0.f; me->iset = 0.f;
//     QTimeEvt_ctorX(&me->tick, &me->super, COTEK_TICK_SIG, 0U);
//     // start periodic every 500ms
//     QTimeEvt_armX(&me->tick, 50U, 50U); // assuming 1 tick = 10 ms (adjust to your BSP tick)
//     me->alive_ms = 1000U;   // start as stale
//     me->present = 0U;
//     me->out_on  = 0U;
//     me->v_out = me->i_out = me->t_out = 0.0f;
//     /* start in "startup sync": confirm real OFF before we publish “OFF” */
//     me->startup_sync = 1U;
//     me->off_acks     = 0U;
//     return Q_TRAN(&Cotek_active);
// }
//
// static QState Cotek_active(CotekAO * const me, QEvt const * const e)
// {
//     switch (e->sig)
//     {
//             case COTEK_TICK_SIG: {
//                 uint16_t rawV = 0, rawI = 0;   uint8_t rawT = 0, ctrl = 0;
//                 uint8_t okV = i2c_read_u16(0x60, &rawV);          // V*100
//                 uint8_t okI = i2c_read_u16(0x62, &rawI);          // A*100
//                 uint8_t okT = i2c_read_u8 (0x68, &rawT);          // °C
//                 uint8_t okC = cotek_read_control(&ctrl);          // bit0=ON
//
//                 uint8_t okAny = (okV || okI || okT || okC);
//                 if (okAny) {
//                     me->alive_ms = 0U;
//                     if (okV) me->v_out = (float)rawV / 100.0f;
//                     if (okI) me->i_out = (float)rawI / 100.0f;
//                     if (okT) me->t_out = (float)rawT;
//                     if (okC) me->out_on = ((ctrl & 0x01U) != 0U);  // bit0 = output enable
//                 } else {
//                     if (me->alive_ms < 5000U) { me->alive_ms += 200U; } // 200 ms tick
//                 }
//                 uint8_t new_present = (me->alive_ms <= 1000U) ? 1U : 0U;
//                 me->present = new_present;
//
//                 static uint8_t last_present = 0xFFU, last_out_on = 0xFFU;
//                 static float   last_v = -999.0f, last_i = -999.0f, last_t = -999.0f;
//
//                 if (   (new_present != last_present)
//                     || (me->out_on   != last_out_on)
//                     || (fabsf(last_v - me->v_out) > 0.05f)
//                     || (fabsf(last_i - me->i_out) > 0.05f)
//                     || (fabsf(last_t - me->t_out) > 0.5f)) {
//
//                     last_present = new_present;
//                     last_out_on  = me->out_on;
//                     last_v       = me->v_out;
//                     last_i       = me->i_out;
//                     last_t       = me->t_out;
//
//                     post_psu(me, new_present, me->out_on, me->v_out, me->i_out, me->t_out);
//                     publish_status(me);
//     }
//                     return Q_HANDLED();
//             }
//             case PSU_REQ_SETPOINT_SIG: {
//                     // refuse if not present (prevents programming into a bus error)
//                     if (me->present == 0U) {
//                         printf("COTEK: IGNORE setpoint (PSU not present)\r\n");
//                         return Q_HANDLED();
//                     }
//                     PsuSetEvt const *se = Q_EVT_CAST(PsuSetEvt);
//                     me->vset = se->voltSet;
//                     me->iset = se->currSet;
//                     me->on = 1U;
//                     /* Program the supply over I2C */
//                     cotek_set_remote_mode();
//                     cotek_set_output_voltage(me->vset);
//                     cotek_set_output_current(me->iset);
//                     cotek_commit_settings();
//                     cotek_power_on();
//                     me->out_on = 1U;
//
//
//             printf("COTEK: ON V=%.2f I=%.2f\r\n", me->vset, me->iset);
//                     /* Push an immediate UI update so pMain shows PSU group “live” */
//                     post_psu(me,
//                              /*present=*/1U,
//                              /*output_on=*/me->out_on,
//                              /*v_out=*/me->vset,   /* show setpoints until readback arrives */
//                              /*i_out=*/0.0f,
//                              /*temp_C=*/NAN);
//
//                     printf("COTEK: ON V=%.2f I=%.2f\r\n", me->vset, me->iset);
//                     return Q_HANDLED();
//             }
//             case PSU_REQ_OFF_SIG: {
//                     me->on = 0U;
//                     cotek_set_remote_mode();
//                     cotek_power_off();   /* actively command OFF */
//                     printf("COTEK: OFF\r\n");
//                     me->startup_sync = 1U;
//                     me->off_acks = 0U;
//
//                     return Q_HANDLED();
//             }
//     }
//
//         return Q_SUPER(&QHsm_top);
//     }
//

static void cotek_set_remote_mode(void) {
    // Write 0x80 to 0x7C (bit 7 = 1 ? Remote mode)
    uint8_t cmd[2] = {0x7C, 0x80};
    cotek_dump_bytes("TX", cmd, 2);
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&hi2c1, COTEK_I2C_ADDR, cmd, 2, I2C_TIMEOUT_MS);
    cotek_log_i2c_result("set_remote_mode reg=0x7C val=0x80", st);
}
static uint16_t clamp_u16(int v) {
    if (v < 0) return 0;
    if (v > 65535) return 65535;
    return (uint16_t)v;
}
static void cotek_set_output_voltage(float voltage) {
    int scaled = (int)lrintf(voltage * 100.0f);   // rounds properly
    uint16_t val = clamp_u16(scaled);

    printf("COTEK: Vset=%.2f -> val=%u (0x%04X)\r\n", (double)voltage, (unsigned)val, (unsigned)val);

    uint8_t cmd[3] = {0x70, (uint8_t)(val & 0xFF), (uint8_t)(val >> 8)};
    cotek_dump_bytes("TX", cmd, 3);
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&hi2c1, COTEK_I2C_ADDR, cmd, 3, I2C_TIMEOUT_MS);
    cotek_log_i2c_result("set_voltage reg=0x70/71", st);
}
static void cotek_set_output_current(float current) {
    int scaled = (int)lrintf(current * 100.0f);
    uint16_t val = clamp_u16(scaled);

    printf("COTEK: Iset=%.2f -> val=%u (0x%04X)\r\n", (double)current, (unsigned)val, (unsigned)val);

    uint8_t cmd[3] = {0x72, (uint8_t)(val & 0xFF), (uint8_t)(val >> 8)};
    cotek_dump_bytes("TX", cmd, 3);
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&hi2c1, COTEK_I2C_ADDR, cmd, 3, I2C_TIMEOUT_MS);
    cotek_log_i2c_result("set_current reg=0x72/73", st);
}
static void cotek_commit_settings(void) {
    uint8_t cmd[2] = {0x7C, 0x84};
    cotek_dump_bytes("TX", cmd, 2);
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&hi2c1, COTEK_I2C_ADDR, cmd, 2, I2C_TIMEOUT_MS);
    cotek_log_i2c_result("commit reg=0x7C val=0x84", st);
}
static void cotek_power_on(void) {
    uint8_t cmd[2] = {0x7C, 0x85};
    cotek_dump_bytes("TX", cmd, 2);
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&hi2c1, COTEK_I2C_ADDR, cmd, 2, I2C_TIMEOUT_MS);
    cotek_log_i2c_result("power_on reg=0x7C val=0x85", st);
}
static void cotek_power_off(void) {
    uint8_t cmd[2] = {0x7C, 0x80};
    cotek_dump_bytes("TX", cmd, 2);
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&hi2c1, COTEK_I2C_ADDR, cmd, 2, I2C_TIMEOUT_MS);
    cotek_log_i2c_result("power_off reg=0x7C val=0x80", st);
}
static float cotek_read_voltage() {
    uint8_t reg = 0x60;
    HAL_I2C_Master_Transmit(&hi2c1, COTEK_I2C_ADDR, &reg, 1, I2C_TIMEOUT_MS);
    HAL_I2C_Master_Receive(&hi2c1, COTEK_I2C_ADDR, rx_data, 2, I2C_TIMEOUT_MS);
    uint16_t raw = rx_data[1] << 8 | rx_data[0];
    return raw / 100.0f;
}
static float cotek_read_current() {
    uint8_t reg = 0x62;
    HAL_I2C_Master_Transmit(&hi2c1, COTEK_I2C_ADDR, &reg, 1, I2C_TIMEOUT_MS);
    HAL_I2C_Master_Receive(&hi2c1, COTEK_I2C_ADDR, rx_data, 2, I2C_TIMEOUT_MS);
    uint16_t raw = rx_data[1] << 8 | rx_data[0];
    return raw / 100.0f;
}
static float cotek_read_temperature() {
    uint8_t reg = 0x68;
    HAL_I2C_Master_Transmit(&hi2c1, COTEK_I2C_ADDR, &reg, 1, I2C_TIMEOUT_MS);
    HAL_I2C_Master_Receive(&hi2c1, COTEK_I2C_ADDR, rx_data, 1, I2C_TIMEOUT_MS);
    return rx_data[0];
}
// simple health accessor for the controller
uint8_t Cotek_isPresent(void) {
    return l_psu.present;    // alive in the last ~1s
}
