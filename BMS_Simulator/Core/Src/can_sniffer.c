/* can_sniffer.c  — TEMPORARY CAN bus logger with microsecond timestamps
 *
 * What it does:
 *  - Accepts ALL incoming CAN frames (STD/EXT/RTR) into FIFO0.
 *  - Pushes them into an ISR-safe ring buffer.
 *  - Prints them from the main loop (pretty or CSV).
 *  - Stamps each frame with HAL_GetTick() (ms) and microseconds from a
 *    user-provided free-running timer (TIMx), or DWT->CYCCNT fallback.
 *
 * Public API (declare these in can_sniffer.h if you like):
 *  -------------------------------------------------------------------
 *  void CanSniffer_AttachMicroTimer(TIM_HandleTypeDef *htim, uint32_t timer_hz);
 *  void CanSniffer_Start(void);     // config accept-all filter + start + IRQs
 *  void CanSniffer_Task(void);      // drain ring & print lines
 *  void CanSniffer_SetCsvMode(bool on);  // optional: CSV output
 *  void CanSniffer_PrintHeader(void);    // optional: CSV headers
 *  -------------------------------------------------------------------
 *
 * Typical main.c usage:
 *  -------------------------------------------------------------------
 *  extern void CanSniffer_AttachMicroTimer(TIM_HandleTypeDef*, uint32_t);
 *  extern void CanSniffer_Start(void);
 *  extern void CanSniffer_Task(void);
 *  extern void CanSniffer_SetCsvMode(bool on);
 *  extern void CanSniffer_PrintHeader(void);
 *
 *  int main(void) {
 *      HAL_Init();
 *      SystemClock_Config();
 *      MX_GPIO_Init();
 *      MX_USART3_UART_Init();   // whichever UART your printf uses
 *      MX_CAN_Init();
 *      MX_TIM2_Init();          // configure TIM2 as 1 MHz free-running
 *      HAL_TIM_Base_Start(&htim2);
 *
 *      // Attach micro-timer (TIM2 @ 1,000,000 Hz):
 *      CanSniffer_AttachMicroTimer(&htim2, 1000000u);
 *
 *      // (optional) CSV mode with header:
 *      // CanSniffer_SetCsvMode(true);
 *      // CanSniffer_PrintHeader();
 *
 *      CanSniffer_Start();
 *      while (1) {
 *          CanSniffer_Task();   // prints lines; call as often as you like
 *      }
 *  }
 *
 * TIM2 @ 1 MHz example (CubeMX-like init):
 *  -------------------------------------------------------------------
 *  void MX_TIM2_Init(void) {
 *      htim2.Instance = TIM2;
 *      htim2.Init.Prescaler = (SystemCoreClock / 1000000u) - 1u; // 1 µs tick
 *      htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
 *      htim2.Init.Period = 0xFFFFFFFFu;     // free-running 32-bit
 *      htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
 *      htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
 *      HAL_TIM_Base_Init(&htim2);
 *  }
 *
 * Revert later by removing:
 *  - CanSniffer_AttachMicroTimer/Start/Task calls
 *  - this file from the build.
 */

/* can_sniffer.c — STM32 CAN logger with microsecond timestamps (L4-fixed) */
#include "main.h"
#include "can_sniffer.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

static UART_HandleTypeDef *s_uart = NULL;
static bool s_csv = true;

void CanSniffer_SetUart(UART_HandleTypeDef *huart) { s_uart = huart; }
void CanSniffer_SetCsvMode(bool enable)            { s_csv = enable; }

static void uart_write(const char *s)
{
    if (!s_uart || !s) return;
    HAL_UART_Transmit(s_uart, (uint8_t*)s, (uint16_t)strlen(s), 100);
}

void CanSniffer_PrintHeader(void)
{
    if (s_csv) {
        uart_write("timestamp_us,can_id,ide,dlc,data_hex\n");
    } else {
        uart_write("[CAN-SNIFFER] Ready.\r\n");
    }
}


/* ---------- HAL family includes (pick your device automatically) ---------- */
#if defined(STM32L4xx) || defined(STM32L452xx) || defined(STM32L4A6xx)
  #include "stm32l4xx_hal.h"
  #include "stm32l4xx_hal_can.h"
#elif defined(STM32F4xx)
  #include "stm32f4xx_hal.h"
  #include "stm32f4xx_hal_can.h"
  #include "stm32f4xx_hal_tim.h"
#elif defined(STM32F1xx)
  #include "stm32f1xx_hal.h"
  #include "stm32f1xx_hal_can.h"
  #include "stm32f1xx_hal_tim.h"
#else
  /* Fallback: generic HAL */
  #include "stm32xx_hal.h"
  #include "stm32xx_hal_can.h"
  #include "stm32xx_hal_tim.h"
#endif

#ifndef COUNTOF
#define COUNTOF(x) (sizeof(x)/sizeof((x)[0]))
#endif

/* externs provided by project */
extern CAN_HandleTypeDef hcan1;

/* ---------------------- Configuration ---------------------- */
#ifndef CANSNIFF_RING_SIZE
#define CANSNIFF_RING_SIZE   256u
#endif

/* ---------------------- Internal state ---------------------- */
typedef struct {
    uint32_t ts_ms;   /* HAL_GetTick() */
    uint32_t ts_us;   /* microseconds (TIMx or DWT) */
    uint32_t id;      /* 11 or 29 bit */
    uint8_t  dlc;     /* 0..8 */
    uint8_t  ide;     /* 0=STD, 1=EXT */
    uint8_t  rtr;     /* 0=data, 1=remote */
    uint8_t  data[8];
} SniffFrame;

static volatile uint16_t s_wr;
static volatile uint16_t s_rd;
static SniffFrame        s_rb[CANSNIFF_RING_SIZE];
static volatile uint32_t s_drop_cnt;

static TIM_HandleTypeDef *s_us_htim = NULL;
static uint32_t           s_us_hz   = 0;
//static bool               s_csv     = false;

/* ---------------------- DWT fallback ---------------------- */
static inline void dwt_enable_if_avail(void) {
#if defined (DWT) && defined (CoreDebug)
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        DWT->CYCCNT = 0;
        DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
    }
#endif
}

static inline uint32_t micros_now_fallback(void) {
#if defined (DWT) && defined (CoreDebug)
    uint32_t cyc = DWT->CYCCNT;
    uint32_t hz  = HAL_RCC_GetSysClockFreq();
    if (hz == 0) return 0;
    return (uint32_t)((uint64_t)cyc / (hz / 1000000u));
#else
    return HAL_GetTick() * 1000u; /* coarse fallback */
#endif
}

/* ---------------------- Time sources ---------------------- */
static inline uint32_t micros_now(void) {
#if defined(HAL_TIM_MODULE_ENABLED)
    if (s_us_htim && s_us_hz) {
        uint32_t cnt = __HAL_TIM_GET_COUNTER(s_us_htim);
        if (s_us_hz == 1000000u) return cnt;               /* already µs */
        return (uint32_t)((uint64_t)cnt * 1000000ull / s_us_hz);
    }
#endif
    return micros_now_fallback();
}

/* ---------------------- Ring helpers ---------------------- */
static inline uint16_t rb_next(uint16_t i) { return (uint16_t)((i + 1u) % CANSNIFF_RING_SIZE); }
static inline bool rb_full(void)  { return rb_next(s_wr) == s_rd; }
static inline bool rb_empty(void) { return s_wr == s_rd; }

/* ---------------------- Accept-all filter ---------------------- */
static void cansniff_config_accept_all(void) {
    CAN_FilterTypeDef f = {0};
    f.FilterBank           = 0;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;
    f.FilterMaskIdLow      = 0x0000;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;
    HAL_StatusTypeDef st = HAL_CAN_ConfigFilter(&hcan1, &f);
    printf("CANSNIFF: ConfigFilter(all) = %s\r\n",
           (st==HAL_OK)?"OK":(st==HAL_ERROR)?"ERR":(st==HAL_BUSY)?"BUSY":"TIMEOUT");
}

/* ---------------------- Public API ---------------------- */
void CanSniffer_AttachMicroTimer(TIM_HandleTypeDef *htim, uint32_t timer_hz) {
#if defined(HAL_TIM_MODULE_ENABLED)
    s_us_htim = htim;
    s_us_hz   = timer_hz;
#else
    (void)htim; (void)timer_hz;
    s_us_htim = NULL; s_us_hz = 0;
#endif
    if (!s_us_htim || !s_us_hz) {
        dwt_enable_if_avail();
        printf("CANSNIFF: TIM module not attached/enabled; using DWT/ms fallback\r\n");
    }
}

// void CanSniffer_SetCsvMode(bool on) {
//     s_csv = on;
// }
//
// void CanSniffer_PrintHeader(void) {
//     if (s_csv) {
//         printf("ms,us,ide,rtr,id,dlc,data0,data1,data2,data3,data4,data5,data6,data7\r\n");
//     } else {
//         printf("# CAN sniffer: pretty mode (use CSV for spreadsheets)\r\n");
//     }
// }

void CanSniffer_Start(void) {
    if (!s_us_htim || !s_us_hz) dwt_enable_if_avail();

    cansniff_config_accept_all();

    HAL_StatusTypeDef st = HAL_CAN_Start(&hcan1);
    printf("CANSNIFF: CAN_Start = %s\r\n",
           (st==HAL_OK)?"OK":(st==HAL_ERROR)?"ERR":(st==HAL_BUSY)?"BUSY":"TIMEOUT");

    uint32_t it = CAN_IT_RX_FIFO0_MSG_PENDING
                | CAN_IT_RX_FIFO0_FULL
                | CAN_IT_RX_FIFO0_OVERRUN
                | CAN_IT_ERROR_WARNING
                | CAN_IT_ERROR_PASSIVE
                | CAN_IT_BUSOFF
                | CAN_IT_LAST_ERROR_CODE
                | CAN_IT_ERROR;

    st = HAL_CAN_ActivateNotification(&hcan1, it);
    printf("CANSNIFF: ActivateNotif = %s\r\n",
           (st==HAL_OK)?"OK":(st==HAL_ERROR)?"ERR":(st==HAL_BUSY)?"BUSY":"TIMEOUT");

    s_wr = s_rd = 0;
    s_drop_cnt = 0;
    printf("CANSNIFF: ready (ring=%u)\r\n", (unsigned)CANSNIFF_RING_SIZE);
}

/* ---------------------- Printing ---------------------- */
static void print_frame_pretty(const SniffFrame *fr) {
    printf("[%9lu ms | %9lu us] %s id=%s%08lX dlc=%u%s data=",
           (unsigned long)fr->ts_ms,
           (unsigned long)fr->ts_us,
           fr->ide ? "EXT" : "STD",
           fr->ide ? "0x"  : "0x00000",
           (unsigned long)fr->id,
           (unsigned)fr->dlc,
           fr->rtr ? " RTR" : "");
    for (uint8_t i = 0; i < fr->dlc; ++i) printf(" %02X", fr->data[i]);
    printf("\r\n");
}

static void print_frame_csv(const SniffFrame *fr) {
    printf("%lu,%lu,%u,%u,0x%lX,%u",
           (unsigned long)fr->ts_ms,
           (unsigned long)fr->ts_us,
           (unsigned)fr->ide,
           (unsigned)fr->rtr,
           (unsigned long)fr->id,
           (unsigned)fr->dlc);
    for (uint8_t i = 0; i < 8; ++i) printf(",%02X", fr->data[i]);
    printf("\r\n");
}

void CanSniffer_Task(void) {
    static uint32_t last_drop = 0;
    if (s_drop_cnt != last_drop) {
        printf("CANSNIFF: WARNING dropped %lu frames (ring full)\r\n",
               (unsigned long)(s_drop_cnt - last_drop));
        last_drop = s_drop_cnt;
    }
    while (!rb_empty()) {
        SniffFrame fr = s_rb[s_rd];
        s_rd = rb_next(s_rd);
        if (s_csv) print_frame_csv(&fr);
        else       print_frame_pretty(&fr);
    }
}

/* ---------------------- HAL callbacks (ISR) ---------------------- */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hh) {
    CAN_RxHeaderTypeDef rxh;
    uint8_t data[8];

    while (HAL_CAN_GetRxFifoFillLevel(hh, CAN_RX_FIFO0) > 0U) {
        if (HAL_CAN_GetRxMessage(hh, CAN_RX_FIFO0, &rxh, data) != HAL_OK) break;

        if (rb_full()) { s_drop_cnt++; continue; }
        SniffFrame *dst = &s_rb[s_wr];
        s_wr = rb_next(s_wr);

        dst->ts_ms = HAL_GetTick();
        dst->ts_us = micros_now();
        dst->ide   = (rxh.IDE == CAN_ID_EXT) ? 1u : 0u;
        dst->rtr   = (rxh.RTR == CAN_RTR_REMOTE) ? 1u : 0u;
        dst->id    = dst->ide ? rxh.ExtId : rxh.StdId;
        dst->dlc   = (rxh.DLC <= 8u) ? rxh.DLC : 8u;
        memset(dst->data, 0, sizeof(dst->data));
        if (dst->dlc) memcpy(dst->data, data, dst->dlc);
    }
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hh) { (void)hh; printf("CANSNIFF: FIFO0 FULL\r\n"); }
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hh) {
    uint32_t e = HAL_CAN_GetError(hh);
    printf("CANSNIFF: ERR 0x%08lX\r\n", (unsigned long)e);
}
