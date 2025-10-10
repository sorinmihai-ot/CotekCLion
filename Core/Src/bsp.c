/*****************************************************************************
* BSP for STM32 NUCLEO-F401RE with QP/C framework
*****************************************************************************/
#include "qpc_cfg.h"
#include "qpc.h"  /* QP/C API */
#include "../Inc/bsp.h"
#include <stdio.h> /* for printf() */
#include "can_app.h"
#include "ao_controller.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_rcc.h"
#include "debug_trace.h"
#include "stm32f1xx.h"

// Local-scope defines -----------------------------------------------------
// #define TICK_INT_PRIORITY 5
// LED pins available on the board (just one user LED LD2--Green on PA.5)
#define LD2_PIN  5U

// external LED to be inserted between GND (short leg) and
// D12 (longer leg) on the CN9 connector
#define LD5_PIN  6U

// Button pins available on the board (just one user Button B1 on PC.13)
#define B1_PIN   13U

/* NUCLEO-F103RB: B1 USER button on PC13, active HIGH when pressed */
#ifndef USER_BTN_PORT
#define USER_BTN_PORT GPIOC
#endif
#ifndef USER_BTN_PIN
#define USER_BTN_PIN  GPIO_PIN_13
#endif

extern UART_HandleTypeDef huart3;       // from main.c
extern uint8_t s_uart3_rxbuf[128];
/* NEW: QF-started flag */
static volatile bool s_qf_started = false;
extern void StartHmiRx(void);
extern volatile uint8_t  g_lastTag;

bool BSP_qfStarted(void) {
    return s_qf_started != 0U;
}

void BSP_markQfStarted(void) {
    s_qf_started = true;
}

/* This is called by HAL from EXTI15_10_IRQHandler() */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (!BSP_qfStarted()) {                 // <<< guard like SysTick does
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin); // clear any spurious pending
        return;
    }
    if (GPIO_Pin == GPIO_PIN_13) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
        // GPIO_PinState s = HAL_GPIO_ReadPin(USER_BTN_PORT, USER_BTN_PIN);
        // if (s == GPIO_PIN_SET) {
        //     static QEvt const pressEvt = { .sig = BUTTON_PRESSED_SIG };
        //     g_lastSig = BUTTON_PRESSED_SIG;  g_lastTag = 3;
        //     printf("BTN: EXTI PC13 pressed\r\n");
        //     (void)QACTIVE_POST_X(AO_Controller, &pressEvt, 3U, 0U);
        //
        // } else {
        //     static QEvt const relEvt   = { .sig = BUTTON_RELEASED_SIG };
        //     g_lastSig = BUTTON_RELEASED_SIG; g_lastTag = 4;
        //     printf("BTN: EXTI PC13 pressed again\r\n");
        //     (void)QACTIVE_POST_X(AO_Controller, &relEvt, 3U, 0U);
        // }
        return;
    }
}
// ------------ LED on PA5 (change if needed) ------------
static void led_init_once(void) {
    static uint8_t inited = 0;
    if (inited) return;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin   = GPIO_PIN_5;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED off on many boards
    inited = 1;
}
static inline void led_toggle(void) {
    GPIOA->ODR ^= GPIO_PIN_5;
}
// ------------ UART2 ready flag ------------
static volatile uint8_t s_uart2_ready = 0;
void BSP_markUart2Ready(void) { s_uart2_ready = 1U; }

// ------------ RAW UART2 output (no HAL, no locks, polling) ------------
static inline void uart2_raw_putc(char ch) {
    // only use once USART2 is clocked + pins configured
    USART_TypeDef *U = USART2;
    while ((U->SR & (uint32_t)USART_SR_TXE) == 0U) { /* wait */ }
    U->DR = (uint16_t)ch;
}
static void uart2_raw_puts(const char *s) {
    while (*s) uart2_raw_putc(*s++);
}

// ------------ Breadcrumbs ------------
void BSP_breadcrumb(uint8_t tag) {
    led_init_once();
    led_toggle();            // visual tick
    if (USART2->CR1 & USART_CR1_UE) {
        // if USART2 is enabled, try to drop a single raw tag
        uart2_raw_putc((char)tag);
    }
}

// ------------ Fatal blinker ------------
void BSP_die(uint8_t code) {
    led_init_once();
    // Try to cough out a last message if USART2 is alive
    if (USART2->CR1 & USART_CR1_UE) {
        uart2_raw_puts("\r\nDIE ");
        uart2_raw_putc('0' + (code / 10));
        uart2_raw_putc('0' + (code % 10));
        uart2_raw_puts("\r\n");
    }
    for (;;) {
        // blink "code" times quickly, pause, repeat
        for (uint8_t i = 0; i < code; ++i) {
            led_toggle();
            HAL_Delay(120);
            led_toggle();
            HAL_Delay(120);
        }
        HAL_Delay(600);
    }
}
static volatile uint8_t s_aos_ready = 0U;
void BSP_markAOsReady(void) { s_aos_ready = 1U; }

static inline uint32_t rd8(volatile uint8_t const *p){ return *p; }
void BSP_dumpIRQs(void) {
    uint32_t group = (SCB->AIRCR >> SCB_AIRCR_PRIGROUP_Pos) & 7U;
    uint32_t prioBits = __NVIC_PRIO_BITS;

    printf("IRQ dump: PRIGROUP=%lu  __NVIC_PRIO_BITS=%lu\r\n",
           (unsigned long)group, (unsigned long)prioBits);

    // core exceptions and a few peripherals you use
    printf("  SysTick=%ld  PendSV=%ld  SVCall=%ld\r\n",
           (long)NVIC_GetPriority(SysTick_IRQn),
           (long)NVIC_GetPriority(PendSV_IRQn),
           (long)NVIC_GetPriority(SVCall_IRQn));

    printf("  USART2=%ld  USART3=%ld  EXTI15_10=%ld  CAN1_TX=%ld  CAN1_RX0=%ld\r\n",
           (long)NVIC_GetPriority(USART2_IRQn),
           (long)NVIC_GetPriority(USART3_IRQn),
           (long)NVIC_GetPriority(EXTI15_10_IRQn),
           (long)NVIC_GetPriority(USB_HP_CAN1_TX_IRQn),
           (long)NVIC_GetPriority(USB_LP_CAN1_RX0_IRQn));
}

void QF_onStartup(void) {
    // bsp.c :: QF_onStartup()
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    // Kernel-aware ISRs — keep them all at QF_AWARE_ISR_CMSIS_PRI (== 5)
    HAL_NVIC_SetPriority(USART2_IRQn, QF_AWARE_ISR_CMSIS_PRI, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    HAL_NVIC_SetPriority(USART3_IRQn, QF_AWARE_ISR_CMSIS_PRI, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    HAL_NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, QF_AWARE_ISR_CMSIS_PRI, 0);
    HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);

    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, QF_AWARE_ISR_CMSIS_PRI, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    // **Important change**: SysTick must also be kernel-aware (same threshold)
    HAL_NVIC_SetPriority(SysTick_IRQn, QF_AWARE_ISR_CMSIS_PRI, 0);

    // Now it’s safe to start HMI RX (IRQ won’t preempt critical sections incorrectly)
    StartHmiRx();
    BSP_markQfStarted();
}

void QF_onCleanup(void) {
    s_qf_started = false;
}

int __io_putchar(int ch) {
    // Only send if USART2 is enabled (avoid spurious writes before init)
    if ((USART2->CR1 & USART_CR1_UE) == 0U) {
        return ch;
    }
    // Poll TXE, then write the byte directly; no HAL, no SysTick needed
    while ((USART2->SR & USART_SR_TXE) == 0U) {
        // spin a few cycles
    }
    USART2->DR = (uint16_t)ch;
    return ch;
}

void BSP_print_banner(void) {
    printf("\r\n=== Cotek / QP/C / STM32F103 ===\r\n");
}

// /* Error/Assertions  =======================================================*/
// /* assertion-handling function */
// Q_NORETURN Q_onAssert(char const *module, int loc) {
//     uint32_t ipsr      = __get_IPSR();                 // nonzero => ISR
//     uint32_t basepri   = __get_BASEPRI();              // threshold in effect
//     uint32_t vectact   = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk);
//
//     printf(">>> Q_onAssert: %s : %d  IPSR=%lu  VECTACTIVE=%lu  BASEPRI=0x%02lX\r\n",
//            (module ? module : "?"), (long)loc,
//            (unsigned long)ipsr,
//            (unsigned long)(vectact >> SCB_ICSR_VECTACTIVE_Pos),
//            (unsigned long)basepri);
//
//     // stay here flashing an LED if you want, or reset
//     NVIC_SystemReset();
//     for (;;) { /* unreachable */ }
// }
//............................................................................
// assert-handling function called by exception handlers in the startup code
void assert_failed(char const * module, int_t id); // prototype
void assert_failed(char const * const module, int_t const id) {
    Q_onAssert(module, id);
}


// ISRs  ======================================================================
void SysTick_Handler(void) {
    /* HAL tick must always run */
    HAL_IncTick();

    if (s_qf_started) {
        /* QP time events */
        static uint8_t q_tick_div;
        if (++q_tick_div >= 10) {   // 1000 Hz / 10 = 100 Hz
            q_tick_div = 0;
            QTIMEEVT_TICK_X(0U, &l_SysTick_Handler);
            }

        /* Button debounce + posts ONLY after kernel started */
        static struct {
            uint32_t depressed;
            uint32_t previous;
        } buttons = { 0U, 0U };

        uint32_t current = GPIOC->IDR;
        uint32_t tmp = buttons.depressed;
        buttons.depressed |= (buttons.previous & current);
        buttons.depressed &= (buttons.previous | current);
        buttons.previous   = current;
        tmp ^= buttons.depressed;
        current = buttons.depressed;
        static uint32_t warmup = 200U; // ~200 ms at 1 kHz
        if (warmup) { --warmup; return; }
        if ((tmp & (1U << B1_PIN)) != 0U) {
            if ((current & (1U << B1_PIN)) != 0U) {
                static QEvt const pressEvt = QEVT_INITIALIZER(BUTTON_PRESSED_SIG);
                g_lastSig = BUTTON_PRESSED_SIG;  g_lastTag = 1;  // tag 1 = SysTick press
                QACTIVE_POST_X(AO_Controller, &pressEvt, 3U, 0U);
                printf("BTN: PC13 pressed\r\n");
            } else {
                static QEvt const releaseEvt = QEVT_INITIALIZER(BUTTON_RELEASED_SIG);
                g_lastSig = BUTTON_RELEASED_SIG; g_lastTag = 2; // tag 2 = SysTick release
                QACTIVE_POST_X(AO_Controller, &releaseEvt, 3U, 0U);
            }
        }
    }
}

//............................................................................
void QV_onIdle(void) {
    QF_INT_ENABLE();
#ifdef NDEBUG
    /* Put the CPU and peripherals to the low-power mode.
    * you might need to customize the clock management for your application,
    * see the datasheet for your particular Cortex-M MCU.
    */
    QV_CPU_SLEEP();  /* atomically go to sleep and enable interrupts */
#else
    QF_INT_ENABLE(); /* just enable interrupts */
#endif
}
/* BSP functions ===========================================================*/
void BSP_init(void) {
    //BSP_print_banner();
}


#define POSTX_TRACE_TAG(tag_, ao_, e_, margin_, sender_)          \
do {                                                          \
g_lastSig = (uint16_t)((e_)->sig);                        \
g_lastTag = (uint8_t)(tag_);                              \
(void)QACTIVE_POST_X((ao_), (e_), (margin_), (sender_));  \
} while (0)

Q_NORETURN Q_onError(char const * const module, int loc) {
    __disable_irq();
    printf(">>> Q_onAssert: %s : %d  (lastSig=%u tag=%u)\r\n",
           module, loc, g_lastSig, g_lastTag);
    while (1) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        for (volatile uint32_t i=0; i<100000; ++i) { __NOP(); }
    }
}


// support for printf() ======================================================
#if 0
int fputc(int c, FILE *stream) { (void)stream; ITM_SendChar(c); return c; }
#endif

