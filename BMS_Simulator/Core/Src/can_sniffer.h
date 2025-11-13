#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

    /* Forward decl to avoid pulling HAL headers into users */
    struct __TIM_HandleTypeDef;
    typedef struct __TIM_HandleTypeDef TIM_HandleTypeDef;

    /* API */
    void CanSniffer_SetUart(UART_HandleTypeDef *huart);
    void CanSniffer_AttachMicroTimer(TIM_HandleTypeDef *htim, uint32_t timer_hz);
    void CanSniffer_SetCsvMode(bool on);
    void CanSniffer_PrintHeader(void);
    void CanSniffer_Start(void);
    void CanSniffer_Task(void);

#ifdef __cplusplus
}
#endif
