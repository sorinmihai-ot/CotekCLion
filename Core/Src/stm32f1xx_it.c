/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "qpc_cfg.h"
#include "app_signals.h"
#include "stm32f1xx_it.h"
#include "bsp.h"

#include <ao_controller.h>

#include "stm32f1xx_hal.h"

#include "bsp.h"
#include "qpc.h"
extern QActive *AO_Cotek;

/* External variables --------------------------------------------------------*/

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
   while (1)  {  }
}
/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  while (1) {  }
}
/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  while (1)  { }
}
/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  while (1)  {  }
}
/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  while (1)  {  }
}
/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
}
/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
}
/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
}
static void EXTI15_10_NVIC_QPaware(void) {
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, QF_AWARE_ISR_CMSIS_PRI, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
void EXTI15_10_IRQHandler(void) {
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);  // PC13 callback
  EXTI15_10_NVIC_QPaware();
}
/**
  * @brief This function handles System tick timer.
  */


/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/
/**
  * @brief This function handles USB high priority or CAN TX interrupts.
  */
void USB_HP_CAN1_TX_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan);
}

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan);
}
/**
  * @brief This function handles CAN RX1 interrupt.
  */
void CAN1_RX1_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan);
}
/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&hi2c1);
}
/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{
  HAL_I2C_ER_IRQHandler(&hi2c1);
}

void USART2_IRQHandler(void) {
  HAL_UART_IRQHandler(&huart2);
}
void USART3_IRQHandler(void) {
  HAL_UART_IRQHandler(&huart3);
}


