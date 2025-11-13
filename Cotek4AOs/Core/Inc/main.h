/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Exported handles (defined in respective modules) */
extern I2C_HandleTypeDef  hi2c1;
extern UART_HandleTypeDef huart2;
extern CAN_HandleTypeDef  hcan;

  /* Simple LED / Button aliases for NUCLEO-F103RB style boards */
#define LD2_GPIO_Port      GPIOA
#define LD2_Pin            GPIO_PIN_5

#define USER_BTN_GPIO_Port GPIOC
#define USER_BTN_Pin       GPIO_PIN_13


/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
