/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "can.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "lcd_2004.h"
#include "buttons.h"
#include "can_tx.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "can_sniffer.h"

// ----- UI STATE -----
typedef enum {
  APP_WELCOME = 0,
  APP_SELECT_BATT,
  APP_SELECT_MODE,
  APP_ACTIVE
} AppState;

// transient status line in ACTIVE mode
static char activeStatusLine[21] = "BACK=stop";
static uint32_t activeStatusUntilMs = 0; // 0 means "no timeout"
static AppState appState = APP_WELCOME;

// battery types to choose from
#define NUM_BATT_TYPES 6
static const char* battTypes[NUM_BATT_TYPES] = {
    "600s",
    "500s Hyper",
    "500s BMZ",
    "400s Hyper",
    "400s DualZ",
    "400s Steatite"
};
static uint8_t battIndex = 0; // which is currently highlighted

// modes
#define NUM_MODES 3
static const char* modes[NUM_MODES] = {
  "Operational",
  "Rec",
  "Non-Rec"
};
static uint8_t modeIndex = 0;

// timestamp markers
static uint32_t bootMs = 0;

// forward declare our helpers
static void UI_Render(void);
static void UI_NextBatt(void);
static void UI_NextMode(void);
static void UI_GotoSelectBatt(void);
static void UI_GotoSelectMode(void);
static void UI_GotoActive(void);

// Position helpers to avoid repeating cursor math
static void lcdWriteLine(uint8_t row, const char* text)
{
    // NOTE: your LCD_SetCursor(col,row) expects (col,row)
    // We'll always start at col=0.
    LCD_SetCursor(0, row);
    LCD_Print("                    "); // clear line (20 spaces)
    LCD_SetCursor(0, row);
    LCD_Print(text);
}

static void UI_Render_Welcome(void)
{
    lcdWriteLine(0, "Ocado Technology");
    lcdWriteLine(1, "Battery Simulator");
    lcdWriteLine(2, "      v1.0");
    lcdWriteLine(3, "   Initializing...");
}

static void UI_Render_SelectBatt(void)
{
    // Row0: title
    lcdWriteLine(0, "Select Battery:");

    // Row1: current batt type highlighted with '>'
    char line1[21];
    snprintf(line1, sizeof(line1), "> %s", battTypes[battIndex]);
    lcdWriteLine(1, line1);

    // Row2: hint
    lcdWriteLine(2, "BATT=cycle  SEL=ok");

    // Row3: blank / or debug
    char dbg[21];
    snprintf(dbg, sizeof(dbg), "%u/%u", battIndex+1, NUM_BATT_TYPES);
    lcdWriteLine(3, dbg);
}

static void UI_Render_SelectMode(void)
{
    lcdWriteLine(0, "Select Mode:");

    char line1[21];
    snprintf(line1, sizeof(line1), "> %s", modes[modeIndex]);
    lcdWriteLine(1, line1);

    lcdWriteLine(2, "MODE=cycle SEL=ok");
    lcdWriteLine(3, "BACK=to batt");
}

static void UI_Render_Active(void)
{
    // Row0: short summary in one line
    char summary0[21];
    // make mode short for fit: we can derive a short label
    const char* modeShort = modes[modeIndex];
    // optional shortening:
    if (strcmp(modeShort, "Operational") == 0) {
        modeShort = "Op";
    } else if (strcmp(modeShort, "Non-Rec") == 0) {
        modeShort = "NRec";
    } else if (strcmp(modeShort, "Rec") == 0) {
        modeShort = "Rec";
    }

    snprintf(summary0, sizeof(summary0), "Type:%s Mode:%s", battTypes[battIndex], modeShort);
    lcdWriteLine(0, summary0);

    // Row1: indicate we're live
    lcdWriteLine(1, "Sending CAN frames");

    // Row2: hint for fault buttons
    lcdWriteLine(2, "NC=warn CRIT=err");

    // Row3: either BACK=stop or temporary injected message
    lcdWriteLine(3, activeStatusLine);
}


static void UI_Render(void)
{
    switch(appState) {
        case APP_WELCOME:      UI_Render_Welcome();      break;
        case APP_SELECT_BATT:  UI_Render_SelectBatt();   break;
        case APP_SELECT_MODE:  UI_Render_SelectMode();   break;
        case APP_ACTIVE:       UI_Render_Active();       break;
        default: break;
    }
}

// state transitions
static void UI_GotoSelectBatt(void)
{
    appState = APP_SELECT_BATT;
    UI_Render();
}

static void UI_GotoSelectMode(void)
{
    appState = APP_SELECT_MODE;
    modeIndex = 0;
    UI_Render();
}

static void UI_GotoActive(void)
{
    appState = APP_ACTIVE;
    strncpy(activeStatusLine, "BACK=stop", sizeof(activeStatusLine));
    activeStatusLine[sizeof(activeStatusLine)-1] = '\0';
    activeStatusUntilMs = 0; // no temporary message yet
    UI_Render();
}

static void UI_ActiveSetTempStatus(const char* msg, uint32_t now, uint32_t durationMs)
{
    // copy message to status line
    strncpy(activeStatusLine, msg, sizeof(activeStatusLine));
    activeStatusLine[sizeof(activeStatusLine)-1] = '\0';

    // set expiry time
    activeStatusUntilMs = now + durationMs;

    // re-render just line 3 to avoid flicker
    lcdWriteLine(3, activeStatusLine);
}

static void UI_ActiveUpdateStatus(uint32_t now)
{
    if (appState != APP_ACTIVE) {
        return;
    }

    if (activeStatusUntilMs != 0U && (int32_t)(now - activeStatusUntilMs) >= 0) {
        // timeout expired -> restore default
        strncpy(activeStatusLine, "BACK=stop", sizeof(activeStatusLine));
        activeStatusLine[sizeof(activeStatusLine)-1] = '\0';
        activeStatusUntilMs = 0;

        // update LCD line 3
        lcdWriteLine(3, activeStatusLine);
    }
}

// cycling helpers
static void UI_NextBatt(void)
{
    battIndex = (uint8_t)((battIndex + 1U) % NUM_BATT_TYPES);
    UI_Render();
}

static void UI_NextMode(void)
{
    modeIndex = (uint8_t)((modeIndex + 1U) % NUM_MODES);
    UI_Render();
}


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
//helpers for the reset issue:
void HardFault_Handler(void)
{
    // try to shout over UART, assuming USART2 is initialized
    const char msg[] = "!!! HARD FAULT !!!\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg)-1, 100);

    // sit here so we don't auto-reset and lose context
    while (1) {
        // blink LED so you see it's dead, optional:
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        HAL_Delay(100);
    }
}
void Error_Handler(void)
{
    const char msg[] = "!!! Error_Handler TRAP !!!\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg)-1, 100);

    while (1)
    {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        HAL_Delay(200);
    }
}
static void PrintResetCause(void)
{
    uint32_t csr = RCC->CSR; // capture first, BEFORE clearing
    // do NOT clear yet

    printf("\r\n=== Reset cause flags: 0x%08lX ===\r\n", (unsigned long)csr);

    if (csr & RCC_CSR_WWDGRSTF)   printf(" -> Window Watchdog reset\r\n");
    if (csr & RCC_CSR_IWDGRSTF)   printf(" -> Independent Watchdog reset\r\n");
    if (csr & RCC_CSR_SFTRSTF)    printf(" -> Software reset\r\n");
    if (csr & RCC_CSR_BORRSTF)    printf(" -> Brown-out or POR reset\r\n");
    if (csr & RCC_CSR_PINRSTF)    printf(" -> NRST pin reset\r\n");
    if (csr & RCC_CSR_LPWRRSTF)   printf(" -> Low-power reset\r\n");
    if (csr & RCC_CSR_OBLRSTF)    printf(" -> Option byte loader reset\r\n");

    // now that we've printed, clear them for NEXT reset
    __HAL_RCC_CLEAR_RESET_FLAGS();
}
static void UART_Sanity(UART_HandleTypeDef *huart)
{
    const char *hello = "\r\n[UART-SANITY] USART OK @ boot\r\n";
    for (int i = 0; i < 5; ++i) {
        HAL_UART_Transmit(huart, (uint8_t*)hello, (uint16_t)strlen(hello), 1000);
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        HAL_Delay(500);
    }
}
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  MX_USART2_UART_Init();
    // // ===== UART SCREAM TEST =====
    // {
    //   const char *msg = "\r\n[UART] HELLO from USART2 @ boot\r\n";
    //   for (int i = 0; i < 5; ++i) {
    //       HAL_UART_Transmit(&huart2, (uint8_t*)msg, (uint16_t)strlen(msg), 1000);
    //       HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    //       HAL_Delay(500);
    //   }
    //
    //   // Now blast 'U' forever so a scope/terminal can see activity
    //   while (1) {
    //       uint8_t U = 'U';
    //       HAL_UART_Transmit(&huart2, &U, 1, 1000);
    //       HAL_Delay(100);                    // ~100 Hz stream of 'U'
    //       HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    //   }
    // }
    // ===== END UART SCREAM TEST =====
  //   UART_Sanity(&huart2);      // <<<<<< should print 5 lines and blink LED
  //   PrintResetCause();
   printf(" -> Main Loop Started\r\n");

  MX_I2C2_Init();
  MX_CAN1_Init();
  HAL_Delay(1000);
  // //   /////////////
  // const char *banner = "\r\n[CAN-SNIFFER] Booting...\r\n";
  // HAL_UART_Transmit(&huart2, (uint8_t*)banner, strlen(banner), 100);
  // /* Sniffer setup */
  // CanSniffer_SetUart(&huart2);    // make sure the sniffer uses the right UART
  // CanSniffer_SetCsvMode(true);    // CSV output for easy logging
  // CanSniffer_PrintHeader();       // print CSV header now, even before frames
  // CanSniffer_Start();             // start CAN (filters + notifications)
  // while (1) {
  //     CanSniffer_Task();   // prints lines; call as often as you like
  //     HAL_Delay(1);
  // }
   CAN_TX_Init();
   bootMs = HAL_GetTick();

   // init LCD
   LCD_Init();
   LCD_Clear();

   // show welcome first
   appState = APP_WELCOME;
   UI_Render();

   // init debouncing
   Buttons_Init();

 while (1)
 {
     uint32_t now = HAL_GetTick();

     // transition from WELCOME → SELECT_BATT after 5000 ms
     if (appState == APP_WELCOME) {
         if ((now - bootMs) >= 5000U) {
             UI_GotoSelectBatt();
         }
     }

     // poll buttons every ~10ms
     static uint32_t lastTick = 0;
     if ((now - lastTick) >= 10U) {
         lastTick = now;

         Buttons_Update();
         // manage temporary "NonCritical sent!" / "CRITICAL sent!"
         UI_ActiveUpdateStatus(now);

         // Handle per-state input
         switch(appState) {
             case APP_SELECT_BATT:
                 if (Button_WasPressed(BTN_BATT)) {
                     UI_NextBatt();
                     printf("BATT cycle -> %s\r\n", battTypes[battIndex]);
                 }

                 if (Button_WasPressed(BTN_SELECT)) {
                     // lock chosen battery type into CAN layer
                     CAN_TX_SetBatteryType(battIndex);
                     printf("SELECT batt %s\r\n", battTypes[battIndex]);
                     UI_GotoSelectMode();
                 }
                 break;


             case APP_SELECT_MODE:
                 if (Button_WasPressed(BTN_MODE)) {
                     UI_NextMode();
                     printf("MODE cycle -> %s\r\n", modes[modeIndex]);
                 }
                 if (Button_WasPressed(BTN_SELECT)) {
                     // lock battery + mode, tell CAN layer
                     CAN_TX_SetBatteryType(battIndex);
                     CAN_TX_SetMode(modeIndex);
                     printf("SELECT mode %s\r\n", modes[modeIndex]);
                     UI_GotoActive();
                 }
                 if (Button_WasPressed(BTN_BACK)) {
                     UI_GotoSelectBatt();
                 }
                 break;



         case APP_ACTIVE:
             // We'll fill these in Step 2 CAN side.
             if (Button_WasPressed(BTN_NONCRIT)) {
                 printf("Inject NON-CRIT\r\n");
                 CAN_TX_SendNonCritical();
                 // TODO: send CAN noncritical fault frame(s)
                 UI_ActiveSetTempStatus("NonCritical sent!", now, 1000U);
             }
             if (Button_WasPressed(BTN_CRIT)) {
                 printf("Inject CRIT\r\n");
                 CAN_TX_SendCritical();
                 // TODO: send CAN critical fault frame(s)
                 UI_ActiveSetTempStatus("CRITICAL sent!", now, 1000U);
             }
             if (Button_WasPressed(BTN_BACK)) {
                 printf("Back to main menu\r\n");
                 UI_GotoSelectBatt();
                 // TODO later: stop CAN spam
             }

             break;

         default:
             break;
         }
         CAN_TX_PeriodicTask(appState == APP_ACTIVE);

         // crude ~10ms pacing
         HAL_Delay(10);
         // you can leave LED toggle how it is for now, or slow it later
     }
}

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
// void Error_Handler(void)
// {
//   /* USER CODE BEGIN Error_Handler_Debug */
//   /* User can add his own implementation to report the HAL error return state */
//   __disable_irq();
//   while (1)
//   {
//   }
//   /* USER CODE END Error_Handler_Debug */
// }
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
