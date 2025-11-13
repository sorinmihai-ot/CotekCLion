//
// Created by sorin.mihai on 26/10/2025.
//
#ifndef LCD_2004_H
#define LCD_2004_H

#include "main.h"
#include "i2c.h"   // for hi2c2

// Change this if your PCF8574 backpack uses a different address.
// Most are 0x27 or 0x3F. We'll start with 0x27.
#define LCD_I2C_ADDR (0x27 << 1)  // 7-bit addr shifted for HAL

// Public API
void LCD_Init(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t col, uint8_t row);
void LCD_Print(const char *text);

#endif
