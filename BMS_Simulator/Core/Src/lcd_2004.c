//
// Created by sorin.mihai on 26/10/2025.
//
#include "lcd_2004.h"
#include "stm32l4xx_hal.h"
#include <string.h>
#include <stdint.h>

// ---- low-level helpers for PCF8574 backpack ----
// Bit layout we assume for typical I2C LCD backpack:
//
// P7  P6  P5  P4  P3  P2   P1   P0
// D7  D6  D5  D4  BL  EN   RW   RS
//
// We will always send RW=0 (write).
// BL = backlight bit (1 = on).
// EN is the strobe.
//
// If your backpack is wired differently and text is garbage,
// we’ll adjust this mapping.

#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE    0x04
#define LCD_RW        0x02
#define LCD_RS        0x01

static void lcd_write_nibble(uint8_t nibble, uint8_t rs);
static void lcd_write_byte(uint8_t value, uint8_t rs);
static void lcd_cmd(uint8_t cmd);
static void lcd_data(uint8_t data);

static void i2c_send(uint8_t val)
{
    HAL_I2C_Master_Transmit(&hi2c2, LCD_I2C_ADDR, &val, 1, 10);
}

// Pulse EN high->low to latch nibble
static void lcd_pulse(uint8_t data)
{
    i2c_send(data | LCD_ENABLE | LCD_BACKLIGHT);
    // small delay - we’ll use a dumb software delay
    for (volatile int i = 0; i < 200; i++) { __NOP(); }

    i2c_send((data & ~LCD_ENABLE) | LCD_BACKLIGHT);
    for (volatile int i = 0; i < 200; i++) { __NOP(); }
}

// Send high 4 bits of 'nibble' on D7..D4 plus RS
static void lcd_write_nibble(uint8_t nibble, uint8_t rs)
{
    uint8_t out = 0;

    // put upper 4 bits of nibble onto D7..D4
    out |= (nibble & 0xF0);

    // RS
    if (rs)
        out |= LCD_RS;

    // RW = 0 always (write)

    // backlight always on
    out |= LCD_BACKLIGHT;

    lcd_pulse(out);
}

// Send full byte (hi nibble then lo nibble)
static void lcd_write_byte(uint8_t value, uint8_t rs)
{
    lcd_write_nibble(value & 0xF0, rs);
    lcd_write_nibble((value << 4) & 0xF0, rs);
}

// command
static void lcd_cmd(uint8_t cmd)
{
    lcd_write_byte(cmd, 0);
    HAL_Delay(2); // safe delay for most cmds
}

// data (a character)
static void lcd_data(uint8_t dataByte)
{
    lcd_write_byte(dataByte, 1);
}

// ---------------- public API ----------------

void LCD_Clear(void)
{
    lcd_cmd(0x01); // Clear display
    HAL_Delay(2);
}

void LCD_SetCursor(uint8_t col, uint8_t row)
{
    // DDRAM mapping for 20x4:
    // row0: 0x00
    // row1: 0x40
    // row2: 0x14
    // row3: 0x54
    static const uint8_t rowAddr[4] = {0x00, 0x40, 0x14, 0x54};
    if (row > 3) row = 3;
    lcd_cmd(0x80 | (rowAddr[row] + col));
}

// Print a C string at current cursor
void LCD_Print(const char *text)
{
    while (*text)
    {
        lcd_data((uint8_t)*text++);
    }
}

// HD44780 4-bit init sequence over I2C backpack.
// This is the slightly-annoying "reset to 4-bit mode" dance.
void LCD_Init(void)
{
    HAL_Delay(50); // wait after power up

    // We directly strobe nibbles 0x30 three times to force 8-bit mode,
    // then 0x20 to drop to 4-bit interface.
    // These are "nibble writes", so we fake a write_nibble with RS=0.

    // send 0x30 high nibble
    lcd_write_nibble(0x30, 0);
    HAL_Delay(5);

    lcd_write_nibble(0x30, 0);
    HAL_Delay(5);

    lcd_write_nibble(0x30, 0);
    HAL_Delay(5);

    // send 0x20 high nibble -> 4-bit mode
    lcd_write_nibble(0x20, 0);
    HAL_Delay(5);

    // now we're in 4-bit mode officially. Configure function set:
    // 0x28: 4-bit, 2-line... (On 20x4, controller is really "2x40")
    lcd_cmd(0x28);

    // Display off
    lcd_cmd(0x08);

    // Clear
    lcd_cmd(0x01);
    HAL_Delay(2);

    // Entry mode set: increment, no shift
    lcd_cmd(0x06);

    // Display on, cursor off, blink off
    lcd_cmd(0x0C);

    // Backlight is kept on automatically in i2c_send()
}
