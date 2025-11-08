//
// Created by Matthew Oduamafu on 07/11/2025.
//
/*
 * lcd_i2c_driver.h
 * Driver for 20x4 LCD with I2C interface (PCF8574)
 */

#ifndef LCD_I2C_DRIVER_H
#define LCD_I2C_DRIVER_H


#include "stm32f0xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// LCD Configuration
#define LCD_I2C_ADDR         0x27U << 1  // Adjust if needed (0x3F << 1 for some modules)
#define LCD_ROWS             4
#define LCD_COLS             20

// LCD Commands
#define LCD_CLEAR            0x01U
#define LCD_HOME             0x02U
#define LCD_ENTRY_MODE       0x04U
#define LCD_DISPLAY_CTRL     0x08U
#define LCD_CURSOR_SHIFT     0x10U
#define LCD_FUNCTION_SET     0x20U
#define LCD_CGRAM_ADDR       0x40U
#define LCD_DDRAM_ADDR       0x80U

// Entry Mode flags
#define LCD_ENTRY_RIGHT      0x00U
#define LCD_ENTRY_LEFT       0x02U
#define LCD_ENTRY_SHIFT_INC  0x01U
#define LCD_ENTRY_SHIFT_DEC  0x00U

// Display Control flags
#define LCD_DISPLAY_ON       0x04U
#define LCD_DISPLAY_OFF      0x00U
#define LCD_CURSOR_ON        0x02U
#define LCD_CURSOR_OFF       0x00U
#define LCD_BLINK_ON         0x01U
#define LCD_BLINK_OFF        0x00U

// Function Set flags
#define LCD_8BIT_MODE        0x10U
#define LCD_4BIT_MODE        0x00U
#define LCD_2_LINE           0x08U
#define LCD_1_LINE           0x00U
#define LCD_5x10_DOTS        0x04U
#define LCD_5x8_DOTS         0x00U

// PCF8574 pins
#define LCD_RS               0x01U
#define LCD_RW               0x02U
#define LCD_EN               0x04U
#define LCD_BACKLIGHT        0x08U

// Animation types
typedef enum {
    ANIM_NONE = 0,
    ANIM_ARROW_RIGHT,
    ANIM_ARROW_LEFT,
    ANIM_ARROW_BOUNCE,
    ANIM_SPINNER,
    ANIM_PROGRESS
} LCD_AnimationType;

// Custom characters
typedef enum {
    CHAR_ARROW_RIGHT = 0,
    CHAR_ARROW_LEFT,
    CHAR_DEGREE,
    CHAR_BELL,
    CHAR_HEART,
    CHAR_BATTERY_FULL,
    CHAR_BATTERY_HALF,
    CHAR_BATTERY_EMPTY
} LCD_CustomChar;

// LCD Handle
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
    uint8_t backlight_state;
    uint8_t display_control;
    uint8_t entry_mode;
    uint8_t current_row;
    uint8_t current_col;
} LCD_Handle;

// Animation state
typedef struct {
    LCD_AnimationType type;
    uint8_t row;
    uint8_t col;
    uint8_t position;
    uint8_t direction;
    uint32_t last_update;
    uint32_t speed;
} LCD_Animation;

// Initialization
HAL_StatusTypeDef LCD_Init(LCD_Handle *hlcd, I2C_HandleTypeDef *hi2c);

// Basic Commands
void LCD_Clear(LCD_Handle *hlcd);
void LCD_Home(LCD_Handle *hlcd);
void LCD_SetCursor(LCD_Handle *hlcd, uint8_t row, uint8_t col);
void LCD_Display(LCD_Handle *hlcd, bool on);
void LCD_Cursor(LCD_Handle *hlcd, bool on);
void LCD_Blink(LCD_Handle *hlcd, bool on);
void LCD_Backlight(LCD_Handle *hlcd, bool on);

// Text Output
void LCD_Print(LCD_Handle *hlcd, const char *str);
void LCD_PrintAt(LCD_Handle *hlcd, uint8_t row, uint8_t col, const char *str);
void LCD_Printf(LCD_Handle *hlcd, uint8_t row, uint8_t col, const char *fmt, ...);
void LCD_PrintChar(LCD_Handle *hlcd, char c);

// Custom Characters
void LCD_CreateCustomChar(LCD_Handle *hlcd, uint8_t location, const uint8_t *charmap);
void LCD_LoadDefaultChars(LCD_Handle *hlcd);
void LCD_PrintCustomChar(LCD_Handle *hlcd, uint8_t location);

// Animation Functions
void LCD_InitAnimation(LCD_Animation *anim, LCD_AnimationType type, uint8_t row, uint8_t col, uint32_t speed);
void LCD_UpdateAnimation(LCD_Handle *hlcd, LCD_Animation *anim);
void LCD_DrawArrow(LCD_Handle *hlcd, uint8_t row, uint8_t col, bool left);
void LCD_DrawSpinner(LCD_Handle *hlcd, uint8_t row, uint8_t col, uint8_t state);
void LCD_DrawProgressBar(LCD_Handle *hlcd, uint8_t row, uint8_t col, uint8_t width, uint8_t percent);

// Advanced Display
void LCD_ClearRow(LCD_Handle *hlcd, uint8_t row);
void LCD_ScrollText(LCD_Handle *hlcd, uint8_t row, const char *text, uint8_t delay_ms);
void LCD_DrawBox(LCD_Handle *hlcd, uint8_t row, uint8_t col, uint8_t width);
void LCD_DrawSeparator(LCD_Handle *hlcd, uint8_t row);

// Utility
void LCD_Delay(uint32_t ms);


#endif //LCD_I2C_DRIVER_H