//
// Created by Matthew Oduamafu on 07/11/2025.
//
/*
 * lcd_i2c_driver.c
 * Implementation of LCD I2C driver with animations
 */

#include "../Inc/lcd_i2c_driver.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

// Custom character patterns
static const uint8_t CHAR_PATTERNS[][8] = {
    // Arrow Right
    {0x00U, 0x04U, 0x02U, 0x1FU, 0x02U, 0x04U, 0x00U, 0x00U},
    // Arrow Left
    {0x00U, 0x04U, 0x08U, 0x1FU, 0x08U, 0x04U, 0x00U, 0x00U},
    // Degree
    {0x06U, 0x09U, 0x09U, 0x06U, 0x00U, 0x00U, 0x00U, 0x00U},
    // Bell
    {0x04U, 0x0EU, 0x0EU, 0x0EU, 0x1FU, 0x00U, 0x04U, 0x00U},
    // Heart
    {0x00U, 0x0AU, 0x1FU, 0x1FU, 0x0EU, 0x04U, 0x00U, 0x00U},
    // Battery Full
    {0x0EU, 0x1FU, 0x1FU, 0x1FU, 0x1FU, 0x1FU, 0x1FU, 0x0EU},
    // Battery Half
    {0x0EU, 0x11U, 0x11U, 0x1FU, 0x1FU, 0x1FU, 0x1FU, 0x0EU},
    // Battery Empty
    {0x0EU, 0x11U, 0x11U, 0x11U, 0x11U, 0x11U, 0x11U, 0x0EU}
};

// Spinner patterns
static const char SPINNER_CHARS[] = {'|', '/', '-', '\\'};

// Private functions
static void LCD_SendCommand(LCD_Handle *hlcd, uint8_t cmd);
static void LCD_SendData(LCD_Handle *hlcd, uint8_t data);
static void LCD_SendNibble(LCD_Handle *hlcd, uint8_t nibble, uint8_t mode);
static void LCD_Pulse(LCD_Handle *hlcd, uint8_t data);

// Initialize LCD
HAL_StatusTypeDef LCD_Init(LCD_Handle *hlcd, I2C_HandleTypeDef *hi2c) {
    hlcd->hi2c = hi2c;
    hlcd->address = LCD_I2C_ADDR;
    hlcd->backlight_state = LCD_BACKLIGHT;
    hlcd->current_row = 0;
    hlcd->current_col = 0;

    HAL_Delay(50); // Wait for LCD power up
    
    // Initialize in 4-bit mode
    LCD_SendNibble(hlcd, 0x03U, 0);
    HAL_Delay(5);
    LCD_SendNibble(hlcd, 0x03U, 0);
    HAL_Delay(1);
    LCD_SendNibble(hlcd, 0x03U, 0);
    HAL_Delay(1);
    LCD_SendNibble(hlcd, 0x02U, 0);
    HAL_Delay(1);

    // Function set: 4-bit, 2 lines, 5x8 dots
    LCD_SendCommand(hlcd, LCD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2_LINE | LCD_5x8_DOTS);

    // Display control: display on, cursor off, blink off
    hlcd->display_control = LCD_DISPLAY_CTRL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF;
    LCD_SendCommand(hlcd, hlcd->display_control);

    // Clear display
    LCD_Clear(hlcd);

    // Entry mode: increment, no shift
    hlcd->entry_mode = LCD_ENTRY_MODE | LCD_ENTRY_LEFT | LCD_ENTRY_SHIFT_DEC;
    LCD_SendCommand(hlcd, hlcd->entry_mode);

    HAL_Delay(2);

    // Load default custom characters
    LCD_LoadDefaultChars(hlcd);

    return HAL_OK;
}

// Send command to LCD
static void LCD_SendCommand(LCD_Handle *hlcd, uint8_t cmd) {
    uint8_t high_nibble = cmd & 0xF0U;
    uint8_t low_nibble = (cmd << 4) & 0xF0U;

    LCD_SendNibble(hlcd, high_nibble, 0);
    LCD_SendNibble(hlcd, low_nibble, 0);
}

// Send data to LCD
static void LCD_SendData(LCD_Handle *hlcd, uint8_t data) {
    uint8_t high_nibble = data & 0xF0U;
    uint8_t low_nibble = (data << 4) & 0xF0U;

    LCD_SendNibble(hlcd, high_nibble, LCD_RS);
    LCD_SendNibble(hlcd, low_nibble, LCD_RS);
}

// Send nibble with pulse
static void LCD_SendNibble(LCD_Handle *hlcd, uint8_t nibble, uint8_t mode) {
    uint8_t data = nibble | mode | hlcd->backlight_state;
    LCD_Pulse(hlcd, data);
}

// Generate enable pulse
static void LCD_Pulse(LCD_Handle *hlcd, uint8_t data) {
    HAL_I2C_Master_Transmit(hlcd->hi2c, hlcd->address, &data, 1, 100);
    HAL_Delay(2);

    data |= LCD_EN;
    HAL_I2C_Master_Transmit(hlcd->hi2c, hlcd->address, &data, 1, 100);
    HAL_Delay(2);

    data &= ~LCD_EN;
    HAL_I2C_Master_Transmit(hlcd->hi2c, hlcd->address, &data, 1, 100);
    HAL_Delay(2);
}

// Clear display
void LCD_Clear(LCD_Handle *hlcd) {
    LCD_SendCommand(hlcd, LCD_CLEAR);
    HAL_Delay(2);
    hlcd->current_row = 0;
    hlcd->current_col = 0;
}

// Return home
void LCD_Home(LCD_Handle *hlcd) {
    LCD_SendCommand(hlcd, LCD_HOME);
    HAL_Delay(2);
    hlcd->current_row = 0;
    hlcd->current_col = 0;
}

// Set cursor position
void LCD_SetCursor(LCD_Handle *hlcd, uint8_t row, uint8_t col) {
    uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};

    if (row >= LCD_ROWS) row = LCD_ROWS - 1;
    if (col >= LCD_COLS) col = LCD_COLS - 1;

    hlcd->current_row = row;
    hlcd->current_col = col;

    LCD_SendCommand(hlcd, LCD_DDRAM_ADDR | (col + row_offsets[row]));
}

// Display control
void LCD_Display(LCD_Handle *hlcd, bool on) {
    if (on) {
        hlcd->display_control |= LCD_DISPLAY_ON;
    } else {
        hlcd->display_control &= ~LCD_DISPLAY_ON;
    }
    LCD_SendCommand(hlcd, hlcd->display_control);
}

// Cursor control
void LCD_Cursor(LCD_Handle *hlcd, bool on) {
    if (on) {
        hlcd->display_control |= LCD_CURSOR_ON;
    } else {
        hlcd->display_control &= ~LCD_CURSOR_ON;
    }
    LCD_SendCommand(hlcd, hlcd->display_control);
}

// Blink control
void LCD_Blink(LCD_Handle *hlcd, bool on) {
    if (on) {
        hlcd->display_control |= LCD_BLINK_ON;
    } else {
        hlcd->display_control &= ~LCD_BLINK_ON;
    }
    LCD_SendCommand(hlcd, hlcd->display_control);
}

// Backlight control
void LCD_Backlight(LCD_Handle *hlcd, bool on) {
    if (on) {
        hlcd->backlight_state = LCD_BACKLIGHT;
    } else {
        hlcd->backlight_state = 0;
    }
    uint8_t data = hlcd->backlight_state;
    HAL_I2C_Master_Transmit(hlcd->hi2c, hlcd->address, &data, 1, 100);
}

// Print string
void LCD_Print(LCD_Handle *hlcd, const char *str) {
    while (*str) {
        LCD_SendData(hlcd, *str++);
        hlcd->current_col++;
    }
}

// Print at specific position
void LCD_PrintAt(LCD_Handle *hlcd, uint8_t row, uint8_t col, const char *str) {
    LCD_SetCursor(hlcd, row, col);
    LCD_Print(hlcd, str);
}

// Printf functionality
void LCD_Printf(LCD_Handle *hlcd, uint8_t row, uint8_t col, const char *fmt, ...) {
    char buffer[LCD_COLS + 1];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    LCD_PrintAt(hlcd, row, col, buffer);
}

// Print single character
void LCD_PrintChar(LCD_Handle *hlcd, char c) {
    LCD_SendData(hlcd, c);
    hlcd->current_col++;
}

// Create custom character
void LCD_CreateCustomChar(LCD_Handle *hlcd, uint8_t location, const uint8_t *charmap) {
    location &= 0x07; // Only 8 custom characters
    LCD_SendCommand(hlcd, LCD_CGRAM_ADDR | (location << 3));

    for (uint8_t i = 0; i < 8; i++) {
        LCD_SendData(hlcd, charmap[i]);
    }

    LCD_Home(hlcd);
}

// Load default custom characters
void LCD_LoadDefaultChars(LCD_Handle *hlcd) {
    for (uint8_t i = 0; i < 8; i++) {
        LCD_CreateCustomChar(hlcd, i, CHAR_PATTERNS[i]);
    }
}

// Print custom character
void LCD_PrintCustomChar(LCD_Handle *hlcd, uint8_t location) {
    LCD_SendData(hlcd, location & 0x07U);
    hlcd->current_col++;
}

// Initialize animation
void LCD_InitAnimation(LCD_Animation *anim, LCD_AnimationType type, uint8_t row, uint8_t col, uint32_t speed) {
    anim->type = type;
    anim->row = row;
    anim->col = col;
    anim->position = 0;
    anim->direction = 1;
    anim->last_update = HAL_GetTick();
    anim->speed = speed;
}

// Update animation
void LCD_UpdateAnimation(LCD_Handle *hlcd, LCD_Animation *anim) {
    if (HAL_GetTick() - anim->last_update < anim->speed) return;

    anim->last_update = HAL_GetTick();

    switch (anim->type) {
        case ANIM_ARROW_RIGHT:
            LCD_DrawArrow(hlcd, anim->row, anim->col, false);
            break;

        case ANIM_ARROW_LEFT:
            LCD_DrawArrow(hlcd, anim->row, anim->col, true);
            break;

        case ANIM_ARROW_BOUNCE:
            LCD_DrawArrow(hlcd, anim->row, anim->col + anim->position, anim->direction == 0);
            anim->position += anim->direction ? 1 : -1;
            if (anim->position >= 3) anim->direction = 0;
            if (anim->position == 0) anim->direction = 1;
            break;

        case ANIM_SPINNER:
            LCD_DrawSpinner(hlcd, anim->row, anim->col, anim->position);
            anim->position = (anim->position + 1) % 4;
            break;

        default:
            break;
    }
}

// Draw arrow
void LCD_DrawArrow(LCD_Handle *hlcd, uint8_t row, uint8_t col, bool left) {
    LCD_SetCursor(hlcd, row, col);
    LCD_PrintCustomChar(hlcd, left ? CHAR_ARROW_LEFT : CHAR_ARROW_RIGHT);
}

// Draw spinner
void LCD_DrawSpinner(LCD_Handle *hlcd, uint8_t row, uint8_t col, uint8_t state) {
    LCD_SetCursor(hlcd, row, col);
    LCD_PrintChar(hlcd, SPINNER_CHARS[state % 4]);
}

// Draw progress bar
void LCD_DrawProgressBar(LCD_Handle *hlcd, uint8_t row, uint8_t col, uint8_t width, uint8_t percent) {
    if (percent > 100) percent = 100;

    LCD_SetCursor(hlcd, row, col);
    LCD_PrintChar(hlcd, '[');

    uint8_t filled = (percent * width) / 100;
    for (uint8_t i = 0; i < width; i++) {
        LCD_PrintChar(hlcd, i < filled ? '=' : ' ');
    }

    LCD_PrintChar(hlcd, ']');
    LCD_Printf(hlcd, row, col + width + 3, "%3d%%", percent);
}

// Clear specific row
void LCD_ClearRow(LCD_Handle *hlcd, uint8_t row) {
    LCD_SetCursor(hlcd, row, 0);
    for (uint8_t i = 0; i < LCD_COLS; i++) {
        LCD_PrintChar(hlcd, ' ');
    }
}

// Draw separator line
void LCD_DrawSeparator(LCD_Handle *hlcd, uint8_t row) {
    LCD_SetCursor(hlcd, row, 0);
    for (uint8_t i = 0; i < LCD_COLS; i++) {
        LCD_PrintChar(hlcd, '-');
    }
}

// Scroll text
void LCD_ScrollText(LCD_Handle *hlcd, uint8_t row, const char *text, uint8_t delay_ms) {
    uint8_t len = strlen(text);
    if (len <= LCD_COLS) {
        LCD_PrintAt(hlcd, row, 0, text);
        return;
    }

    for (uint8_t i = 0; i <= len - LCD_COLS; i++) {
        LCD_SetCursor(hlcd, row, 0);
        for (uint8_t j = 0; j < LCD_COLS; j++) {
            LCD_PrintChar(hlcd, text[i + j]);
        }
        HAL_Delay(delay_ms);
    }
}

// Draw box
void LCD_DrawBox(LCD_Handle *hlcd, uint8_t row, uint8_t col, uint8_t width) {
    LCD_SetCursor(hlcd, row, col);
    LCD_PrintChar(hlcd, '[');
    for (uint8_t i = 0; i < width - 2; i++) {
        LCD_PrintChar(hlcd, ' ');
    }
    LCD_PrintChar(hlcd, ']');
}