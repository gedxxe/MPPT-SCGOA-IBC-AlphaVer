#ifndef LCD_DISPLAY_H
#define	LCD_DISPLAY_H

#include "stm32f1xx_hal.h"

#include "u8g2.h"

#include <stdio.h>
#include <string.h>

typedef enum {
	UNIT_VOLTAGE,
	UNIT_CURRENT,
	UNIT_TEMPERATURE,
	UNIT_PERCENTAGE,
	UNIT_POWER
} UnitLcd_e;

extern I2C_HandleTypeDef hi2c1;

extern u8g2_t display;

void lcd_init(void);
void DisplayRabbitAnimation(void);
void lcd_print_unit(uint8_t col, uint8_t row, uint16_t data, UnitLcd_e uint);
void lcd_main_display(void);

#endif
