/*
 * File Role    : Deklarasi utilitas LCD OLED (u8g2) untuk menampilkan data sistem/MPPT.
 * Dependencies : stm32f1xx_hal.h untuk akses HAL I2C/delay, u8g2.h sebagai driver display,
 *                stdio/string untuk format teks.
 * Fungsi inti  : lcd_init(), DisplayRabbitAnimation(), lcd_main_display(), dan lcd_print_unit()
 *                sebagai kontrak rendering UI; objek display diekspos untuk modul lain jika perlu.
 */
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
void lcd_print_unit(uint8_t col, uint8_t row, uint16_t data, UnitLcd_e unit);
void lcd_main_display(void);

#endif
