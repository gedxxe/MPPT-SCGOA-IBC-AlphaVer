#ifndef WORK_PROTECT_H
#define WORK_PROTECT_H

#include "stm32f1xx_hal.h"

typedef enum {
	NO_ERROR,
	BAT_HIGH,
	INPUT_LOW,
	INPUT_HIGH,
	CURRENT_HIGH,
	TEMPERATURE_HIGH
} ErrorStatus_e;

extern ErrorStatus_e mpptError;

void work_protect_charging(void);

#endif
