#ifndef PWM_H
#define PWM_H

#include "stm32f1xx_hal.h"

#define PERIOD      300
#define MAX_PERIOD  600   /* Dua fase interleaved: PERIOD x2. */

extern TIM_HandleTypeDef htim1;

extern uint16_t PWM_VALUE;

extern uint8_t  shift_pwm_channel; /* Kanal PWM yang sedang di-drive (1 atau 2). */
extern uint16_t duty_cycle;        /* Duty register terakhir yang disiapkan. */

extern uint8_t flag_pwm_off;
extern uint8_t flag_pwm_on;

extern uint8_t flag_up;

extern int duty_percent;

void pwm_shift_out(uint8_t ch, uint16_t val);
void pwm_off(void);
void pwm_test(void);


#endif
