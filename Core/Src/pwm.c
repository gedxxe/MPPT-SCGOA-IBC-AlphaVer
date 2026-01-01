#include "main.h"

uint8_t shift_pwm_channel	= 0;
uint16_t duty_cycle			= 0;

uint8_t flag_pwm_off	= 0;
uint8_t flag_pwm_on		= 0;

uint16_t PWM_VALUE	= 0;

int duty_percent	= 0;

uint8_t flag_up = 0;

void pwm_shift_out(uint8_t ch, uint16_t val) {
	if(!flag_pwm_on) {
		flag_pwm_on		= 1;
	    flag_pwm_off	= 0;

	    __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_2);
	    __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_3);
	}
	if(val >= MAX_PERIOD) {
		val = MAX_PERIOD;
	}

	// Channel 1 - 0 degrees
	if (ch == 1) {
		if (val >= MAX_PERIOD) {
			TIM1->CCR3 = PERIOD;
			TIM1->CCR2 = PERIOD;
		}
		else if (val >= PERIOD) {
			TIM1->CCR3 = PERIOD;
			TIM1->CCR2 = val - PERIOD;
		}
		else {
			TIM1->CCR3 = val;
			TIM1->CCR2 = 0;
		}
	}
	// Channel 2 - 180 degrees
	else if (ch == 2) {
		if (val >= MAX_PERIOD) {
			TIM1->CCR3 = PERIOD;
			TIM1->CCR2 = PERIOD;
	    } else if (val >= PERIOD) {
	    	TIM1->CCR3 = val - PERIOD;
	    	TIM1->CCR2 = PERIOD;
	    } else {
	    	TIM1->CCR3 = 0;
	    	TIM1->CCR2 = val;
	    }
	}
}

void pwm_off(void) {
    if(!flag_pwm_off) {
    	flag_pwm_on		= 0;
        flag_pwm_off	= 1;

        __HAL_TIM_DISABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_2);
        __HAL_TIM_DISABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_3);
    }

	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
}

void pwm_test() {
	if(PWM_VALUE < 100 && flag_up) {
		flag_up = 1;
			if(PWM_VALUE>100) flag_up = 0;
	}
	else {
		flag_up = 0;
		if (PWM_VALUE<=0) flag_up = 1;
	}
	if(flag_up){
		PWM_VALUE++;
	}
	else if (!flag_up) {
		PWM_VALUE--;
	}
}
