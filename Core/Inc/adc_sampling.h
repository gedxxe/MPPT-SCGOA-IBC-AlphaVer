#ifndef	ADC_SAMPLING_H
#define ADC_SAMPLING_H

#include "stm32f1xx_hal.h"
#include <math.h>

#define BAT_VOLT_COEF	9.5f
#define PV_VOLT_COEF	9.6f
#define BAT_CURR_COEF	13.1f
#define PV_CURR_COEF	16.1f

extern volatile uint16_t ADC_RAW[5];

extern uint16_t adc_voltage_bat;
extern uint16_t adc_voltage_pv;
extern uint16_t adc_current_bat;
extern uint16_t adc_current_pv;
extern uint16_t adc_temperature;

extern uint32_t sum_voltage_bat;
extern uint32_t sum_voltage_pv;
extern uint32_t sum_current_bat;
extern uint32_t sum_current_pv;
extern uint32_t sum_temperature;

extern uint16_t avg_voltage_bat;
extern uint16_t avg_voltage_pv;
extern uint16_t avg_current_bat;
extern uint16_t avg_current_pv;
extern uint16_t avg_temperature;

extern uint16_t voltage_bat;
extern uint16_t voltage_pv;
extern uint16_t current_bat;
extern uint16_t current_pv;
extern float temperature;

extern uint16_t dis_voltage_bat;
extern uint16_t dis_voltage_pv;
extern uint16_t dis_current_bat;
extern uint16_t dis_current_pv;
extern uint16_t dis_power_bat;
extern uint16_t dis_power_pv;
extern int dis_temperature;

extern uint16_t soc_battery;

extern uint8_t adc_conv_flag;

void adc_sampling(void);

#endif
