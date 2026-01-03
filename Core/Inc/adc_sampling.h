/*
 * File Role    : Deklarasi fungsi dan variabel pembacaan ADC beserta scaling-nya.
 * Dependencies : stm32f1xx_hal.h untuk akses HAL ADC/GPIO, math.h untuk log() temperatur.
 * Fungsi inti  : adc_sampling() memproses batch ADC; variabel display (dis_*) diekspos
 *                sebagai kontrak pembacaan sensor bagi modul MPPT, LCD, dan proteksi.
 */
#ifndef ADC_SAMPLING_H
#define ADC_SAMPLING_H

#include "stm32f1xx_hal.h"
#include <math.h>

/* Koefisien pembagi tegangan/arus hasil kalibrasi hardware
 * untuk mengonversi nilai ADC mentah ke satuan fisik. */
#define BAT_VOLT_COEF   9.5f  // Skala tegangan baterai
#define PV_VOLT_COEF    9.6f  // Skala tegangan PV
#define BAT_CURR_COEF   13.1f // Skala arus baterai
#define PV_CURR_COEF    16.1f // Skala arus PV

#define ADC_CHANNEL_COUNT 5

extern volatile uint16_t ADC_RAW[ADC_CHANNEL_COUNT];

extern uint16_t dis_voltage_bat;
extern uint16_t dis_voltage_pv;
extern uint16_t dis_current_bat;
extern uint16_t dis_current_pv;
extern uint16_t dis_power_bat;
extern uint16_t dis_power_pv;
extern int dis_temperature;

extern uint16_t soc_battery;

void adc_sampling(void);

#endif
