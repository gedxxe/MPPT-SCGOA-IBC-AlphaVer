#include "main.h"

volatile uint16_t ADC_RAW[5];

uint16_t adc_voltage_pv		= 0;
uint16_t adc_voltage_bat	= 0;
uint16_t adc_current_pv		= 0;
uint16_t adc_current_bat	= 0;
uint16_t adc_temperature	= 0;

uint32_t sum_voltage_pv		= 0;
uint32_t sum_voltage_bat	= 0;
uint32_t sum_current_pv		= 0;
uint32_t sum_current_bat	= 0;
uint32_t sum_temperature	= 0;

uint16_t avg_voltage_pv		= 0;
uint16_t avg_voltage_bat	= 0;
uint16_t avg_current_pv		= 0;
uint16_t avg_current_bat	= 0;
uint16_t avg_temperature	= 0;

uint16_t voltage_bat	= 0;
uint16_t voltage_pv		= 0;
uint16_t current_bat	= 0;
uint16_t current_pv		= 0;
float temperature		= 0;

uint16_t dis_voltage_bat	= 0;
uint16_t dis_voltage_pv		= 0;
uint16_t dis_current_bat	= 0;
uint16_t dis_current_pv		= 0;
uint16_t dis_power_bat		= 0;
uint16_t dis_power_pv		= 0;
int dis_temperature			= 0;

uint16_t soc_battery = 0;

float rNTC	= 0;

uint8_t sum_adc_count	= 0;

uint8_t flag_adc_conv	= 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);

	sum_adc_count++;

	sum_voltage_pv	+= ADC_RAW[0];
	sum_voltage_bat	+= ADC_RAW[1];
	sum_current_pv	+= ADC_RAW[3];
	sum_current_bat	+= ADC_RAW[2];
	sum_temperature	+= ADC_RAW[4];

	//Membuat rata" nilai ADC dari DMA
	if (sum_adc_count >= 10) {
		avg_voltage_pv	= sum_voltage_pv / 10;
		avg_voltage_bat	= sum_voltage_bat / 10;
		avg_current_pv	= sum_current_pv / 10;
		avg_current_bat	= sum_current_bat / 10;
		avg_temperature	= sum_temperature / 10;

		flag_adc_conv = 1;

		sum_voltage_pv	= 0;
		sum_voltage_bat	= 0;
		sum_current_pv	= 0;
		sum_current_bat	= 0;
		sum_temperature	= 0;

		sum_adc_count	= 0;
	}
}

void adc_sampling() {
	uint32_t tempVal;
	if (flag_adc_conv){
		//Mengonversi Voltage Pembacaan PV
		voltage_pv	= avg_voltage_pv / PV_VOLT_COEF;
		tempVal = voltage_pv;
		tempVal *= 512;
		tempVal >>= 9;
		dis_voltage_pv = tempVal;

		//Mengonversi Voltage Pembacaan Baterai
		voltage_bat	= avg_voltage_bat / BAT_VOLT_COEF;
		tempVal = voltage_bat;
		tempVal *= 512;
		tempVal >>= 9;
		dis_voltage_bat = tempVal;

		//Mengonversi Pembacaan Arus PV
		current_pv = avg_current_pv / PV_CURR_COEF;
		tempVal = current_pv;
		tempVal *= 512;
		tempVal >>= 9;
		dis_current_pv = tempVal;

		//Mengonversi Pembacaan Arus Baterai
		current_bat = avg_current_bat / BAT_CURR_COEF;
		tempVal = current_bat;
		tempVal *= 512;
		tempVal >>= 9;
		dis_current_bat = tempVal;

		//Mengonversi Pembacaan Suhu
		temperature = avg_temperature * (3.3 / 4095);
		rNTC = (3.3f * 9840.0f / temperature) - 9840.0f;
		dis_temperature = ((3950 * 298.15f) / (298.15f * log(rNTC / 10000.0f) + 3950)) - 273.15f;

		dis_power_pv = dis_current_pv * dis_voltage_pv;
		dis_power_bat = dis_current_bat * dis_voltage_bat;

		soc_battery = (dis_voltage_bat * 100) / MAX_BATTERY_CHARGE;

		// Membuat Flag/tanda bahwa konversi ADC sudah selesai, flag_adc_done akan digunakan di charging_flow();
		flag_adc_conv = 0;
		flag_adc_done = 1;
	}
}
