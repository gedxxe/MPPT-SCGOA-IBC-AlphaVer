#include "main.h"

ErrorStatus_e mpptError = 0;

//uint8_t flag_charge_allow	= 0;
uint8_t flag_charge_allow	= 1;
uint8_t count_wait_charge	= 0;


void work_protect_charging() {
	if (dis_temperature > 25) {
		HAL_GPIO_WritePin(FAN_PIN_GPIO_Port, FAN_PIN_Pin, GPIO_PIN_SET);
	} else if (dis_temperature < 35) {
		HAL_GPIO_WritePin(FAN_PIN_GPIO_Port, FAN_PIN_Pin, GPIO_PIN_RESET);
	}
	//Proteksi lainnya
	if (dis_voltage_bat > BATTERY_PROTECT_VOLT) {
		mpptError = BAT_HIGH;
	}
	else if (dis_current_bat > BATTERY_PROTECT_CURRENT) {
		mpptError = CURRENT_HIGH;
	}
	else if (dis_voltage_pv > PV_PROTECT_VOLT) {
		mpptError = INPUT_HIGH;
	}
	else {
		mpptError = NO_ERROR;
	}

	//Memyalakan relay dan mulai algoritma charging
#ifdef CHARGING_TEST
	if (dis_voltage_pv >= 80 && dis_voltage_bat >= 100 && mpptError <= 0) {
#endif

#ifdef POWER_TEST
		if (dis_voltage_pv >= 80) {
#endif
		HAL_GPIO_WritePin(RLY_PV_PIN_GPIO_Port, RLY_PV_PIN_Pin, GPIO_PIN_SET);

		count_wait_charge++;
		if (count_wait_charge >= 6) {
			//PWM_VALUE = 0;
			HAL_GPIO_WritePin(RLY_BAT_PIN_GPIO_Port, RLY_BAT_PIN_Pin, GPIO_PIN_SET);

			//flag untu memulai charging dalam stage bulk dan menyalakan PWM,
			flag_enter_charge = 1;
			flag_charging_Bulk = 1;
			flag_charging_CV = 0;
		}
	}
	else {
		HAL_GPIO_WritePin(RLY_PV_PIN_GPIO_Port, RLY_PV_PIN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RLY_BAT_PIN_GPIO_Port, RLY_BAT_PIN_Pin, GPIO_PIN_RESET);

		flag_enter_charge = 0;
		flag_charging_Bulk = 0;
		flag_charging_CV = 0;

		MPPT_Hybrid_Reset(); // Tambahan baru
	}
}
