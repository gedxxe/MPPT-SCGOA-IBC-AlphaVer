#include "main.h"

ErrorStatus_e mpptError = 0; /* Status fault global untuk UI/MPPT. */
static uint8_t is_fan_on = 0; /* State kipas untuk menjaga hysteresis sederhana. */

uint8_t count_wait_charge   = 0; /* Debounce waktu sebelum relay baterai ON. */


void work_protect_charging() {
    /* Kendali kipas dengan hysteresis sederhana (ON di >=35°C, OFF di <=25°C). */
    if ((dis_temperature >= 35) && !is_fan_on) {
        HAL_GPIO_WritePin(FAN_PIN_GPIO_Port, FAN_PIN_Pin, GPIO_PIN_SET);
        is_fan_on = 1;
    } else if ((dis_temperature <= 25) && is_fan_on) {
        HAL_GPIO_WritePin(FAN_PIN_GPIO_Port, FAN_PIN_Pin, GPIO_PIN_RESET);
        is_fan_on = 0;
    }

    /* Proteksi tegangan/arus input & baterai. */
    if (dis_voltage_bat > BATTERY_PROTECT_VOLT) {
        mpptError = BAT_HIGH;        /* Tegangan baterai terlalu tinggi. */
    }
    else if (dis_current_bat > BATTERY_PROTECT_CURRENT) {
        mpptError = CURRENT_HIGH;    /* Arus baterai melewati batas aman. */
    }
    else if (dis_voltage_pv > PV_PROTECT_VOLT) {
        mpptError = INPUT_HIGH;      /* Tegangan input PV berlebih. */
    }
    else {
        mpptError = NO_ERROR;        /* Tidak ada fault terdeteksi. */
    }

    /* Menyalakan relay dan mulai algoritma charging hanya jika input aman. */
#ifdef CHARGING_TEST
    if (dis_voltage_pv >= 80 && dis_voltage_bat >= 100 && mpptError == NO_ERROR) {
#endif

#ifdef POWER_TEST
        if (dis_voltage_pv >= 80) {
#endif
            HAL_GPIO_WritePin(RLY_PV_PIN_GPIO_Port, RLY_PV_PIN_Pin, GPIO_PIN_SET);

        count_wait_charge++; /* Tambah counter untuk menunggu relay stabil. */
        if (count_wait_charge >= 6) {
            HAL_GPIO_WritePin(RLY_BAT_PIN_GPIO_Port, RLY_BAT_PIN_Pin, GPIO_PIN_SET);

            /* Jalankan inisialisasi charging sekali setelah relay ON. */
            if (!flag_enter_charge) {
                check_initial_state();
            }
        }
    }
    else {
        HAL_GPIO_WritePin(RLY_PV_PIN_GPIO_Port, RLY_PV_PIN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RLY_BAT_PIN_GPIO_Port, RLY_BAT_PIN_Pin, GPIO_PIN_RESET);

        /* Matikan seluruh state charging saat input tidak memenuhi. */
        flag_enter_charge = 0;
        flag_charging_Bulk = 0;
        flag_charging_CV = 0;
        count_wait_charge = 0; /* Reset debounce agar relay butuh waktu stabil lagi. */

        MPPT_Hybrid_Reset(); // Bersihkan state internal MPPT.
    }
}
