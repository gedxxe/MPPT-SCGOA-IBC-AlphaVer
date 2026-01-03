/*
 * File Role    : Menjalankan proteksi dasar (tegangan/arus/suhu) dan mengelola relay/kipas.
 * Dependencies : main.h untuk akses variabel global pembacaan sensor dan flag state MPPT.
 * Fungsi inti  : work_protect_charging() dipanggil periodik (500 ms) oleh main loop.
 */
#include "main.h"

ErrorStatus_e mpptError = 0; /* Status fault global untuk UI/MPPT. */
static uint8_t is_fan_on = 0; /* State kipas untuk menjaga hysteresis sederhana. */

uint8_t count_wait_charge   = 0; /* Debounce waktu sebelum relay baterai ON. */


void work_protect_charging() {
    /* Kendali kipas: pakai batas suhu absolut agar MOSFET/diode tidak overheat.
     * Hysteresis sederhana mencegah on/off cepat yang bisa menambah stress relay. */
    if ((dis_temperature >= 35) && !is_fan_on) {
        HAL_GPIO_WritePin(FAN_PIN_GPIO_Port, FAN_PIN_Pin, GPIO_PIN_SET); /* Aktifkan kipas saat panas. */
        is_fan_on = 1; /* Simpan state supaya tidak menulis GPIO berulang. */
    } else if ((dis_temperature <= 25) && is_fan_on) {
        HAL_GPIO_WritePin(FAN_PIN_GPIO_Port, FAN_PIN_Pin, GPIO_PIN_RESET); /* Matikan kipas ketika cukup dingin. */
        is_fan_on = 0; /* Reset state untuk siklus berikutnya. */
    }

    /* Proteksi tegangan/arus input & baterai diprioritaskan demi keamanan sel dan komponen daya. */
    if (dis_voltage_bat > BATTERY_PROTECT_VOLT) {
        mpptError = BAT_HIGH;        /* Tegangan baterai terlalu tinggi; hindari over-charge. */
    }
    else if (dis_current_bat > BATTERY_PROTECT_CURRENT) {
        mpptError = CURRENT_HIGH;    /* Arus baterai melewati batas aman; lindungi shunt/PCB. */
    }
    else if (dis_voltage_pv > PV_PROTECT_VOLT) {
        mpptError = INPUT_HIGH;      /* Tegangan input PV berlebih; cegah MOSFET gate driver overvoltage. */
    }
    else {
        mpptError = NO_ERROR;        /* Tidak ada fault terdeteksi; charging boleh lanjut. */
    }

    /* Menyalakan relay dan mulai algoritma charging hanya jika input aman. */
#ifdef CHARGING_TEST
    if (dis_voltage_pv >= 80 && dis_voltage_bat >= 100 && mpptError == NO_ERROR) {
#endif

#ifdef POWER_TEST
        if (dis_voltage_pv >= 80) {
#endif
            HAL_GPIO_WritePin(RLY_PV_PIN_GPIO_Port, RLY_PV_PIN_Pin, GPIO_PIN_SET); /* Tutup relay PV lebih dulu untuk hindari lonjakan. */

        count_wait_charge++; /* Tambah counter untuk memberi waktu kontak relay PV stabil mekanis. */
        if (count_wait_charge >= 6) {
            HAL_GPIO_WritePin(RLY_BAT_PIN_GPIO_Port, RLY_BAT_PIN_Pin, GPIO_PIN_SET); /* Setelah stabil, hubungkan ke baterai. */

            /* Jalankan inisialisasi charging sekali setelah relay ON.
             * check_initial_state akan mengevaluasi tegangan baterai untuk memilih Bulk/CV/Float. */
            if (!flag_enter_charge) {
                check_initial_state(); /* Set flag_enter_charge + state awal dengan data sensor terkini. */
            }
        }
    }
    else {
        HAL_GPIO_WritePin(RLY_PV_PIN_GPIO_Port, RLY_PV_PIN_Pin, GPIO_PIN_RESET); /* Lepas PV untuk keamanan. */
        HAL_GPIO_WritePin(RLY_BAT_PIN_GPIO_Port, RLY_BAT_PIN_Pin, GPIO_PIN_RESET); /* Lepas baterai untuk mencegah arus tak terkendali. */

        /* Matikan seluruh state charging saat input tidak memenuhi agar siklus berikutnya bersih. */
        flag_enter_charge = 0;      /* Blokir MPPT hingga kondisi aman kembali. */
        flag_charging_Bulk = 0;     /* Reset flag stage untuk menghindari state salah. */
        flag_charging_CV = 0;
        count_wait_charge = 0;      /* Reset debounce agar relay memulai ulang dengan jeda mekanis. */

        MPPT_Hybrid_Reset(); /* Bersihkan state internal MPPT (PnO/GOA, debouncer). */
    }
}
