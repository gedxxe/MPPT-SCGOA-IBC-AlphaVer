/*
 * File Role    : Mengelola output PWM interleaved dua fase (TIM1 CH2/CH3) untuk konverter boost.
 * Dependencies : main.h untuk akses handle TIM1 dan konstanta periode.
 * Fungsi inti  : pwm_shift_out(), pwm_off(), pwm_test() sesuai deklarasi pwm.h.
 * Catatan      : Variabel global (PWM_VALUE, duty_percent, flag_pwm_on/off) dipakai lintas modul.
 */
#include "main.h"

uint8_t shift_pwm_channel = 0; /* Channel aktif (1 atau 2) yang sedang di-drive. */
uint16_t duty_cycle       = 0; /* Duty cycle terakhir yang di-request (register mentah). */

uint8_t flag_pwm_off = 0;      /* Status bahwa PWM sedang dimatikan untuk menghindari double-disable. */
uint8_t flag_pwm_on  = 0;      /* Status bahwa PWM sedang aktif untuk menghindari double-enable. */

uint16_t PWM_VALUE   = 0;      /* Nilai duty mentah yang dipakai MPPT dan LCD. */

int duty_percent     = 0;      /* Representasi persentase duty (display). */

uint8_t flag_up      = 0;      /* Penentu arah sweep di pwm_test. */

void pwm_shift_out(uint8_t ch, uint16_t val) {
    if (!flag_pwm_on) {
        flag_pwm_on  = 1; /* Tandai PWM sudah aktif supaya tidak enable ulang. */
        flag_pwm_off = 0; /* Reset flag off agar konsisten. */

        __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_2); /* Aktifkan preload agar update sinkron. */
        __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_3);
    }
    if (val >= MAX_PERIOD) {
        val = MAX_PERIOD; /* Clamp duty agar tidak keluar domain dua fase. */
    }

    /* Channel 1 - 0 degrees (fase pertama interleaved). */
    if (ch == 1) {
        if (val >= MAX_PERIOD) {
            TIM1->CCR3 = PERIOD;     /* Kedua channel full ketika duty 100%. */
            TIM1->CCR2 = PERIOD;
        }
        else if (val >= PERIOD) {
            TIM1->CCR3 = PERIOD;     /* Fase pertama penuh, sisanya diberikan ke fase kedua. */
            TIM1->CCR2 = val - PERIOD;
        }
        else {
            TIM1->CCR3 = val;        /* Duty hanya pada fase pertama. */
            TIM1->CCR2 = 0;
        }
    }
    /* Channel 2 - 180 degrees (fase kedua interleaved). */
    else if (ch == 2) {
        if (val >= MAX_PERIOD) {
            TIM1->CCR3 = PERIOD;     /* Kedua channel full ketika duty 100%. */
            TIM1->CCR2 = PERIOD;
        } else if (val >= PERIOD) {
            TIM1->CCR3 = val - PERIOD; /* Fase kedua diisi setelah fase pertama penuh. */
            TIM1->CCR2 = PERIOD;
        } else {
            TIM1->CCR3 = 0;          /* Duty hanya pada fase kedua. */
            TIM1->CCR2 = val;
        }
    }
}

void pwm_off(void) {
    if (!flag_pwm_off) {
        flag_pwm_on  = 0;
        flag_pwm_off = 1;

        __HAL_TIM_DISABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_2);
        __HAL_TIM_DISABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_3);
    }

    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
}

void pwm_test() {
    /* Test sweep untuk melihat respon driver; tidak dipakai di produksi. */
    if (flag_up) {
        if (PWM_VALUE < 100) {
            PWM_VALUE++;
        } else {
            flag_up = 0; /* Balik arah saat mencapai batas atas. */
        }
    } else {
        if (PWM_VALUE > 0) {
            PWM_VALUE--;
        } else {
            flag_up = 1; /* Balik arah saat mencapai nol. */
        }
    }
}
