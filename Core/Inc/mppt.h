#ifndef MPPT_H
#define MPPT_H

#include "stm32f1xx_hal.h"

/* Flag yang menandakan konversi ADC 10 ms terakhir selesai.
 * Dipakai sebagai guard agar state machine hanya berjalan dengan data baru. */
extern uint8_t flag_adc_done;

/* Flag global untuk menandakan charging loop diizinkan berjalan
 * (diset di check_initial_state ketika relai baterai aktif). */
extern uint8_t flag_enter_charge;

/* Status tahapan charging; hanya satu yang aktif pada satu waktu.
 * - Bulk   : MPPT aktif, mengejar daya maksimum.
 * - CV     : Absorption/constant voltage.
 * - FLOAT  : Penjagaan tegangan float. */
extern uint8_t flag_charging_Bulk;
extern uint8_t flag_charging_CV;
extern uint8_t flag_charging_FLOAT;

/* Menggerakkan MPPT hybrid (PnO + GOA) tiap 10 ms. */
void MPPT_Hybrid(void);

/* Mereset seluruh state internal MPPT (GOA, PnO, debounce). */
void MPPT_Hybrid_Reset(void);

/* Memilih state awal charging berdasar tegangan baterai dan mengizinkan loop. */
void check_initial_state(void);

/* State machine utama charging (Bulk/CV/Float + proteksi input). */
void charging_flow(void);

#endif
