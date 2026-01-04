/*
 * File Role    : Deklarasi algoritma MPPT hibrid (PnO + Goat Optimizer Algorithm/GOA) dan state machine charging.
 * Dependencies : stm32f1xx_hal.h untuk tipe dasar dan akses ke waktu/HAL.
 * Fungsi inti  : MPPT_Hybrid(), MPPT_Hybrid_Reset(), check_initial_state(), charging_flow()
 *                beserta flag status (flag_charging_*, flag_adc_done, dbg_limit_psu).
 */
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
extern uint8_t dbg_limit_psu;

/* Compile-time switch:
 * - ENABLE_PSU_ESCAPE=1 (default): aktifkan escape hatch PSU current-limited.
 * - ENABLE_PSU_ESCAPE=0: matikan jika sumber adalah panel PV murni.
 *   Dapat di-set via -D atau dengan mendefinisikan sebelum include mppt.c. */

/* Menggerakkan MPPT hybrid (PnO + GOA) tiap 10 ms. */
void MPPT_Hybrid(void);

/* Mereset seluruh state internal MPPT (GOA, PnO, debounce). */
void MPPT_Hybrid_Reset(void);

/* Memilih state awal charging berdasar tegangan baterai dan mengizinkan loop. */
void check_initial_state(void);

/* State machine utama charging (Bulk/CV/Float + proteksi input). */
void charging_flow(void);

#endif
