#include "main.h"
#include <math.h>

/* ============================================================
 *  STATE MACHINE & VERIF SNAPSHOT (2024-11)
 *  - States:
 *      BULK  : MPPT_Hybrid() mengejar daya, guard konduksi & proteksi.
 *      CV    : Penahan Vabs dengan hysteresis ±VABS_HYST, anti-flap via timer.
 *      FLOAT : Penjagaan Vflt dengan hysteresis ±VFLT_HYST + rebulk timer.
 *      IDLE  : Semua flag 0 ketika PV absen / relay off.
 *  - Guards & hysteresis:
 *      • PV-loss debounce (AND, 500 ms) mematikan PWM, reset MPPT.
 *      • BULK→CV pakai tegangan + arus + timer 10 s; BULK→FLOAT fallback pakai
 *        tegangan tinggi + arus rendah + timer 3 s.
 *      • CV→FLOAT butuh dwell 30 s + arus rendah + timer 1 s.
 *      • FLOAT→BULK via V_REBULK 5 s dengan counter naik/turun.
 *  - Safety interlock:
 *      • Over-current/over-voltage: duty step-down + reset GOA gate.
 *      • PV dicabut: PWM=0, flags=0, MPPT reset, trend log dihapus.
 *  - Escape hatch PSU current-limited:
 *      • Deteksi arus mentok + Vpv sag + daya stagnan ≥0.8 s → duty dibekukan
 *        2 s + flag dbg_limit_psu untuk log ringan (ENABLE_PSU_ESCAPE dapat
 *        di-compile-off via -DENABLE_PSU_ESCAPE=0 atau override di mppt.h).
 *  - Skenario verif (ringkas):
 *      1) Cold start 27.6V → langsung FLOAT, PWM=0 (arus≈0), hysteresis float.
 *      2) Cold start 26V → BULK→CV→FLOAT berurutan; guard keluar BULK via timer
 *         dan fallback high-V; CV menjaga tegangan dengan band ±0.4V.
 *      3) PSU current limited → duty tidak naik tak terhingga; escape hatch
 *         aktif dan tercatat di dbg_limit_psu.
 *      4) PV dicabut saat BULK → PWM=0, flags reset, MPPT_Hybrid_Reset().
 *      5) PV dicabut saat FLOAT → saat PV kembali, check_initial_state() ulang
 *         (tetap FLOAT bila Vbat tinggi) dengan hysteresis rebulk 5 s.
 * ============================================================ */

/* Flag state FLOAT; dipisah agar mudah dicek oleh UI/pengendali. */
uint8_t flag_charging_FLOAT = 0;

/* Flag data ADC baru tersedia (disetel di adc_sampling.c). */
uint8_t flag_adc_done		= 0;
/* Flag bahwa charging boleh berjalan (dinyalakan setelah relay siap). */
uint8_t flag_enter_charge	= 0;

/* Flag tahapan charging aktif. */
uint8_t flag_charging_Bulk 	= 0;
uint8_t flag_charging_CV	= 0;

/* Telemetry ringan untuk mendeteksi “bench current limited” escape hatch. */
uint8_t dbg_limit_psu       = 0;

/* Memori PnO untuk membandingkan daya/tegangan langkah sebelumnya. */
uint16_t prevPowerInput		= 0;
uint16_t prevVoltagePv		= 0;


//=======================================================
/* ============================================================
 *  MPPT Hybrid (PnO Startup -> GOA refine) - STM32
 *  - MPPT dipanggil tiap 10 ms (charging_flow()).
 *  - PnO yang kamu punya dipakai untuk "bring-up conduction"
 *    (karena PnO kamu terbukti robust di hardware).
 *  - GOA dipakai setelah arus benar-benar sudah masuk (konduksi).
 *
 *  REVIEW & HW-NOTES (2024-xx):
 *  - Loop 10 ms tidak mengandung blocking/malloc; satu-satunya jitter
 *    berasal dari RNG metaheuristik (sengaja). ISR ADC -> flag_adc_done
 *    sudah jadi gating utama.
 *  - Risiko wrap duty saat decrement PnO (uint16 underflow) diperbaiki
 *    dengan saturating-decrement agar tidak loncat ke MAX_PERIOD.
 *  - PnO hanya starter/anchor; setelah handover ke GOA tidak ada
 *    rollback kecuali PV/input benar-benar hilang (lihat reset).
 *  - State PnO (prev power/voltage) ikut di-reset supaya handoff antar
 *    siklus bersih dan deterministik.
 *
 *  KUNCI FIX dibanding versi bug:
 *  (1) PWM_VALUE dijadikan source of truth duty.
 *      duty_cycle SELALU disinkronkan ke PWM_VALUE sebelum update.
 *  (2) PV-loss detector pakai AND + debounce (bukan OR).
 *  (3) Reset state ketika charging disable / PV benar-benar hilang.
 * ============================================================ */

/* ---------- helper clamp ---------- */
static inline uint16_t clamp_u16(uint16_t x, uint16_t lo, uint16_t hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* ---------- RNG ringan (lebih “STM32-friendly” daripada rand()) ---------- */
static float rand01(void)
{
    static uint32_t seed = 2463534242u;
    seed = 1664525u * seed + 1013904223u;
    return (float)(seed & 0x00FFFFFFu) / 16777216.0f; // 0..~1
}

/* approx gaussian (cukup untuk metaheuristik) */
static float randn_approx(void)
{
    float u1 = rand01();
    float u2 = rand01();
    float u3 = rand01();
    return (u1 + u2 + u3 - 1.5f) * 1.1547f;
}

/* ============================================================
 *  PARAMETER TUNING (ikut MATLAB hybrid kamu)
 * ============================================================ */
#define N_GOAT              10

/* Domain duty dalam bentuk PWM (0..MAX_PERIOD=600) */
#define DUTY_LB_F           0.10f
#define DUTY_UB_F           0.60f

/* GOA parameters (sesuai MATLAB kamu) */
#define GOA_ALPHA           0.00125f
#define GOA_BETA            0.90f
#define GOA_JPROB           0.10f
#define GOA_STAG_LIMIT      10
#define GOA_REPLACE_RATIO   0.15f
#define GOA_IMPROVE_TH      0.02f     // 2% improve

/* “near convergence” (sesuai MATLAB) */
#define CONV_WINDOW         0.12f
#define CONV_COUNT_LIMIT    15

/* Slew limiter (dalam PWM count per 10 ms)
 * MATLAB: maxDeltaD = 1*Ts. Kalau Ts=10ms => 0.01 duty.
 * 0.01 * 600 = 6 count.
 */
#define MAX_DELTA_PWM       6u

/* Hold & averaging untuk evaluasi fitness goat
 * (biar power yang dinilai tidak random akibat ripple/noise)
 *  - settle 1 step = 10ms
 *  - avg 3 step = 30ms
 * Total per goat ~40ms -> 10 goat ~400ms per cycle
 */
#define EVAL_SETTLE_STEPS   1u
#define EVAL_AVG_STEPS      3u

/* ---------- PV-loss detector (DISPLAY UNIT) ----------
 * Karena unit dis_voltage_pv/dis_current_pv kamu “display-scaled”,
 * threshold juga harus di unit yang sama.
 *
 * Safe: PV dianggap hilang hanya jika Vpv rendah DAN Ipv rendah
 * selama beberapa siklus MPPT (debounce).
 */
#define PV_LOSS_V_TH        5u      // contoh: 0.5V kalau display unit 0.1V
#define PV_LOSS_I_TH        1u      // contoh: 0.1A kalau display unit 0.1A
#define PV_LOSS_COUNT_N     50u     // 50 * 10ms = 500ms

/* ---------- Conduction gate ----------
 * GOA jangan jalan sebelum ada arus masuk beneran.
 * Threshold pakai arus baterai (lebih “jujur” untuk charging).
 */
#define COND_IBAT_TH        1u      // contoh: 0.1A kalau unit 0.1A
#define COND_STABLE_N       10u     // 10 * 10ms = 100ms stabil

/* ---------- Bench PSU escape hatch (compile-time gate) ----------
 * Deteksi ketika arus sudah mentok, Vpv turun, tapi daya tidak naik.
 * Dipakai untuk menghentikan eskalasi duty yang sia-sia di PSU current-limited.
 *
 * NON-PSU (panel PV) MODE:
 *  - Set ENABLE_PSU_ESCAPE ke 0 (misal via -DENABLE_PSU_ESCAPE=0 atau
 *    #define di mppt.h) untuk mematikan heuristik ini jika sumber bukan PSU.
 */
#ifndef ENABLE_PSU_ESCAPE
#define ENABLE_PSU_ESCAPE 1
#endif

#if ENABLE_PSU_ESCAPE
#define PSU_SAG_DVPV_TH     8u      // 0.8V drop dari sampel sebelumnya
#define PSU_STALL_COUNT     80u     // 0.8s berturut-turut
#define PSU_RELAX_TICKS     200u    // 2.0s menahan duty agar tidak naik
#define PSU_RECOVER_DVPV    5u      // butuh recovery tegangan sebelum relaks dihapus
#endif

/* ============================================================
 *  STATE (persistent)
 * ============================================================ */
static uint8_t  hyb_isInit = 0;

/* phase flags */
static uint8_t  pno_active = 1;

/* PV loss debounce */
static uint16_t pv_loss_cnt = 0;

/* conduction stable debounce */
static uint16_t cond_cnt = 0;

/* escape hatch PSU current limited */
#if ENABLE_PSU_ESCAPE
static uint16_t psu_limit_cnt     = 0;
static uint16_t psu_limit_relax   = 0;
static uint16_t psu_limit_ceiling = 0;

/* trend memori untuk escape hatch (PV sag & power stall) */
static uint16_t Vpv_last = 0;
static uint32_t Ppv_last = 0;
#endif

/* GOA arrays */
static float goats_D[N_GOAT];
static float goats_P[N_GOAT];
static float goats_prevP[N_GOAT];
static uint8_t stag_cnt[N_GOAT];

/* GOA best */
static float gbest_D = 0.10f;
static float gbest_P = 0.0f;
static float lbest_D = 0.10f;
static float lbest_P = 0.0f;

/* GOA machine */
static uint8_t state_goa = 1;        // 1=EVAL, 2=UPDATE
static uint8_t goat_idx = 1;         // 1..N
static uint8_t prev_goat_idx = 0;    // 1..N
static uint16_t conv_counter = 0;
static uint8_t  t_goa = 1;

/* evaluator hold+avg */
static uint8_t  eval_holding = 0;
static uint8_t  eval_settle = 0;
static uint8_t  eval_avg_cnt = 0;
static uint32_t eval_sumP = 0;

/* duty memory */
static uint16_t Dold_pwm = 0;

/* Update histori PV untuk escape hatch PSU (no-op if disabled). */
#if ENABLE_PSU_ESCAPE
#define UPDATE_TRENDS(Vpv_now, Ppv_now) \
    do { Vpv_last = (Vpv_now); Ppv_last = (Ppv_now); } while (0)
#else
#define UPDATE_TRENDS(Vpv_now, Ppv_now) \
    do { (void)(Vpv_now); (void)(Ppv_now); } while (0)
#endif

/* ============================================================
 *  RESET FUNCTION (panggil saat PV putus / charging stop)
 * ============================================================ */
void MPPT_Hybrid_Reset(void)
{
    /* Tandai perlu inisialisasi ulang pada pemanggilan berikutnya. */
    hyb_isInit     = 0;
    /* Mulai lagi dari fase PnO agar konduksi aman. */
    pno_active     = 1;
    /* Hapus debounce PV loss & konduksi. */
    pv_loss_cnt    = 0;
    cond_cnt       = 0;

#if ENABLE_PSU_ESCAPE
    /* bersihkan escape hatch PSU */
    psu_limit_cnt     = 0;
    psu_limit_relax   = 0;
    psu_limit_ceiling = 0;
    dbg_limit_psu     = 0;
    Vpv_last          = 0;
    Ppv_last          = 0;
#endif

    /* Reset mesin GOA ke siklus awal. */
    state_goa      = 1;
    goat_idx       = 1;
    prev_goat_idx  = 0;
    conv_counter   = 0;
    t_goa          = 1;

    /* Bersihkan akumulator evaluasi fitness. */
    eval_holding   = 0;
    eval_settle    = 0;
    eval_avg_cnt   = 0;
    eval_sumP      = 0;

    /* Reset solusi terbaik ke batas bawah duty. */
    gbest_P        = 0.0f;
    gbest_D        = DUTY_LB_F;

    /* Reset memori PnO supaya tidak bawa sejarah tegangan/daya lama. */
    prevPowerInput = 0;
    prevVoltagePv  = 0;

    /* NOTE:
     * Jangan paksa PWM_VALUE=0 di reset function kalau kamu panggil reset
     * di kondisi tertentu. Kalau PV-loss hard, baru kamu matikan PWM.
     */
}

//=======================================================


#include <stdint.h>
#include "adc_sampling.h"   // extern uint16_t dis_voltage_pv, dis_current_pv;
#include "pwm.h"            // extern uint16_t PWM_VALUE; extern int duty_percent; #define MAX_PERIOD ...

/* ============================================================
 *  MPPT HYBRID MAIN
 * ============================================================ */
void MPPT_Hybrid(void)
{
    /* ========================================================
     * 0) Kalau charging tidak aktif, reset state dan keluar.
     *    Ini penting biar "previous value" tidak nyangkut
     *    saat PSU dimatiin / relay off.
     * ======================================================== */
    if (!flag_enter_charge || !flag_charging_Bulk) {
        /* Jika charging tidak diizinkan, jaga state bersih lalu keluar. */
        MPPT_Hybrid_Reset();
        return;
    }

    /* ========================================================
     * 1) Ambil data sensor (DISPLAY SCALED, konsisten dengan PnO)
     * ======================================================== */
    uint16_t Vpv  = dis_voltage_pv;  /* Tegangan PV display-scaled. */
    uint16_t Ipv  = dis_current_pv;  /* Arus PV display-scaled. */
    uint16_t Vbat = dis_voltage_bat; /* Tegangan baterai display-scaled. */
    uint16_t Ibat = dis_current_bat; /* Arus baterai display-scaled. */

    /* Hitung power 32-bit aman (tidak overflow) */
    uint32_t Ppv32 = (uint32_t)Vpv * (uint32_t)Ipv;

    /* ========================================================
     * 2) PV-loss detector (AND + debounce)
     *    Tujuan: kalau PV benar-benar putus, reset MPPT total.
     * ======================================================== */
    uint8_t pv_low = (Vpv <= PV_LOSS_V_TH) && (Ipv <= PV_LOSS_I_TH);

    if (pv_low) {
        if (pv_loss_cnt < PV_LOSS_COUNT_N) pv_loss_cnt++;
    } else {
        pv_loss_cnt = 0;
    }

    if (pv_loss_cnt >= PV_LOSS_COUNT_N) {
        /* PV benar-benar hilang -> matikan PWM + reset state */
        PWM_VALUE = 0;        /* Matikan PWM fisik. */
        duty_cycle = 0;       /* Sinkron duty internal. */
        duty_percent = 0;     /* Nol-kan persentase untuk UI. */

        MPPT_Hybrid_Reset();  /* Bersihkan state algoritma. */
        pv_loss_cnt = 0;      /* Reset debounce agar siap deteksi ulang. */
        UPDATE_TRENDS(Vpv, Ppv32);
        return;
    }

    /* ========================================================
     * 3) Sinkronisasi DUTY (FIX paling krusial)
     *    PWM_VALUE harus jadi satu-satunya truth.
     * ======================================================== */
    duty_cycle = PWM_VALUE;          /* Sinkron duty internal dengan nilai PWM terakhir. */

    /* ========================================================
     * 4) Proteksi charge (ikut gaya robust PnO kamu)
     *    Kalau overcurrent/overvoltage -> turunin duty 1 step dan keluar.
     * ======================================================== */
    if (dis_current_bat > MAX_CURRENT_CHARGE) {
        /* Kurangi duty satu langkah untuk meredam arus berlebih. */
        if (duty_cycle > 0) duty_cycle--;
        PWM_VALUE = duty_cycle;
        duty_percent = (PWM_VALUE * 100) / MAX_PERIOD;

        /* Balik ke PnO agar stabil setelah kondisi aman. */
        pno_active = 1;
        cond_cnt = 0;
        UPDATE_TRENDS(Vpv, Ppv32);
        return;
    }
    if (dis_voltage_bat > MAX_BATTERY_CHARGE) {
        /* Turunkan duty jika tegangan baterai melewati batas bulk. */
        if (duty_cycle > 0) duty_cycle--;
        PWM_VALUE = duty_cycle;
        duty_percent = (PWM_VALUE * 100) / MAX_PERIOD;

        /* Reset gating konduksi agar GOA tidak aktif saat proteksi. */
        pno_active = 1;
        cond_cnt = 0;
        UPDATE_TRENDS(Vpv, Ppv32);
        return;
    }

    /* ========================================================
     * 5) INIT (sekali)
     * ======================================================== */
    if (!hyb_isInit)
    {
        /* start minimal duty domain */
        uint16_t pwm_min = (uint16_t)(DUTY_LB_F * (float)MAX_PERIOD + 0.5f);
        uint16_t pwm_max = (uint16_t)(DUTY_UB_F * (float)MAX_PERIOD + 0.5f);

        /* kalau PWM masih 0, naikkan ke minimal domain */
        if (PWM_VALUE < pwm_min) {
            PWM_VALUE  = pwm_min;
            duty_cycle = pwm_min;
        }

        Dold_pwm = PWM_VALUE;

        /* init GOA population random domain */
        for (int i = 0; i < N_GOAT; i++) {
            goats_D[i]      = DUTY_LB_F + (DUTY_UB_F - DUTY_LB_F) * rand01();
            goats_P[i]      = 0.0f;
            goats_prevP[i]  = 0.0f;
            stag_cnt[i]     = 0;
        }

        /* anchor gbest ke duty real sekarang (bukan hardcode) */
        gbest_D = (float)PWM_VALUE / (float)MAX_PERIOD;
        gbest_P = (float)Ppv32;

        state_goa     = 1;
        goat_idx      = 1;
        prev_goat_idx = 0;
        conv_counter  = 0;
        t_goa         = 1;

        eval_holding  = 0;
        eval_settle   = 0;
        eval_avg_cnt  = 0;
        eval_sumP     = 0;

        /* mulai dari PnO startup (biar masuk konduksi dulu) */
        pno_active = 1;
        cond_cnt   = 0;

        hyb_isInit = 1;
    }

    /* ========================================================
     * 6) Conduction detection (harus ada arus masuk stabil)
     * ======================================================== */
    uint8_t is_conducting = (Ibat >= COND_IBAT_TH);

    if (is_conducting) {
        if (cond_cnt < COND_STABLE_N) cond_cnt++;
    } else {
        cond_cnt = 0;
    }

#if ENABLE_PSU_ESCAPE
    /* ========================================================
     * 7) Escape hatch: bench PSU current-limited (BULK only)
     *    - Deteksi: arus sudah dekat limit, Vpv turun, daya tidak naik.
     *    - Aksi: tahan duty (atau step down sedikit) + log flag ringan.
     * ======================================================== */
    uint8_t near_current_ceiling = (Ipv >= (uint16_t)(MAX_CURRENT_CHARGE - 1u)) || (Ibat >= (uint16_t)(MAX_CURRENT_CHARGE - 1u));
    uint8_t pv_sagging           = (Vpv_last > 0) && (Vpv + PSU_SAG_DVPV_TH < Vpv_last);
    uint8_t power_not_better     = (Ppv_last > 0) && (Ppv32 + (uint32_t)Ipv <= Ppv_last); /* tambah Ipv sebagai margin noise */

    if (near_current_ceiling && pv_sagging && power_not_better && PWM_VALUE > 0) {
        if (psu_limit_cnt < PSU_STALL_COUNT) psu_limit_cnt++;
    } else {
        if (psu_limit_cnt > 0) psu_limit_cnt--;
    }

    /* jika limit terdeteksi lama, bekukan duty agar tidak terus naik */
    if (psu_limit_cnt >= PSU_STALL_COUNT) {
        psu_limit_ceiling = PWM_VALUE;
        psu_limit_relax   = PSU_RELAX_TICKS;
        dbg_limit_psu     = 1;                 /* logging ringan untuk UI / debug */

        if (PWM_VALUE > 0) PWM_VALUE--;        /* redam 1 step supaya sag berhenti */
        duty_cycle   = PWM_VALUE;
        duty_percent = (PWM_VALUE * 100) / MAX_PERIOD;
    }

    /* selama relaksasi, jangan biarkan duty melampaui ceiling */
    if (psu_limit_relax > 0) {
        psu_limit_relax--;

        if (psu_limit_ceiling > 0 && PWM_VALUE > psu_limit_ceiling) {
            PWM_VALUE = psu_limit_ceiling;
            duty_cycle = PWM_VALUE;
        }

        /* lepas relaksasi hanya jika Vpv sudah recovery dan arus turun */
        if (!near_current_ceiling && (Vpv + PSU_RECOVER_DVPV) > Vpv_last) {
            psu_limit_relax   = 0;
            psu_limit_ceiling = 0;
            dbg_limit_psu     = 0;
        }
    } else if (psu_limit_cnt == 0) {
        /* limit sudah pulih total */
        psu_limit_ceiling = 0;
        dbg_limit_psu     = 0;
    }
#endif

    /* ========================================================
     * 8) PHASE 1: PnO proven kamu (startup / anti-stuck)
     *    - PnO akan “mendorong” duty sampai ada arus masuk stabil.
     * ======================================================== */
    if (pno_active)
    {
        /* jalankan PnO kamu (yang sudah proven robust) */
        MPPT_PnO();

        /* setelah MPPT_PnO(), duty_cycle & PWM_VALUE sudah di-set oleh PnO */
        duty_percent = (PWM_VALUE * 100) / MAX_PERIOD;

        /* Handoff ke GOA kalau arus sudah masuk stabil */
        if (cond_cnt >= COND_STABLE_N)
        {
            pno_active = 0;          // masuk GOA

            /* anchor GOA */
            gbest_D = (float)PWM_VALUE / (float)MAX_PERIOD;
            gbest_P = (float)Ppv32;

            /* reseed populasi di sekitar anchor (sesuai MATLAB) */
            float span  = 0.20f; // ±0.10
            float newLB = gbest_D - 0.5f * span;
            float newUB = gbest_D + 0.5f * span;
            if (newLB < DUTY_LB_F) newLB = DUTY_LB_F;
            if (newUB > DUTY_UB_F) newUB = DUTY_UB_F;

            for (int i = 0; i < N_GOAT; i++) {
                goats_D[i]      = newLB + (newUB - newLB) * rand01();
                goats_P[i]      = 0.0f;
                goats_prevP[i]  = 0.0f;
                stag_cnt[i]     = 0;
            }

            state_goa     = 1;
            goat_idx      = 1;
            prev_goat_idx = 0;
            conv_counter  = 0;
            t_goa         = 1;

            eval_holding  = 0;
            eval_settle   = 0;
            eval_avg_cnt  = 0;
            eval_sumP     = 0;

            /* jangan return, biar loop berikutnya GOA mulai rapi */
        }

        UPDATE_TRENDS(Vpv, Ppv32);
        return; // PHASE 1 selesai di sini
    }

    /* ========================================================
     * 9) PHASE 2: GOA refine (MATLAB-like) dengan hold+avg fitness
     * ======================================================== */
    uint16_t pwm_min = (uint16_t)(DUTY_LB_F * (float)MAX_PERIOD + 0.5f);
    uint16_t pwm_max = (uint16_t)(DUTY_UB_F * (float)MAX_PERIOD + 0.5f);

    /* ---------- STATE 1: EVALUATE ---------- */
    if (state_goa == 1)
    {
        /* Kalau sedang hold duty goat -> settle/avg power */
        if (eval_holding)
        {
            if (eval_settle > 0) {
                eval_settle--;
            } else {
                eval_sumP += Ppv32;
                eval_avg_cnt++;

                if (eval_avg_cnt >= EVAL_AVG_STEPS)
                {
                    float P_fit = (float)eval_sumP / (float)EVAL_AVG_STEPS;

                    if (prev_goat_idx >= 1 && prev_goat_idx <= N_GOAT) {
                        goats_P[prev_goat_idx - 1] = P_fit;
                    }

                    eval_holding = 0;
                    eval_sumP    = 0;
                    eval_avg_cnt = 0;
                    prev_goat_idx = 0;
                }
            }

            UPDATE_TRENDS(Vpv, Ppv32);
            return; // selama holding, jangan ubah PWM lagi
        }

        /* Pilih goat berikutnya */
        if (goat_idx > N_GOAT) {
            state_goa = 2; // masuk UPDATE
        } else {
            /* set PWM sesuai duty goat */
            float Dg = goats_D[goat_idx - 1];
            if (Dg < DUTY_LB_F) Dg = DUTY_LB_F;
            if (Dg > DUTY_UB_F) Dg = DUTY_UB_F;

            uint16_t pwm_cmd = (uint16_t)(Dg * (float)MAX_PERIOD + 0.5f);
            pwm_cmd = clamp_u16(pwm_cmd, pwm_min, pwm_max);
#if ENABLE_PSU_ESCAPE
            if (psu_limit_relax > 0 && psu_limit_ceiling > 0 && pwm_cmd > psu_limit_ceiling) {
                pwm_cmd = psu_limit_ceiling;   /* honor escape hatch ceiling */
            }
#endif

            /* slew limit biar nggak brutal */
            int diff = (int)pwm_cmd - (int)Dold_pwm;
            uint16_t maxStep = (conv_counter >= CONV_COUNT_LIMIT) ? 1u : MAX_DELTA_PWM;
            if (diff > (int)maxStep) pwm_cmd = Dold_pwm + maxStep;
            else if (diff < -(int)maxStep) pwm_cmd = Dold_pwm - maxStep;

            PWM_VALUE   = pwm_cmd;
            duty_cycle  = pwm_cmd; // sync
            duty_percent = (PWM_VALUE * 100) / MAX_PERIOD;
            Dold_pwm    = pwm_cmd;

            prev_goat_idx = goat_idx;
            goat_idx++;

            eval_holding = 1;
            eval_settle  = EVAL_SETTLE_STEPS;
            eval_avg_cnt = 0;
            eval_sumP    = 0;

            UPDATE_TRENDS(Vpv, Ppv32);
            return;
        }
    }

    /* ---------- STATE 2: UPDATE ---------- */
    if (state_goa == 2)
    {
        /* (1) local best */
        lbest_P = goats_P[0];
        lbest_D = goats_D[0];
        for (int i = 1; i < N_GOAT; i++) {
            if (goats_P[i] > lbest_P) {
                lbest_P = goats_P[i];
                lbest_D = goats_D[i];
            }
        }

        /* (2) update global best jika improve > 2% */
        if (gbest_P > 0.0f) {
            float rel = (lbest_P - gbest_P) / gbest_P;
            if (rel > GOA_IMPROVE_TH) {
                gbest_P = lbest_P;
                gbest_D = lbest_D;
            }
        } else {
            gbest_P = lbest_P;
            gbest_D = lbest_D;
        }

        /* (3) init goats_prevP sekali */
        uint8_t all_zero = 1;
        for (int i = 0; i < N_GOAT; i++) {
            if (goats_prevP[i] != 0.0f) { all_zero = 0; break; }
        }
        if (all_zero) {
            for (int i = 0; i < N_GOAT; i++) goats_prevP[i] = goats_P[i];
        }

        /* (4) near convergence by D_span */
        float dmax = goats_D[0], dmin = goats_D[0];
        for (int i = 1; i < N_GOAT; i++) {
            if (goats_D[i] > dmax) dmax = goats_D[i];
            if (goats_D[i] < dmin) dmin = goats_D[i];
        }
        float D_span = dmax - dmin;
        if (D_span < CONV_WINDOW) conv_counter++;
        else conv_counter = 0;

        uint8_t isNear = (conv_counter >= CONV_COUNT_LIMIT);

        /* (5) effective params (MATLAB-like) */
        float alpha_eff = isNear ? (0.05f * GOA_ALPHA) : GOA_ALPHA;
        float jprob_eff = isNear ? (0.6f  * GOA_JPROB) : GOA_JPROB;
        float repl_eff  = isNear ? 0.0f : GOA_REPLACE_RATIO;

        float beta_eff  = (t_goa < 5) ? (0.5f * GOA_BETA) : GOA_BETA;

        /* (6) update goats */
        float BW = (DUTY_UB_F - DUTY_LB_F);

        for (int i = 0; i < N_GOAT; i++)
        {
            /* stagnation counter */
            if (goats_P[i] < goats_prevP[i]) {
                if (stag_cnt[i] < GOA_STAG_LIMIT) stag_cnt[i]++;
            } else {
                stag_cnt[i] = 0;
            }

            float stagn_norm = (float)stag_cnt[i] / (float)GOA_STAG_LIMIT;
            if (stagn_norm > 1.0f) stagn_norm = 1.0f;

            float jump_prob = jprob_eff + 0.5f * stagn_norm;
            if (jump_prob > 0.9f) jump_prob = 0.9f;

            float Dnew = goats_D[i];

            /* exploit */
            Dnew += beta_eff * rand01() * (gbest_D - Dnew);

            /* explore */
            Dnew += alpha_eff * randn_approx() * BW;

            /* jump */
            if (rand01() < jump_prob) {
                int j = (int)(rand01() * (float)N_GOAT);
                if (j >= N_GOAT) j = N_GOAT - 1;
                Dnew += rand01() * (goats_D[j] - Dnew);
                stag_cnt[i] = 0;
            }

            /* clamp */
            if (Dnew < DUTY_LB_F) Dnew = DUTY_LB_F;
            if (Dnew > DUTY_UB_F) Dnew = DUTY_UB_F;

            goats_D[i] = Dnew;
        }

        /* (7) parasite avoidance */
        if (repl_eff > 0.0f)
        {
            int num_worst = (int)floorf(repl_eff * (float)N_GOAT);
            if (num_worst < 1) num_worst = 1;

            for (int k = 0; k < num_worst; k++) {
                int worst_i = 0;
                float worstP = goats_P[0];
                for (int i = 1; i < N_GOAT; i++) {
                    if (goats_P[i] < worstP) { worstP = goats_P[i]; worst_i = i; }
                }
                goats_D[worst_i] = DUTY_LB_F + BW * rand01();
            }
        }

        for (int i = 0; i < N_GOAT; i++) goats_prevP[i] = goats_P[i];

        t_goa++; if (t_goa > 10) t_goa = 1;

        /* next cycle */
        state_goa     = 1;
        goat_idx      = 1;
        prev_goat_idx = 0;

        /* command gbest (apply PWM with slew) */
        float Dcmd = gbest_D;
        if (Dcmd < DUTY_LB_F) Dcmd = DUTY_LB_F;
        if (Dcmd > DUTY_UB_F) Dcmd = DUTY_UB_F;

        uint16_t pwm_cmd = (uint16_t)(Dcmd * (float)MAX_PERIOD + 0.5f);
        pwm_cmd = clamp_u16(pwm_cmd, pwm_min, pwm_max);
#if ENABLE_PSU_ESCAPE
        if (psu_limit_relax > 0 && psu_limit_ceiling > 0 && pwm_cmd > psu_limit_ceiling) {
            pwm_cmd = psu_limit_ceiling;
        }
#endif

        int diff = (int)pwm_cmd - (int)Dold_pwm;
        uint16_t maxStep = (conv_counter >= CONV_COUNT_LIMIT) ? 1u : MAX_DELTA_PWM;
        if (diff > (int)maxStep) pwm_cmd = Dold_pwm + maxStep;
        else if (diff < -(int)maxStep) pwm_cmd = Dold_pwm - maxStep;

        PWM_VALUE   = pwm_cmd;
        duty_cycle  = pwm_cmd; // sync
        duty_percent = (PWM_VALUE * 100) / MAX_PERIOD;
        Dold_pwm    = pwm_cmd;
        UPDATE_TRENDS(Vpv, Ppv32);
        return;
    }

    /* fallback kalau state corrupt -> balik PnO */
    UPDATE_TRENDS(Vpv, Ppv32);
    pno_active = 1;
}
// =============================================================


/* Perturb-and-Observe sederhana untuk fase startup/anti-stuck. */
void MPPT_PnO(void) {
	/* Proteksi cepat berbasis arus/tegangan baterai. */
	if(dis_current_bat > MAX_CURRENT_CHARGE)		{ if (duty_cycle > 0) duty_cycle--; }
	else if(dis_voltage_bat > MAX_BATTERY_CHARGE)	{ if (duty_cycle > 0) duty_cycle--; }
	else {
		/* Bandingkan daya/tegangan saat ini dengan sebelumnya
		 * untuk memutuskan arah perturbasi duty. */
		if(dis_power_pv > prevPowerInput && dis_voltage_pv > prevVoltagePv)		{ if (duty_cycle > 0) duty_cycle--; }
		else if(dis_power_pv > prevPowerInput && dis_voltage_pv < prevVoltagePv)	{ duty_cycle++; }
		else if(dis_power_pv < prevPowerInput && dis_voltage_pv > prevVoltagePv)	{ duty_cycle++; }
		else if(dis_power_pv < prevPowerInput && dis_voltage_pv < prevVoltagePv)	{ if (duty_cycle > 0) duty_cycle--; }
		else if(dis_voltage_bat < MAX_BATTERY_CHARGE)								{ duty_cycle++; }

		/* Simpan daya/tegangan sebagai referensi langkah berikutnya. */
		prevPowerInput = dis_power_pv;
		prevVoltagePv = dis_voltage_pv;
	}
	/* Jaga duty dalam batas PWM yang diizinkan. */
	if(duty_cycle >= MAX_PERIOD) {
		duty_cycle = MAX_PERIOD;
	}
	else if(duty_cycle <= 0) {
		duty_cycle = 0;
	}
    /* jika escape hatch PSU aktif, jangan melewati ceiling */
#if ENABLE_PSU_ESCAPE
    if (psu_limit_relax > 0 && psu_limit_ceiling > 0 && duty_cycle > psu_limit_ceiling) {
        duty_cycle = psu_limit_ceiling;
    }
#endif
	/* Propagasi hasil perturbasi ke register PWM & persen untuk UI. */
	PWM_VALUE = duty_cycle;
	duty_percent = (PWM_VALUE * 100) / MAX_PERIOD;
}
/* ============================================================
 *  CHARGING FLOW ROBUST (deci-Volt & deci-Amp: 0.1V, 0.1A)
 *  MPPT dipanggil tiap 10 ms
 *  - BULK: MPPT (Hybrid / PnO)
 *  - CV  : jaga Vabs
 *  - FLOAT: jaga Vfloat
 *  Transisi pakai hysteresis + "counter naik/turun" (anti noise)
 * ============================================================ */

/* ---------- ABSORPTION (24V VRLA, 2x12V 7Ah) ----------
 * Tegangan/arus berbasis unit tampilan (0.1V / 0.1A).
 * Hysteresis + counter dipakai untuk meredam ripple sensing kecil
 * supaya tidak membatalkan perpindahan state.
 */
#define VABS_SET            288u   // 28.8V
#define VABS_ENTER_MIN      284u   // 28.4V (lebih realistis untuk "mulai CV")
#define VABS_HYST           4u     // 0.4V band (anti hunting)

/* ---------- FLOAT ---------- */
#define VFLT_SET            272u   // 27.2V
#define VFLT_HYST           3u     // 0.3V

/* ---------- Current thresholds (max charge 2.2A elsewhere) ---------- */
#define I_MIN_TO_ENTER_CV   3u     // 0.3A (pastikan memang ada charging)
#define I_END_ABS           4u     // 0.4A (kompensasi resolusi 0.1A)
#define I_IDLE              2u     // 0.2A (anggap tidak charging)

/* ---------- Full-battery at startup fallback (BULK -> FLOAT) ---------- */
#define V_BULK_TO_FLOAT_MIN 274u   // 27.4V (baterai sudah tinggi)

/* ---------- Rebulk ---------- */
#define V_REBULK            REBULK_VOLTAGE   // pakai define kamu (25.5V)

/* ---------- Timer ticks (10ms per tick) ---------- */
#define T_ENTER_CV_TICKS    1000u  // 10.0s (hindari flapping)
#define T_TO_FLOAT_TICKS    100u   // 1.0s
#define T_BULK_FLOAT_TICKS  300u   // 3.0s (cukup 3–10 detik sesuai request)
#define T_REBULK_TICKS      500u   // 5.0s
#define T_ABS_MIN_DWELL_TICKS 3000u // 30.0s minimum di absorption sebelum keluar

/* ---------- helper counter anti noise (naik cepat, turun pelan) ---------- */
static inline uint16_t cnt_updown(uint16_t cnt, uint8_t ok, uint16_t cnt_max)
{
    if (ok) {
        if (cnt < cnt_max) cnt++;
    } else {
        if (cnt > 0) cnt--;   // turun pelan, bukan reset 0
    }
    return cnt;
}

/* ---------- Charging timers (10 ms tick) ---------- */
static uint16_t t_enter_cv    = 0;
static uint16_t t_to_float    = 0;
static uint16_t t_bulk_float  = 0;
static uint16_t t_rebulk      = 0;
static uint16_t t_abs_dwell   = 0;
static uint16_t t_bulk_highV  = 0;

/* timestamp 10ms tick untuk catat waktu masuk CV */
static uint32_t t_flow_ticks      = 0;
static uint32_t t_abs_enter_ticks = 0;

/* latch PV hilang untuk re-evaluasi state saat PV kembali */
static uint8_t pv_absent_latched = 0;

/* ============================================================
 *  INITIAL STATE SELECTION (dipanggil saat relay bat di-on-kan)
 *  Menentukan titik awal charging berdasarkan tegangan baterai.
 * ============================================================ */
void check_initial_state(void)
{
    /* reset seluruh mesin MPPT */
    MPPT_Hybrid_Reset();

    /* default: matikan semua state dulu */
    flag_charging_Bulk  = 0;
    flag_charging_CV    = 0;
    flag_charging_FLOAT = 0;

    /* pembacaan dalam unit display (0.1V) */
    uint16_t Vbat = dis_voltage_bat;

    /*
     * Hysteresis start-up (anti flapping di sekitar ambang):
     *  - ≥28.4V -> mulai di CV (absorption)
     *  - 27.5V..28.3V -> FLOAT (baterai sudah tinggi)
     *  - lainnya -> BULK
     */
    t_flow_ticks      = 0;
    t_abs_enter_ticks = 0;

    if (Vbat >= VABS_ENTER_MIN) {
        /* mendekati/past absorption -> mulai di CV */
        flag_charging_CV = 1;
        t_abs_enter_ticks = t_flow_ticks;  /* pastikan dwell dihitung dari start CV */
        /* mulai dengan duty rendah agar tidak langsung overshoot */
        PWM_VALUE    = (uint16_t)(0.05f * (float)MAX_PERIOD);
        duty_cycle   = PWM_VALUE;
        duty_percent = (PWM_VALUE * 100) / MAX_PERIOD;
    } else if (Vbat >= 275u) {
        /* baterai sudah tinggi -> langsung FLOAT */
        flag_charging_FLOAT = 1;
        PWM_VALUE    = 0;    /* biarkan mengapung, duty nanti naik perlahan bila perlu */
        duty_cycle   = 0;
        duty_percent = 0;
    } else {
        /* default: mulai BULK (MPPT) */
        flag_charging_Bulk = 1;
        PWM_VALUE    = 0;
        duty_cycle   = 0;
        duty_percent = 0;
    }

    /* izinkan charging */
    flag_enter_charge = 1;
}

void charging_flow(void)
{
    if (!flag_adc_done) {
        /* pastikan counter transisi tidak nyangkut saat charging tidak aktif */
        t_enter_cv   = 0;
        t_to_float   = 0;
        t_bulk_float = 0;
        t_rebulk     = 0;
        t_bulk_highV = 0;
        t_flow_ticks = 0;
        t_abs_enter_ticks = 0;

        flag_adc_done = 0;
        return;
    }

    /* baca sensor */
    uint16_t Vbat = dis_voltage_bat;   // 0.1V
    uint16_t Ibat = dis_current_bat;   // 0.1A
    t_flow_ticks++;                    // tiap 10 ms

    /* standby jika input PV/PSU benar-benar tidak ada */
    uint8_t pv_absent = (dis_voltage_pv <= PV_LOSS_V_TH) && (dis_current_pv <= PV_LOSS_I_TH);
    if (pv_absent) {
        flag_charging_Bulk  = 0;
        flag_charging_CV    = 0;
        flag_charging_FLOAT = 0;
        flag_enter_charge   = 0;

        PWM_VALUE    = 0;
        duty_cycle   = 0;
        duty_percent = 0;

        MPPT_Hybrid_Reset();

        t_enter_cv   = 0;
        t_to_float   = 0;
        t_bulk_float = 0;
        t_rebulk     = 0;
        t_bulk_highV = 0;
        t_flow_ticks = 0;
        t_abs_enter_ticks = 0;
        pv_absent_latched = 1;

        flag_adc_done = 0;
        return;
    }
    else {
        /* PV sudah kembali; jika sebelumnya absent, evaluasi ulang state awal. */
        if (pv_absent_latched && !flag_enter_charge) {
            pv_absent_latched = 0;
            check_initial_state();
        } else {
            pv_absent_latched = 0;
        }
    }

    /* jika charging belum diizinkan (mis. relay belum siap), jangan lanjut loop */
    if (!flag_enter_charge) {
        t_enter_cv   = 0;
        t_to_float   = 0;
        t_bulk_float = 0;
        t_rebulk     = 0;
        t_bulk_highV = 0;
        t_abs_enter_ticks = 0;
        flag_adc_done = 0;
        return;
    }

    /* ===================== STAGE 1: BULK (MPPT) ===================== */
    if (flag_charging_Bulk)
    {
        MPPT_Hybrid();  // atau MPPT_PnO() untuk test A/B

        /* A) Masuk CV kalau Vbat cukup tinggi dan arus masih ada */
        uint8_t ok_voltage_cv = (Vbat >= VABS_ENTER_MIN);
        uint8_t ok_enter_cv   = ok_voltage_cv && (Ibat >= I_MIN_TO_ENTER_CV); // arus hanya untuk memastikan masih charging
        t_enter_cv = cnt_updown(t_enter_cv, ok_enter_cv, T_ENTER_CV_TICKS);

        if (t_enter_cv >= T_ENTER_CV_TICKS) {
            flag_charging_Bulk  = 0;
            flag_charging_CV    = 1;
            flag_charging_FLOAT = 0;

            /* reset timer lain */
            t_enter_cv   = 0;
            t_to_float   = 0;
            t_bulk_float = 0;
            t_rebulk     = 0;
            t_abs_dwell  = 0;
            t_abs_enter_ticks = t_flow_ticks;

            /* reset MPPT state biar nggak nyangkut kalau nanti balik BULK */
            MPPT_Hybrid_Reset();
        }

        /* B) BULK -> FLOAT fallback:
              baterai sudah tinggi tapi arus nol (baterai penuh / tidak bisa nyedot lagi)
              Stabil 5 detik -> masuk FLOAT */
        uint8_t ok_bulk_float = (Vbat >= V_BULK_TO_FLOAT_MIN) && (Ibat <= I_IDLE);
        t_bulk_float = cnt_updown(t_bulk_float, ok_bulk_float, T_BULK_FLOAT_TICKS);
        uint8_t ok_bulk_highV = (Vbat > (uint16_t)(VFLT_SET + 5u));
        t_bulk_highV = cnt_updown(t_bulk_highV, ok_bulk_highV, 3000u); // 30s high-V guard

        if (t_bulk_float >= T_BULK_FLOAT_TICKS) {
            flag_charging_Bulk  = 0;
            flag_charging_CV    = 0;
            flag_charging_FLOAT = 1;

            t_bulk_float = 0;
            t_enter_cv   = 0;
            t_to_float   = 0;
            t_rebulk     = 0;
            t_bulk_highV = 0;

            MPPT_Hybrid_Reset();
        }
        /* escape hatch jika Vbat tinggi tetapi arus tidak pernah memenuhi syarat CV/FLOAT */
        else if (t_bulk_highV >= 3000u) {
            flag_charging_Bulk  = 0;
            flag_charging_CV    = 0;
            flag_charging_FLOAT = 1;

            t_bulk_highV = 0;
            t_bulk_float = 0;
            t_enter_cv   = 0;
            t_to_float   = 0;
            t_rebulk     = 0;
            t_abs_enter_ticks = t_flow_ticks; // start float timing cleanly

            MPPT_Hybrid_Reset();
        }
    }

    /* ===================== STAGE 2: ABSORPTION (CV) ===================== */
    else if (flag_charging_CV)
    {
        /* hitung durasi berada di CV (10ms tick) dari timestamp masuk */
        t_abs_dwell = (uint16_t)(t_flow_ticks - t_abs_enter_ticks);

        /* kontrol CV dengan hysteresis */
        if (Vbat < (uint16_t)(VABS_SET - VABS_HYST)) {
            if (PWM_VALUE < MAX_PERIOD) PWM_VALUE++;
        } else if (Vbat > (uint16_t)(VABS_SET + VABS_HYST)) {
            if (PWM_VALUE > 0) PWM_VALUE--;
        }

        /* tetap hormati limit arus */
        if (dis_current_bat > MAX_CURRENT_CHARGE) {
            if (PWM_VALUE > 0) PWM_VALUE--;
        }

        duty_cycle   = PWM_VALUE;
        duty_percent = (PWM_VALUE * 100) / MAX_PERIOD;

        /* Masuk FLOAT kalau:
           - Vbat tetap tinggi (>= 28.4V)
           - arus sudah kecil (<= 0.3A)
           - sudah cukup lama di absorption (anti loncat cepat)
           stabil 5 detik (pakai counter anti noise) */
        uint8_t ok_to_float = (Vbat >= VABS_ENTER_MIN) && (Ibat <= I_END_ABS);
        uint8_t dwell_met   = (t_abs_dwell >= T_ABS_MIN_DWELL_TICKS);

        if (dwell_met) {
            t_to_float = cnt_updown(t_to_float, ok_to_float, T_TO_FLOAT_TICKS);
        } else {
            t_to_float = 0;
        }

        if (t_to_float >= T_TO_FLOAT_TICKS) {
            flag_charging_Bulk  = 0;
            flag_charging_CV    = 0;
            flag_charging_FLOAT = 1;

            t_to_float = 0;
            t_rebulk   = 0;
            t_abs_dwell = 0;
        }
    }

    /* ===================== STAGE 3: FLOAT ===================== */
    else if (flag_charging_FLOAT)
    {
        /* kontrol float dengan hysteresis */
        if (Vbat < (uint16_t)(VFLT_SET - VFLT_HYST)) {
            if (PWM_VALUE < MAX_PERIOD) PWM_VALUE++;
        } else if (Vbat > (uint16_t)(VFLT_SET + VFLT_HYST)) {
            if (PWM_VALUE > 0) PWM_VALUE--;
        }

        /* optional: current limit tetap */
        if (dis_current_bat > MAX_CURRENT_CHARGE) {
            if (PWM_VALUE > 0) PWM_VALUE--;
        }

        duty_cycle   = PWM_VALUE;
        duty_percent = (PWM_VALUE * 100) / MAX_PERIOD;

        /* rebulk kalau drop stabil 2 detik */
        uint8_t ok_rebulk = (Vbat < V_REBULK);
        t_rebulk = cnt_updown(t_rebulk, ok_rebulk, T_REBULK_TICKS);

        if (t_rebulk >= T_REBULK_TICKS) {
            flag_charging_FLOAT = 0;
            flag_charging_Bulk  = 1;
            flag_charging_CV    = 0;

            t_rebulk     = 0;
            t_enter_cv   = 0;
            t_to_float   = 0;
            t_bulk_float = 0;

            MPPT_Hybrid_Reset();
        }
    }

    flag_adc_done = 0;
}
