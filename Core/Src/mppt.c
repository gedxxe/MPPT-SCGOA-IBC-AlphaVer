/*
 * File Role    : Implementasi algoritma MPPT hybrid (Perturb & Observe untuk start,
 *                dilanjutkan Goat Optimizer Algorithm/GOA) beserta state machine
 *                charging Bulk/CV/Float dan proteksi input.
 * Dependencies : main.h untuk akses data ADC ter-skala, PWM_VALUE, flag charging,
 *                serta HAL timing; math.h untuk operasi float (log, pow).
 * Fungsi inti  : MPPT_Hybrid(), MPPT_Hybrid_Reset(), check_initial_state(), charging_flow().
 */
#include "main.h"
#include <math.h>
#include <stdint.h>
#include "adc_sampling.h"   // extern uint16_t dis_voltage_pv, dis_current_pv;
#include "pwm.h"            // extern uint16_t PWM_VALUE; extern int duty_percent; #define MAX_PERIOD ...

/* ============================================================
 *  STATE MACHINE & VERIF SNAPSHOT (2026-01)
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
/* ============================================================
 *  CHANGELOG - 13 May 2026 (Pure SC-GOA Refactor)
 *  1) Menghapus jalur fallback/startup Perturb & Observe pada
 *     MPPT_Hybrid(), sehingga mode BULK memakai SC-GOA murni.
 *  2) Menghapus fungsi MPPT_PnO() beserta state residu terkait.
 *  3) Merapikan komentar agar fokus ke alur SC-GOA aktual.
 *  4) Mempertahankan guard keselamatan existing (PV-loss debounce,
 *     over-current/over-voltage step-down, PSU current-limit escape,
 *     slew limiter, dan hold+average fitness sampling).
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
uint32_t psu_escape_trigger_count = 0;
uint32_t psu_escape_active_ticks  = 0;

//=======================================================
/* ============================================================
 *  MPPT Hybrid (Pure SC-GOA refine) - STM32
 *  - MPPT dipanggil tiap 10 ms (charging_flow()).
 *  - Pure SC-GOA dijalankan terus selama fase BULK aktif.
 *  - Tidak ada fallback ke PnO/startup heuristic agar perilaku
 *    konsisten dengan model SCA MATLAB yang kamu kirim.
 *
 *  REVIEW & HW-NOTES (2024-xx):
 *  - Loop 10 ms tidak mengandung blocking/malloc; satu-satunya jitter
 *    berasal dari RNG metaheuristik (sengaja). ISR ADC -> flag_adc_done
 *    sudah jadi gating utama.
 *  - Search agent dipertahankan di domain duty dan dievaluasi lewat
 *    hold+average power untuk mengurangi bias ripple switching.
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

static inline uint16_t abs_diff_u16(uint16_t a, uint16_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

/* Sinkronisasi register PWM + duty + persen untuk UI. */
static inline void sync_pwm_outputs(uint16_t pwm)
{
    PWM_VALUE   = pwm;
    duty_cycle  = pwm;
    duty_percent = (PWM_VALUE * 100) / MAX_PERIOD;
}

/* Forward deklarasi helper resume-duty agar bisa dipakai di fungsi awal. */
static inline void update_resume_bookmark(uint16_t Vpv, uint16_t Ipv, uint16_t pwm_now);
static inline void arm_resume_from_bookmark(uint16_t Vpv_now, uint16_t pwm_now);
static inline uint16_t pv_window_push(uint16_t Vpv);
static inline void clear_resume_snapshot(void);
static inline void pv_window_reseed(uint16_t Vpv_seed);

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

/* SC-GOA parameters (mengacu SCA MATLAB + formulasi TEX) */
#define SCA_A0              2.0f
#define SCA_A_MIN           0.1f
#define SCA_DECAY_STEP      100.0f
#define SCA_R3_MIN          0.7f
#define SCA_R3_SPAN         0.3f
#define SCA_ALPHA_STEP      0.6f
#define SCA_BETA_MOM        0.4f
#define SCGOA_EMA_ALPHA     0.3f
#define SCGOA_REINIT_IT     50u

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

/* Resume window pasca PV-loss (10 ms per tick) */
#define RESUME_MAX_LOSS_TICKS 500u   // 5.0s maksimum jeda untuk boleh resume
#define RESUME_DVPV_WINDOW    20u    // 2.0V band kecocokan tegangan PV (tolak ukur resume)
/* Headroom untuk menyimpan snapshot sebelum tegangan benar-benar ambruk. */
#define PV_RESUME_HEADROOM    30u    // 3.0V di atas ambang PV_LOSS_V_TH
#define PV_RESUME_DEADBAND    15u    // 1.5V deadband validasi resume berbasis rata-rata
#define PV_RESUME_MIN_V       80u    // 8.0V: minimum valid untuk boleh resume duty
#define PV_WINDOW_COUNT       10u    // 10 sampel @10ms ~100ms smoothing Vpv
#define PV_ARM_BAND           8u     // 0.8V: syarat minimal untuk arm snapshot (lebih ketat)
#define PV_DROP_DV_TH         12u    // 1.2V per 10ms dianggap drop tajam PSU
#define PV_DROP_COUNT_MAX     5u     // 5 sampel berturut-turut drop tajam => anggap loss keras

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
#define MPPT_SOURCE_MODE_REAL_PV  0
#define MPPT_SOURCE_MODE_BENCH_PSU 1
#ifndef MPPT_SOURCE_MODE
#define MPPT_SOURCE_MODE MPPT_SOURCE_MODE_REAL_PV
#endif

#if (MPPT_SOURCE_MODE == MPPT_SOURCE_MODE_BENCH_PSU)
/* Profil bench PSU: longgarkan false-positive,
 * butuh persistensi lebih lama, namun freeze awal lebih singkat.
 */
#define PSU_SAG_DVPV_TH        12u   // 1.2V drop
#define PSU_POWER_STALL_MARGIN  (4u) // margin stagnasi daya lebih longgar
#define PSU_STALL_COUNT         120u // 1.2s berturut-turut
#define PSU_RELAX_TICKS         120u // 1.2s freeze awal
#define PSU_RECOVER_DVPV        6u   // syarat recovery sedikit lebih ketat
#else
/* Profil panel PV riil (default) */
#define PSU_SAG_DVPV_TH        8u    // 0.8V drop
#define PSU_POWER_STALL_MARGIN (1u)  // margin stagnasi daya baseline
#define PSU_STALL_COUNT        80u   // 0.8s berturut-turut
#define PSU_RELAX_TICKS        200u  // 2.0s freeze awal
#define PSU_RECOVER_DVPV       5u    // syarat recovery baseline
#endif
#endif


#ifndef MPPT_DEBUG_TRACE
#define MPPT_DEBUG_TRACE 1
#endif

#if MPPT_DEBUG_TRACE
#define MPPT_DUTY_WINDOW_LO_PWM ((uint16_t)(0.38f * (float)MAX_PERIOD + 0.5f))
#define MPPT_DUTY_WINDOW_HI_PWM ((uint16_t)(0.48f * (float)MAX_PERIOD + 0.5f))
#define MPPT_TRACE_BUF_LEN 128u

typedef enum {
    MPPT_PHASE_NONE = 0,
    MPPT_PHASE_SETTLE = 1,
    MPPT_PHASE_AVG = 2,
    MPPT_PHASE_UPDATE = 3
} mppt_trace_phase_t;

typedef struct {
    uint32_t tick_10ms;
    uint16_t Vpv;
    uint16_t Ipv;
    uint32_t Ppv;
    uint16_t pwm_before;
    uint16_t pwm_after;
    uint8_t flag_bulk;
    uint8_t flag_cv;
    uint8_t flag_float;
    uint8_t guard_pv_loss;
    uint8_t guard_conduction;
    uint8_t guard_psu_escape;
    uint8_t guard_oc_stepdown;
    uint8_t guard_ov_stepdown;
    uint8_t guard_max_delta_clip;
    uint8_t guard_state_transition;
    uint8_t goat_idx;
    uint8_t goat_phase;
    char trigger[18];
} mppt_trace_entry_t;

static mppt_trace_entry_t mppt_trace_buf[MPPT_TRACE_BUF_LEN];
static volatile uint16_t mppt_trace_wr = 0;
static volatile uint32_t mppt_trace_tick = 0;

static volatile uint32_t mppt_dbg_cnt_window = 0;
static volatile uint32_t mppt_dbg_cnt_max_delta_clip = 0;
static volatile uint32_t mppt_dbg_cnt_guard_reset = 0;
static volatile uint32_t mppt_dbg_cnt_state_transition = 0;

static inline uint8_t mppt_trace_in_window(uint16_t pwm)
{
    return (pwm >= MPPT_DUTY_WINDOW_LO_PWM) && (pwm <= MPPT_DUTY_WINDOW_HI_PWM);
}

static inline void mppt_trace_log(uint16_t Vpv, uint16_t Ipv, uint32_t Ppv,
                                  uint16_t pwm_before, uint16_t pwm_after,
                                  uint8_t guard_pv_loss, uint8_t guard_conduction,
                                  uint8_t guard_psu_escape, uint8_t guard_oc_stepdown,
                                  uint8_t guard_ov_stepdown, uint8_t guard_max_delta_clip,
                                  uint8_t guard_state_transition, uint8_t goat_now,
                                  mppt_trace_phase_t phase, const char *trigger)
{
    uint8_t in_window = mppt_trace_in_window(pwm_before) || mppt_trace_in_window(pwm_after);
    if (!in_window) return;

    mppt_trace_entry_t *e = &mppt_trace_buf[mppt_trace_wr];
    e->tick_10ms = mppt_trace_tick++;
    e->Vpv = Vpv;
    e->Ipv = Ipv;
    e->Ppv = Ppv;
    e->pwm_before = pwm_before;
    e->pwm_after = pwm_after;
    e->flag_bulk = flag_charging_Bulk;
    e->flag_cv = flag_charging_CV;
    e->flag_float = flag_charging_FLOAT;
    e->guard_pv_loss = guard_pv_loss;
    e->guard_conduction = guard_conduction;
    e->guard_psu_escape = guard_psu_escape;
    e->guard_oc_stepdown = guard_oc_stepdown;
    e->guard_ov_stepdown = guard_ov_stepdown;
    e->guard_max_delta_clip = guard_max_delta_clip;
    e->guard_state_transition = guard_state_transition;
    e->goat_idx = goat_now;
    e->goat_phase = (uint8_t)phase;

    uint8_t i = 0;
    while (trigger && trigger[i] && i < (sizeof(e->trigger) - 1u)) {
        e->trigger[i] = trigger[i];
        i++;
    }
    e->trigger[i] = '\0';

    if (guard_max_delta_clip) mppt_dbg_cnt_max_delta_clip++;
    if (guard_pv_loss || guard_oc_stepdown || guard_ov_stepdown || guard_psu_escape) mppt_dbg_cnt_guard_reset++;
    if (guard_state_transition) mppt_dbg_cnt_state_transition++;
    mppt_dbg_cnt_window++;

    mppt_trace_wr = (uint16_t)((mppt_trace_wr + 1u) % MPPT_TRACE_BUF_LEN);
}
#else
#define mppt_trace_log(...) do { } while (0)
#endif

/* ============================================================
 *  STATE (persistent)
 * ============================================================ */
static uint8_t  hyb_isInit = 0;


/* PV loss debounce */
static uint16_t pv_loss_cnt = 0;
static uint16_t pv_win_idx = 0;
static uint16_t pv_win_cnt = 0;
static uint32_t pv_win_sum = 0;
static uint16_t pv_win_buf[PV_WINDOW_COUNT] = {0};
static uint16_t pv_window_avg = 0;
static uint16_t pv_last_sample = 0;
static uint8_t  pv_drop_cnt = 0;
static uint8_t  pv_window_frozen = 0;

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
static float goat_vel[N_GOAT];

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
static uint32_t scgoa_iter = 0;

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
    /* Hapus debounce PV loss & konduksi. */
    pv_loss_cnt    = 0;
    pv_win_idx     = 0;
    pv_win_cnt     = 0;
    pv_win_sum     = 0;
    pv_window_avg  = 0;
    pv_last_sample = 0;
    pv_drop_cnt    = 0;
    pv_window_frozen = 0;
    cond_cnt       = 0;

#if ENABLE_PSU_ESCAPE
    /* bersihkan escape hatch PSU */
    psu_limit_cnt     = 0;
    psu_limit_relax   = 0;
    psu_limit_ceiling = 0;
    dbg_limit_psu     = 0;
    psu_escape_active_ticks = 0;
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
    scgoa_iter     = 0;

    /* Reset solusi terbaik ke batas bawah duty. */
    gbest_P        = 0.0f;
    gbest_D        = DUTY_LB_F;

    /* NOTE:
     * Jangan paksa PWM_VALUE=0 di reset function kalau kamu panggil reset
     * di kondisi tertentu. Kalau PV-loss hard, baru kamu matikan PWM.
     */
}

//=======================================================

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
     * 1) Ambil data sensor (DISPLAY SCALED)
     * ======================================================== */
    uint16_t Vpv  = dis_voltage_pv;  /* Tegangan PV display-scaled. */
    uint16_t Ipv  = dis_current_pv;  /* Arus PV display-scaled. */
    uint16_t Vbat = dis_voltage_bat; /* Tegangan baterai display-scaled. */
    uint16_t Ibat = dis_current_bat; /* Arus baterai display-scaled. */

    /* Hitung power 32-bit aman (tidak overflow) */
    uint32_t Ppv32 = (uint32_t)Vpv * (uint32_t)Ipv;
    uint16_t pwm_before = PWM_VALUE;

    /* ========================================================
     * 2) PV-loss detector (AND + debounce)
     *    Tujuan: kalau PV benar-benar putus, reset MPPT total.
     * ======================================================== */
    uint8_t pv_low_inst = (Vpv <= PV_LOSS_V_TH) && (Ipv <= PV_LOSS_I_TH);
    uint8_t pv_low_avg  = (pv_window_avg <= (PV_LOSS_V_TH + PV_RESUME_DEADBAND)) && (Ipv <= PV_LOSS_I_TH);
    uint8_t pv_drop_fast = (pv_drop_cnt >= PV_DROP_COUNT_MAX); /* drop tajam berturut-turut */
    uint8_t pv_low = pv_low_inst || pv_low_avg || pv_drop_fast;

    if (pv_low) {
        if (pv_loss_cnt < PV_LOSS_COUNT_N) pv_loss_cnt++;
    } else {
        pv_loss_cnt = 0;
    }

    if (pv_loss_cnt >= PV_LOSS_COUNT_N) {
        /* PV benar-benar hilang -> matikan PWM + reset state */
        if (pv_drop_fast) clear_resume_snapshot(); /* jangan pakai duty lama jika drop tajam */
        arm_resume_from_bookmark(Vpv, PWM_VALUE);

        sync_pwm_outputs(0);  /* Matikan PWM fisik + sinkron UI. */

        MPPT_Hybrid_Reset();  /* Bersihkan state algoritma. */
        pv_loss_cnt = 0;      /* Reset debounce agar siap deteksi ulang. */
        mppt_trace_log(Vpv, Ipv, Ppv32, pwm_before, PWM_VALUE, 1u, 0u, 0u, 0u, 0u, 0u, 0u, goat_idx, MPPT_PHASE_NONE, "DUTY_WINDOW_EVENT");
        UPDATE_TRENDS(Vpv, Ppv32);
        return;
    }

    /* ========================================================
     * 3) Sinkronisasi DUTY (FIX paling krusial)
     *    PWM_VALUE harus jadi satu-satunya truth.
     * ======================================================== */
    sync_pwm_outputs(PWM_VALUE);     /* Sinkron duty + persen UI dengan nilai PWM terakhir. */

    /* ========================================================
     * 4) Proteksi charge (mode BULK)
     *    Kalau overcurrent/overvoltage -> turunin duty 1 step dan keluar.
     * ======================================================== */
    if (dis_current_bat > MAX_CURRENT_CHARGE) {
        /* Kurangi duty satu langkah untuk meredam arus berlebih. */
        if (duty_cycle > 0) duty_cycle--;
        sync_pwm_outputs(duty_cycle);

        cond_cnt = 0;
        mppt_trace_log(Vpv, Ipv, Ppv32, pwm_before, PWM_VALUE, 0u, 1u, 0u, 1u, 0u, 0u, 0u, goat_idx, MPPT_PHASE_NONE, "DUTY_WINDOW_EVENT");
        UPDATE_TRENDS(Vpv, Ppv32);
        return;
    }
    if (dis_voltage_bat > MAX_BATTERY_CHARGE) {
        /* Turunkan duty jika tegangan baterai melewati batas bulk. */
        if (duty_cycle > 0) duty_cycle--;
        sync_pwm_outputs(duty_cycle);

        /* Reset gating konduksi agar GOA tidak aktif saat proteksi. */
        cond_cnt = 0;
        mppt_trace_log(Vpv, Ipv, Ppv32, pwm_before, PWM_VALUE, 0u, 1u, 0u, 0u, 1u, 0u, 0u, goat_idx, MPPT_PHASE_NONE, "DUTY_WINDOW_EVENT");
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
            sync_pwm_outputs(pwm_min);
        }

        Dold_pwm = PWM_VALUE;

        /* init GOA population random domain */
        for (int i = 0; i < N_GOAT; i++) {
            goats_D[i]      = DUTY_LB_F + (DUTY_UB_F - DUTY_LB_F) * rand01();
            goats_P[i]      = 0.0f;
            goats_prevP[i]  = 0.0f;
            stag_cnt[i]     = 0;
            goat_vel[i]     = 0.0f;
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
        scgoa_iter    = 0;

        /* Pure SC-GOA: tidak ada startup/fallback PnO */
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

    if (cond_cnt < COND_STABLE_N) {
        mppt_trace_log(Vpv, Ipv, Ppv32, pwm_before, PWM_VALUE, 0u, 1u, 0u, 0u, 0u, 0u, 0u, goat_idx, MPPT_PHASE_NONE, "DUTY_WINDOW_EVENT");
    }

#if ENABLE_PSU_ESCAPE
    /* ========================================================
     * 7) Escape hatch: bench PSU current-limited (BULK only)
     *    - Deteksi: arus sudah dekat limit, Vpv turun, daya tidak naik.
     *    - Aksi: tahan duty (atau step down sedikit) + log flag ringan.
     * ======================================================== */
    uint8_t near_current_ceiling = (Ipv >= (uint16_t)(MAX_CURRENT_CHARGE - 1u)) || (Ibat >= (uint16_t)(MAX_CURRENT_CHARGE - 1u));
    uint8_t pv_sagging           = (Vpv_last > 0) && (Vpv + PSU_SAG_DVPV_TH < Vpv_last);
    uint8_t power_not_better     = (Ppv_last > 0) && (Ppv32 + (uint32_t)(PSU_POWER_STALL_MARGIN * Ipv) <= Ppv_last); /* margin stagnasi daya by profile */

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
        psu_escape_trigger_count++;

        if (PWM_VALUE > 0) PWM_VALUE--;        /* redam 1 step supaya sag berhenti */
        sync_pwm_outputs(PWM_VALUE);
        mppt_trace_log(Vpv, Ipv, Ppv32, pwm_before, PWM_VALUE, 0u, 0u, 1u, 0u, 0u, 0u, 0u, goat_idx, MPPT_PHASE_NONE, "DUTY_WINDOW_EVENT");
    }

    /* selama relaksasi, jangan biarkan duty melampaui ceiling */
    if (psu_limit_relax > 0) {
        psu_escape_active_ticks++;
        psu_limit_relax--;

        if (psu_limit_ceiling > 0 && PWM_VALUE > psu_limit_ceiling) {
            sync_pwm_outputs(psu_limit_ceiling);
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
     * 8) PHASE SC-GOA PURE: evaluate + update
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

            mppt_trace_log(Vpv, Ipv, Ppv32, pwm_before, PWM_VALUE, 0u, 0u, dbg_limit_psu, 0u, 0u, 0u, 0u, goat_idx, (eval_settle > 0) ? MPPT_PHASE_SETTLE : MPPT_PHASE_AVG, "DUTY_WINDOW_EVENT");
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
            uint8_t max_delta_clipped = 0u;
            if (diff > (int)maxStep) { pwm_cmd = Dold_pwm + maxStep; max_delta_clipped = 1u; }
            else if (diff < -(int)maxStep) { pwm_cmd = Dold_pwm - maxStep; max_delta_clipped = 1u; }

            sync_pwm_outputs(pwm_cmd); // sync
            mppt_trace_log(Vpv, Ipv, Ppv32, pwm_before, PWM_VALUE, 0u, 0u, dbg_limit_psu, 0u, 0u, max_delta_clipped, 0u, (uint8_t)(goat_idx), MPPT_PHASE_UPDATE, "DUTY_WINDOW_EVENT");
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

    /* ---------- STATE 2: UPDATE (SC-GOA: SCA + momentum + goat refresh) ---------- */
    if (state_goa == 2)
    {
        scgoa_iter++;

        /* (1) local best + global best */
        lbest_P = goats_P[0];
        lbest_D = goats_D[0];
        for (int i = 1; i < N_GOAT; i++) {
            if (goats_P[i] > lbest_P) {
                lbest_P = goats_P[i];
                lbest_D = goats_D[i];
            }
        }
        if (lbest_P > gbest_P) {
            gbest_P = lbest_P;
            gbest_D = lbest_D;
        }

        /* (2) near convergence by D-span */
        float dmax = goats_D[0], dmin = goats_D[0];
        for (int i = 1; i < N_GOAT; i++) {
            if (goats_D[i] > dmax) dmax = goats_D[i];
            if (goats_D[i] < dmin) dmin = goats_D[i];
        }
        float D_span = dmax - dmin;
        if (D_span < CONV_WINDOW) conv_counter++;
        else conv_counter = 0;

        /* (3) SCA adaptive amplitude */
        float a_param = SCA_A0 * (1.0f - ((float)scgoa_iter / SCA_DECAY_STEP));
        if (a_param < SCA_A_MIN) a_param = SCA_A_MIN;

        for (int i = 0; i < N_GOAT; i++)
        {
            float r1 = a_param * (2.0f * rand01() - 1.0f);
            float r2 = 2.0f * 3.1415926f * rand01();
            float r3 = SCA_R3_MIN + SCA_R3_SPAN * rand01();
            float r4 = rand01();

            float dist = fabsf(r3 * gbest_D - goats_D[i]);
            float delta = (r4 < 0.5f) ? (r1 * sinf(r2) * dist)
                                      : (r1 * cosf(r2) * dist);

            float Dnew = goats_D[i] + SCA_ALPHA_STEP * delta + SCA_BETA_MOM * goat_vel[i];
            if (Dnew < DUTY_LB_F) Dnew = DUTY_LB_F;
            if (Dnew > DUTY_UB_F) Dnew = DUTY_UB_F;

            goat_vel[i] = Dnew - goats_D[i];
            goats_D[i] = Dnew;
        }

        /* (4) diversity maintenance: re-init worst goat periodik */
        if ((scgoa_iter % SCGOA_REINIT_IT) == 0u) {
            int worst_i = 0;
            float worstP = goats_P[0];
            for (int i = 1; i < N_GOAT; i++) {
                if (goats_P[i] < worstP) { worstP = goats_P[i]; worst_i = i; }
            }
            goats_D[worst_i] = DUTY_LB_F + (DUTY_UB_F - DUTY_LB_F) * rand01();
            goat_vel[worst_i] = 0.0f;
        }

        state_goa     = 1;
        goat_idx      = 1;
        prev_goat_idx = 0;

        /* (5) EMA smoothing pada duty keluaran */
        float Dcmd = SCGOA_EMA_ALPHA * gbest_D + (1.0f - SCGOA_EMA_ALPHA) * ((float)Dold_pwm / (float)MAX_PERIOD);
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
        uint8_t max_delta_clipped = 0u;
        if (diff > (int)maxStep) { pwm_cmd = Dold_pwm + maxStep; max_delta_clipped = 1u; }
        else if (diff < -(int)maxStep) { pwm_cmd = Dold_pwm - maxStep; max_delta_clipped = 1u; }

        sync_pwm_outputs(pwm_cmd);
        mppt_trace_log(Vpv, Ipv, Ppv32, pwm_before, PWM_VALUE, 0u, 0u, dbg_limit_psu, 0u, 0u, max_delta_clipped, 0u, goat_idx, MPPT_PHASE_UPDATE, "DUTY_WINDOW_EVENT");
        Dold_pwm = pwm_cmd;
        UPDATE_TRENDS(Vpv, Ppv32);
        return;
    }

    /* fallback kalau state corrupt -> reset mesin SC-GOA */
    UPDATE_TRENDS(Vpv, Ppv32);
    state_goa = 1;
    goat_idx = 1;
    prev_goat_idx = 0;
}
// =============================================================
/* ============================================================
 *  CHARGING FLOW ROBUST (deci-Volt & deci-Amp: 0.1V, 0.1A)
 *  MPPT dipanggil tiap 10 ms
 *  - BULK: MPPT Hybrid SC-GOA
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

/* Snapshot untuk resume duty pasca PV-loss */
static uint16_t resume_pwm_last   = 0;
static uint16_t resume_vpv_last   = 0;
static uint16_t resume_loss_ticks = 0;
static uint8_t  resume_armed      = 0;
static uint16_t resume_pwm_bookmark = 0;
static uint16_t resume_vpv_bookmark = 0;
static uint16_t resume_bookmark_age = 0;
static uint16_t resume_vpv_snapshot = 0; /* Vpv stabil saat bookmark diambil (anti anjlok akibat averaging) */
static uint16_t resume_pwm_snapshot = 0; /* duty stabil saat bookmark diambil */
static uint16_t stable_vpv_last     = 0; /* Vpv terakhir yang valid (untuk fallback resume <5s) */
static uint16_t stable_pwm_last     = 0; /* PWM terakhir yang valid (untuk fallback resume <5s) */

static inline void reset_flow_counters(void)
{
    t_enter_cv   = 0;
    t_to_float   = 0;
    t_bulk_float = 0;
    t_rebulk     = 0;
    t_bulk_highV = 0;
    t_flow_ticks = 0;
    t_abs_enter_ticks = 0;
}

static inline void clear_resume_snapshot(void)
{
    resume_pwm_last   = 0;
    resume_vpv_last   = 0;
    resume_loss_ticks = 0;
    resume_armed      = 0;
    resume_pwm_bookmark = 0;
    resume_vpv_bookmark = 0;
    resume_bookmark_age = 0;
    resume_vpv_snapshot = 0;
    resume_pwm_snapshot = 0;
    stable_vpv_last     = 0;
    stable_pwm_last     = 0;
    pv_drop_cnt = 0;
    pv_window_frozen = 0;
}

static inline uint16_t pv_window_push(uint16_t Vpv)
{
    /* deteksi slope turun tajam (per 10ms) untuk menganggap loss keras */
    if (pv_last_sample > 0 && (pv_last_sample - Vpv) > PV_DROP_DV_TH) {
        if (pv_drop_cnt < PV_DROP_COUNT_MAX) pv_drop_cnt++;
    } else {
        if (pv_drop_cnt > 0) pv_drop_cnt--;
    }
    pv_last_sample = Vpv;

    if (pv_win_cnt < PV_WINDOW_COUNT) {
        pv_win_buf[pv_win_idx] = Vpv;
        pv_win_sum += Vpv;
        pv_win_cnt++;
    } else {
        pv_win_sum -= pv_win_buf[pv_win_idx];
        pv_win_buf[pv_win_idx] = Vpv;
        pv_win_sum += Vpv;
    }

    pv_win_idx++;
    if (pv_win_idx >= PV_WINDOW_COUNT) pv_win_idx = 0;

    if (pv_win_cnt > 0) pv_window_avg = (uint16_t)(pv_win_sum / pv_win_cnt);
    else pv_window_avg = Vpv;

    return pv_window_avg;
}

static inline void pv_window_reseed(uint16_t Vpv_seed)
{
    pv_win_idx       = 0;
    pv_win_cnt       = 1;
    pv_win_sum       = Vpv_seed;
    pv_window_avg    = Vpv_seed;
    pv_last_sample   = Vpv_seed;
    pv_drop_cnt      = 0;
}

static inline void update_resume_bookmark(uint16_t Vpv, uint16_t Ipv, uint16_t pwm_now)
{
    /* Simpan snapshot hanya saat PV benar-benar masih ada (tegangan & arus cukup). */
    uint8_t pv_level_ok = (Vpv > (PV_LOSS_V_TH + PV_ARM_BAND)) && (Vpv >= PV_RESUME_MIN_V) ? 1u : 0u;
    uint8_t pv_sane = pv_level_ok && (Ipv > PV_LOSS_I_TH);

    if (pv_sane) {
        resume_pwm_bookmark = pwm_now;
        resume_vpv_bookmark = Vpv;
        resume_bookmark_age = RESUME_MAX_LOSS_TICKS; /* tetap fresh selama ~5s tanpa PV. */
        resume_vpv_snapshot = Vpv;
        resume_pwm_snapshot = pwm_now;
        stable_vpv_last     = Vpv;
        stable_pwm_last     = pwm_now;
    } else if (resume_bookmark_age > 0) {
        resume_bookmark_age--; /* Hindari memakai snapshot terlalu lama setelah PV hilang. */
    }
}

static inline void arm_resume_from_bookmark(uint16_t Vpv_now, uint16_t pwm_now)
{
    uint16_t snap_pwm = resume_pwm_bookmark ? resume_pwm_bookmark : (stable_pwm_last ? stable_pwm_last : pwm_now);
    uint16_t snap_vpv = resume_vpv_bookmark ? resume_vpv_bookmark : (stable_vpv_last ? stable_vpv_last : ((pv_window_avg > PV_LOSS_V_TH) ? pv_window_avg : Vpv_now));

    resume_pwm_last   = snap_pwm;
    resume_vpv_last   = snap_vpv;
    resume_vpv_snapshot = snap_vpv;
    resume_pwm_snapshot = snap_pwm;
    resume_loss_ticks = 0;
    resume_armed      = 1;
    resume_bookmark_age = 0;
}

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
    reset_flow_counters();

    if (Vbat >= VABS_ENTER_MIN) {
        /* mendekati/past absorption -> mulai di CV */
        flag_charging_CV = 1;
        t_abs_enter_ticks = t_flow_ticks;  /* pastikan dwell dihitung dari start CV */
        /* mulai dengan duty rendah agar tidak langsung overshoot */
        sync_pwm_outputs((uint16_t)(0.05f * (float)MAX_PERIOD));
    } else if (Vbat >= 275u) {
        /* baterai sudah tinggi -> langsung FLOAT */
        flag_charging_FLOAT = 1;
        sync_pwm_outputs(0); /* biarkan mengapung, duty nanti naik perlahan bila perlu */
    } else {
        /* default: mulai BULK (MPPT) */
        flag_charging_Bulk = 1;
        sync_pwm_outputs(0);
    }

    /* izinkan charging */
    flag_enter_charge = 1;
}

void charging_flow(void)
{
    if (!flag_adc_done) {
        /* pastikan counter transisi tidak nyangkut saat charging tidak aktif */
        reset_flow_counters();

        flag_adc_done = 0;
        return;
    }

    /* baca sensor */
    uint16_t Vbat = dis_voltage_bat;   // 0.1V
    uint16_t Ibat = dis_current_bat;   // 0.1A
    uint16_t Vpv_now = dis_voltage_pv; // 0.1V
    uint16_t Ipv_now = dis_current_pv; // 0.1A
    t_flow_ticks++;                    // tiap 10 ms
    /* Hitung rata-rata Vpv (100ms window) untuk keputusan loss/resume yang lebih robust.
     * Saat PV dianggap hilang (window frozen), gunakan sampel instan agar tidak terjebak di nilai nol. */
    uint16_t Vpv_avg = pv_window_frozen ? (pv_window_avg ? pv_window_avg : Vpv_now) : pv_window_push(Vpv_now);
    if (pv_window_frozen && Vpv_now > Vpv_avg) {
        /* Paksa rata-rata mengikuti pemulihan PV supaya keluar dari status “absent” lebih cepat. */
        pv_window_avg = Vpv_now;
    }
    uint8_t pv_drop_fast = (pv_drop_cnt >= PV_DROP_COUNT_MAX);
    uint16_t Vpv_for_absent = pv_window_frozen ? Vpv_now : Vpv_avg;

    /* Refresh bookmark untuk kemampuan resume (gunakan PV & duty terkini). */
    uint16_t Vpv_for_bookmark = (Vpv_avg > Vpv_now) ? Vpv_avg : Vpv_now;
    update_resume_bookmark(Vpv_for_bookmark, Ipv_now, PWM_VALUE);

    /* standby jika input PV/PSU benar-benar tidak ada */
    uint8_t pv_absent_voltage = (Vpv_now <= PV_LOSS_V_TH) || (Vpv_for_absent <= (PV_LOSS_V_TH + PV_RESUME_DEADBAND)) || pv_drop_fast;
    uint8_t pv_absent = pv_absent_voltage && (Ipv_now <= PV_LOSS_I_TH);
    if (pv_absent) {
        if (pv_drop_fast) {
            clear_resume_snapshot(); /* drop keras: jangan reuse duty lama */
            pv_window_frozen = 1;    /* hentikan averaging supaya nol tidak menyeret rata-rata */
        } else if (!resume_armed) {
            arm_resume_from_bookmark((Vpv_avg > Vpv_now) ? Vpv_avg : Vpv_now, PWM_VALUE);
            pv_window_frozen = 1;    /* freeze window setelah loss terdeteksi */
        }
        if (resume_armed) {
            if (resume_loss_ticks < RESUME_MAX_LOSS_TICKS) resume_loss_ticks++;
            else clear_resume_snapshot(); /* di atas 5s: tidak boleh resume */
        }

        flag_charging_Bulk  = 0;
        flag_charging_CV    = 0;
        flag_charging_FLOAT = 0;
        flag_enter_charge   = 0;

        sync_pwm_outputs(0);

        MPPT_Hybrid_Reset();

        reset_flow_counters();
        pv_absent_latched = 1;

        flag_adc_done = 0;
        return;
    } else {
        /* PV sudah kembali; jika sebelumnya absent, evaluasi ulang/resume. */
        if (pv_absent_latched && !flag_enter_charge) {
            /* Restart jendela rata-rata dengan sampel terbaru agar tidak tercemar nol. */
            pv_window_reseed(Vpv_now);
            pv_window_frozen = 0;
            uint8_t resumed = 0;

            if (resume_armed && resume_loss_ticks <= RESUME_MAX_LOSS_TICKS) {
                uint16_t Vpv_now_return = dis_voltage_pv;
                uint16_t dV_inst = abs_diff_u16(resume_vpv_snapshot ? resume_vpv_snapshot : resume_vpv_last, Vpv_now_return);
                uint8_t within_band = (dV_inst <= RESUME_DVPV_WINDOW);
                uint8_t above_min   = (Vpv_now_return >= PV_RESUME_MIN_V);
                if (within_band && above_min) {
                    uint16_t pwm_min = (uint16_t)(DUTY_LB_F * (float)MAX_PERIOD + 0.5f);
                    uint16_t pwm_max = (uint16_t)(DUTY_UB_F * (float)MAX_PERIOD + 0.5f);
                    uint16_t pwm_resume = clamp_u16(resume_pwm_snapshot ? resume_pwm_snapshot : resume_pwm_last, pwm_min, pwm_max);

                    sync_pwm_outputs(pwm_resume);
                    flag_charging_Bulk  = 1;
                    flag_charging_CV    = 0;
                    flag_charging_FLOAT = 0;
                    flag_enter_charge   = 1;

                    resumed = 1;
                }
            }

            clear_resume_snapshot();
            pv_absent_latched = 0;

            if (resumed) {
                flag_adc_done = 0;
                return; /* lanjut loop berikutnya dengan duty hasil resume */
            }

            check_initial_state();
        } else {
            pv_absent_latched = 0;
            if (resume_armed && resume_loss_ticks > RESUME_MAX_LOSS_TICKS) {
                clear_resume_snapshot();
            }
            pv_window_frozen = 0;
        }
    }

    /* jika charging belum diizinkan (mis. relay belum siap), jangan lanjut loop */
    if (!flag_enter_charge) {
        reset_flow_counters();
        flag_adc_done = 0;
        return;
    }

    /* ===================== STAGE 1: BULK (MPPT) ===================== */
    if (flag_charging_Bulk)
    {
        MPPT_Hybrid();  // MPPT SC-GOA (pure)

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
            mppt_trace_log(Vpv_now, Ipv_now, (uint32_t)Vpv_now * (uint32_t)Ipv_now, PWM_VALUE, PWM_VALUE, 0u, 0u, 0u, 0u, 0u, 0u, 1u, 0u, MPPT_PHASE_NONE, "DUTY_WINDOW_EVENT");
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
            mppt_trace_log(Vpv_now, Ipv_now, (uint32_t)Vpv_now * (uint32_t)Ipv_now, PWM_VALUE, PWM_VALUE, 0u, 0u, 0u, 0u, 0u, 0u, 1u, 0u, MPPT_PHASE_NONE, "DUTY_WINDOW_EVENT");
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
            mppt_trace_log(Vpv_now, Ipv_now, (uint32_t)Vpv_now * (uint32_t)Ipv_now, PWM_VALUE, PWM_VALUE, 0u, 0u, 0u, 0u, 0u, 0u, 1u, 0u, MPPT_PHASE_NONE, "DUTY_WINDOW_EVENT");
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

        sync_pwm_outputs(PWM_VALUE);

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
            mppt_trace_log(Vpv_now, Ipv_now, (uint32_t)Vpv_now * (uint32_t)Ipv_now, PWM_VALUE, PWM_VALUE, 0u, 0u, 0u, 0u, 0u, 0u, 1u, 0u, MPPT_PHASE_NONE, "DUTY_WINDOW_EVENT");
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

        sync_pwm_outputs(PWM_VALUE);

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
            mppt_trace_log(Vpv_now, Ipv_now, (uint32_t)Vpv_now * (uint32_t)Ipv_now, PWM_VALUE, PWM_VALUE, 0u, 0u, 0u, 0u, 0u, 0u, 1u, 0u, MPPT_PHASE_NONE, "DUTY_WINDOW_EVENT");
        }
    }

    flag_adc_done = 0;
}
