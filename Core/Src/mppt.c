#include "main.h"
#include <math.h>
#include <stdlib.h>

uint8_t flag_charging_FLOAT = 0;
uint16_t count_float_stable = 0;

uint8_t flag_adc_done		= 0;
uint8_t flag_enter_charge	= 0;

uint16_t duty_GOA_update	= 0;

uint8_t flag_charging_Bulk 	= 0;
uint8_t flag_charging_CV	= 0;

uint16_t count_charging_CV	= 0;

uint16_t prevPowerInput		= 0;
uint16_t prevVoltagePv		= 0;


// =============================== KODE TUNE ABSORP DAN FLOAT

/* ---------------- helper: duty clamp aman ---------------- */
static inline void pwm_inc(void)
{
    if (PWM_VALUE < MAX_PERIOD) PWM_VALUE++;
}
static inline void pwm_dec(void)
{
    if (PWM_VALUE > 0) PWM_VALUE--;
}
// ========================END

//=======================================================
/* ============================================================
 *  MPPT Hybrid (PnO Startup -> GOA refine) - STM32
 *  - MPPT dipanggil tiap 10 ms (charging_flow()).
 *  - PnO yang kamu punya dipakai untuk "bring-up conduction"
 *    (karena PnO kamu terbukti robust di hardware).
 *  - GOA dipakai setelah arus benar-benar sudah masuk (konduksi).
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

/* ============================================================
 *  RESET FUNCTION (panggil saat PV putus / charging stop)
 * ============================================================ */
void MPPT_Hybrid_Reset(void)
{
    hyb_isInit     = 0;
    pno_active     = 1;
    pv_loss_cnt    = 0;
    cond_cnt       = 0;

    state_goa      = 1;
    goat_idx       = 1;
    prev_goat_idx  = 0;
    conv_counter   = 0;
    t_goa          = 1;

    eval_holding   = 0;
    eval_settle    = 0;
    eval_avg_cnt   = 0;
    eval_sumP      = 0;

    gbest_P        = 0.0f;
    gbest_D        = DUTY_LB_F;

    /* NOTE:
     * Jangan paksa PWM_VALUE=0 di reset function kalau kamu panggil reset
     * di kondisi tertentu. Kalau PV-loss hard, baru kamu matikan PWM.
     */
}

//=======================================================


#include <stdint.h>
#include "adc_sampling.h"   // extern uint16_t dis_voltage_pv, dis_current_pv;
#include "pwm.h"            // extern uint16_t PWM_VALUE; extern int duty_percent; #define MAX_PERIOD ...

// Logika Startup (Inisialisasi)
void check_initial_state() {
    flag_enter_charge = 1; // Izinkan pengisian berjalan

    if (dis_voltage_bat < MAX_BATTERY_CHARGE) {
        flag_charging_Bulk = 1;  // Mulai dari MPPT jika baterai belum penuh
        flag_charging_CV = 0;
        flag_charging_FLOAT = 0;
    } else {
        flag_charging_FLOAT = 1; // Langsung Float jika baterai sudah penuh
    }
}

// ==========================================
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
        MPPT_Hybrid_Reset();
        return;
    }

    /* ========================================================
     * 1) Ambil data sensor (DISPLAY SCALED, konsisten dengan PnO)
     * ======================================================== */
    uint16_t Vpv  = dis_voltage_pv;
    uint16_t Ipv  = dis_current_pv;
    uint16_t Vbat = dis_voltage_bat;
    uint16_t Ibat = dis_current_bat;

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
        PWM_VALUE = 0;
        duty_cycle = 0;
        duty_percent = 0;

        MPPT_Hybrid_Reset();
        pv_loss_cnt = 0;
        return;
    }

    /* ========================================================
     * 3) Sinkronisasi DUTY (FIX paling krusial)
     *    PWM_VALUE harus jadi satu-satunya truth.
     * ======================================================== */
    duty_cycle = PWM_VALUE;          // sync sebelum update algorithm

    /* ========================================================
     * 4) Proteksi charge (ikut gaya robust PnO kamu)
     *    Kalau overcurrent/overvoltage -> turunin duty 1 step dan keluar.
     * ======================================================== */
    if (dis_current_bat > MAX_CURRENT_CHARGE) {
        if (duty_cycle > 0) duty_cycle--;
        PWM_VALUE = duty_cycle;
        duty_percent = (PWM_VALUE * 100) / MAX_PERIOD;

        /* Saat proteksi aktif, jangan biarkan GOA “berantem”.
           Paksa balik ke PnO setelah proteksi reda. */
        pno_active = 1;
        cond_cnt = 0;
        return;
    }
    if (dis_voltage_bat > MAX_BATTERY_CHARGE) {
        if (duty_cycle > 0) duty_cycle--;
        PWM_VALUE = duty_cycle;
        duty_percent = (PWM_VALUE * 100) / MAX_PERIOD;

        pno_active = 1;
        cond_cnt = 0;
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

    /* ========================================================
     * 7) PHASE 1: PnO proven kamu (startup / anti-stuck)
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

        return; // PHASE 1 selesai di sini
    }

    /* ========================================================
     * 8) PHASE 2: GOA refine (MATLAB-like) dengan hold+avg fitness
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

        int diff = (int)pwm_cmd - (int)Dold_pwm;
        uint16_t maxStep = (conv_counter >= CONV_COUNT_LIMIT) ? 1u : MAX_DELTA_PWM;
        if (diff > (int)maxStep) pwm_cmd = Dold_pwm + maxStep;
        else if (diff < -(int)maxStep) pwm_cmd = Dold_pwm - maxStep;

        PWM_VALUE   = pwm_cmd;
        duty_cycle  = pwm_cmd; // sync
        duty_percent = (PWM_VALUE * 100) / MAX_PERIOD;
        Dold_pwm    = pwm_cmd;
        return;
    }

    /* fallback kalau state corrupt -> balik PnO */
    pno_active = 1;
}
// =============================================================


void MPPT_PnO(void) {
	//duty=duty_cycle;
	if(dis_current_bat > MAX_CURRENT_CHARGE)		{duty_cycle--;}
	else if(dis_voltage_bat > MAX_BATTERY_CHARGE)	{duty_cycle--;}
	else {
		if(dis_power_pv > prevPowerInput && dis_voltage_pv > prevVoltagePv)		{duty_cycle--;}
		else if(dis_power_pv > prevPowerInput && dis_voltage_pv < prevVoltagePv)	{duty_cycle++;}
		else if(dis_power_pv < prevPowerInput && dis_voltage_pv > prevVoltagePv)	{duty_cycle++;}
		else if(dis_power_pv < prevPowerInput && dis_voltage_pv < prevVoltagePv)	{duty_cycle--;}
		else if(dis_voltage_bat < MAX_BATTERY_CHARGE)								{duty_cycle++;}

		prevPowerInput = dis_power_pv;
		prevVoltagePv = dis_voltage_pv;
	}
	if(duty_cycle >= MAX_PERIOD) {
		duty_cycle = MAX_PERIOD;
	}
	else if(duty_cycle <= 0) {
		duty_cycle = 0;
	}
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

/* ---------- ABSORPTION (24V VRLA) ---------- */
#define VABS_SET            288u   // 28.8V
#define VABS_ENTER_MIN      284u   // 28.4V (lebih realistis untuk "mulai CV")
#define VABS_HYST           4u     // 0.4V band (anti hunting)

/* ---------- FLOAT ---------- */
#define VFLT_SET            272u   // 27.2V
#define VFLT_HYST           3u     // 0.3V

/* ---------- Current thresholds ---------- */
#define I_MIN_TO_ENTER_CV   3u     // 0.3A (pastikan memang ada charging)
#define I_END_ABS           3u     // 0.3A (end current absorption)
#define I_IDLE              1u     // 0.1A (anggap tidak charging)

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

void charging_flow(void)
{
    /* timers persist across calls */
    static uint16_t t_enter_cv    = 0;
    static uint16_t t_to_float    = 0;
    static uint16_t t_bulk_float  = 0;
    static uint16_t t_rebulk      = 0;
    static uint16_t t_abs_dwell   = 0;

    /* timestamp 10ms tick untuk catat waktu masuk CV */
    static uint32_t t_flow_ticks      = 0;
    static uint32_t t_abs_enter_ticks = 0;

    if (!(flag_adc_done && flag_enter_charge)) {
        /* pastikan counter transisi tidak nyangkut saat charging tidak aktif */
        t_enter_cv   = 0;
        t_to_float   = 0;
        t_bulk_float = 0;
        t_rebulk     = 0;

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

        if (t_bulk_float >= T_BULK_FLOAT_TICKS) {
            flag_charging_Bulk  = 0;
            flag_charging_CV    = 0;
            flag_charging_FLOAT = 1;

            t_bulk_float = 0;
            t_enter_cv   = 0;
            t_to_float   = 0;
            t_rebulk     = 0;

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
