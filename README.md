# MPPT-SCGOA-IBC-AlphaVer

## Overview
This repository contains an embedded implementation of a pure SC-GOA based MPPT controller for an interleaved boost converter on STM32F103CB. The firmware is integrated with a charging state machine (Bulk, CV, Float, Idle) and includes runtime guards for stable converter operation.

This README describes the implemented firmware behavior in `Core/Src/mppt.c`.

## 1. Firmware Implementation Summary

### 1.1 Runtime execution model
- The MPPT routine is executed from the charging control flow every 10 ms.
- SC-GOA search is active only during the Bulk charging state.
- Candidate fitness uses staged evaluation (settling steps followed by averaging steps) to reduce switching ripple and sampling noise impact.
- Duty command output is updated through bounded increments using a slew limiter.

### 1.2 Duty domain and representation
- Duty search bounds:
  - `DUTY_LB_F = 0.10`
  - `DUTY_UB_F = 0.60`
- Internal runtime output uses PWM counts (`0..MAX_PERIOD`) and is synchronized to duty display/control variables.

### 1.3 Active runtime guards and limiters
The firmware applies the following runtime safeguards:
- PV-loss debounce using low-voltage and low-current conjunction over a debounce window.
- Conduction gate before full optimizer progression.
- Over-current and over-voltage step-down interlocks.
- Optional PSU current-limit escape behavior through compile-time mode selection.
- Adaptive or fixed slew limiting on duty updates per 10 ms tick.

## 2. Important Runtime Parameters

| Category | Firmware parameter(s) | Default value | Runtime role |
|---|---|---:|---|
| Population | `N_GOAT` | 10 | Number of search agents |
| Duty domain | `DUTY_LB_F`, `DUTY_UB_F` | 0.10, 0.60 | Search bounds |
| Tick loop | MPPT call period | 10 ms | Optimizer execution interval |
| Fitness evaluation | `EVAL_SETTLE_STEPS`, `EVAL_AVG_STEPS` | 1, 3 | Settling and averaging per candidate |
| Slew limiter | `MIN_DELTA_PWM`, `MAX_DELTA_PWM` | 1, 6 | Maximum duty increment per tick |
| Near-convergence | `CONV_WINDOW`, `CONV_COUNT_LIMIT` | 0.12, 15 | Stability indicator in firmware loop |
| Re-initialization | `SCGOA_REINIT_IT` | 50 | Periodic population refresh |
| PV-loss guard | `PV_LOSS_V_TH`, `PV_LOSS_I_TH`, `PV_LOSS_COUNT_N` | 5, 1, 50 | Loss detection with debounce |
| Conduction guard | `COND_IBAT_TH`, `COND_STABLE_N` | 1, 10 | Enable condition for stable charge current |
| PSU escape | `PSU_STALL_COUNT`, `PSU_RELAX_TICKS` | profile dependent | Temporary duty freeze under prolonged current-limit condition |

Notes:
- `PSU_*` values depend on compile-time source mode (`REAL_PV` or `BENCH_PSU`).
- Threshold values are in project-specific scaled ADC/display units.

## 3. Charging State Flow Used by Firmware

The implemented charging sequence uses four states:

1. **Bulk**
   - SC-GOA MPPT is active.
   - Conduction, PV-loss, and electrical protection guards remain active.
2. **CV (Constant Voltage)**
   - Battery voltage is regulated around the absorption target using hysteresis and dwell timing.
3. **Float**
   - Battery voltage is maintained around the float target with hysteresis and re-bulk condition checks.
4. **Idle**
   - Entered when PV is absent or charging path is disabled.
   - Charging flags are cleared and MPPT context is reset.

State transitions use voltage/current checks and timing conditions to avoid oscillatory toggling.

## 4. Fundamental Formula

### 4.1 SC-GOA update equation used in firmware context
For agent `i` at iteration `t`:

\[
x_i^{(t+1)} = x_i^{(t)} + \alpha\,\phi(r_2)\,r_3\,\left|x^{*} - x_i^{(t)}\right|
\]

Where:
- \(\phi(r_2)\) is either \(\sin(r_2)\) or \(\cos(r_2)\).
- \(r_2\) is the random phase term.
- \(r_3\) is the random step scaling term.
- \(\alpha\) is the exploration amplitude.

Notation mapping note:
- `r1` represents amplitude-related control in model notation.
- Firmware amplitude behavior is represented by `SCA_A0`, `SCA_A_MIN`, `SCA_DECAY_STEP`, and `SCA_ALPHA_STEP`.

### 4.2 Power fitness definition
Candidate fitness is based on average measured PV power:

\[
f_i^{(t)} = \frac{1}{n_s}\sum_{k=1}^{n_s} P_k, \qquad P_k = V_{pv,k} I_{pv,k}
\]

In firmware execution, this is implemented with settle steps and averaging steps (`EVAL_SETTLE_STEPS`, `EVAL_AVG_STEPS`) rather than a single instantaneous sample.

### 4.3 Smoothing, EMA, and momentum rules
When enabled, runtime smoothing follows these standard forms.

**EMA duty smoothing**
\[
D_{ema}(t) = \lambda D_{cmd}(t) + (1-\lambda) D_{ema}(t-1)
\]
with \(\lambda\) corresponding to `SCGOA_EMA_ALPHA`.

**Momentum form**
\[
v_i^{(t+1)} = \beta v_i^{(t)} + \Delta x_i^{(t)}, \qquad
x_i^{(t+1)} = x_i^{(t)} + v_i^{(t+1)}
\]
with \(\beta\) corresponding to `SCA_BETA_MOM`.

**Convergence metric reference**
- Runtime near-convergence behavior uses `CONV_WINDOW` and `CONV_COUNT_LIMIT`.

## 5. Model vs Firmware Mapping

| MATLAB/model notation | Meaning | Firmware macro or implementation in `Core/Src/mppt.c` |
|---|---|---|
| \(N\) | population size | `N_GOAT` |
| \(D_{\min}, D_{\max}\) | duty search bounds | `DUTY_LB_F`, `DUTY_UB_F` |
| \(\alpha\), `r1` | exploration amplitude control | `SCA_A0`, `SCA_A_MIN`, `SCA_DECAY_STEP`, `SCA_ALPHA_STEP` |
| `r2` | sinusoidal phase term | random phase generation in update routine |
| `r3` | step scaling | `SCA_R3_MIN`, `SCA_R3_SPAN` |
| \(\beta\) | momentum factor | `SCA_BETA_MOM` |
| EMA coefficient | duty smoothing coefficient | `SCGOA_EMA_ALPHA` |
| convergence metric | stability assessment | `CONV_WINDOW`, `CONV_COUNT_LIMIT` |
| re-initialization interval | periodic refresh policy | `SCGOA_REINIT_IT` |
| \(f_i\) | objective value per agent | power calculation and averaging (`EVAL_*`) |

## 6. Note on Reported "Optimal Duty" Values

Observed operating values such as 0.43 or 0.44 are tuning and operating outcomes under specific hardware and irradiance conditions. They are not fixed hard-coded optimum constants. The firmware continues online search within configured duty bounds.

## 7. Project Structure
```
SC-GOA_MPPT_IBC
┣ Core
┃ ┣ Inc (Header Files)
┃ ┃ ┣ adc_sampling.h
┃ ┃ ┣ lcd_display.h
┃ ┃ ┣ main.h
┃ ┃ ┣ mppt.h
┃ ┃ ┣ pwm.h
┃ ┃ ┣ stm32f1xx_hal_conf.h
┃ ┃ ┣ stm32f1xx_it.h
┃ ┃ ┗ work_protect.h
┃ ┣ Src (Source Files)
┃ ┃ ┣ adc_sampling.c
┃ ┃ ┣ lcd_display.c
┃ ┃ ┣ main.c
┃ ┃ ┣ mppt.c
┃ ┃ ┣ pwm.c
┃ ┃ ┣ stm32f1xx_hal_msp.c
┃ ┃ ┣ stm32f1xx_it.c
┃ ┃ ┣ syscalls.c
┃ ┃ ┣ sysmem.c
┃ ┃ ┣ system_stm32f1xx.c
┃ ┃ ┗ work_protect.c
┃ ┗ Startup
┃   ┗ startup_stm32f103cbtx.s
┣ Drivers
┃ ┣ CMSIS (Hardware Interface Standard)
┃ ┣ STM32F1xx_HAL_Driver (Hardware Abstraction Layer)
┃ ┗ csrc (U8g2 Graphics Library for LCD)
┣ .settings (IDE Project Settings)
┣ .project
┣ .cproject
┗ README.md
```

## License
This project is licensed under the GNU General Public License v3.0.
