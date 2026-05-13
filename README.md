# MPPT-SCGOA-IBC-AlphaVer

Internal testing repository for a **Hybrid Sine Cosine-Goat Optimizer Algorithm (GOA)** for MPPT applications. This project integrates metaheuristic global search with conventional local tracking to ensure fast convergence and minimal steady-state oscillation. Designed for MATLAB/Simulink with a focus on codegen compatibility for STM32/embedded deployment.

## 1. Algorithm Overview: Hybrid SC-GOA

The **SC-GOA** is a specialized hybrid metaheuristic designed to solve the Maximum Power Point Tracking (MPPT) problem, particularly under Partial Shading Conditions (PSC). It combines the oscillatory exploration of the **Sine Cosine Algorithm (SCA)** with the robust exploitation and jump-escape strategies of the **Goat Optimization Algorithm (GOA)**.

### Core Novelty:

* **SCA Exploration (Mode 1):** Uses sinusoidal updates to generate oscillatory trajectories around the best-known solution, effectively probing the duty cycle space for the Global MPP (GMPP).


* **GOA Exploitation (Mode 2):** Provides targeted linear attraction toward the best solution for exponential convergence speed.


* **GOA Jump Escape (Mode 3):** Executes stochastic jumps when stagnation is detected, allowing the algorithm to escape local optima basins.



## 2. Key Technical Features

* **Adaptive 3-Mode Switching:** Transitions between exploration, exploitation, and escape based on real-time convergence metrics like the Coefficient of Variation ($CV_f$) and Spatial Spread Ratio ($R_s$).


* **Convergence-Based Lock:** Automatically freezes the duty cycle output once the population converges within a tight tolerance ($CV_f < 0.03$), eliminating steady-state power ripple.


* **Statistical Partial Shading Detection:** Employs a dual-criterion test using **Z-scores** and power drop ratios to distinguish between normal irradiance fluctuations and major shading events that require re-initialization.


* **Memory-Guided Parasite Avoidance:** Replaces the worst-performing agents with Gaussian perturbations of high-quality solutions stored in an archived memory bank.


* **Adaptive Sample Sizing:** Dynamically adjusts the number of measurement samples per agent based on environmental noise levels.



## 3. Implementation Details (MATLAB/Simulink)

The implementation is optimized for stability and real-time execution:

* **Population Strategy:** Utilizes 5–10 agents with stratified initialization to ensure uniform coverage of the duty cycle range ($D_{min}=0.10$ to $D_{max}=0.50$).


* **Smoothing Mechanisms:**
* **Momentum Filter:** Incorporates agent velocity to smooth out sudden jumps during position updates.
* **EMA Smoothing:** An Exponential Moving Average (EMA) filter is applied to the final duty cycle output to prevent hardware chattering.


* **Rate Limiting:** The algorithm logic is configured to update every $N$ timesteps (e.g., 100 cycles) to allow for hardware settling time and prevent high-frequency oscillations.

## 4. Hardware Architecture

The repository includes configurations for a **2-Phase Interleaved Boost Converter** targeting the **STM32F103CB** (Cortex-M3) platform:

* **PWM Frequency:** 30 kHz with 180° phase shift between phases to minimize input current ripple.


* **Control Loop:** MPPT tick runs at 10ms (synchronized with timer ISRs).


* **Sensors:** 12-bit ADC sampling for $V_{pv}$ and $I_{pv}$ with moving average filtering.



## 5. Project Structure
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


**License:** This project is licensed under the MIT License - see the `LICENSE` file for details.
