/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* File Role    : Titik masuk aplikasi STM32, mendeklarasikan GPIO/pin, dependensi modul.
 * Dependencies : stm32f1xx_hal.h beserta modul aplikasi (adc_sampling, lcd_display, mppt, work_protect, pwm).
 * Fungsi inti  : Error_Handler() serta HAL_TIM_MspPostInit() digunakan startup/driver PWM. */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc_sampling.h"
#include "lcd_display.h"
#include "mppt.h"
#include "work_protect.h"
#include "pwm.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_RUN_PIN_Pin GPIO_PIN_2
#define LED_RUN_PIN_GPIO_Port GPIOB
#define FAN_PIN_Pin GPIO_PIN_13
#define FAN_PIN_GPIO_Port GPIOB
#define RLY_PV_PIN_Pin GPIO_PIN_14
#define RLY_PV_PIN_GPIO_Port GPIOB
#define RLY_BAT_PIN_Pin GPIO_PIN_15
#define RLY_BAT_PIN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
// BULK STAGE
#define MAX_BATTERY_CHARGE 288
#define MAX_CURRENT_CHARGE 22
// FLOAT STAGE
#define FLOAT_BATTERY_CHARGE  270  // 27.0V (Float)
#define CV_DONE_CURRENT         2    // 0.2A (Arus kecil penanda baterai penuh untuk pindah ke Float)

#define BATTERY_PROTECT_CURRENT	70
#define BATTERY_PROTECT_VOLT	300
#define PV_PROTECT_VOLT			250

/* PROTEKSI */
#define REBULK_VOLTAGE          255  // 25.5V (Jika turun ke sini, ulangi dari Bulk)

//#define POWER_TEST
#define CHARGING_TEST
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
