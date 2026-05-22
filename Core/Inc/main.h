/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "comm.h"
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define mt6701_1_Pin GPIO_PIN_4
#define mt6701_1_GPIO_Port GPIOA
#define mpu6500_cs_Pin GPIO_PIN_6
#define mpu6500_cs_GPIO_Port GPIOC
#define mt6701_0_Pin GPIO_PIN_4
#define mt6701_0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/** @brief 通用数学常量 */
#define RAD_TO_DEG              57.29578f   /**< 180/π, rad→° */
#define RPM_FROM_RADPS           9.5493f    /**< 60/(2π), rad/s→RPM */

/** @brief 编译器屏障 — 强制从内存重载全局变量, 用于跨任务共享数据 */
#define COMPILER_BARRIER()      asm volatile("" ::: "memory")

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
