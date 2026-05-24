/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../Inc/buzzer.h"
#include "ws2812b.h"
#include "mt6701.h"
#include "ina240.h"
#include "mpu6500.h"
#include "vbus.h"
#include "foc.h"
#include "motor_hal.h"
#include "calibration.h"
#include "comm.h"
#include "bt_comm.h"
#include "build_time.h"  /* BUILD_TIMESTAMP — 每次构建自动生成 */
#include <stdio.h>
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern osThreadId TaskBalanceLoopHandle;
static uint8_t g_printf_port = 0;  /**< 0=USART1, 1=USART2, 由 CLI 端口切换控制 */

void CLI_SetOutputPort(uint8_t port) { g_printf_port = port; }
uint8_t CLI_GetOutputPort(void) { return g_printf_port; }
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  MX_ADC2_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM17_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  // 禁用 stdout 缓冲，确保 printf 立即输出
  setvbuf(stdout, NULL, _IONBF, 0);
  WS2812B_Init();
  Buzzer_Init();
  COMM_Init();
  BT_COMM_Init();
  MT6701_Init();
  INA240_Init();

  // 初始化完成提示音 — 两声短响
  Buzzer_Beep(2000, 80);
  HAL_Delay(120);
  Buzzer_Beep(2000, 80);
  HAL_Delay(150);

  // 先启动 TIM3 提供 ADC TRGO 触发，但 PC14 保持低电平（MP6536 禁能，零电流）
  HAL_TIM_Base_Start(&htim3);
  HAL_Delay(10);
  INA240_Calibrate();

  // 从 Flash 加载校准参数到 g_calib（仅用于打印，实际应用在 FOC_Init 之后）
  CALIB_FlashLoad(&g_calib);
  if (!CALIB_FlashIsValid(&g_calib)) {
      g_calib.magic   = CALIB_MAGIC;
      g_calib.version = CALIB_VERSION;
  }

  printf("\r\n=== STM32G4-Balance v3.2 (构建: " BUILD_TIMESTAMP ") ===\r\n");
  printf("IWDG: LSI/256=125Hz, RL=1000 (8s timeout)\r\n");

  FOC_Init();
  // 先中性化 PWM 再使能 MP6536，避免门驱输入浮空导致电机抖动
  for (int i = 0; i < 2; i++) {
      Motor_StartPWM(&g_motor[i]);
      Motor_SetDuty(&g_motor[i], 0.50f, 0.50f, 0.50f);
  }
  Motor_Enable();

  // 电流环初始化
  for (int i = 0; i < 2; i++) {
      g_motor[i].id_ref = 0.0f;
      g_motor[i].iq_ref = 0.0f;
      g_motor[i].mode = MOTOR_MODE_CURRENT_LOOP;
      PI_Reset(&g_motor[i].id_pi);
      PI_Reset(&g_motor[i].iq_pi);
  }

  // FOC_Init 之后应用 Flash 校准参数
  if (CALIB_FlashIsValid(&g_calib)) {
      CALIB_ApplyToMotor(&g_calib, 0);
      CALIB_ApplyToMotor(&g_calib, 1);
      MT6701_SetEncDirection(0, g_calib.enc_direction[0]);
      MT6701_SetEncDirection(1, g_calib.enc_direction[1]);
      INA240_SetAllZeroOffsets(g_calib.zero_offset);
  }
  printf("g_motor active: phase_comp M1=%.3f M2=%.3f rad\r\n", g_motor[0].phase_comp, g_motor[1].phase_comp);

  VBUS_Init();
  printf("VBUS=%.2fV (%s)\r\n", VBUS_Read(),
         VBUS_IsCalibrated() ? "calibrated" : "uncalibrated, use VCAL");

  if (MPU6500_Init() != 0) {
    printf("[FATAL] MPU6500 init FAILED — system halted\r\n");
    /* 等待 UART DMA 发送完成，确保错误消息完整输出 */
    HAL_Delay(5);
    while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC));
    /* 错误提示音: 3声短促 4000Hz (每声100ms, 间隔100ms) */
    for (int i = 0; i < 3; i++) {
      Buzzer_Beep(4000, 100);
      HAL_Delay(100);
    }
    /* 死循环 — 不初始化 MT6701, 不启动 FreeRTOS, 系统完全停止 */
    while (1);
  }
  MT6701_CSDelay_Init();
  MT6701_StartDMA(0);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // FreeRTOS 已接管调度，此处不应被执行
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// printf 重定向 — CLI 可切换输出端口 (USART1↔USART2)
int __io_putchar(int ch)
{
    if (g_printf_port == 1) {
        BT_COMM_SendByte((uint8_t)ch);
    } else {
        COMM_SendByte((uint8_t)ch);
    }
    return ch;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM6)
  {
      MT6701_OnCSDelayComplete();
  }
  if (htim->Instance == TIM17)
  {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      vTaskNotifyGiveFromISR(TaskBalanceLoopHandle, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  Fault_DisableMotors();  // CR-023: 安全停止电机
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
