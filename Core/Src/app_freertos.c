/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "mt6701.h"
#include "ina240.h"
#include "mpu6050.h"
#include "kf_angle.h"
#include "comm_protocol.h"
#include "foc.h"
#include "six_step.h"
#include "motor_hal.h"
#include "svpwm.h"
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
/* USER CODE BEGIN Variables */
osThreadId encoderTaskHandle;
osThreadId adcTaskHandle;
osThreadId mpuTaskHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void TaskEncoderReport(void const * argument);
void TaskADCMonitor(void const * argument);
void TaskMPU6500(void const * argument);
void TaskSixStep(void const * argument);
void TaskVoltageSine(void const * argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(encoderTask, TaskEncoderReport, osPriorityBelowNormal, 0, 64);
  encoderTaskHandle = osThreadCreate(osThread(encoderTask), NULL);
  osThreadDef(adcTask, TaskADCMonitor, osPriorityNormal, 0, 64);
  adcTaskHandle = osThreadCreate(osThread(adcTask), NULL);
  osThreadDef(mpuTask, TaskMPU6500, osPriorityNormal, 0, 384);
  mpuTaskHandle = osThreadCreate(osThread(mpuTask), NULL);
  // osThreadDef(sixStepTask, TaskSixStep, osPriorityNormal, 0, 128);
  // osThreadCreate(osThread(sixStepTask), NULL);
  osThreadDef(voltageSineTask, TaskVoltageSine, osPriorityNormal, 0, 256);
  osThreadCreate(osThread(voltageSineTask), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;
  for(;;)
  {
    while (COMM_Available() > 0)
    {
      COMM_SendByte(COMM_ReadByte());
    }
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  6 步换相任务：开环方波驱动 M1
  * @param  argument: 未使用
  * @retval 无
  */
void TaskSixStep(void const * argument)
{
    (void)argument;
    SixStep_Init(&g_motor[0], 0.3f);  // M1, 30% 占空比起步
    Motor_StartPWM(&g_motor[0]);

    for (;;)
    {
        SixStep_Run(&g_motor[0]);
        osDelay(1);  // 1ms 周期
    }
}

/**
  * @brief  编码器读取任务：每 500ms 读取 MT6701 数据（仅供内部使用，不输出串口）
  * @param  argument: 未使用
  * @retval 无
  */
void TaskEncoderReport(void const * argument)
{
  (void)argument;
  for(;;)
  {
    for (uint8_t i = 0; i < MT6701_NUM_ENCODERS; i++)
    {
      MT6701_Data_t enc;
      MT6701_GetData(i, &enc);
    }
    osDelay(500);
  }
}

/**
  * @brief  ADC 监控任务：每 100ms 读取 INA240 电流并通过串口输出
  * @param  argument: 未使用
  * @retval 无
  */
void TaskADCMonitor(void const * argument)
{
  (void)argument;
  for(;;)
  {
    osDelay(100);
  }
}

/**
  * @brief  MPU6500 姿态解算任务：每 200ms 更新卡尔曼滤波器
  * @param  argument: 未使用
  * @retval 无
  */
void TaskMPU6500(void const * argument)
{
  (void)argument;
  MPU6050_SetAccelRange(MPU6050_ACCEL_RANGE_4G);

  /* 初始化卡尔曼滤波器 */
  KalmanAngle_t kf;
  KalmanAngle_Init(&kf, 0.0f, 0.001f, 0.003f, 0.03f);
  const float dt = 0.01f;

  /* 等待滤波器收敛 */
  for (int i = 0; i < 100; i++) osDelay(10);

  for(;;)
  {
    MPU6050_Accel_t accel;
    MPU6050_Gyro_t gyro;
    MPU6050_ReadAccel(&accel);
    MPU6050_ReadGyro(&gyro);

    float accel_angle = atan2f(accel.y, accel.z) * 57.29578f;
    KalmanAngle_Predict(&kf, gyro.x, dt);
    KalmanAngle_Update(&kf, accel_angle);

    osDelay((int)(dt * 1000));
  }
}

/**
  * @brief  电压模式正弦波任务：开环 SVPWM 驱动 M1
  * @param  argument: 未使用
  * @retval 无
  */
void TaskVoltageSine(void const * argument)
{
    (void)argument;
    g_motor[0].mode = MOTOR_MODE_VOLTAGE_SINE;
    g_motor[0].voltage_mag = 0.5f;  // 0.5V 起转
    Motor_StartPWM(&g_motor[0]);

    for (;;)
    {
        // 读编码器机械角度 → 电气角度
        uint16_t raw = MT6701_ReadAngle(g_motor[0].motor_id);
        float mech_angle = (float)raw * 6.283185307f / 16384.0f;
        float elec_angle = mech_angle * (float)MOTOR_POLE_PAIRS;
        g_motor[0].elec_angle = elec_angle;

        // Vα = Vm * cos(θ), Vβ = Vm * sin(θ)
        float vm = g_motor[0].voltage_mag;
        float v_alpha = vm * cosf(elec_angle);
        float v_beta  = vm * sinf(elec_angle);

        SVPWM_SetVab(v_alpha, v_beta, &g_motor[0]);
        osDelay(1);  // 1ms 周期（vTaskDelayUntil 未使能，用 osDelay 代替）
    }
}
/* USER CODE END Application */

