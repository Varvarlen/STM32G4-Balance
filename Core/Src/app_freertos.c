/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
#include <stdlib.h>
#include <math.h>
#include "mt6701.h"
#include "ina240.h"
#include "mpu6500.h"
#include "kf_angle.h"

#include "comm_protocol.h"
#include "comm.h"
#include "foc.h"

#include "motor_hal.h"
#include "svpwm.h"
#include "current_ctrl.h"
#include "encoder_cache.h"
#include "calibration.h"
#include "speed_ctrl.h"
#include "buzzer.h"
#include "cli_parser.h"
#include "cli.h"
#include "balance_ctrl.h"
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
SpeedCtrl_t g_speed[2];
BalanceCtrl_t g_balance;
osThreadId TaskCLIHandle;
osThreadId TaskBalanceLoopHandle;
extern TIM_HandleTypeDef htim17;
/* USER CODE END Variables */
osThreadId DefaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartCLITask(void const * argument);
void StartBalanceLoopTask(void const * argument);
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
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of DefaultTask */
  osThreadDef(DefaultTask, StartDefaultTask, osPriorityIdle, 0, 192);
  DefaultTaskHandle = osThreadCreate(osThread(DefaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  if (g_calib_mode) {
      osThreadDef(TaskCLI, StartCLITask, osPriorityNormal, 0, 1024);
      TaskCLIHandle = osThreadCreate(osThread(TaskCLI), NULL);
  } else {
      osThreadDef(TaskCLI, StartCLITask, osPriorityNormal, 0, 256);
      TaskCLIHandle = osThreadCreate(osThread(TaskCLI), NULL);
      osThreadDef(TaskBalance, StartBalanceLoopTask, osPriorityHigh, 0, 768);
      TaskBalanceLoopHandle = osThreadCreate(osThread(TaskBalance), NULL);
  }
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the DefaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;
  vTaskDelete(NULL);  // 占位任务, 立即自删, 满足 CubeMX 至少一个任务的约束
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/** @brief CLI 任务 — 串口命令处理 */
void StartCLITask(void const * argument)
{
  (void)argument;
  CLI_Init();
  for(;;)
  {
    CLI_Process();
    osDelay(1);
  }
}

/** @brief 平衡环任务 — 1kHz TIM17 触发, IMU→卡尔曼滤波→平衡PID→速度环→差速 */
void StartBalanceLoopTask(void const * argument)
{
  (void)argument;
  BalanceCtrl_Init(&g_balance);

  SpeedCtrl_Init(&g_speed[0], SPEED_PI_DEFAULT_KP, SPEED_PI_DEFAULT_KI,
                 2.0f, -2.0f, MT6701_GetEncDirection(0),
                 MOTOR_KT, MOTOR_J);
  SpeedCtrl_Init(&g_speed[1], SPEED_PI_DEFAULT_KP, SPEED_PI_DEFAULT_KI,
                 2.0f, -2.0f, MT6701_GetEncDirection(1),
                 MOTOR_KT, MOTOR_J);

  MPU6500_SetAccelRange(MPU6500_ACCEL_RANGE_4G);

  const float dt = 0.001f;  // 1ms

  // 等待 IMU 上电稳定
  for (int i = 0; i < 50; i++) osDelay(10);

  // ===== MPU6500 倾角基准校准 =====
  // 蜂鸣提示: 升调 1k→1.5k→2kHz (校准开始)
  Buzzer_Beep(1000, 80);
  osDelay(100);
  Buzzer_Beep(1500, 80);
  osDelay(100);
  Buzzer_Beep(2000, 80);
  osDelay(100);

  // 采样 2s 加速度计倾角 (用于笛卡尔曼初始角度)
  float accel_sum = 0.0f;
  const int calib_samples = 200;  // 2s × 100Hz
  for (int i = 0; i < calib_samples; i++) {
      MPU6500_Accel_t accel;
      MPU6500_Gyro_t gyro;
      MPU6500_ReadAll(&accel, &gyro);
      accel_sum += atan2f(accel.y, accel.z) * 57.29578f;
      osDelay(10);
  }
  float accel_mean = accel_sum / (float)calib_samples;  // 安装偏置角 (°)

  // 初始化卡尔曼滤波器
  // Q_angle=0.001: 角度过程噪声, 平衡"响应速度 vs 平滑度"
  // Q_bias=0.003:  零偏过程噪声, 平衡"漂移跟踪速度 vs 稳态噪声"
  // R_measure=0.03: 加速度计观测噪声, 基于MPU6500噪声密度 300μg/√Hz × √92Hz ≈ 0.003g
  KalmanAngle_t kf;
  KalmanAngle_Init(&kf, accel_mean, 0.001f, 0.003f, 0.03f);

  // 应用校准值: 以当前机械直立角为 0° 基准
  g_balance.target_angle = accel_mean;

  // 蜂鸣提示: 降调 (校准完成) — 单长音
  Buzzer_Beep(2000, 200);

  printf("[CAL] Tilt offset=%.2f\r\n", accel_mean);

  HAL_TIM_Base_Start_IT(&htim17);

  static uint32_t tick = 0;
  for (;;) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      tick++;

      // 1. IMU 读取 + 卡尔曼滤波倾角估计 (前后倾斜 = 绕X轴)
      MPU6500_Accel_t accel;
      MPU6500_Gyro_t gyro;
      MPU6500_ReadAll(&accel, &gyro);

      // 卡尔曼预测: 陀螺仪积分 (卡尔曼内部维护零偏估计)
      KalmanAngle_Predict(&kf, gyro.x, dt);

      // 卡尔曼更新: 加速度计观测 (不做EMA预滤波, 卡尔曼本身就是最优滤波器)
      float accel_angle = atan2f(accel.y, accel.z) * 57.29578f;
      KalmanAngle_Update(&kf, accel_angle);

      g_balance.tilt_angle = KalmanAngle_GetAngle(&kf);
      g_balance.gyro_rate  = gyro.x - KalmanAngle_GetBias(&kf);

      // 2. 平衡 PID + 差速混合
      BalanceCtrl_Run(&g_balance);

      // 3. 速度环（双电机交替）
      int order[2];
      if (tick & 1) { order[0] = 1; order[1] = 0; }
      else          { order[0] = 0; order[1] = 1; }

      float speed_refs[2];
      speed_refs[0] = g_balance.speed_ref_l;
      speed_refs[1] = g_balance.speed_ref_r;

      for (int j = 0; j < 2; j++) {
          int i = order[j];
          SpeedCtrl_UpdateRPM(&g_speed[i], g_foc_snap[i].mech_angle,
                               g_foc_snap[i].iq);

          if (g_balance.active) {
              g_speed[i].speed_ref = speed_refs[i];
              if (!g_speed[i].speed_mode) {
                  g_speed[i].speed_mode = 1;
                  PI_Reset(&g_speed[i].pi);
              }
              Motor_SetIqRef(&g_motor[i], SpeedCtrl_Run(&g_speed[i]));
              g_motor[i].speed_mode = 1;
          } else if (g_motor[i].speed_mode) {
              // 平衡已停但speed_mode还在 → 倾倒保护或STOP触发 → 强制停机
              SpeedCtrl_ExitMode(&g_speed[i]);
              g_motor[i].speed_mode = 0;
              Motor_SetIqRef(&g_motor[i], 0.0f);
              Motor_Neutralize(&g_motor[i]);
              if (i == 1) {  // 两个电机都处理后报警一次
                  Buzzer_Beep(4000, 100);
                  printf("[PROTECT] Balance deactivated, motors stopped\r\n");
              }
          }
      }

      // 4. 遥测（可选, 200Hz = 每5次发一帧）
      if (CLI_TelemetryEnabled() && (tick % 5 == 0)) {
          float frame[10];
          frame[0] = g_balance.tilt_angle;              // ch0: 卡尔曼滤波倾角 (°)
          frame[1] = g_balance.gyro_rate;               // ch1: 角速度 (°/s)
          frame[2] = g_balance.balance_out;              // ch2: 平衡PID输出 (RPM)
          frame[3] = g_speed[0].speed_fb;               // ch3: 左轮速度 (RPM)
          frame[4] = g_speed[1].speed_fb;               // ch4: 右轮速度 (RPM)
          frame[5] = g_motor[0].iq;                     // ch5: 左轮电流 (A)
          frame[6] = g_motor[1].iq;                     // ch6: 右轮电流 (A)
          frame[7] = accel.x;                           // ch7: 加速度计X (g)
          frame[8] = accel.y;                           // ch8: 加速度计Y (g)
          frame[9] = accel.z;                           // ch9: 加速度计Z (g)
          COMM_SendFloatFrame(frame, 10);
      }
  }
}

/** @brief FreeRTOS 栈溢出钩子 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    printf("STACK OVERFLOW: %s\r\n", pcTaskName);
    __disable_irq();
    while(1);
}

/** @brief FreeRTOS 内存分配失败钩子 */
void vApplicationMallocFailedHook(void)
{
    printf("MALLOC FAILED: heap exhausted\r\n");
    __disable_irq();
    while(1);
}

/* USER CODE END Application */

