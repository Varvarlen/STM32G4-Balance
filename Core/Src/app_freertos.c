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
#include "mpu6050.h"
#include "kf_angle.h"
#include "comm_protocol.h"
#include "comm.h"
#include "foc.h"

#include "motor_hal.h"
#include "svpwm.h"
#include "current_ctrl.h"
#include "encoder_cache.h"
#include "calibration.h"
#include "debug_capture.h"
#include "speed_ctrl.h"
#include "speed_capture.h"
#include "buzzer.h"
#include "cli_parser.h"
#include "cli.h"
#include "pos_ctrl.h"
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
PosCtrl_t g_pos[2];
osThreadId TaskCLIHandle;
osThreadId TaskSpeedLoopHandle;
extern uint8_t g_test_mode;
extern volatile uint8_t g_capture_dumping;
extern TIM_HandleTypeDef htim17;
/* USER CODE END Variables */
osThreadId DefaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartCLITask(void const * argument);
void StartTaskTelemetry(void const * argument);
void StartTaskIMU(void const * argument);
void StartTaskSpeedLoop(void const * argument);
void TaskDebugCapture(void const *argument);
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
  } else if (g_test_mode) {
      osThreadDef(TaskCLI, StartCLITask, osPriorityNormal, 0, 384);
      TaskCLIHandle = osThreadCreate(osThread(TaskCLI), NULL);
      osThreadDef(debugCaptureTask, TaskDebugCapture, osPriorityNormal, 0, 512);
      osThreadCreate(osThread(debugCaptureTask), NULL);
  } else {
      osThreadDef(TaskCLI, StartCLITask, osPriorityNormal, 0, 256);
      TaskCLIHandle = osThreadCreate(osThread(TaskCLI), NULL);
      osThreadDef(TaskSpeedLoop, StartTaskSpeedLoop, osPriorityHigh, 0, 512);
      TaskSpeedLoopHandle = osThreadCreate(osThread(TaskSpeedLoop), NULL);
      osThreadDef(TaskTelemetry, StartTaskTelemetry, osPriorityNormal, 0, 320);
      osThreadCreate(osThread(TaskTelemetry), NULL);
      osThreadDef(TaskIMU, StartTaskIMU, osPriorityNormal, 0, 384);
      osThreadCreate(osThread(TaskIMU), NULL);
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

/** @brief CLI 任务 — 串口命令处理 + 负载实验/阶跃测试状态机 */
void StartCLITask(void const * argument)
{
  (void)argument;
  CLI_Init();
  for(;;)
  {
    if (g_load_test.active) {
        uint32_t elapsed = xTaskGetTickCount() - g_load_test.phase_start;
        switch (g_load_test.phase) {
        case 0:
            if (elapsed >= 2000) {
                g_load_test.phase = 1;
                g_load_test.phase_start = xTaskGetTickCount();
                Buzzer_Beep(2000, 0);
                printf("BUZZER ON [M%d]\r\n", g_load_test.motor_idx + 1);
            }
            break;
        case 1:
            if (elapsed >= 3000) {
                g_load_test.phase = 2;
                g_load_test.phase_start = xTaskGetTickCount();
                Buzzer_Stop();
                printf("BUZZER OFF [M%d]\r\n", g_load_test.motor_idx + 1);
            }
            break;
        case 2:
            if (elapsed >= 3000) {
                uint8_t mi = g_load_test.motor_idx;
                SpeedCtrl_ExitMode(&g_speed[mi]);
                g_motor[mi].speed_mode = 0;
                Motor_SetIqRef(&g_motor[mi], 0.0f);
                g_load_test.active = 0;
                Buzzer_Beep(1000, 80);
                printf("LOAD TEST M%d OK\r\n", mi + 1);
            }
            break;
        }
    }

    if (g_step_test.active) {
        uint32_t elapsed = xTaskGetTickCount() - g_step_test.phase_start;
        uint8_t mi = g_step_test.motor_idx;
        switch (g_step_test.phase) {
        case 0:
            if (elapsed >= 500) {
                SpeedCapture_Start(SC_BURST_SAMPLES);
                g_step_test.phase = 1;
                g_step_test.phase_start = xTaskGetTickCount();
            }
            break;
        case 1:
            if (elapsed >= 50) {
                g_speed[mi].speed_ref = g_step_test.to_rpm;
                g_speed[mi].speed_ref_ramp = g_step_test.to_rpm;
                g_step_test.phase = 2;
                g_step_test.phase_start = xTaskGetTickCount();
            }
            break;
        case 2:
            if (SpeedCapture_IsReady() || elapsed >= 2000) {
                g_step_test.phase = 3;
                g_step_test.phase_start = xTaskGetTickCount();
                SpeedCtrl_ExitMode(&g_speed[mi]);
                g_motor[mi].speed_mode = 0;
                g_speed[mi].no_ramp = 0;
                Motor_SetIqRef(&g_motor[mi], 0.0f);
            }
            break;
        case 3:
            SpeedCapture_Dump();
            g_step_test.active = 0;
            printf("STEP M%d done (%d samples)\r\n", mi+1, SC_BURST_SAMPLES);
            break;
        }
    }

    CLI_Process();
    osDelay(1);
  }
}

/** @brief 遥测任务 — 200Hz 11 通道浮点帧 */
void StartTaskTelemetry(void const * argument)
{
  (void)argument;
  for(;;)
  {
    if (!CLI_TelemetryEnabled()) {
        osDelay(5);
        continue;
    }
    if (g_capture_dumping) {
        osDelay(1);
        continue;
    }
    float frame[11];
    for (int i = 0; i < 2; i++) {
        // 位置环观测: pos_ref | pos_est | speed_fb | iq | speed_ref
        frame[i*5+0] = g_pos[i].active ? g_pos[i].pos_ref : g_speed[i].speed_ref_ramp;
        frame[i*5+1] = g_speed[i].pos_est;
        frame[i*5+2] = g_speed[i].speed_fb;
        frame[i*5+3] = g_motor[i].iq;
        frame[i*5+4] = g_speed[i].speed_ref;
    }
    frame[10] = g_enc[0].mech_angle * MT6701_GetEncDirection(0);  // ch11: M1 方向校正后机械角 (rad)
    COMM_SendFloatFrame(frame, 11);
    osDelay(5);
  }
}

/** @brief IMU 任务 — MPU6050 姿态角解算 */
void StartTaskIMU(void const * argument)
{
  (void)argument;
  MPU6050_SetAccelRange(MPU6050_ACCEL_RANGE_4G);

  KalmanAngle_t kf;
  KalmanAngle_Init(&kf, 0.0f, 0.001f, 0.003f, 0.03f);
  const float dt = 0.01f;

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

/** @brief 速度环任务 — TIM17 1kHz 触发, EKF + PI + 采集 */
void StartTaskSpeedLoop(void const * argument)
{
  (void)argument;
  SpeedCapture_Init();
  SpeedCtrl_Init(&g_speed[0], SPEED_PI_DEFAULT_KP, SPEED_PI_DEFAULT_KI,
                 2.0f, -2.0f, MT6701_GetEncDirection(0),
                 MOTOR_KT, MOTOR_J);
  SpeedCtrl_Init(&g_speed[1], SPEED_PI_DEFAULT_KP, SPEED_PI_DEFAULT_KI,
                 2.0f, -2.0f, MT6701_GetEncDirection(1),
                 MOTOR_KT, MOTOR_J);

  PosCtrl_Init(&g_pos[0], POS_P_DEFAULT_KP, POS_SPEED_MAX);
  PosCtrl_Init(&g_pos[1], POS_P_DEFAULT_KP, POS_SPEED_MAX);

  HAL_TIM_Base_Start_IT(&htim17);  // 任务内启动, 句柄已有效

  static uint32_t speed_loop_tick = 0;
  for (;;) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      speed_loop_tick++;

      // 交替轮询: 偶数周期 M0→M1, 奇数周期 M1→M0
      int order[2];
      if (speed_loop_tick & 1) { order[0] = 1; order[1] = 0; }
      else                     { order[0] = 0; order[1] = 1; }

      for (int j = 0; j < 2; j++) {
          int i = order[j];
          SpeedCtrl_UpdateRPM(&g_speed[i], g_foc_snap[i].mech_angle,
                               g_foc_snap[i].iq);
          if (g_pos[i].active) {
              // 位置模式: P → 级联速度 PI, 反馈用编码器原始值 (meas_cont)
              float pos_fb = SpeedCtrl_GetPosition(&g_speed[i]);
              g_speed[i].speed_ref = PosCtrl_Run(&g_pos[i], pos_fb);
              // 速度环 SPEED_RAMP_MAX (20000 RPM/s) 接管斜坡平滑
              if (!g_speed[i].speed_mode) {
                  g_speed[i].speed_mode = 1;
                  PI_Reset(&g_speed[i].pi);
              }
              float iq_ref = SpeedCtrl_Run(&g_speed[i]);
              Motor_SetIqRef(&g_motor[i], iq_ref);
              g_motor[i].speed_mode = 1;
          } else if (g_motor[i].speed_mode) {
              float iq_ref = SpeedCtrl_Run(&g_speed[i]);
              Motor_SetIqRef(&g_motor[i], iq_ref);
          }
          if (g_step_test.active && g_step_test.motor_idx == i) {
              SpeedCapture_Write(g_speed[i].speed_fb, g_motor[i].iq,
                                 g_speed[i].speed_ref, g_speed[i].t_load_est);
          }
      }
  }
}

/** @brief 阶跃测试 debug 任务 — 采集完成后下传数据 */
void TaskDebugCapture(void const *argument)
{
    (void)argument;
    DebugCapture_Init();
    for (;;) {
        DebugCapture_Task();
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

