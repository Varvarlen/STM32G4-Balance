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
#include "six_step.h"
#include "motor_hal.h"
#include "svpwm.h"
#include "current_ctrl.h"
#include "encoder_cache.h"
#include "calibration.h"
#include "debug_capture.h"
#include "speed_ctrl.h"
#include "buzzer.h"
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
extern uint8_t g_test_mode;
extern TIM_HandleTypeDef htim17;

// 负载实验状态
typedef struct {
    uint8_t  active;
    uint8_t  motor_idx;
    uint8_t  phase;
    uint32_t phase_start;
    float    rpm;
} LoadTest_t;
static LoadTest_t g_load_test;
/* USER CODE END Variables */
osThreadId TaskCLIHandle;
osThreadId TaskSpeedLoopHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void TaskDebugCapture(void const *argument);
/* USER CODE END FunctionPrototypes */

void StartCLITask(void const * argument);
void StartTaskTelemetry(void const * argument);
void StartTaskIMU(void const * argument);
void StartTaskSpeedLoop(void const * argument);

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
      osThreadDef(TaskCLI, StartCLITask, osPriorityNormal, 0, 192);
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

/* USER CODE BEGIN Header_StartCLITask */
/**
  * @brief  Function implementing the TaskCLI thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCLITask */
void StartCLITask(void const * argument)
{
  /* USER CODE BEGIN StartCLITask */
  (void)argument;
  for(;;)
  {
    // 负载实验状态机
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

    while (COMM_Available() > 0)
    {
      uint8_t ch = COMM_ReadByte();
      if (g_calib_mode) {
          switch (ch)
          {
          case 'c': case 'C':
          case 'd': case 'D':
          {
              uint8_t motor_idx = (ch == 'd' || ch == 'D') ? 1 : 0;
              for (int wait = 0; wait < 50 && COMM_Available() == 0; wait++)
                  osDelay(1);
              ch = COMM_ReadByte();
              if (ch < '1' || ch > '5') break;
              if (!CALIB_TryLock()) {
                  printf("Calibration busy, wait or press 'q' to abort\r\n");
                  break;
              }
              printf("=== M%d calib #%c ===\r\n", motor_idx + 1, ch);
              switch (ch) {
              case '1': CALIB_CurrentOffset(); break;
              case '2': CALIB_PhaseWireMap(&g_motor[motor_idx]); break;
              case '3': CALIB_EncoderDir(&g_motor[motor_idx]); break;
              case '4': CALIB_EncoderOffset(&g_motor[motor_idx]); break;
              case '5': CALIB_MotorParams(&g_motor[motor_idx]); break;
              }
              break;
          }
          case 's': case 'S':
              CALIB_PrintParams(&g_calib);
              break;
          case 'q': case 'Q':
              CALIB_Abort();
              printf("CALIB ABORT requested\r\n");
              break;
          case '\r': case '\n':
              break;
          default:
              printf("calib: c1-5=M1 d1-5=M2 s=params q=abort\r\n");
              break;
          }
      } else if (g_test_mode) {
          if (ch == 'r' || ch == 'R') {
              DebugCapture_Resend();
          } else if (ch == 'S' || ch == 's') {
              uint8_t cmd = 0;
              for (int w = 0; w < 20 && COMM_Available() == 0; w++) osDelay(1);
              cmd = COMM_ReadByte();
              uint8_t motor_idx = (cmd == 'L' || cmd == 'l') ? 1 : 0;
              if (cmd != 'R' && cmd != 'r' && cmd != 'L' && cmd != 'l') continue;
              char buf[16]; uint8_t pos = 0;
              for (int w = 0; w < 30 && pos < 15; w++) {
                  if (COMM_Available() == 0) { osDelay(1); continue; }
                  uint8_t c = COMM_ReadByte();
                  if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+')
                      buf[pos++] = (char)c;
                  else break;
              }
              buf[pos] = '\0';
              if (pos > 0) DebugCapture_Start(motor_idx, (float)atof(buf));
          }
      } else {
          // E/F 负载实验 — 自动化时序: 斜坡2s → 蜂鸣器3s → 恢复3s
          if (ch == 'E' || ch == 'e' || ch == 'F' || ch == 'f') {
              if (g_load_test.active) { printf("Busy\r\n"); continue; }
              uint8_t motor_idx = (ch == 'F' || ch == 'f') ? 1 : 0;
              char buf[16]; uint8_t pos = 0;
              for (int w = 0; w < 30 && pos < 15; w++) {
                  if (COMM_Available() == 0) { osDelay(1); continue; }
                  uint8_t c = COMM_ReadByte();
                  if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+')
                      buf[pos++] = (char)c;
                  else break;
              }
              buf[pos] = '\0';
              if (pos > 0) {
                  float rpm = (float)atof(buf);
                  SpeedCtrl_EnterMode(&g_speed[motor_idx], rpm);
                  g_motor[motor_idx].speed_mode = 1;
                  g_load_test.active = 1;
                  g_load_test.motor_idx = motor_idx;
                  g_load_test.phase = 0;
                  g_load_test.phase_start = xTaskGetTickCount();
                  g_load_test.rpm = rpm;
                  Buzzer_Beep(1000, 80);
                  printf("TEST M%d @ %d RPM\r\n", motor_idx + 1, (int)rpm);
              }
              continue;
          }

          // V/W 速度指令
          if (ch == 'V' || ch == 'v' || ch == 'W' || ch == 'w') {
              uint8_t motor_idx = (ch == 'W' || ch == 'w') ? 1 : 0;
              char buf[16]; uint8_t pos = 0;
              for (int w = 0; w < 30 && pos < 15; w++) {
                  if (COMM_Available() == 0) { osDelay(1); continue; }
                  uint8_t c = COMM_ReadByte();
                  if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+')
                      buf[pos++] = (char)c;
                  else break;
              }
              buf[pos] = '\0';
              if (pos > 0) {
                  float rpm = (float)atof(buf);
                  SpeedCtrl_EnterMode(&g_speed[motor_idx], rpm);
                  g_motor[motor_idx].speed_mode = 1;
              }
              continue;
          }
          // R/L 电流指令
          if (ch == 'R' || ch == 'r' || ch == 'L' || ch == 'l') {
              uint8_t next_ch = ch;
              do {
                  uint8_t motor_idx = (next_ch == 'L' || next_ch == 'l') ? 1 : 0;
                  char buf[16]; uint8_t pos = 0; uint8_t term = 0;
                  for (int w = 0; w < 30 && pos < 15; w++) {
                      if (COMM_Available() == 0) { osDelay(1); continue; }
                      uint8_t c = COMM_ReadByte();
                      if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+') {
                          buf[pos++] = (char)c;
                      } else { term = c; break; }
                  }
                  buf[pos] = '\0';
                  if (pos > 0) {
                      if (g_motor[motor_idx].speed_mode) {
                          SpeedCtrl_ExitMode(&g_speed[motor_idx]);
                          g_motor[motor_idx].speed_mode = 0;
                      }
                      Motor_SetIqRef(&g_motor[motor_idx], (float)atof(buf));
                  }
                  next_ch = term;
              } while (next_ch == 'R' || next_ch == 'r' || next_ch == 'L' || next_ch == 'l');
          }
      }
    }
    osDelay(1);
  }
  /* USER CODE END StartCLITask */
}

/* USER CODE BEGIN Header_StartTaskTelemetry */
/**
* @brief Function implementing the TaskTelemetry thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskTelemetry */
void StartTaskTelemetry(void const * argument)
{
  /* USER CODE BEGIN StartTaskTelemetry */
  (void)argument;
  for(;;)
  {
    float frame[10];
    for (int i = 0; i < 2; i++) {
        frame[i*5+0] = g_speed[i].speed_ref_ramp;
        frame[i*5+1] = g_speed[i].speed_fb;
        frame[i*5+2] = g_motor[i].iq_ref;
        frame[i*5+3] = g_motor[i].iq;
        frame[i*5+4] = g_enc[i].mech_angle;
    }
    COMM_SendFloatFrame(frame, 10);
    osDelay(10);
  }
  /* USER CODE END StartTaskTelemetry */
}

/* USER CODE BEGIN Header_StartTaskIMU */
/**
* @brief Function implementing the TaskIMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskIMU */
void StartTaskIMU(void const * argument)
{
  /* USER CODE BEGIN StartTaskIMU */
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
  /* USER CODE END StartTaskIMU */
}

/* USER CODE BEGIN Header_StartTaskSpeedLoop */
/**
* @brief Function implementing the TaskSpeedLoop thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskSpeedLoop */
void StartTaskSpeedLoop(void const * argument)
{
  /* USER CODE BEGIN StartTaskSpeedLoop */
  (void)argument;
  SpeedCtrl_Init(&g_speed[0], SPEED_PI_DEFAULT_KP, SPEED_PI_DEFAULT_KI,
                 2.0f, -2.0f, MT6701_GetEncDirection(0),
                 MOTOR_KT, MOTOR_J);
  SpeedCtrl_Init(&g_speed[1], SPEED_PI_DEFAULT_KP, SPEED_PI_DEFAULT_KI,
                 2.0f, -2.0f, MT6701_GetEncDirection(1),
                 MOTOR_KT, MOTOR_J);

  // 启动 TIM17 必须在任务内进行 — 此时 TaskSpeedLoopHandle 已有效
  // 若在 main.c 中启动，TIM17 首帧中断可能在 osKernelStart 前触发，
  // 导致 vTaskNotifyGiveFromISR(NULL) 触发 configASSERT 死锁
  HAL_TIM_Base_Start_IT(&htim17);

  for (;;) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

      for (int i = 0; i < 2; i++) {
          SpeedCtrl_UpdateRPM(&g_speed[i], g_enc[i].mech_angle,
                               g_motor[i].iq);
          if (g_motor[i].speed_mode) {
              float iq_ref = SpeedCtrl_Run(&g_speed[i]);
              Motor_SetIqRef(&g_motor[i], iq_ref);
          }
      }
  }
  /* USER CODE END StartTaskSpeedLoop */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  FreeRTOS 栈溢出钩子 — 输出诊断信息后停机
  */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    printf("STACK OVERFLOW: %s\r\n", pcTaskName);
    __disable_irq();
    while(1);
}

/**
  * @brief  FreeRTOS 内存分配失败钩子 — 输出诊断信息后停机
  */
void vApplicationMallocFailedHook(void)
{
    printf("MALLOC FAILED: heap exhausted\r\n");
    __disable_irq();
    while(1);
}

/**
  * @brief  阶跃测试任务 — 采集完成后下传数据
  */
void TaskDebugCapture(void const *argument)
{
    (void)argument;
    DebugCapture_Init();
    for (;;) {
        DebugCapture_Task();
    }
}
/* USER CODE END Application */

