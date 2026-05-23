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
#include "vbus.h"
#include "bt_comm.h"
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
extern IWDG_HandleTypeDef hiwdg;  /**< 独立看门狗句柄，main.c 定义 */

// 运行时故障/状态计数器 (CLI help 可查看)
uint32_t g_imu_fault_total   = 0;  /**< IMU SPI 累计故障次数 */
uint32_t g_nan_fault_cnt     = 0;  /**< NaN 累计检测次数 */
uint32_t g_cycle_overrun_cnt = 0;  /**< 平衡循环超时 (>1ms) 累计次数 */
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
      osThreadDef(TaskCLI, StartCLITask, osPriorityNormal, 0, 512);
      TaskCLIHandle = osThreadCreate(osThread(TaskCLI), NULL);
      osThreadDef(TaskBalance, StartBalanceLoopTask, osPriorityHigh, 0, 896);
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

  SpeedCtrl_Init(&g_speed[MOTOR_LEFT], SPEED_PI_DEFAULT_KP, SPEED_PI_DEFAULT_KI,
                 2.0f, -2.0f, MT6701_GetEncDirection(MOTOR_LEFT),
                 MOTOR_KT, MOTOR_J);
  SpeedCtrl_Init(&g_speed[MOTOR_RIGHT], SPEED_PI_DEFAULT_KP, SPEED_PI_DEFAULT_KI,
                 2.0f, -2.0f, MT6701_GetEncDirection(MOTOR_RIGHT),
                 MOTOR_KT, MOTOR_J);

  MPU6500_SetAccelRange(MPU6500_ACCEL_RANGE_4G);

  const float dt = 0.001f;  // 1ms

  // 等待 IMU 上电稳定
  for (int i = 0; i < 20; i++) osDelay(10);

  // 初始化卡尔曼滤波器 (机械平衡点 0°, 无需校准)
  KalmanAngle_t kf;
  g_balance.target_angle = 0.0f;

  {
      // 多次采样加速度计, 获取稳态初始倾角 (避免单次瞬时噪声)
      float accel_sum = 0.0f;
      for (int i = 0; i < 10; i++) {
          MPU6500_Accel_t accel;
          MPU6500_Gyro_t gyro;
          MPU6500_ReadAll(&accel, &gyro);
          accel_sum += atan2f(accel.y, accel.z) * RAD_TO_DEG;
          osDelay(10);
      }
      float init_tilt = accel_sum / 10.0f;

      KalmanAngle_Init(&kf, init_tilt, 1e-6f, 0.003f, 0.03f);

      if (fabsf(init_tilt) <= 10.0f) {
          g_balance.active = 1;
          g_balance.gyro_filt = 0.0f;
          for (int i = 0; i < 2; i++) {
              g_motor[i].mode = MOTOR_MODE_CURRENT_LOOP;
              PI_Reset(&g_motor[i].id_pi);
              PI_Reset(&g_motor[i].iq_pi);
              SpeedCtrl_EnterMode(&g_speed[i], 0.0f);
              g_motor[i].speed_mode = 1;
          }
          Buzzer_Sweep(800, 2000, 400);
          printf("[INIT] Tilt=%.1f, auto-balance ON\r\n", init_tilt);
      } else {
          Buzzer_Sweep(2000, 800, 400);
          printf("[INIT] Tilt=%.1f > 10, send B to start\r\n", init_tilt);
      }
  }

  HAL_TIM_Base_Start_IT(&htim17);

  // 启用 DWT 周期计数器 (用于控制循环超时检测)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  static uint32_t tick = 0;
  static uint8_t imu_fault_cnt = 0;
  static uint8_t imu_faulted = 0;
  for (;;) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      uint32_t t_start = DWT->CYCCNT;  // 记录本周期起始时刻, DWT 自由运行不重置
      tick++;

      // 1. IMU 读取 + 卡尔曼滤波倾角估计 (前后倾斜 = 绕X轴)
      MPU6500_Accel_t accel;
      MPU6500_Gyro_t gyro;
      if (MPU6500_ReadAll(&accel, &gyro) != 0) {
          imu_fault_cnt++;
          if (imu_fault_cnt >= 5 && !imu_faulted) {
              imu_faulted = 1;
              g_imu_fault_total++;
              g_balance.active = 0;
              for (int i = 0; i < 2; i++) {
                  g_motor[i].speed_mode = 0;
                  Motor_SetIqRef(&g_motor[i], 0.0f);
                  Motor_Neutralize(&g_motor[i]);
              }
              printf("[FAULT] IMU SPI timeout x5, motors stopped\r\n");
              Buzzer_Beep(4000, 200);
          }
          continue;
      }
      if (imu_faulted) {
          imu_faulted = 0;
          printf("[FAULT] IMU recovered\r\n");
      }
      imu_fault_cnt = 0;

      // 卡尔曼预测: 陀螺仪积分 (卡尔曼内部维护零偏估计)
      KalmanAngle_Predict(&kf, gyro.x, dt);

      // 卡尔曼更新: 加速度计观测 (不做EMA预滤波, 卡尔曼本身就是最优滤波器)
      float accel_angle = atan2f(accel.y, accel.z) * RAD_TO_DEG;
      KalmanAngle_Update(&kf, accel_angle);

      g_balance.tilt_angle = KalmanAngle_GetAngle(&kf);
      g_balance.gyro_rate  = gyro.x - KalmanAngle_GetBias(&kf);

      // 2. 速度外环 (100Hz): 差分测速 + PI → target_angle
      {
          static float last_pos[2] = {0.0f, 0.0f};
          static float speed_i = 0.0f;
          static uint8_t pos_valid = 0;

          // STOP 或保护后清零积分和位置历史
          if (!g_balance.active) {
              speed_i = 0.0f;
              pos_valid = 0;
              g_balance.target_angle = 0.0f;
          } else if (tick % SPEED_OUTER_DIV == 0) {
              COMPILER_BARRIER();  // CLI 可能已修改 g_speed_outer_kp/ki
              float avg_speed = 0.0f;
              if (pos_valid) {
                  for (int i = 0; i < 2; i++) {
                      float delta = g_foc_snap[i].mech_angle - last_pos[i];
                      if (delta > M_PI) delta -= 2.0f * M_PI;
                      if (delta < -M_PI) delta += 2.0f * M_PI;
                      avg_speed += delta / SPEED_OUTER_DT * RPM_FROM_RADPS * g_speed[i].enc_dir;
                  }
                  avg_speed *= 0.5f;

                  float err = avg_speed - g_balance.target_speed;
                  speed_i += g_speed_outer_ki * err * SPEED_OUTER_DT;
                  if (speed_i >  SPEED_OUTER_MAX) speed_i =  SPEED_OUTER_MAX;
                  if (speed_i < -SPEED_OUTER_MAX) speed_i = -SPEED_OUTER_MAX;

                  g_balance.target_angle = g_speed_outer_kp * err + speed_i;
                  if (g_balance.target_angle >  SPEED_OUTER_MAX)
                      g_balance.target_angle =  SPEED_OUTER_MAX;
                  if (g_balance.target_angle < -SPEED_OUTER_MAX)
                      g_balance.target_angle = -SPEED_OUTER_MAX;
              }
              for (int i = 0; i < 2; i++) {
                  last_pos[i] = g_foc_snap[i].mech_angle;
              }
              pos_valid = 1;
          }
      }

      // 3a. 偏航角度积分 (1kHz): 编码器差速→连续角度
      {
          if (!g_balance.active) {
              g_balance.yaw_angle = 0.0f;
              g_balance.target_yaw_angle = 0.0f;
              g_balance.yaw_mode = 0;
          } else {
              float odom_rate = (g_speed[MOTOR_RIGHT].speed_fb - g_speed[MOTOR_LEFT].speed_fb) * YAW_RPM_TO_DPS;
              g_balance.yaw_angle += odom_rate * dt;  // dt=0.001f
          }
      }

      // 3b. 偏航控制 (50Hz): 互补滤波 + 角度外环(25Hz) + 速率PI → steer
      {
          static float gyro_bias_z = 0.0f;
          static float yaw_i = 0.0f;
          static float yaw_angle_i = 0.0f;  // 角度外环积分
          static float target_yaw_rate = 0.0f;  // 角度外环输出, 速率PI的目标

          if (!g_balance.active) {
              gyro_bias_z = 0.0f;
              yaw_i = 0.0f;
              yaw_angle_i = 0.0f;
              target_yaw_rate = 0.0f;
              g_balance.steer = 0.0f;
          } else if (tick % YAW_OUTER_DIV == 0) {
              // 互补滤波: 陀螺偏置缓慢收敛到 (gyro.z - odom_rate)
              float odom_rate = (g_speed[MOTOR_RIGHT].speed_fb - g_speed[MOTOR_LEFT].speed_fb) * YAW_RPM_TO_DPS;
              gyro_bias_z += YAW_COMP_ALPHA * (gyro.z - odom_rate - gyro_bias_z);

              // 偏航角度外环 (25Hz): angle_error → target_yaw_rate (PI)
              if (tick % YAW_ANGLE_OUTER_DIV == 0) {
                  COMPILER_BARRIER();  // g_yaw_angle_kp/ki 由 CLI 写入
                  float angle_err = g_balance.target_yaw_angle - g_balance.yaw_angle;
                  if (isnan(angle_err)) { angle_err = 0.0f; g_nan_fault_cnt++; }
                  yaw_angle_i += g_yaw_angle_ki * angle_err * YAW_ANGLE_OUTER_DT;
                  if (yaw_angle_i >  YAW_ANGLE_MAX_I) yaw_angle_i =  YAW_ANGLE_MAX_I;
                  if (yaw_angle_i < -YAW_ANGLE_MAX_I) yaw_angle_i = -YAW_ANGLE_MAX_I;
                  target_yaw_rate = g_yaw_angle_kp * angle_err + yaw_angle_i;
                  if (target_yaw_rate >  YAW_ANGLE_MAX_RATE) target_yaw_rate =  YAW_ANGLE_MAX_RATE;
                  if (target_yaw_rate < -YAW_ANGLE_MAX_RATE) target_yaw_rate = -YAW_ANGLE_MAX_RATE;
              }

              // 偏航速率 PI (50Hz): yaw_rate error → steer
              {
                  COMPILER_BARRIER();  // g_yaw_kp/ki 由 CLI 写入
                  float yaw_rate = gyro.z - gyro_bias_z;
                  float err = yaw_rate - target_yaw_rate;  // 正yaw→正steer→CW→对抗CCW
                  yaw_i += g_yaw_ki * err * YAW_OUTER_DT;
                  if (yaw_i >  YAW_PI_MAX) yaw_i =  YAW_PI_MAX;
                  if (yaw_i < -YAW_PI_MAX) yaw_i = -YAW_PI_MAX;

                  g_balance.steer = g_yaw_kp * err + yaw_i;
                  if (g_balance.steer >  YAW_PI_MAX) g_balance.steer =  YAW_PI_MAX;
                  if (g_balance.steer < -YAW_PI_MAX) g_balance.steer = -YAW_PI_MAX;
              }
          }
      }

      // 4. 平衡 PID + 差速混合 (target_angle 含速度外环, steer 含偏航PI)
      BalanceCtrl_Run(&g_balance);

      // 5. 直接力矩 + 差速转向
      int order[2];
      if (tick & 1) { order[0] = 1; order[1] = 0; }
      else          { order[0] = 0; order[1] = 1; }

      for (int j = 0; j < 2; j++) {
          int i = order[j];
          // EKF 速度估计 (保留用于遥测 ch3/ch4)
          SpeedCtrl_UpdateRPM(&g_speed[i], g_foc_snap[i].mech_angle,
                               g_foc_snap[i].iq);

          if (g_balance.active) {
              // 差速转矩转向: 左轮 +steer, 右轮 -steer
              float iq_base = BALANCE_DIRECT_GAIN * g_balance.balance_out;
              float iq_diff = BALANCE_DIRECT_GAIN * g_balance.steer;
              float iq_cmd = (i == MOTOR_LEFT) ? iq_base + iq_diff
                                              : iq_base - iq_diff;
              if (isnan(iq_cmd)) { iq_cmd = 0.0f; g_nan_fault_cnt++; }
              if (isinf(iq_cmd)) { iq_cmd = (iq_cmd > 0.0f) ? 2.0f : -2.0f; g_nan_fault_cnt++; }
              if (iq_cmd >  2.0f) iq_cmd =  2.0f;
              if (iq_cmd < -2.0f) iq_cmd = -2.0f;
              Motor_SetIqRef(&g_motor[i], iq_cmd);
          } else if (g_motor[i].speed_mode) {
              g_motor[i].speed_mode = 0;
              Motor_SetIqRef(&g_motor[i], 0.0f);
              Motor_Neutralize(&g_motor[i]);
              if (i == 1) {
                  Buzzer_Beep(3000, 100);
                  printf("[PROTECT] Balance deactivated, motors stopped\r\n");
              }
          }
      }

      // 5. IWDG 喂狗 + 欠压保护 + SVPWM Vbus 更新 (每 100ms)
      if (tick % 100 == 0) {
          HAL_IWDG_Refresh(&hiwdg);  // 独立看门狗喂狗 (8s 超时, 100ms 喂一次)
          float vbus = VBUS_Read();
          g_foc_vbus = vbus;  // 更新 SVPWM 母线电压缓存
          if (vbus <= 6.4f) {
              if (g_balance.active) {
                  g_balance.active = 0;
                  for (int i = 0; i < 2; i++) {
                      g_motor[i].speed_mode = 0;
                      Motor_SetIqRef(&g_motor[i], 0.0f);
                      Motor_Neutralize(&g_motor[i]);
                  }
                  printf("[FAULT] VBUS=%.2fV low, motors stopped\r\n", vbus);
              }
              if (tick % 100 == 0) Buzzer_Beep(4000, 80);
          } else if (vbus <= 6.6f) {
              if (tick % 800 == 0) Buzzer_Beep(3000, 80);
          }
      }

      // 6. 遥测（可选, 200Hz = 每5次发一帧）
      if (CLI_TelemetryEnabled() && (tick % 5 == 0)) {
          float frame[10];
          frame[0] = g_balance.tilt_angle;              // ch0: 倾角 (°)
          frame[1] = g_balance.gyro_rate;               // ch1: 角速度 (°/s)
          frame[2] = g_balance.balance_out;              // ch2: 平衡PID输出 (RPM)
          frame[3] = g_speed[MOTOR_RIGHT].speed_fb;       // ch3: 右轮速度 (RPM)
          frame[4] = g_speed[MOTOR_LEFT].speed_fb;        // ch4: 左轮速度 (RPM)
          frame[5] = g_motor[MOTOR_RIGHT].iq;             // ch5: 右轮电流 (A)
          frame[6] = g_motor[MOTOR_LEFT].iq;              // ch6: 左轮电流 (A)
          frame[7] = g_balance.target_angle;            // ch7: 目标倾角 (°)
          frame[8] = g_balance.yaw_angle;                   // ch8: 偏航角度 (°)
          frame[9] = g_balance.target_yaw_angle;          // ch9: 目标偏航角度 (°)
          COMM_SendFloatFrame(frame, 10);
      }

      // 6.5 蓝牙遥测 (50Hz = 每20 tick推送, 手机控制帧 bit2 开关)
      if (g_bt_telem_enabled && (tick % 20 == 0)) {
          int16_t avg_speed = (int16_t)((g_balance.speed_ref_l + g_balance.speed_ref_r) / 2.0f);
          int16_t vbus_mv = (int16_t)(VBUS_Read() * 100.0f);
          int32_t uptime = (int32_t)(xTaskGetTickCount() / 1000);
          uint8_t bt_flags = 0;
          if (g_balance.active) bt_flags |= 0x01;
          BT_SendTelemetryFrame(g_balance.tilt_angle, avg_speed, vbus_mv, uptime, bt_flags);
      }

      // 7. 控制循环超时检测 (>1ms @170MHz → 丢帧)
      if ((DWT->CYCCNT - t_start) > 170000) {
          g_cycle_overrun_cnt++;
      }

      // 8. 蜂鸣器非阻塞到期检查
      Buzzer_Update();
  }
}

/** @brief FreeRTOS 栈溢出钩子 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    printf("STACK OVERFLOW: %s\r\n", pcTaskName);
    Fault_DisableMotors();
    __disable_irq();
    while(1);
}

/** @brief FreeRTOS 内存分配失败钩子 */
void vApplicationMallocFailedHook(void)
{
    printf("MALLOC FAILED: heap exhausted\r\n");
    Fault_DisableMotors();
    __disable_irq();
    while(1);
}

/* USER CODE END Application */

