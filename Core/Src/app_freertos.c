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
osThreadId mpuTaskHandle;
volatile uint8_t g_trigger_report;
extern uint8_t g_test_mode;
/* USER CODE END Variables */
osThreadId TaskCLIHandle;
osThreadId TaskTelemetryHandle;
osThreadId TaskIMUHandle;
osThreadId TaskSpeedLoopHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void TaskMPU6500(void const * argument);
void TaskSixStep(void const * argument);
void TaskVoltageSine(void const * argument);
void TaskCurrentLoop(void const *argument);
void TaskSpeedReport(void const *argument);
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

  /* Create the thread(s) */
  /* definition and creation of TaskCLI */
  osThreadDef(TaskCLI, StartCLITask, osPriorityNormal, 0, 192);
  TaskCLIHandle = osThreadCreate(osThread(TaskCLI), NULL);

  /* definition and creation of TaskTelemetry */
  osThreadDef(TaskTelemetry, StartTaskTelemetry, osPriorityLow, 0, 320);
  TaskTelemetryHandle = osThreadCreate(osThread(TaskTelemetry), NULL);

  /* definition and creation of TaskIMU */
  osThreadDef(TaskIMU, StartTaskIMU, osPriorityNormal, 0, 384);
  TaskIMUHandle = osThreadCreate(osThread(TaskIMU), NULL);

  /* definition and creation of TaskSpeedLoop */
  osThreadDef(TaskSpeedLoop, StartTaskSpeedLoop, osPriorityHigh, 0, 512);
  TaskSpeedLoopHandle = osThreadCreate(osThread(TaskSpeedLoop), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  if (g_calib_mode) {
      // 校准模式：printf 浮点格式化需要大栈 (newlib ~800B)
      osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
      defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
  } else if (g_test_mode) {
      // 阶跃测试模式：CLI + 采集任务，无遥测输出
      osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 384);
      defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
      osThreadDef(debugCaptureTask, TaskDebugCapture, osPriorityNormal, 0, 512);
      osThreadCreate(osThread(debugCaptureTask), NULL);
  } else {
      // 正常模式：小栈 + 完整任务
      osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
      defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
      osThreadDef(mpuTask, TaskMPU6500, osPriorityNormal, 0, 384);
      mpuTaskHandle = osThreadCreate(osThread(mpuTask), NULL);
      // osThreadDef(voltageSineTask, TaskVoltageSine, osPriorityNormal, 0, 384);
      // osThreadCreate(osThread(voltageSineTask), NULL);
      // osThreadDef(speedReportTask, TaskSpeedReport, osPriorityNormal, 0, 256);
      // osThreadCreate(osThread(speedReportTask), NULL);
      // osThreadDef(sixStepTask, TaskSixStep, osPriorityNormal, 0, 384);
      // osThreadCreate(osThread(sixStepTask), NULL);
      osThreadDef(currentLoopTask, TaskCurrentLoop, osPriorityNormal, 0, 384);
      osThreadCreate(osThread(currentLoopTask), NULL);
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
  /* Infinite loop */
  for(;;)
  {
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskSpeedLoop */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  测速任务：每 10ms 上报 M1/M2 RPM + 电压幅值
  */
void TaskSpeedReport(void const *argument)
{
    (void)argument;
    uint16_t last_angle[2] = {0};
    uint32_t last_tick = xTaskGetTickCount();

    for (;;)
    {
        uint32_t now = xTaskGetTickCount();
        float dt = (float)(now - last_tick) / 1000.0f;
        last_tick = now;

        float frame[4];
        for (int i = 0; i < 2; i++)
        {
            uint16_t raw = MT6701_ReadAngle(g_motor[i].motor_id);
            int16_t diff = (int16_t)(raw - last_angle[i]);
            if (diff > 8192)       diff -= 16384;
            else if (diff < -8192) diff += 16384;
            float revs = (float)diff / 16384.0f;
            frame[i] = -revs / dt * 60.0f;
            if (i == 1) frame[i] = -frame[i];  // enc_direction 已在编码器层处理（mt6701.c）
            frame[i + 2] = g_motor[i].voltage_mag;
            last_angle[i] = raw;
        }
        COMM_SendFloatFrame(frame, 4);

        osDelay(10);
    }
}

/**
  * @brief  6 步换相任务（保留，未启用）
  * @note   注意：此函数使用 motor->direction 控制换相方向，未使用 enc_direction。
  *         启用电前需确认 enc_direction 已通过 MT6701_SetEncDirection 同步。
  */
void TaskSixStep(void const * argument)
{
    (void)argument;
    float frame[3];

    g_motor[0].direction = 1;
    SixStep_Init(&g_motor[0], 0.5f);
    Motor_StartPWM(&g_motor[0]);

    g_motor[1].direction = 1;
    SixStep_Init(&g_motor[1], 0.25f);
    Motor_StartPWM(&g_motor[1]);

    uint8_t  report_active = 0;
    uint32_t report_deadline = 0;
    uint32_t loop_cnt = 0;

    for (;;)
    {
        SixStep_Run(&g_motor[0]);
        SixStep_Run(&g_motor[1]);

        if (g_trigger_report)
        {
            g_trigger_report = 0;
            report_active = 1;
            report_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
            loop_cnt = 0;
        }
        if (report_active && xTaskGetTickCount() >= report_deadline)
            report_active = 0;

        if (report_active)
        {
            float Ia1 = INA240_GetCurrentFast(INA240_MOTOR1_U);
            float Ib1 = INA240_GetCurrentFast(INA240_MOTOR1_W);
            frame[0] = Ia1;
            frame[1] = Ib1;
            frame[2] = -(Ia1 + Ib1);
            COMM_SendFloatFrame(frame, 3);
        }
        loop_cnt++;
        osDelay(1);
    }
}

/**
  * @brief  MPU6500 姿态解算任务
  */
void TaskMPU6500(void const * argument)
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

/**
  * @brief  SVPWM 电压模式正弦波任务
  */
void TaskVoltageSine(void const * argument)
{
    (void)argument;
    float vm1 = 0.5f;
    float vm2 = 0.5f;
    float frame[3];

    g_motor[0].mode = MOTOR_MODE_VOLTAGE_SINE;
    g_motor[0].voltage_mag = vm1;
    g_motor[0].direction = -1;   // 开环旋转方向（正转：电角递增加 enc_direction=方向）
    Motor_StartPWM(&g_motor[0]);

    g_motor[1].mode = MOTOR_MODE_VOLTAGE_SINE;
    g_motor[1].voltage_mag = vm2;
    g_motor[1].direction = -1;
    Motor_StartPWM(&g_motor[1]);

    uint8_t  report_active = 0;
    uint32_t report_deadline = 0;
    uint32_t loop_cnt = 0;

    for (;;)
    {
        // M1 正转, M2 反转（翻转 v_alpha 符号即可反转旋转方向）
        float vm[2] = {vm1, vm2};
        int8_t rev[2] = {1, 1};
        for (int i = 0; i < 2; i++)
        {
            uint16_t raw = MT6701_ReadAngle(g_motor[i].motor_id);
            float mech_rad = (float)raw * 6.283185307f / 16384.0f;
            float elec_rad = mech_rad * (float)MOTOR_POLE_PAIRS * g_motor[i].direction;
            g_motor[i].elec_angle = elec_rad;

            // rev=+1 → 正转（高效）, rev=-1 → 反转
            float v_alpha = (vm[i] * rev[i]) * sinf(elec_rad);
            float v_beta  = -(vm[i] * rev[i]) * cosf(elec_rad);
            SVPWM_SetVab(v_alpha, v_beta, &g_motor[i]);
        }

        if (g_trigger_report)
        {
            g_trigger_report = 0;
            report_active = 1;
            report_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
        }
        if (report_active && xTaskGetTickCount() >= report_deadline)
            report_active = 0;

        if (report_active)
        {
            float Ia1 = INA240_GetCurrentFast(INA240_MOTOR1_U);
            float Ib1 = INA240_GetCurrentFast(INA240_MOTOR1_W);
            frame[0] = Ia1;
            frame[1] = Ib1;
            frame[2] = -(Ia1 + Ib1);
            COMM_SendFloatFrame(frame, 3);
        }
        loop_cnt++;
        osDelay(1);
    }
}

/**
  * @brief  电流闭环任务（层 2，当前启用）
  */
void TaskCurrentLoop(void const *argument)
{
    (void)argument;

    // M1（iq_ref 由串口 R 指令设置，初始为 0 安全）
    g_motor[0].id_ref = 0.0f;
    g_motor[0].iq_ref = 0.0f;
    PI_Reset(&g_motor[0].id_pi);
    PI_Reset(&g_motor[0].iq_pi);
    Motor_StartPWM(&g_motor[0]);
    g_motor[0].mode = MOTOR_MODE_CURRENT_LOOP;

    // M2（iq_ref 由串口 L 指令设置，初始为 0 安全）
    g_motor[1].id_ref = 0.0f;
    g_motor[1].iq_ref = 0.0f;
    PI_Reset(&g_motor[1].id_pi);
    PI_Reset(&g_motor[1].iq_pi);
    Motor_StartPWM(&g_motor[1]);
    g_motor[1].mode = MOTOR_MODE_CURRENT_LOOP;

    osDelay(500);

    float   last_mech[2] = {0};
    uint32_t last_tick = xTaskGetTickCount();

    for (;;)
    {
        uint32_t now = xTaskGetTickCount();
        float dt = (float)(now - last_tick) / 1000.0f;
        last_tick = now;

        // 计算两电机 RPM（10ms 采样 → Nyquist ~3000 RPM，无混叠）
        float rpm[2];
        for (int i = 0; i < 2; i++) {
            float cur = g_enc[i].mech_angle;
            float delta = cur - last_mech[i];
            // 有界折返 — 防止 NaN/inf/极端值死锁, 最多 ±32 圈
            int w = 0;
            while (delta > 3.14159265f && w++ < 32)  delta -= 6.283185307f;
            while (delta < -3.14159265f && w++ < 32) delta += 6.283185307f;
            if (w >= 32 || dt <= 0.0f) {
                rpm[i] = 0.0f;  // 数据异常, RPM 置零
            } else {
                rpm[i] = (delta / 6.283185307f) / dt * 60.0f
                        * (float)MT6701_GetEncDirection(i);
            }
            last_mech[i] = cur;
        }

        float frame[10];
        frame[0] = g_motor[0].id;  frame[1] = g_motor[0].iq;
        frame[2] = g_motor[0].vd;  frame[3] = g_motor[0].vq;
        frame[4] = g_motor[1].id;  frame[5] = g_motor[1].iq;
        frame[6] = g_motor[1].vd;  frame[7] = g_motor[1].vq;
        frame[8] = rpm[0];         frame[9] = rpm[1];
        if (!CALIB_IsBusy() && !DebugCapture_IsActive()) {
            COMM_SendFloatFrame(frame, 10);
        }
        osDelay(10);
    }
}
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

