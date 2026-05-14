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
#include "speed_capture.h"
#include "buzzer.h"
#include "cli_parser.h"
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
extern volatile uint8_t g_capture_dumping;
static uint8_t g_telem_enabled = 0;  // 遥测开关，默认关闭
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

// 阶跃测试状态
typedef struct {
    uint8_t  active;
    uint8_t  motor_idx;
    uint8_t  phase;          // 0=等待稳态, 1=采集中, 2=dump
    uint32_t phase_start;
    float    from_rpm;
    float    to_rpm;
} StepTest_t;
static StepTest_t g_step_test;
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

/* ===== CLI 辅助函数 ===== */

static void CMD_Help(void)
{
    printf("\r\n=== STATUS ===\r\n");
    for (int i = 0; i < 2; i++) {
        const char *mode_str = g_motor[i].speed_mode ? "SPEED" : "CURRENT";
        if (g_motor[i].speed_mode) {
            printf("M%d %s ref=%.0fRPM fb=%.0fRPM iq=%.3fA\r\n",
                   i+1, mode_str, g_speed[i].speed_ref, g_speed[i].speed_fb, g_motor[i].iq);
        } else {
            printf("M%d %s iq_ref=%.3fA iq=%.3fA\r\n",
                   i+1, mode_str, g_motor[i].iq_ref, g_motor[i].iq);
        }
        printf("  Speed PI: Kp=%.3f Ki=%.3f | Current PI: Kp=%.1f Ki=%.0f\r\n",
               g_speed[i].kp, g_speed[i].ki, g_motor[i].iq_pi.kp, g_motor[i].iq_pi.ki);
    }
    printf("\r\n=== COMMANDS ===\r\n");
    printf("R<A> / L<A>      电流模式 (A)\r\n");
    printf("RS<RPM> / LS<RPM>    速度模式 [ramp]\r\n");
    printf("RT<RPM> / LT<RPM>    阶跃测试 [no ramp] [burst]\r\n");
    printf("RT<A> <B> / LT<A> <B>    阶跃 A→B\r\n");
    printf("RE<RPM> / LE<RPM>    负载实验\r\n");
    printf("PRS / PLS         查询速度 PI\r\n");
    printf("PRC / PLC         查询电流 PI\r\n");
    printf("PRS P=X I=Y       设置速度 PI\r\n");
    printf("PRC P=X I=Y       设置电流 PI\r\n");
    printf("T                 开关遥测输出\r\n");
    printf("?                 帮助\r\n\r\n");
}

/** @brief 电流模式. first_char 是 R/L 后的第一个已读取字符（数字/小数点/符号）
 *  @note  支持堆叠命令 R0.2L0.3: 扫描到下一个 R/L 时返回 1 */
static uint8_t CMD_Current(uint8_t motor_idx, uint8_t first_char)
{
    char buf[16]; uint8_t pos = 0;
    if ((first_char >= '0' && first_char <= '9') || first_char == '.' ||
        first_char == '-' || first_char == '+')
        buf[pos++] = (char)first_char;

    for (uint8_t w = 0; w < 30 && pos < 15; w++) {
        if (COMM_Available() == 0) { osDelay(1); continue; }
        uint8_t c = COMM_ReadByte();
        if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+')
            buf[pos++] = (char)c;
        else {
            buf[pos] = '\0';
            float val = (float)atof(buf);
            if (g_motor[motor_idx].speed_mode) {
                SpeedCtrl_ExitMode(&g_speed[motor_idx]);
                g_motor[motor_idx].speed_mode = 0;
            }
            Motor_SetIqRef(&g_motor[motor_idx], val);
            printf("M%d CURRENT iq_ref=%.3fA\r\n", motor_idx + 1, val);
            if (c == 'R' || c == 'r' || c == 'L' || c == 'l') return 1;
            return 0;
        }
    }
    buf[pos] = '\0';
    if (pos > 0) {
        float val = (float)atof(buf);
        if (g_motor[motor_idx].speed_mode) {
            SpeedCtrl_ExitMode(&g_speed[motor_idx]);
            g_motor[motor_idx].speed_mode = 0;
        }
        Motor_SetIqRef(&g_motor[motor_idx], val);
        printf("M%d CURRENT iq_ref=%.3fA\r\n", motor_idx + 1, val);
    }
    return 0;
}

/** @brief 速度模式 */
static void CMD_Speed(uint8_t motor_idx)
{
    float rpm;
    if (!CLI_ReadFloat(&rpm)) { printf("RS/LS: 需要转速值 (RPM)\r\n"); return; }
    SpeedCtrl_EnterMode(&g_speed[motor_idx], rpm);
    g_motor[motor_idx].speed_mode = 1;
    printf("M%d SPEED %.0fRPM [ramp=%.0f RPM/s]\r\n", motor_idx + 1, rpm, SPEED_RAMP_MAX);
}

/** @brief 阶跃测试（当前转速→目标 或 A→B） */
static void CMD_Step(uint8_t motor_idx)
{
    if (g_step_test.active) { printf("M%d STEP 忙\r\n", motor_idx + 1); return; }
    if (SpeedCapture_IsBusy()) { printf("M%d STEP capture 忙\r\n", motor_idx + 1); return; }

    float from_rpm, to_rpm;
    if (!CLI_ReadFloat(&to_rpm)) { printf("RT/LT: 需要目标转速 (RPM)\r\n"); return; }
    from_rpm = g_speed[motor_idx].speed_fb;

    /* 检查是否有第二个参数 (空格后跟数字) */
    uint8_t next = CLI_ReadChar(5);
    if (next == ' ' || (next >= '0' && next <= '9') || next == '.' || next == '-' || next == '+') {
        if (next == ' ') next = CLI_ReadChar(5);
        if ((next >= '0' && next <= '9') || next == '.' || next == '-' || next == '+') {
            char buf2[16]; uint8_t p2 = 0;
            buf2[p2++] = (char)next;
            for (uint8_t w = 0; w < 30 && p2 < 15; w++) {
                if (COMM_Available() == 0) { osDelay(1); continue; }
                uint8_t c = COMM_ReadByte();
                if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+')
                    buf2[p2++] = (char)c;
                else break;
            }
            buf2[p2] = '\0';
            from_rpm = to_rpm;
            to_rpm = (float)atof(buf2);
        }
    }

    g_speed[motor_idx].no_ramp = 1;
    SpeedCtrl_EnterMode(&g_speed[motor_idx], from_rpm);
    g_motor[motor_idx].speed_mode = 1;
    g_speed[motor_idx].speed_ref_ramp = from_rpm;
    g_step_test.active = 1;
    g_step_test.motor_idx = motor_idx;
    g_step_test.phase = 0;
    g_step_test.phase_start = xTaskGetTickCount();
    g_step_test.from_rpm = from_rpm;
    g_step_test.to_rpm = to_rpm;
    printf("M%d STEP %d→%dRPM [no ramp] [burst %df]\r\n",
           motor_idx + 1, (int)from_rpm, (int)to_rpm, SC_BURST_SAMPLES);
}

/** @brief 负载实验 */
static void CMD_Load(uint8_t motor_idx)
{
    if (g_load_test.active) { printf("M%d LOAD 忙\r\n", motor_idx + 1); return; }
    float rpm;
    if (!CLI_ReadFloat(&rpm)) { printf("RE/LE: 需要转速值 (RPM)\r\n"); return; }

    SpeedCtrl_EnterMode(&g_speed[motor_idx], rpm);
    g_motor[motor_idx].speed_mode = 1;
    g_load_test.active = 1;
    g_load_test.motor_idx = motor_idx;
    g_load_test.phase = 0;
    g_load_test.phase_start = xTaskGetTickCount();
    g_load_test.rpm = rpm;
    Buzzer_Beep(1000, 80);
    printf("M%d LOAD %.0fRPM [2s ramp→3s buzz→3s idle]\r\n", motor_idx + 1, rpm);
}

/** @brief PI 参数查询/设置: PRS / PRS P=X I=Y / PRS 0.015 0.1 */
static void CMD_PI_Param(void)
{
    uint8_t ch = CLI_ReadChar(20);
    uint8_t motor_idx;
    if (ch == 'R' || ch == 'r') motor_idx = 0;
    else if (ch == 'L' || ch == 'l') motor_idx = 1;
    else { printf("P: 需要 R 或 L (如 PRS, PRC)\r\n"); return; }

    uint8_t sub = CLI_ReadChar(20);
    if (sub == 0 || sub == '\r' || sub == '\n') {
        /* PR 或 PL: 查询全部 PI */
        printf("M%d Speed  PI: Kp=%.3f Ki=%.3f Out=±%.1fA\r\n",
               motor_idx + 1, g_speed[motor_idx].kp, g_speed[motor_idx].ki, 2.0f);
        printf("M%d Current PI: Kp=%.1f Ki=%.0f Out=±%.1fV\r\n",
               motor_idx + 1, g_motor[motor_idx].iq_pi.kp, g_motor[motor_idx].iq_pi.ki, FOC_VBUS);
        return;
    }

    uint8_t is_speed;
    if (sub == 'S' || sub == 's') is_speed = 1;
    else if (sub == 'C' || sub == 'c') is_speed = 0;
    else { printf("P: 未知子系统 '%c', 使用 S=Speed C=Current\r\n", sub); return; }

    /* 检查是否有参数 */
    uint8_t peek = CLI_ReadChar(5);
    while (peek == ' ') peek = CLI_ReadChar(5);  /* 跳过空格 */
    if (peek == 0 || peek == '\r' || peek == '\n') {
        /* 纯查询 */
        if (is_speed) {
            float kp = g_speed[motor_idx].kp, ki = g_speed[motor_idx].ki;
            printf("M%d Speed PI: Kp=%.3f Ki=%.3f ω0=%.2fHz Out=±%.1fA\r\n",
                   motor_idx + 1, kp, ki, ki / kp / 6.283f, 2.0f);
        } else {
            float kp = g_motor[motor_idx].iq_pi.kp, ki = g_motor[motor_idx].iq_pi.ki;
            printf("M%d Current PI: Kp=%.1f Ki=%.0f ω0=%.1fHz Out=±%.1fV\r\n",
                   motor_idx + 1, kp, ki, ki / kp / 6.283f, FOC_VBUS);
        }
        return;
    }

    /* 解析设置值: PRS P=0.015 I=0.1 或 PRS 0.015 0.1 */
    float kp = 0, ki = 0;
    uint8_t kp_set = 0, ki_set = 0;

    if (peek == 'P' || peek == 'p' || peek == 'I' || peek == 'i') {
        do {
            uint8_t ch2 = peek;
            uint8_t eq = CLI_ReadChar(10);
            if (eq != '=') break;
            float val;
            if (!CLI_ReadFloat(&val)) break;
            if (ch2 == 'P' || ch2 == 'p') { kp = val; kp_set = 1; }
            if (ch2 == 'I' || ch2 == 'i') { ki = val; ki_set = 1; }
            peek = CLI_ReadChar(5);
            if (peek == ' ') peek = CLI_ReadChar(5);
        } while (peek == 'P' || peek == 'p' || peek == 'I' || peek == 'i');
    } else if ((peek >= '0' && peek <= '9') || peek == '.' || peek == '-' || peek == '+') {
        char buf[16]; uint8_t p = 0;
        buf[p++] = (char)peek;
        for (uint8_t w = 0; w < 30 && p < 15; w++) {
            if (COMM_Available() == 0) { osDelay(1); continue; }
            uint8_t c = COMM_ReadByte();
            if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+')
                buf[p++] = (char)c;
            else break;
        }
        buf[p] = '\0';
        kp = (float)atof(buf); kp_set = 1;
        if (CLI_ReadFloat(&ki)) ki_set = 1;
    }

    if (!kp_set && !ki_set) { printf("P: 需要 P=X I=Y 或直接跟 Kp Ki 值\r\n"); return; }

    if (is_speed) {
        if (!kp_set) kp = g_speed[motor_idx].kp;
        if (!ki_set) ki = g_speed[motor_idx].ki;
        SpeedCtrl_SetGains(&g_speed[motor_idx], kp, ki);
        printf("M%d Speed PI: Kp=%.3f Ki=%.3f ω0=%.2fHz Out=±%.1fA\r\n",
               motor_idx + 1, kp, ki, ki / kp / 6.283f, 2.0f);
    } else {
        if (!kp_set) kp = g_motor[motor_idx].iq_pi.kp;
        if (!ki_set) ki = g_motor[motor_idx].iq_pi.ki;
        Motor_SetCurrentPI(&g_motor[motor_idx], kp, ki);
        printf("M%d Current PI: Kp=%.1f Ki=%.0f ω0=%.1fHz Out=±%.1fV\r\n",
               motor_idx + 1, kp, ki, ki / kp / 6.283f, FOC_VBUS);
    }
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

    // 阶跃测试状态机
    if (g_step_test.active) {
        uint32_t elapsed = xTaskGetTickCount() - g_step_test.phase_start;
        uint8_t mi = g_step_test.motor_idx;
        switch (g_step_test.phase) {
        case 0: // 等待稳态 500ms, 然后开始 pre-trigger 采集
            if (elapsed >= 500) {
                SpeedCapture_Start(SC_BURST_SAMPLES);
                g_step_test.phase = 1;
                g_step_test.phase_start = xTaskGetTickCount();
            }
            break;
        case 1: // pre-trigger 50ms 基线, 然后阶跃
            if (elapsed >= 50) {
                g_speed[mi].speed_ref = g_step_test.to_rpm;
                g_speed[mi].speed_ref_ramp = g_step_test.to_rpm;
                g_step_test.phase = 2;
                g_step_test.phase_start = xTaskGetTickCount();
            }
            break;
        case 2: // 采集中, 等待完成 (g_sc.ready 由 SpeedCapture_Write 置位)
            if (SpeedCapture_IsReady() || elapsed >= 2000) {
                g_step_test.phase = 3;
                g_step_test.phase_start = xTaskGetTickCount();
                SpeedCtrl_ExitMode(&g_speed[mi]);
                g_motor[mi].speed_mode = 0;
                g_speed[mi].no_ramp = 0;
                Motor_SetIqRef(&g_motor[mi], 0.0f);
            }
            break;
        case 3: // dump 数据 (抑制遥测)
            SpeedCapture_Dump();
            g_step_test.active = 0;
            printf("STEP M%d done (%d samples)\r\n", mi+1, SC_BURST_SAMPLES);
            break;
        }
    }

    while (COMM_Available() > 0)
    {
      uint8_t ch = COMM_ReadByte();

      // ===== 校准模式 CLI =====
      if (g_calib_mode) {
          if (ch == '?' || ch == 'h' || ch == 'H') {
              printf("\r\n=== 校准模式 ===\r\n");
              printf("R1-5  M1校准  L1-5  M2校准\r\n");
              printf("RS    查看校准参数\r\n");
              printf("q     中止校准\r\n");
              printf("?     帮助\r\n\r\n");
          } else if (ch == 'R' || ch == 'r' || ch == 'L' || ch == 'l') {
              uint8_t motor_idx = (ch == 'L' || ch == 'l') ? 1 : 0;
              uint8_t func = CLI_ReadChar(20);
              if (func >= '1' && func <= '5') {
                  if (!CALIB_TryLock()) {
                      printf("校准忙, 等待或按 'q' 中止\r\n");
                  } else {
                      printf("=== M%d 校准 #%c ===\r\n", motor_idx + 1, func);
                      switch (func) {
                      case '1': CALIB_CurrentOffset(); break;
                      case '2': CALIB_PhaseWireMap(&g_motor[motor_idx]); break;
                      case '3': CALIB_EncoderDir(&g_motor[motor_idx]); break;
                      case '4': CALIB_EncoderOffset(&g_motor[motor_idx]); break;
                      case '5': CALIB_MotorParams(&g_motor[motor_idx]); break;
                      }
                  }
              } else if (func == 's' || func == 'S') {
                  CALIB_PrintParams(&g_calib);
              }
          } else if (ch == 'q' || ch == 'Q') {
              CALIB_Abort();
              printf("校准中止\r\n");
          }
          continue;
      }

      // ===== 阶跃测试模式 CLI =====
      if (g_test_mode) {
          if (ch == '?' || ch == 'h' || ch == 'H') {
              printf("\r\n=== 阶跃测试模式 ===\r\n");
              printf("R<A>   M1 电流阶跃 (A)\r\n");
              printf("L<A>   M2 电流阶跃 (A)\r\n");
              printf("r     重发上次采集\r\n");
              printf("?     帮助\r\n\r\n");
          } else if (ch == 'R' || ch == 'r' || ch == 'L' || ch == 'l') {
              uint8_t motor_idx = (ch == 'L' || ch == 'l') ? 1 : 0;
              // peek 下一个字符判断是 resend 还是电流阶跃
              uint8_t peek = CLI_ReadChar(5);
              if (peek == 0 || peek == '\r' || peek == '\n') {
                  // 裸 r → resend
                  DebugCapture_Resend();
              } else if ((peek >= '0' && peek <= '9') || peek == '.' || peek == '-' || peek == '+') {
                  char buf[16]; uint8_t p = 0;
                  buf[p++] = (char)peek;
                  for (uint8_t w = 0; w < 30 && p < 15; w++) {
                      if (COMM_Available() == 0) { osDelay(1); continue; }
                      uint8_t c = COMM_ReadByte();
                      if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+')
                          buf[p++] = (char)c;
                      else break;
                  }
                  buf[p] = '\0';
                  DebugCapture_Start(motor_idx, (float)atof(buf));
              }
          }
          continue;
      }

      if (ch == '?' || ch == 'H' || ch == 'h') {
          CMD_Help();
          continue;
      }

      if (ch == 'T' || ch == 't') {
          g_telem_enabled = !g_telem_enabled;
          printf("TELEMETRY %s\r\n", g_telem_enabled ? "ON" : "OFF");
          continue;
      }

      if (ch == 'P' || ch == 'p') {
          CMD_PI_Param();
          continue;
      }

      if (ch == 'R' || ch == 'r' || ch == 'L' || ch == 'l') {
          uint8_t motor_idx = (ch == 'L' || ch == 'l') ? 1 : 0;
          uint8_t ch2 = CLI_ReadChar(30);
          if (ch2 == 0) continue;

          if ((ch2 >= '0' && ch2 <= '9') || ch2 == '.' || ch2 == '-' || ch2 == '+') {
              while (CMD_Current(motor_idx, ch2)) {
                  ch = CLI_ReadChar(5);
                  if (ch == 'R' || ch == 'r') motor_idx = 0;
                  else if (ch == 'L' || ch == 'l') motor_idx = 1;
                  else break;
                  ch2 = CLI_ReadChar(30);
                  if (!((ch2 >= '0' && ch2 <= '9') || ch2 == '.' ||
                        ch2 == '-' || ch2 == '+'))
                      break;
              }
          } else {
              switch (ch2) {
              case 'S': case 's': CMD_Speed(motor_idx);  break;
              case 'T': case 't': CMD_Step(motor_idx);   break;
              case 'E': case 'e': CMD_Load(motor_idx);   break;
              default:
                  printf("M%d: 未知子命令 '%c'. 使用 S=Speed T=Step E=Load\r\n",
                         motor_idx + 1, ch2);
                  CLI_FlushLine();
                  break;
              }
          }
          continue;
      }

      if (ch != '\r' && ch != '\n') {
          // 未识别字符 — 静默忽略（避免遥测数据误触发）
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
    // 遥测关闭时跳过发送
    if (!g_telem_enabled) {
        osDelay(5);
        continue;
    }
    // 阶跃测试 dump 期间抑制遥测, 避免串口竞争
    if (g_capture_dumping) {
        osDelay(1);
        continue;
    }
    float frame[10];
    for (int i = 0; i < 2; i++) {
        frame[i*5+0] = g_speed[i].speed_ref_ramp;
        frame[i*5+1] = g_speed[i].speed_fb;
        frame[i*5+2] = g_motor[i].iq_ref;
        frame[i*5+3] = g_motor[i].iq;
        frame[i*5+4] = g_enc[i].mech_angle;
    }
    COMM_SendFloatFrame(frame, 10);
    osDelay(5);  // 200Hz 遥测
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
  SpeedCapture_Init();
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
          // 阶跃测试: 记录 burst 数据
          if (g_step_test.active && g_step_test.motor_idx == i) {
              SpeedCapture_Write(g_speed[i].speed_fb, g_motor[i].iq,
                                 g_speed[i].speed_ref, g_speed[i].t_load_est);
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

