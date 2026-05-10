# 速度环实现计划

**目标:** 在电流环之上实现级联速度 PI 闭环，1kHz 任务中运行，TIM15 定时器精确触发，自适应窗口 RPM 测量保证低速精度。

**架构:** 速度环封装为独立 SpeedCtrl_t 模块（speed_ctrl.c/h）。TaskSpeedLoop（High 优先级）等待 TIM15 TaskNotify 后执行。串口 V/W 指令进入速度模式，R/L 切回电流模式。遥测帧改为 8 通道速度环数据。

---

## 前置条件 — CubeMX 配置（用户操作）

### FREERTOS → Tasks

| 当前 | 操作 | 新名称 | 栈 | 优先级 | 入口 |
|------|:--:|------|:--:|:------:|------|
| defaultTask | **改名** | TaskCLI | 128 | Normal | StartCLITask |
| currentLoopTask | **改名** | TaskTelemetry | 384 | Low | TaskTelemetry |
| (新建) | **新建** | TaskSpeedLoop | 512 | High | TaskSpeedLoop |
| mpuTask | 保留 | TaskIMU | 384 | Normal | TaskIMU |

### TIM15 配置

| 参数 | 值 |
|------|-----|
| Mode → Clock Source | Internal Clock |
| Prescaler | 169 |
| Counter Period (ARR) | 999 |
| NVIC → Enabled, Priority | 6 |

---

## 文件结构

| 文件 | 操作 | 职责 |
|------|:--:|------|
| `Core/Inc/speed_ctrl.h` | **新建** | SpeedCtrl_t + API 声明 |
| `Core/Src/speed_ctrl.c` | **新建** | 自适应窗口 RPM + EMA + PI + 斜坡 |
| `Core/Inc/foc.h` | 修改 | Motor_t 添加 speed_mode |
| `Core/Src/foc.c` | 修改 | FOC_Init 初始化 speed_mode=0 |
| `Core/Src/app_freertos.c` | 修改 | 任务迁移+新任务+CLI(V/W)+遥测帧改造 |
| `Core/Src/main.c` | 修改 | 启动流程中电流环初始化迁移 + TIM15 启动 |
| `Core/Src/stm32g4xx_it.c` | 修改 | TIM15 ISR → vTaskNotifyGiveFromISR |
| `CMakeLists.txt` | 修改 | 添加 speed_ctrl.c |
| `docs/FREERTOS_TASKS.md` | 修改 | 任务表更新 |
| `docs/COMM_PROTOCOL.md` | 修改 | 遥测帧更新 |

---

## Task 1: speed_ctrl.h — 速度环模块头文件

**文件:** 创建 `Core/Inc/speed_ctrl.h`

```c
#ifndef SPEED_CTRL_H
#define SPEED_CTRL_H

#include "pi.h"
#include <stdint.h>

#define SPEED_LOOP_FREQ     1000.0f
#define SPEED_LOOP_DT       (1.0f / SPEED_LOOP_FREQ)
#define SPEED_EMA_SHIFT     3           // τ≈1.7ms @1kHz
#define SPEED_RAMP_MAX      5000.0f     // 加速度限制 (RPM/s)
#define SPEED_MIN_DELTA     4.0f        // 最小角度增量 (counts)
#define SPEED_MAX_WINDOW_MS 20

typedef struct {
    PI_t    pi;
    float   kp, ki;
    float   speed_ref;         // 目标转速 (RPM)
    float   speed_ref_ramp;    // 斜坡后给定
    float   speed_fb;          // EMA 滤波反馈 (RPM)
    float   last_mech;         // 上一时刻机械角度
    float   accum_delta;       // 累积角度增量
    uint16_t accum_ms;         // 累积毫秒
    float   raw_rpm;           // 最新原始 RPM
    uint8_t speed_mode;        // 0=电流模式, 1=速度模式
    int8_t  enc_dir;
} SpeedCtrl_t;

void SpeedCtrl_Init(SpeedCtrl_t *sc, float kp, float ki,
                    float out_max, float out_min, int8_t enc_dir);
void SpeedCtrl_UpdateRPM(SpeedCtrl_t *sc, float mech_angle);
float SpeedCtrl_Run(SpeedCtrl_t *sc);
void SpeedCtrl_EnterMode(SpeedCtrl_t *sc, float speed_ref);
void SpeedCtrl_ExitMode(SpeedCtrl_t *sc);

#endif
```

提交: `feat: 添加速度环模块头文件 speed_ctrl.h`

---

## Task 2: speed_ctrl.c — 速度环核心实现

**文件:** 创建 `Core/Src/speed_ctrl.c`

```c
#include "speed_ctrl.h"
#include <math.h>

#define RAD_TO_COUNTS  2607.5946f   // 16384/(2π)

void SpeedCtrl_Init(SpeedCtrl_t *sc, float kp, float ki,
                    float out_max, float out_min, int8_t enc_dir)
{
    PI_Init(&sc->pi, kp, ki, out_max, out_min);
    sc->kp = kp; sc->ki = ki;
    sc->speed_ref = 0.0f; sc->speed_ref_ramp = 0.0f; sc->speed_fb = 0.0f;
    sc->last_mech = 0.0f; sc->accum_delta = 0.0f; sc->accum_ms = 0;
    sc->raw_rpm = 0.0f; sc->speed_mode = 0; sc->enc_dir = enc_dir;
}

void SpeedCtrl_UpdateRPM(SpeedCtrl_t *sc, float mech_angle)
{
    float delta = mech_angle - sc->last_mech;
    sc->last_mech = mech_angle;
    // 折返修正
    if (delta > 3.14159265f)  delta -= 6.283185307f;
    if (delta < -3.14159265f) delta += 6.283185307f;

    sc->accum_delta += delta;
    sc->accum_ms++;

    // 自适应窗口：≥4 counts 或超时 20ms
    float accum_counts = fabsf(sc->accum_delta) * RAD_TO_COUNTS;
    if (accum_counts >= SPEED_MIN_DELTA || sc->accum_ms >= SPEED_MAX_WINDOW_MS) {
        if (sc->accum_ms > 0) {
            float dt_sec = (float)sc->accum_ms / 1000.0f;
            sc->raw_rpm = (sc->accum_delta / 6.283185307f) / dt_sec
                          * 60.0f * (float)sc->enc_dir;
        }
        sc->accum_delta = 0.0f;
        sc->accum_ms = 0;
    }
    // EMA 滤波 — 每 1ms 平滑输出
    int32_t diff = (int32_t)((sc->raw_rpm - sc->speed_fb) * 1000.0f);
    sc->speed_fb += (float)diff / 1000.0f * (1.0f / (float)(1 << SPEED_EMA_SHIFT));
}

float SpeedCtrl_Run(SpeedCtrl_t *sc)
{
    if (!sc->speed_mode) return 0.0f;
    // 斜坡
    float error = sc->speed_ref - sc->speed_ref_ramp;
    float step = SPEED_RAMP_MAX * SPEED_LOOP_DT;
    if (error > step) sc->speed_ref_ramp += step;
    else if (error < -step) sc->speed_ref_ramp -= step;
    else sc->speed_ref_ramp = sc->speed_ref;
    // 速度 PI
    return PI_Step(&sc->pi, sc->speed_ref_ramp - sc->speed_fb, SPEED_LOOP_DT);
}

void SpeedCtrl_EnterMode(SpeedCtrl_t *sc, float speed_ref)
{
    sc->speed_ref = speed_ref;
    sc->speed_ref_ramp = sc->speed_fb;
    sc->speed_mode = 1;
    PI_Reset(&sc->pi);
}

void SpeedCtrl_ExitMode(SpeedCtrl_t *sc)
{
    sc->speed_mode = 0;
    sc->speed_ref = 0.0f;
    sc->speed_ref_ramp = 0.0f;
    PI_Reset(&sc->pi);
}
```

提交: `feat: 实现速度环核心 — 自适应窗口RPM + EMA + PI + 斜坡`

---

## Task 3: Motor_t 添加 speed_mode

**文件:** 修改 `Core/Inc/foc.h`, `Core/Src/foc.c`

`foc.h` — Motor_t 末尾添加:
```c
    uint8_t speed_mode;             // 0=电流模式, 1=速度模式
```

`foc.c` — FOC_Init 中 `virtual_angle=0` 之后添加:
```c
    g_motor[0].speed_mode = 0;
    g_motor[1].speed_mode = 0;
```

提交: `feat: Motor_t 新增 speed_mode 字段`

---

## Task 4: CubeMX 生成后的适配 — 任务迁移 + CLI 更新

**前提:** 用户完成 CubeMX 配置并重新生成代码。

**文件:** 修改 `Core/Src/app_freertos.c`

**4a. Includes 添加:**
```c
#include "speed_ctrl.h"
```

**4b. 全局变量添加:**
```c
SpeedCtrl_t g_speed[2];
osThreadId speedLoopTaskHandle;
```

**4c. MX_FREERTOS_Init — 模式分支调整:**

CubeMX 生成样板创建代码后，在 `USER CODE BEGIN RTOS_THREADS` 替换为:

```c
if (g_calib_mode) {
    osThreadDef(cliTask, StartCLITask, osPriorityNormal, 0, 1024);
    cliTaskHandle = osThreadCreate(osThread(cliTask), NULL);
} else if (g_test_mode) {
    osThreadDef(cliTask, StartCLITask, osPriorityNormal, 0, 384);
    cliTaskHandle = osThreadCreate(osThread(cliTask), NULL);
    osThreadDef(debugCaptureTask, TaskDebugCapture, osPriorityNormal, 0, 512);
    osThreadCreate(osThread(debugCaptureTask), NULL);
} else {
    osThreadDef(cliTask, StartCLITask, osPriorityNormal, 0, 128);
    cliTaskHandle = osThreadCreate(osThread(cliTask), NULL);
    osThreadDef(speedLoopTask, TaskSpeedLoop, osPriorityHigh, 0, 512);
    speedLoopTaskHandle = osThreadCreate(osThread(speedLoopTask), NULL);
    osThreadDef(telemetryTask, TaskTelemetry, osPriorityLow, 0, 384);
    osThreadCreate(osThread(telemetryTask), NULL);
    osThreadDef(imuTask, TaskIMU, osPriorityNormal, 0, 384);
    imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);
}
```

**4d. StartDefaultTask → StartCLITask 改名 + V/W 指令:**

在正常模式 CLI 中，R/L 指令前新增 V/W:

```c
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
```

R/L 处理中切回电流模式:

```c
    if (ch == 'R' || ch == 'r' || ch == 'L' || ch == 'l') {
        // ... 现有解析获取 motor_idx ...
        if (g_motor[motor_idx].speed_mode) {
            SpeedCtrl_ExitMode(&g_speed[motor_idx]);
            g_motor[motor_idx].speed_mode = 0;
        }
        Motor_SetIqRef(&g_motor[motor_idx], (float)atof(buf));
        // ...
    }
```

提交: `feat: app_freertos 任务迁移 + 速度指令 V/W + 模式切换`

---

## Task 5: TaskSpeedLoop — 1kHz 速度环任务

**文件:** 修改 `Core/Src/app_freertos.c`

```c
void TaskSpeedLoop(void const *argument)
{
    (void)argument;
    SpeedCtrl_Init(&g_speed[0], 0.05f, 1.0f, 2.0f, -2.0f,
                   MT6701_GetEncDirection(0));
    SpeedCtrl_Init(&g_speed[1], 0.05f, 1.0f, 2.0f, -2.0f,
                   MT6701_GetEncDirection(1));

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // 等待 TIM15 ISR 通知

        for (int i = 0; i < 2; i++) {
            SpeedCtrl_UpdateRPM(&g_speed[i], g_enc[i].mech_angle);
            if (g_motor[i].speed_mode) {
                float iq_ref = SpeedCtrl_Run(&g_speed[i]);
                Motor_SetIqRef(&g_motor[i], iq_ref);
            }
        }
    }
}
```

提交: `feat: TaskSpeedLoop — 1kHz 速度环控制 (TIM15 触发)`

---

## Task 6: TIM15 ISR — vTaskNotifyGiveFromISR

**文件:** 修改 `Core/Src/stm32g4xx_it.c`

ISR 头部声明 + TIM15 处理:

```c
/* USER CODE BEGIN Includes */
extern osThreadId speedLoopTaskHandle;
/* USER CODE END Includes */

/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim15;
/* USER CODE END EV */

// 在 TIM1_BRK_TIM15_IRQHandler 中:
void TIM1_BRK_TIM15_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 1 */
  if (__HAL_TIM_GET_FLAG(&htim15, TIM_FLAG_UPDATE)) {
      if (__HAL_TIM_GET_IT_SOURCE(&htim15, TIM_IT_UPDATE)) {
          __HAL_TIM_CLEAR_FLAG(&htim15, TIM_FLAG_UPDATE);
          BaseType_t xHigherPriorityTaskWoken = pdFALSE;
          vTaskNotifyGiveFromISR(speedLoopTaskHandle, &xHigherPriorityTaskWoken);
          portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
  }
  /* USER CODE END TIM1_BRK_TIM15_IRQn 1 */
}
```

提交: `feat: TIM15 ISR — vTaskNotifyGiveFromISR 唤醒速度环任务`

---

## Task 7: main.c — 电流环初始化迁移 + TIM15 启动

**文件:** 修改 `Core/Src/main.c`

正常模式分支中 `Motor_Enable()` 之后:

```c
      // 电流环初始化（从 TaskCurrentLoop 迁移到启动流程）
      for (int i = 0; i < 2; i++) {
          g_motor[i].id_ref = 0.0f;
          g_motor[i].iq_ref = 0.0f;
          g_motor[i].mode = MOTOR_MODE_CURRENT_LOOP;
          PI_Reset(&g_motor[i].id_pi);
          PI_Reset(&g_motor[i].iq_pi);
      }
```

`MT6701_CSDelay_Init()` 之后:

```c
      HAL_TIM_Base_Start_IT(&htim15);  // 启动 1kHz 速度环触发
```

提交: `refactor: 电流环初始化迁移到 main.c + 启动 TIM15`

---

## Task 8: TaskTelemetry — 遥测帧改造

**文件:** 修改 `Core/Src/app_freertos.c`

函数 `TaskCurrentLoop` 改名为 `TaskTelemetry`，删除已迁移的初始化代码，替换帧内容:

```c
void TaskTelemetry(void const *argument)
{
    (void)argument;
    for (;;) {
        float frame[8];
        for (int i = 0; i < 2; i++) {
            frame[i*4+0] = g_speed[i].speed_ref_ramp;
            frame[i*4+1] = g_speed[i].speed_fb;
            frame[i*4+2] = g_motor[i].iq_ref;
            frame[i*4+3] = g_motor[i].iq;
        }
        COMM_SendFloatFrame(frame, 8);
        osDelay(10);
    }
}
```

提交: `refactor: TaskTelemetry 遥测帧改造为 8 通道速度环数据`

---

## Task 9: 文档更新

**文件:** 修改 `docs/FREERTOS_TASKS.md`, `docs/COMM_PROTOCOL.md`

更新任务表、遥测帧格式、V/W 指令说明（详见讨论中确定的格式）。

提交: `docs: 更新任务表和遥测协议 — 速度环`

---

## 速度 PI 参数起点

| 参数 | 值 | 说明 |
|------|-----|------|
| Kp | 0.05 A/RPM | 保守起步 |
| Ki | 1.0 A/(RPM·s) | 零点 20 rad/s |
| 输出限幅 | ±2A | Motor_SetIqRef 范围 |
| 斜坡 | 5000 RPM/s | 0→500 RPM 约 0.1s |
| EMA τ | ~1.7ms | SPEED_EMA_SHIFT=3 |

## 注意事项

- CMSIS_V1 的 `osSignalSet` 不支持 ISR 上下文，ISR 中必须用 FreeRTOS 原生 `vTaskNotifyGiveFromISR`
- 任务侧 `ulTaskNotifyTake(pdTRUE, portMAX_DELAY)` 在收到通知后自动清零
- CubeMX 生成后 `MX_FREERTOS_Init` 的样板代码被覆盖，需在 `USER CODE` 保护区手动恢复模式分支
- TIM15 与 TIM1_BRK 共享中断向量，ISR 中通过标志位区分
