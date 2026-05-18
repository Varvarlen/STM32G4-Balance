# 平衡车实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 清理调试/遗留代码并将项目重构为平衡车控制架构（串级 PID + 差速转向 + 双串口遥控）

**Architecture:** 保留 FOC 电流环/速度环（EKF+PI），新增 TaskBalanceLoop (1kHz, TIM17触发) 合并 IMU 读取→卡尔曼倾角→平衡 PID→速度环→差速分配，CLI 精简为平衡车指令集。USART1 调试口 + USART2 蓝牙口共用解析核心。

**Tech Stack:** STM32G431CBU6 + FreeRTOS V10.3.1 (CMSIS_V1) + arm-none-eabi-gcc 13.3.1 + CMake + Ninja

---

### Task 1: 创建 worktree 并验证初始状态

**Files:**
- 无代码修改

- [ ] **Step 1: 创建 balance_pid worktree**

```bash
git -C D:/code/ST/STM32G431workspace/Demo/STM32G431Demo worktree add -b balance_pid ../STM32G431Demo-balance_pid
```

- [ ] **Step 2: 验证 worktree 可编译**

在 worktree 目录执行：
```bash
cd D:/code/ST/STM32G431workspace/Demo/STM32G431Demo-balance_pid
cmake --preset Debug
cmake --build --preset Debug
```

Expected: 编译成功，无错误。

---

### Task 2: 删除 six_step 遗留代码

**Files:**
- Delete: `Core/Src/six_step.c`
- Delete: `Core/Inc/six_step.h`
- Modify: `CMakeLists.txt` — 移除 six_step 引用
- Modify: `Core/Inc/foc.h` — 删除死枚举值
- Modify: `Core/Src/app_freertos.c` — 移除 `#include "six_step.h"`

- [ ] **Step 1: 删除文件**

```bash
rm Core/Src/six_step.c Core/Inc/six_step.h
```

- [ ] **Step 2: 修改 CMakeLists.txt — 移除 six_step 行**

删除以下两行（第 66-67 行附近）：
```
        Core/Inc/six_step.h
        Core/Src/six_step.c
```

- [ ] **Step 3: 修改 foc.h — 精简枚举**

将：
```c
typedef enum {
    MOTOR_MODE_OFF = 0,
    MOTOR_MODE_SIX_STEP,
    MOTOR_MODE_VOLTAGE_SINE,
    MOTOR_MODE_CURRENT_LOOP
} MotorMode_t;
```

改为：
```c
typedef enum {
    MOTOR_MODE_OFF = 0,
    MOTOR_MODE_CURRENT_LOOP
} MotorMode_t;
```

- [ ] **Step 4: 修改 app_freertos.c — 移除 six_step include**

删除行：
```c
#include "six_step.h"
```

- [ ] **Step 5: 编译验证**

```bash
cmake --build --preset Debug
```

Expected: 编译成功。

- [ ] **Step 6: 提交**

```bash
git add -A
git commit -m "cleanup: 移除 six_step 六步换相遗留代码及死枚举值"
```

---

### Task 3: 删除 debug_capture + speed_capture

**Files:**
- Delete: `Core/Src/debug_capture.c`, `Core/Inc/debug_capture.h`
- Delete: `Core/Src/speed_capture.c`, `Core/Inc/speed_capture.h`
- Modify: `CMakeLists.txt` — 移除引用
- Modify: `Core/Src/app_freertos.c` — 移除 include 和 debugCaptureTask 创建
- Modify: `Core/Src/cli.c` — 移除 speed_capture include

- [ ] **Step 1: 删除文件**

```bash
rm Core/Src/debug_capture.c Core/Inc/debug_capture.h
rm Core/Src/speed_capture.c Core/Inc/speed_capture.h
```

- [ ] **Step 2: 修改 CMakeLists.txt**

删除以下 4 行：
```
        Core/Inc/debug_capture.h
        Core/Src/debug_capture.c
        Core/Inc/speed_ctrl.h
        Core/Src/speed_ctrl.c
```

注意：`speed_ctrl.h/c` 保留（速度环核心），只删除 `speed_capture.h/c`（采集工具）。
实际要删除的是 speed_capture 相关行，确认 CMakeLists.txt 第 76-79 行：
```
        Core/Src/speed_capture.c   ← 删除此行
```
以及第 74-75 行的 debug_capture：
```
        Core/Inc/debug_capture.h   ← 删除
        Core/Src/debug_capture.c   ← 删除
```

- [ ] **Step 3: 修改 app_freertos.c — 移除相关 include 和任务**

删除以下 include 行：
```c
#include "debug_capture.h"
#include "speed_capture.h"
```

删除外部变量声明（第 65 行）：
```c
extern volatile uint8_t g_capture_dumping;
```

删除任务声明：
```c
void TaskDebugCapture(void const *argument);
```

删除阶跃测试模式下任务创建代码块（第 117-118 行）：
```c
osThreadDef(debugCaptureTask, TaskDebugCapture, osPriorityNormal, 0, 512);
osThreadCreate(osThread(debugCaptureTask), NULL);
```

删除 `TaskDebugCapture` 函数体（第 345-352 行）。

删除 SpeedLoop 任务中的阶跃采集写入（第 336-339 行）：
```c
if (g_step_test.active && g_step_test.motor_idx == i) {
    SpeedCapture_Write(g_speed[i].speed_fb, g_motor[i].iq,
                       g_speed[i].speed_ref, g_speed[i].t_load_est);
}
```

- [ ] **Step 4: 修改 cli.c — 移除 speed_capture include**

删除：
```c
#include "speed_capture.h"
```

- [ ] **Step 5: 编译验证**

```bash
cmake --build --preset Debug
```

Expected: 编译失败（CLI 和 app_freertos 中还引用了 g_step_test/g_load_test 等）。没关系，后续任务会逐步修复。

- [ ] **Step 6: 提交**

```bash
git add -A
git commit -m "cleanup: 移除 debug_capture/speed_capture 电流及速度阶跃采集工具"
```

---

### Task 4: 重命名 MPU6050 → MPU6500

**Files:**
- Create: `Core/Inc/mpu6500.h`（内容与 mpu6050.h 相同，仅改文件头注释）
- Create: `Core/Src/mpu6500.c`（内容与 mpu6050.c 相同，仅改 include 路径和注释）
- Delete: `Core/Inc/mpu6050.h`, `Core/Src/mpu6050.c`
- Modify: `CMakeLists.txt` — 更新文件名
- Modify: 所有 `#include "mpu6050.h"` → `#include "mpu6500.h"`
- Modify: 所有 `mpu6050` → `mpu6500`（函数名、宏前缀、类型名）

- [ ] **Step 1: 复制文件并全局替换符号名**

```bash
# 复制
cp Core/Inc/mpu6050.h Core/Inc/mpu6500.h
cp Core/Src/mpu6050.c Core/Src/mpu6500.c
```

- [ ] **Step 2: 修改 mpu6500.h — 替换所有符号名和注释**

将所有 `MPU6050` 字样替换为 `MPU6500`：
- 文件头 `@file` 注释
- 宏定义 `MPU6050_WHO_AM_I_VAL` → `MPU6500_WHO_AM_I_VAL`
- 寄存器宏 `MPU6050_REG_*` → `MPU6500_REG_*`
- 枚举类型 `MPU6050_AccelRange_t` → `MPU6500_AccelRange_t`
- 枚举值 `MPU6050_ACCEL_RANGE_*` → `MPU6500_ACCEL_RANGE_*`
- 枚举类型 `MPU6050_GyroRange_t` → `MPU6500_GyroRange_t`
- 枚举值 `MPU6050_GYRO_RANGE_*` → `MPU6500_GYRO_RANGE_*`
- 结构体 `MPU6050_Accel_t` → `MPU6500_Accel_t`
- 结构体 `MPU6050_Gyro_t` → `MPU6500_Gyro_t`
- CS 引脚宏
- 所有函数声明
- 头文件保护宏 `__MPU6050_H__` → `__MPU6500_H__`

- [ ] **Step 3: 修改 mpu6500.c — 替换所有符号名**

将 `#include "mpu6050.h"` 改为 `#include "mpu6500.h"`，所有 `mpu6050`/`MPU6050` 替换为 `mpu6500`/`MPU6500`（包括函数名、类型引用、宏引用、注释）。

- [ ] **Step 4: 修改 CMakeLists.txt**

将：
```
        Core/Inc/mpu6050.h
        Core/Src/mpu6050.c
```
改为：
```
        Core/Inc/mpu6500.h
        Core/Src/mpu6500.c
```

- [ ] **Step 5: 修改所有引用文件 — 更新 include 和符号名**

在以下文件中将 `mpu6050`/`MPU6050` 替换为 `mpu6500`/`MPU6500`：
- `Core/Src/app_freertos.c` — include + 类型引用
- `Core/Src/main.c` — include + 函数调用

- [ ] **Step 6: 删除旧文件**

```bash
rm Core/Inc/mpu6050.h Core/Src/mpu6050.c
```

- [ ] **Step 7: 编译验证**

```bash
cmake --build --preset Debug
```

Expected: 编译失败（函数名变化导致未匹配，且之前未删完的引用仍存在问题）。继续后续任务逐步修复。

- [ ] **Step 8: 提交**

```bash
git add -A
git commit -m "refactor: 重命名 MPU6050→MPU6500 匹配实际硬件芯片型号"
```

---

### Task 5: 修改 foc.h — 添加 Motor_t 新字段

**Files:**
- Modify: `Core/Inc/foc.h`

- [ ] **Step 1: 在 Motor_t 末尾添加新成员**

在 `foc.h` 的 `Motor_t` 结构体末尾（`uint8_t speed_mode;` 行之后，`} Motor_t;` 之前）添加：

```c
    float speed_balance;    // 平衡环输出的速度指令 (RPM, 未差速混合前)
```

- [ ] **Step 2: 编译验证**

```bash
cmake --build --preset Debug
```

Expected: 编译失败（前面任务残留）。继续。

- [ ] **Step 3: 提交**

```bash
git add Core/Inc/foc.h
git commit -m "feat: Motor_t 添加 balance_speed 字段用于平衡环速度指令注入"
```

---

### Task 6: 创建 balance_ctrl.h

**Files:**
- Create: `Core/Inc/balance_ctrl.h`
- Modify: `CMakeLists.txt` — 添加新文件

- [ ] **Step 1: 创建 balance_ctrl.h**

```c
#ifndef BALANCE_CTRL_H
#define BALANCE_CTRL_H

#include <stdint.h>

// 平衡 PID 默认参数（第一版保守值，后续根据实验调参）
#define BALANCE_KP_DEFAULT      15.0f   // 角度比例增益 (RPM/°)
#define BALANCE_KD_DEFAULT       2.0f   // 角速度阻尼增益 (RPM per °/s)
#define BALANCE_OUTPUT_MAX     500.0f   // 平衡输出限幅 (RPM)
#define BALANCE_STEER_MAX      100.0f   // 转向差速限幅 (RPM)

typedef struct {
    // 参数（可通过 CLI 在线调整）
    float   kp_angle;       // 角度比例增益 (RPM/°)
    float   kd_gyro;        // 角速度阻尼增益 (RPM per °/s)
    float   kff_speed;      // 速度前馈增益 (暂未使用, 预留)
    float   target_angle;   // 目标倾角 (°), 通常 0
    float   target_speed;   // 遥控前向速度指令 (RPM)
    float   steer;          // 转向指令 (RPM 差速量)
    float   output_max;     // 平衡输出限幅 (RPM)

    // 运行状态（只读）
    float   tilt_angle;     // 当前倾角 (°), 卡尔曼估计值
    float   gyro_rate;      // 当前陀螺仪角速度 (°/s)
    float   balance_out;    // 平衡 PID 输出 (RPM)
    float   speed_ref_l;    // 左轮速度指令 (RPM)
    float   speed_ref_r;    // 右轮速度指令 (RPM)

    uint8_t active;         // 平衡控制激活标志
} BalanceCtrl_t;

void BalanceCtrl_Init(BalanceCtrl_t *bc);
void BalanceCtrl_Run(BalanceCtrl_t *bc);

#endif
```

- [ ] **Step 2: 更新 CMakeLists.txt — 添加 balance_ctrl 引用**

在 `add_executable` 的源文件列表中添加（在 `Core/Inc/cli.h` 行之后）：
```
        Core/Inc/balance_ctrl.h
        Core/Src/balance_ctrl.c
```

- [ ] **Step 3: 添加编译验证（预期失败，balance_ctrl.c 未创建）**

```bash
cmake --build --preset Debug
```

Expected: 链接错误 "undefined reference to BalanceCtrl_Init/Run"。

- [ ] **Step 4: 提交**

```bash
git add Core/Inc/balance_ctrl.h CMakeLists.txt
git commit -m "feat: 新增 balance_ctrl.h — 平衡控制结构体与参数定义"
```

---

### Task 7: 创建 balance_ctrl.c

**Files:**
- Create: `Core/Src/balance_ctrl.c`

- [ ] **Step 1: 创建 balance_ctrl.c**

```c
#include "balance_ctrl.h"
#include <math.h>

void BalanceCtrl_Init(BalanceCtrl_t *bc)
{
    bc->kp_angle     = BALANCE_KP_DEFAULT;
    bc->kd_gyro      = BALANCE_KD_DEFAULT;
    bc->kff_speed    = 0.0f;
    bc->target_angle = 0.0f;
    bc->target_speed = 0.0f;
    bc->steer        = 0.0f;
    bc->output_max   = BALANCE_OUTPUT_MAX;

    bc->tilt_angle   = 0.0f;
    bc->gyro_rate    = 0.0f;
    bc->balance_out  = 0.0f;
    bc->speed_ref_l  = 0.0f;
    bc->speed_ref_r  = 0.0f;
    bc->active       = 0;
}

void BalanceCtrl_Run(BalanceCtrl_t *bc)
{
    if (!bc->active) {
        bc->balance_out = 0.0f;
        bc->speed_ref_l = 0.0f;
        bc->speed_ref_r = 0.0f;
        return;
    }

    // 平衡 PID: 角度比例 + 角速度阻尼 + 遥控速度偏置
    float tilt_err = bc->target_angle - bc->tilt_angle;
    float output = bc->kp_angle * tilt_err
                 + bc->kd_gyro  * bc->gyro_rate
                 + bc->target_speed;

    // 限幅
    if (output >  bc->output_max) output =  bc->output_max;
    if (output < -bc->output_max) output = -bc->output_max;

    bc->balance_out = output;

    // 差速混合: 左右轮 = 平衡输出 ± 转向偏置
    float steer = bc->steer;
    if (steer >  BALANCE_STEER_MAX) steer =  BALANCE_STEER_MAX;
    if (steer < -BALANCE_STEER_MAX) steer = -BALANCE_STEER_MAX;

    bc->speed_ref_l = output + steer;
    bc->speed_ref_r = output - steer;
}
```

- [ ] **Step 2: 编译验证**

```bash
cmake --build --preset Debug
```

Expected: 编译成功（balance_ctrl.c 编译通过，链接解决）。

- [ ] **Step 3: 提交**

```bash
git add Core/Src/balance_ctrl.c
git commit -m "feat: 实现 balance_ctrl — 平衡 PID + 差速混合"
```

---

### Task 8: 简化 main.c 启动流程

**Files:**
- Modify: `Core/Src/main.c`

- [ ] **Step 1: 移除 g_test_mode 和相关启动选择器**

删除：
```c
uint8_t g_test_mode = 0;
```

将 3 秒倒计时选择器（第 145-169 行）替换为简化的单模式启动。

**删除的原始代码块**（第 145-169 行，约 25 行）：启动模式选择器（`for (int t = 3; ...)` 循环 + `g_calib_mode` / `g_test_mode` 检测）。

**替换为：**
```c
    // 简短倒计时，期间可按 'c' 进入校准模式
    printf("\r\n=== STM32G431 Balance Car ===\r\n");
    printf("Press 'c' to enter calibration mode (1.5s)...\r\n");
    while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC));
    HAL_Delay(50);

    for (int w = 0; w < 15; w++) {
        HAL_Delay(100);
        while (COMM_Available() > 0) {
            uint8_t ch = COMM_ReadByte();
            if (ch == 'c' || ch == 'C') { g_calib_mode = 1; break; }
        }
        if (g_calib_mode) break;
    }
    if (!g_calib_mode) printf("Normal mode\r\n");
```

- [ ] **Step 2: 移除 else if (g_test_mode) 分支**

删除第 187-208 行的 `else if (g_test_mode)` 整个代码块（阶跃测试模式初始化）。

- [ ] **Step 3: 修改 else 分支（正常模式）**

这是 main.c 最后的 else 分支（第 209 行起），保持大部分逻辑不变，但做以下调整：

移除对 `g_test_mode` 的检查（无此变量）。

删除遥测任务创建相关的外部变量声明（不再需要）：
```c
extern osThreadId TaskSpeedLoopHandle;  // 改为 TaskBalanceLoopHandle
```

将第 63 行改为：
```c
extern osThreadId TaskBalanceLoopHandle;
```

- [ ] **Step 4: 编译验证（预期失败）**

```bash
cmake --build --preset Debug
```

Expected: 链接错误（app_freertos 尚未修改，TaskBalanceLoopHandle 不存在）。

- [ ] **Step 5: 提交**

```bash
git add Core/Src/main.c
git commit -m "refactor: 简化 main.c 启动流程 — 移除阶跃测试模式/3秒选择器"
```

---

### Task 9: 重构 app_freertos.c — 前半（移除旧任务）

**Files:**
- Modify: `Core/Src/app_freertos.c`

- [ ] **Step 1: 修改 include 列表**

删除不再需要的 include：
```c
#include "six_step.h"
#include "debug_capture.h"
#include "speed_capture.h"
```

将 `#include "mpu6050.h"` 改为 `#include "mpu6500.h"`。

添加：
```c
#include "balance_ctrl.h"
```

移除 `#include "pos_ctrl.h"`（位置环在平衡车中不使用，由平衡 PID 替代）。

移除 `#include "encoder_cache.h"`（不再直接引用，由 speed_ctrl 内部使用）。

- [ ] **Step 2: 修改全局变量声明**

删除：
```c
PosCtrl_t g_pos[2];
extern uint8_t g_test_mode;
extern volatile uint8_t g_capture_dumping;
extern TIM_HandleTypeDef htim17;
```

替换为：
```c
BalanceCtrl_t g_balance;
extern TIM_HandleTypeDef htim17;
```

SpeedCtrl_t g_speed[2] 保留。

将 `osThreadId TaskSpeedLoopHandle` 改为 `osThreadId TaskBalanceLoopHandle`。

- [ ] **Step 3: 修改任务函数声明**

删除：
```c
void StartTaskSpeedLoop(void const * argument);
void StartTaskIMU(void const * argument);
void TaskDebugCapture(void const *argument);
```

替换为：
```c
void StartBalanceLoopTask(void const * argument);
```

保留：
```c
void StartCLITask(void const * argument);
```

删除 `StartTaskTelemetry` 声明。

- [ ] **Step 4: 修改 MX_FREERTOS_Init — 任务创建**

删除整个 `if (g_calib_mode)` / `else if (g_test_mode)` / `else` 三分支，替换为：

```c
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
```

- [ ] **Step 5: 编译验证（预期失败）**

```bash
cmake --build --preset Debug
```

Expected: StartBalanceLoopTask 未定义。

- [ ] **Step 6: 提交**

```bash
git add Core/Src/app_freertos.c
git commit -m "refactor: 重构 app_freertos 任务布局 — 移除旧任务/模式分支"
```

---

### Task 10: 实现 StartBalanceLoopTask

**Files:**
- Modify: `Core/Src/app_freertos.c` — 添加 StartBalanceLoopTask 函数体

- [ ] **Step 1: 删除旧任务函数体**

删除以下完整函数：
- `StartTaskTelemetry`（第 234-259 行）
- `StartTaskIMU`（第 262-287 行）
- `StartTaskSpeedLoop`（第 290-342 行）— 其逻辑将合并入 StartBalanceLoopTask
- `TaskDebugCapture`（第 345-352 行）

- [ ] **Step 2: 删除 CLI 任务中的测试状态机**

在 `StartCLITask` 函数体中删除：
- `g_load_test` 状态机（第 158-189 行，约 30 行）
- `g_step_test` 状态机（第 191-226 行，约 35 行）

替换为简单循环：
```c
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
```

- [ ] **Step 3: 添加 StartBalanceLoopTask 函数体**

在 `StartCLITask` 函数之后添加：

```c
/** @brief 平衡环任务 — 1kHz TIM17 触发, IMU→卡尔曼→平衡PID→速度环→差速 */
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

  KalmanAngle_t kf;
  KalmanAngle_Init(&kf, 0.0f, 0.001f, 0.003f, 0.03f);
  const float dt = 0.001f;  // 1ms

  // 等待 IMU 稳定
  for (int i = 0; i < 100; i++) osDelay(10);

  HAL_TIM_Base_Start_IT(&htim17);

  static uint32_t tick = 0;
  for (;;) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      tick++;

      // 1. IMU 读取 + 卡尔曼倾角估计
      MPU6500_Accel_t accel;
      MPU6500_Gyro_t gyro;
      MPU6500_ReadAccel(&accel);
      MPU6500_ReadGyro(&gyro);

      float accel_angle = atan2f(accel.y, accel.z) * 57.29578f;
      KalmanAngle_Predict(&kf, gyro.x, dt);
      KalmanAngle_Update(&kf, accel_angle);

      g_balance.tilt_angle = KalmanAngle_GetAngle(&kf);
      g_balance.gyro_rate  = gyro.x - KalmanAngle_GetBias(&kf);

      // 2. 平衡 PID + 差速混合
      BalanceCtrl_Run(&g_balance);

      // 3. 速度环（双电机交替, 保持现有交替轮询逻辑）
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
              // 平衡模式: 直接写入速度指令, 启用速度环
              g_speed[i].speed_ref = speed_refs[i];
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
      }

      // 4. 遥测（可选, 按需发送）
      if (CLI_TelemetryEnabled() && (tick % 5 == 0)) {
          float frame[7];
          frame[0] = g_balance.tilt_angle;
          frame[1] = g_balance.gyro_rate;
          frame[2] = g_balance.balance_out;
          frame[3] = g_speed[0].speed_fb;
          frame[4] = g_speed[1].speed_fb;
          frame[5] = g_motor[0].iq;
          frame[6] = g_motor[1].iq;
          COMM_SendFloatFrame(frame, 7);
      }
  }
}
```

- [ ] **Step 4: 编译验证**

```bash
cmake --build --preset Debug
```

Expected: 编译失败（CLI 尚未更新，引用了 g_step_test/g_load_test 等）。

- [ ] **Step 5: 提交**

```bash
git add Core/Src/app_freertos.c
git commit -m "feat: 实现 StartBalanceLoopTask — IMU→卡尔曼→平衡PID→速度环合并任务"
```

---

### Task 11: 重构 CLI — 头文件与类型

**Files:**
- Modify: `Core/Inc/cli.h`
- Modify: `Core/Src/cli.c`

- [ ] **Step 1: 重写 cli.h**

```c
#ifndef CLI_H
#define CLI_H

#include <stdint.h>

void CLI_Init(void);
void CLI_Process(void);
uint8_t CLI_TelemetryEnabled(void);

#endif
```

- [ ] **Step 2: 重写 cli.c — 删旧留新**

完整重写 cli.c，简化为平衡车指令集：

```c
#include "cli.h"
#include "cli_parser.h"
#include "comm.h"
#include "foc.h"
#include "motor_hal.h"
#include "speed_ctrl.h"
#include "balance_ctrl.h"
#include "calibration.h"
#include "buzzer.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// 遥测开关
static uint8_t g_telem_enabled = 0;

// 外部引用
extern SpeedCtrl_t g_speed[2];
extern BalanceCtrl_t g_balance;
extern uint8_t g_calib_mode;

void CLI_Init(void)
{
    g_telem_enabled = 0;
}

uint8_t CLI_TelemetryEnabled(void)
{
    return g_telem_enabled;
}

// ===== 帮助 =====
static void CMD_Help(void)
{
    printf("\r\n=== STATUS ===\r\n");
    if (g_balance.active) {
        printf("Balance: Kp=%.1f Kd=%.1f Tilt=%.2f° Gyro=%.1f°/s Out=%.0fRPM\r\n",
               g_balance.kp_angle, g_balance.kd_gyro,
               g_balance.tilt_angle, g_balance.gyro_rate,
               g_balance.balance_out);
        printf("Speed L=%.0fRPM R=%.0fRPM fb_L=%.0f fb_R=%.0f\r\n",
               g_balance.speed_ref_l, g_balance.speed_ref_r,
               g_speed[0].speed_fb, g_speed[1].speed_fb);
    } else {
        printf("Balance: INACTIVE\r\n");
    }
    for (int i = 0; i < 2; i++) {
        printf("M%d Speed PI: Kp=%.3f Ki=%.3f | Current PI: Kp=%.1f Ki=%.0f\r\n",
               i+1, g_speed[i].kp, g_speed[i].ki,
               g_motor[i].iq_pi.kp, g_motor[i].iq_pi.ki);
    }
    printf("\r\n=== COMMANDS ===\r\n");
    printf("B             激活平衡控制\r\n");
    printf("STOP          紧急停止\r\n");
    printf("S<RPM>        前进速度指令\r\n");
    printf("T<val>        转向指令\r\n");
    printf("PK            查询平衡参数\r\n");
    printf("PK ANG=<val>  设置角度 Kp\r\n");
    printf("PK GYR=<val>  设置角速度 Kd\r\n");
    printf("PK ANG0=<val> 设置目标倾角 (°)\r\n");
    printf("PK MAX=<val>  设置输出限幅 (RPM)\r\n");
    printf("PS P=X I=Y    设置速度 PI\r\n");
    printf("PC P=X I=Y    设置电流 PI\r\n");
    printf("P             查询全部参数\r\n");
    printf("T             开关遥测\r\n");
    printf("CAL           进入校准模式\r\n");
    printf("?             帮助\r\n\r\n");
}

// ===== 运行指令 =====

static void CMD_Balance(void)
{
    if (!g_balance.active) {
        g_balance.active = 1;
        // 初始化左右轮速度模式
        for (int i = 0; i < 2; i++) {
            SpeedCtrl_EnterMode(&g_speed[i], 0.0f);
            g_motor[i].speed_mode = 1;
        }
        printf("BALANCE ON\r\n");
    }
}

static void CMD_Stop(void)
{
    g_balance.active = 0;
    for (int i = 0; i < 2; i++) {
        SpeedCtrl_ExitMode(&g_speed[i]);
        g_motor[i].speed_mode = 0;
        Motor_SetIqRef(&g_motor[i], 0.0f);
    }
    g_balance.target_speed = 0.0f;
    g_balance.steer = 0.0f;
    Motor_Neutralize(&g_motor[0]);
    Motor_Neutralize(&g_motor[1]);
    Buzzer_Beep(1000, 80);
    printf("STOP\r\n");
}

static void CMD_Speed(uint8_t first_char)
{
    char buf[16]; uint8_t p = 0;
    buf[p++] = (char)first_char;
    for (uint8_t w = 0; w < 30 && p < 15; w++) {
        if (COMM_Available() == 0) { osDelay(1); continue; }
        uint8_t c = COMM_ReadByte();
        if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+')
            buf[p++] = (char)c;
        else break;
    }
    buf[p] = '\0';
    if (p == 0) { printf("S: 需要转速值 (RPM)\r\n"); return; }
    float rpm = (float)atof(buf);
    g_balance.target_speed = rpm;
    printf("SPEED %.0fRPM\r\n", rpm);
}

static void CMD_Steer(void)
{
    float val;
    if (!CLI_ReadFloat(&val)) {
        printf("T: 需要转向值\r\n");
        return;
    }
    g_balance.steer = val;
    printf("STEER %.1f\r\n", val);
}

// ===== 参数查询/设置 =====

static void CMD_BalanceParam(void)
{
    uint8_t peek = CLI_ReadChar(5);
    while (peek == ' ') peek = CLI_ReadChar(5);

    if (peek == 0 || peek == '\r' || peek == '\n') {
        // 纯查询
        printf("Balance: Kp=%.1f Kd=%.1f Kff=%.1f TargetAngle=%.1f° Max=%.0fRPM\r\n",
               g_balance.kp_angle, g_balance.kd_gyro, g_balance.kff_speed,
               g_balance.target_angle, g_balance.output_max);
        return;
    }

    // 解析 KEY=VAL
    char key[8];
    float val;
    uint8_t got = 0;

    // 读取键名（ANG, GYR, SP, MAX, ANG0）
    char keybuf[8] = {0};
    uint8_t kp = 0;
    keybuf[kp++] = (char)peek;
    for (uint8_t w = 0; w < 5 && kp < 7; w++) {
        uint8_t c = CLI_ReadChar(5);
        if (c == 0 || c == '\r' || c == '\n' || c == '=' || c == ' ') break;
        if ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))
            keybuf[kp++] = (char)c;
        else break;
    }
    keybuf[kp] = '\0';

    // 跳过 = 和空格
    uint8_t eq = CLI_ReadChar(10);
    if (eq != '=') {
        // 可能是空格
        if (eq == ' ') eq = CLI_ReadChar(10);
        if (eq != '=') {
            printf("PK: 需要 KEY=VAL 格式\r\n");
            return;
        }
    }

    if (CLI_ReadFloat(&val)) got = 1;

    if (!got) {
        printf("PK: 需要数值\r\n");
        return;
    }

    // 应用参数
    if (strcmp(keybuf, "ANG") == 0 || strcmp(keybuf, "ang") == 0) {
        g_balance.kp_angle = val;
        printf("Balance Kp_angle=%.1f RPM/°\r\n", val);
    } else if (strcmp(keybuf, "GYR") == 0 || strcmp(keybuf, "gyr") == 0) {
        g_balance.kd_gyro = val;
        printf("Balance Kd_gyro=%.1f RPM/(°/s)\r\n", val);
    } else if (strcmp(keybuf, "SP") == 0 || strcmp(keybuf, "sp") == 0) {
        g_balance.kff_speed = val;
        printf("Balance Kff_speed=%.1f\r\n", val);
    } else if (strcmp(keybuf, "MAX") == 0 || strcmp(keybuf, "max") == 0) {
        g_balance.output_max = val;
        printf("Balance OutputMax=%.0f RPM\r\n", val);
    } else if (strcmp(keybuf, "ANG0") == 0 || strcmp(keybuf, "ang0") == 0) {
        g_balance.target_angle = val;
        printf("Balance TargetAngle=%.1f°\r\n", val);
    } else {
        printf("PK: 未知键 '%s', 可用: ANG GYR SP MAX ANG0\r\n", keybuf);
    }
}

static void CMD_AllParams(void)
{
    printf("Balance: Kp=%.1f Kd=%.1f Kff=%.1f TargetAngle=%.1f° Max=%.0fRPM\r\n",
           g_balance.kp_angle, g_balance.kd_gyro, g_balance.kff_speed,
           g_balance.target_angle, g_balance.output_max);
    for (int mi = 0; mi < 2; mi++) {
        printf("M%d Speed  PI: Kp=%.3f Ki=%.3f ω0=%.2fHz Out=±%.1fA\r\n",
               mi+1, g_speed[mi].kp, g_speed[mi].ki,
               g_speed[mi].ki / g_speed[mi].kp / 6.283f, 2.0f);
        printf("M%d Current PI: Kp=%.1f Ki=%.0f ω0=%.1fHz Out=±%.1fV\r\n",
               mi+1, g_motor[mi].iq_pi.kp, g_motor[mi].iq_pi.ki,
               g_motor[mi].iq_pi.ki / g_motor[mi].iq_pi.kp / 6.283f, FOC_VBUS);
    }
}

static void CMD_SetSpeedPI(void)
{
    uint8_t ch = CLI_ReadChar(10);
    uint8_t motor_start = 0, motor_end = 1;
    if (ch == 'R' || ch == 'r') { motor_start = motor_end = 0; }
    else if (ch == 'L' || ch == 'l') { motor_start = motor_end = 1; }
    // 否则 PC/PS → 双电机

    float kp = 0, ki = 0;
    uint8_t kp_set = 0, ki_set = 0;
    uint8_t peek = CLI_ReadChar(20);
    while (peek == ' ') peek = CLI_ReadChar(20);

    if (peek == 0 || peek == '\r' || peek == '\n') {
        for (int mi = motor_start; mi <= motor_end; mi++) {
            printf("M%d Speed PI: Kp=%.3f Ki=%.3f ω0=%.2fHz\r\n",
                   mi+1, g_speed[mi].kp, g_speed[mi].ki,
                   g_speed[mi].ki / g_speed[mi].kp / 6.283f);
        }
        return;
    }

    // 解析 P=X I=Y
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
    }

    for (int mi = motor_start; mi <= motor_end; mi++) {
        if (kp_set) g_speed[mi].kp = g_speed[mi].pi.kp = kp;
        if (ki_set) g_speed[mi].ki = g_speed[mi].pi.ki = ki;
        printf("M%d Speed PI: Kp=%.3f Ki=%.3f\r\n", mi+1,
               g_speed[mi].kp, g_speed[mi].ki);
    }
}

static void CMD_SetCurrentPI(void)
{
    uint8_t ch = CLI_ReadChar(10);
    uint8_t motor_start = 0, motor_end = 1;
    if (ch == 'R' || ch == 'r') { motor_start = motor_end = 0; }
    else if (ch == 'L' || ch == 'l') { motor_start = motor_end = 1; }

    float kp = 0, ki = 0;
    uint8_t kp_set = 0, ki_set = 0;
    uint8_t peek = CLI_ReadChar(20);
    while (peek == ' ') peek = CLI_ReadChar(20);

    if (peek == 0 || peek == '\r' || peek == '\n') {
        for (int mi = motor_start; mi <= motor_end; mi++) {
            printf("M%d Current PI: Kp=%.1f Ki=%.0f\r\n",
                   mi+1, g_motor[mi].iq_pi.kp, g_motor[mi].iq_pi.ki);
        }
        return;
    }

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
    }

    for (int mi = motor_start; mi <= motor_end; mi++) {
        if (kp_set || ki_set) {
            float use_kp = kp_set ? kp : g_motor[mi].iq_pi.kp;
            float use_ki = ki_set ? ki : g_motor[mi].iq_pi.ki;
            Motor_SetCurrentPI(&g_motor[mi], use_kp, use_ki);
        }
        printf("M%d Current PI: Kp=%.1f Ki=%.0f\r\n", mi+1,
               g_motor[mi].iq_pi.kp, g_motor[mi].iq_pi.ki);
    }
}

// ===== 主 CLI 循环 =====

void CLI_Process(void)
{
    while (COMM_Available() > 0)
    {
      uint8_t ch = COMM_ReadByte();

      // 校准模式 CLI
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

      // 正常模式 CLI
      if (ch == '?' || ch == 'H' || ch == 'h') {
          CMD_Help(); continue;
      }

      if (ch == 'B' || ch == 'b') {
          // peek 确认不是其他 B 开头命令
          uint8_t nxt = CLI_ReadChar(5);
          if (nxt == 0 || nxt == '\r' || nxt == '\n') {
              CMD_Balance();
          }
          continue;
      }

      if (ch == 'S' || ch == 's') {
          uint8_t nxt = CLI_ReadChar(20);
          if (nxt == 'T' || nxt == 't') {
              // STOP
              uint8_t nxt2 = CLI_ReadChar(5);
              if (nxt2 == 'O' || nxt2 == 'o') {
                  uint8_t nxt3 = CLI_ReadChar(5);
                  if (nxt3 == 'P' || nxt3 == 'p') {
                      CMD_Stop();
                      continue;
                  }
              }
          } else if (nxt == 0 || nxt == '\r' || nxt == '\n') {
              printf("S: 用 S<RPM> 或 STOP\r\n");
          } else if ((nxt >= '0' && nxt <= '9') || nxt == '.' || nxt == '-' || nxt == '+') {
              // S<RPM> — nxt 是数字/符号首字符
              CMD_Speed(nxt);
          } else {
              printf("S: 未知子命令 '%c'\r\n", nxt);
          }
          continue;
      }

      if (ch == 'T' || ch == 't') {
          uint8_t nxt = CLI_ReadChar(5);
          if (nxt == 0 || nxt == '\r' || nxt == '\n') {
              // 裸 T → 开关遥测
              g_telem_enabled = !g_telem_enabled;
              printf("TELEMETRY %s\r\n", g_telem_enabled ? "ON" : "OFF");
          } else if ((nxt >= '0' && nxt <= '9') || nxt == '.' || nxt == '-' || nxt == '+') {
              // T<val> 转向指令
              char buf[16]; uint8_t p = 0;
              buf[p++] = (char)nxt;
              for (uint8_t w = 0; w < 30 && p < 15; w++) {
                  if (COMM_Available() == 0) { osDelay(1); continue; }
                  uint8_t c = COMM_ReadByte();
                  if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+')
                      buf[p++] = (char)c;
                  else break;
              }
              buf[p] = '\0';
              g_balance.steer = (float)atof(buf);
              printf("STEER %.1f\r\n", g_balance.steer);
          }
          continue;
      }

      if (ch == 'P' || ch == 'p') {
          uint8_t nxt = CLI_ReadChar(10);
          if (nxt == 'K' || nxt == 'k') {
              CMD_BalanceParam();
          } else if (nxt == 'S' || nxt == 's') {
              CMD_SetSpeedPI();
          } else if (nxt == 'C' || nxt == 'c') {
              CMD_SetCurrentPI();
          } else if (nxt == 0 || nxt == '\r' || nxt == '\n') {
              CMD_AllParams();
          }
          continue;
      }

      if (ch == 'C' || ch == 'c') {
          uint8_t nxt1 = CLI_ReadChar(10);
          uint8_t nxt2 = CLI_ReadChar(10);
          if ((nxt1 == 'A' || nxt1 == 'a') && (nxt2 == 'L' || nxt2 == 'l')) {
              printf("Entering calibration mode...\r\n");
              g_calib_mode = 1;
              // 软复位进入校准
              NVIC_SystemReset();
          }
          continue;
      }

      if (ch != '\r' && ch != '\n') {
          // 静默忽略未识别字符
      }
    }
}
```

- [ ] **Step 2: 编译验证**

```bash
cmake --build --preset Debug
```

Expected: 编译成功。

- [ ] **Step 3: 提交**

```bash
git add Core/Inc/cli.h Core/Src/cli.c
git commit -m "refactor: CLI 精简为平衡车指令集 — B/S/T/PK/PS/PC/P/T/CAL"
```

---

### Task 12: 清理残留引用并验证编译

**Files:**
- 全局搜索确保无残留引用

- [ ] **Step 1: 搜索残留符号**

```bash
# 搜索已删除的符号
grep -r "six_step\|SixStep" Core/ --include="*.c" --include="*.h"
grep -r "debug_capture\|DebugCapture\|g_cap" Core/ --include="*.c" --include="*.h"
grep -r "speed_capture\|SpeedCapture" Core/ --include="*.c" --include="*.h"
grep -r "g_test_mode" Core/ --include="*.c" --include="*.h"
grep -r "g_step_test\|g_load_test" Core/ --include="*.c" --include="*.h"
grep -r "g_capture_buf\|g_capture_dumping" Core/ --include="*.c" --include="*.h"
```

- [ ] **Step 2: 处理残留引用**

若有残留引用，修改对应文件移除。

- [ ] **Step 3: 编译**

```bash
cmake --build --preset Debug
```

Expected: 编译成功，0 错误 0 警告。

- [ ] **Step 4: 提交**

```bash
git add -A
git commit -m "fix: 清理残留引用, 验证全量编译通过"
```

---

### Task 13: 更新文档

**Files:**
- Modify: `docs/FREERTOS_TASKS.md`
- Modify: `docs/COMM_PROTOCOL.md`

- [ ] **Step 1: 更新 FREERTOS_TASKS.md**

将任务表替换为平衡车布局：

```markdown
# FreeRTOS 任务列表

## 正常模式（平衡车）

| 任务 | 函数 | 周期 | 栈 (words) | 优先级 | 核心操作 |
|------|------|:----:|:---------:|:------:|------|
| TaskCLI | StartCLITask | 1ms | 256 | Normal | USART1/USART2 串口命令 + 遥测开关 + 参数调优 |
| TaskBalanceLoop | StartBalanceLoopTask | 1kHz | 768 | High | IMU→卡尔曼→平衡PID→速度环(双电机交替)→差速 (TIM17 触发) |

## 校准模式

| 任务 | 函数 | 周期 | 栈 (words) | 优先级 | 核心操作 |
|------|------|:----:|:---------:|:------:|------|
| TaskCLI | StartCLITask | 1ms | 1024 | Normal | 校准 CLI (R1-5, L1-5, RS, q) |

## 栈用量

| 模式 | TaskCLI | TaskBalanceLoop | 合计 |
|------|:------:|:---------------:|:----:|
| 正常 | 256 | 768 | 1024 words (4.1KB) |
| 校准 | 1024 | — | 1024 words (4KB) |

## 指令

| 指令 | 功能 | 例 |
|------|------|-----|
| `B` | 激活平衡控制 | |
| `STOP` | 紧急停止 | |
| `S<RPM>` | 前进速度指令 | `S100` |
| `T<val>` | 转向指令 | `T30` |
| `PK` | 查询平衡参数 | |
| `PK ANG=<val>` | 设置角度 Kp | `PK ANG=15` |
| `PK GYR=<val>` | 设置角速度 Kd | `PK GYR=2.0` |
| `PK ANG0=<val>` | 设置目标倾角 (°) | `PK ANG0=0.5` |
| `PK MAX=<val>` | 设置输出限幅 (RPM) | `PK MAX=500` |
| `PS P=X I=Y` | 设置速度 PI | `PS P=0.044 I=1.221` |
| `PC P=X I=Y` | 设置电流 PI | `PC P=15 I=5295` |
| `P` | 查询全部参数 | |
| `T` | 开关遥测 | |
| `CAL` | 进入校准模式 | |
| `?` | 帮助 | |

## 遥测帧格式（按需, 200Hz 子采样）

```
[Tilt_angle(°)] [Gyro(°/s)] [Balance_out(RPM)] [Speed_fb_L(RPM)] [Speed_fb_R(RPM)] [iq_L(A)] [iq_R(A)]
```
```

- [ ] **Step 2: 更新 COMM_PROTOCOL.md**

在遥测/指令章节添加平衡车命令和遥测通道说明。

- [ ] **Step 3: 提交**

```bash
git add docs/FREERTOS_TASKS.md docs/COMM_PROTOCOL.md
git commit -m "docs: 更新文档 — 平衡车任务布局/指令集/遥测格式"
```

---

### Task 14: USART2 蓝牙配置（需 CubeMX）

> **注意**：USART2 外设配置（引脚/中断/DMA）需通过 CubeMX `.ioc` 完成，遵循项目约束。

**CubeMX 配置**：
- 启用 USART2：Mode = Asynchronous，Baud Rate = 115200（匹配蓝牙模块）
- 在 NVIC Settings 中启用 USART2 global interrupt
- 生成代码后，在 `stm32g4xx_it.c` 的 `USART2_IRQHandler` 中将接收字节路由至 COMM 模块

**COMM 模块扩展**：
- `comm.c` 中 USART2 RX ISR 调用 `COMM_PushByte()`，使两路串口共享同一接收缓冲区
- `comm.c` 中新增 `COMM_IsTxBusy()` 用于 USART2 发送流控
- CLI 解析不变，`COMM_Available()`/`COMM_ReadByte()` 透明消费两路数据

- [ ] **Step 1: 通过 CubeMX 配置 USART2 并重新生成代码**
- [ ] **Step 2: 扩展 comm.c 支持双路 RX**
- [ ] **Step 3: 编译验证**
- [ ] **Step 4: 提交**

---

### Task 15: 最终编译验证

- [ ] **Step 1: 清理并全量重新编译**

```bash
cmake --build --preset Debug --clean-first
```

Expected: 编译成功，0 错误。

- [ ] **Step 2: 检查二进制尺寸**

```bash
arm-none-eabi-size build/Debug/STM32G431Demo.elf
```

- [ ] **Step 3: 提交（如有未提交变更）**

```bash
git add -A
git commit -m "chore: 最终编译验证通过"
```

---
