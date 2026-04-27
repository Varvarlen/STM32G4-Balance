# FOC 无刷电机驱动 — 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 基于现有 STM32G431 Demo 项目，逐层实现双电机 FOC 驱动（6步换相→电压正弦→电流闭环）

**Architecture:** 在独立的 git worktree 中开发。每层对应一组独立模块，从简到繁增量搭建。电流环在 ADC EOC 中断中 10kHz 执行，编码器通过 SPI3 DMA 乒乓读取。FreeRTOS 任务仅管理上层控制。

**Tech Stack:** STM32G431CBU6 + HAL + FreeRTOS V10.3.1 + arm-none-eabi-gcc 13.3.1，CMake + Ninja

**设计文档:** `docs/superpowers/specs/2026-04-27-foc-bldc-driver-design.md`

---

## 前置要求

- [ ] 创建设计文档中提到的 git worktree 进行隔离开发
- [ ] 确保当前 FreeRTOS 项目基线可正常编译烧录运行

---

## 任务 1: 硬件连接文档

**文件:**
- 创建: `docs/HARDWARE_CONNECTIONS.md`

- [ ] **Step 1: 写入完整硬件连接文档**

内容覆盖：两台电机的 PWM 引脚分配、INA240 电流采样通道对应关系、MT6701 编码器片选、MP6536 使能引脚、供电电源。

```markdown
# 硬件连接

## 电机驱动 PWM

| 电机 | 驱动芯片 | PWM1 | PWM2 | PWM3 | SHDNB |
|------|---------|------|------|------|-------|
| M1 | MP6536 (#1) | PB9 (TIM4_CH4) | PB7 (TIM4_CH2) | PB6 (TIM4_CH1) | PC14 |
| M2 | MP6536 (#2) | PB1 (TIM3_CH4) | PB0 (TIM3_CH3) | PA7 (TIM3_CH2) | PC14 |

## 电流采样 INA240

| 通道 | 枚举名 | 引脚 | ADC 通道 | 对应电机相位 |
|------|--------|------|----------|-------------|
| 0 | INA240_MOTOR1_U | PA5 | ADC2_IN13 | M1 V 相 |
| 1 | INA240_MOTOR1_W | PA6 | ADC2_IN3 | M1 U 相 |
| 2 | INA240_MOTOR2_U | PC4 | ADC2_IN5 | M2 V 相 |
| 3 | INA240_MOTOR2_W | PB2 | ADC2_IN12 | M2 W 相 |

> 注意：枚举名中的 U/W 与实际物理相位名不完全对应。FOC 只需两相电流（第三相由 Ia+Ib+Ic=0 推出），不影响功能。

## 编码器 MT6701

| 编码器 | 片选引脚 | SPI |
|--------|---------|-----|
| M1 | PB4 | SPI3 (SCK=PC10, MISO=PC11) |
| M2 | PA4 | SPI3 (SCK=PC10, MISO=PC11) |

## 电源

| 项目 | 值 |
|------|-----|
| 电机供电 | 7.4V |
| MCU 供电 | 3.3V |
| INA240 偏置 | 1.65V (Vs/2) |
| INA240 增益 | 20 V/V |
| 采样电阻 | 2 mΩ |
```

- [ ] **Step 2: 提交**

```bash
git add docs/HARDWARE_CONNECTIONS.md
git commit -m "docs: 添加硬件连接文档"
```

---

## 任务 2: CubeMX 重新配置（手动操作）

> 此任务由用户在 CubeMX GUI 中完成，不涉及代码修改。按照设计文档第 10 节表格逐项配置。

- [ ] **Step 1: TIM3 配置为 10kHz 中心对齐主定时器**

在 `.ioc` 文件中：

| 标签页 | 路径 | 值 |
|--------|------|-----|
| Timers → TIM3 | Mode | 勾选 Channel 2/3/4 的 PWM Generation CH2/3/4 |
| Timers → TIM3 | Counter Settings → Prescaler | 0 |
| Timers → TIM3 | Counter Settings → Counter Mode | Center Aligned mode 1 |
| Timers → TIM3 | Counter Settings → AutoReload | 8499 |
| Timers → TIM3 | Trigger Output (TRGO) Parameters → Trigger Event Selection | Update Event |
| Timers → TIM3 | PWM Generation CH2/3/4 → Pulse | 0 |
| Timers → TIM3 | PWM Generation CH2/3/4 → Mode | PWM mode 1 |
| Timers → TIM3 | PWM Generation CH2/3/4 → CH Polarity | High |

- [ ] **Step 2: TIM4 配置为 TIM3 同步从定时器**

| 标签页 | 路径 | 值 |
|--------|------|-----|
| Timers → TIM4 | Mode | 勾选 Channel 1/2/4 的 PWM Generation CH1/2/4 |
| Timers → TIM4 | Slave Mode → Slave Mode | Trigger Mode |
| Timers → TIM4 | Slave Mode → Trigger Source | ITR2 (TIM3 Trigger) |
| Timers → TIM4 | Counter Settings → Prescaler | 0 |
| Timers → TIM4 | Counter Settings → Counter Mode | Center Aligned mode 1 |
| Timers → TIM4 | Counter Settings → AutoReload | 8499 |
| Timers → TIM4 | PWM Generation CH1/2/4 → Pulse | 0 |
| Timers → TIM4 | PWM Generation CH1/2/4 → Mode | PWM mode 1 |
| Timers → TIM4 | PWM Generation CH1/2/4 → CH Polarity | High |

- [ ] **Step 3: ADC2 配置为 TIM3 TRGO 触发**

| 标签页 | 路径 | 值 |
|--------|------|-----|
| Analog → ADC2 | Mode | 勾选 IN3/IN5/IN12/IN13 |
| Analog → ADC2 | ADC_Settings → External Trigger Source | Timer 3 Trigger Out event |
| Analog → ADC2 | ADC_Settings → External Trigger → Trigger Detection | Rising edge |
| Analog → ADC2 | ADC_Settings → Continuous Conversion Mode | Disabled |
| Analog → ADC2 | ADC_Settings → Discontinuous Conversion Mode | Disabled |
| Analog → ADC2 | ADC_Settings → Scan Conversion Mode | Enabled |
| Analog → ADC2 | ADC_Settings → Number of Conversions | 4 |
| Analog → ADC2 | Rank 1-4 → Channel | IN3, IN5, IN12, IN13（按设计顺序） |
| Analog → ADC2 | Rank 1-4 → Sampling Time | 2.5 Cycles |
| Analog → ADC2 | ADC_Regular_ConversionMode → DMA Continuous Requests | Enabled |
| Analog → ADC2 | NVIC Settings → ADC1 and ADC2 interrupts | Enabled |
| Analog → ADC2 | NVIC Settings → Preemption Priority | 5（运行时覆盖为 4） |

- [ ] **Step 4: SPI3 配置为 8MHz**

| 标签页 | 路径 | 值 |
|--------|------|-----|
| Connectivity → SPI3 | Mode | Full-Duplex Master |
| Connectivity → SPI3 | Parameter Settings → Data Size | 8 Bits |
| Connectivity → SPI3 | Parameter Settings → CPOL | Low |
| Connectivity → SPI3 | Parameter Settings → CPHA | 2 Edge |
| Connectivity → SPI3 | Parameter Settings → Baud Rate Prescaler | 使 SCK ≈ 8 MHz |

- [ ] **Step 5: 设置 USART1 中断优先级**（确保不被 FOC ISR 影响）

| 标签页 | 路径 | 值 |
|--------|------|-----|
| Connectivity → USART1 | NVIC Settings → Preemption Priority | 8 |

- [ ] **Step 6: 设置 SPI3 DMA 中断优先级**

| 标签页 | 路径 | 值 |
|--------|------|-----|
| Connectivity → SPI3 | DMA Settings → SPI3_RX | 添加 DMA 通道 |
| Connectivity → SPI3 | DMA Settings → Mode | Normal（非 Circular） |
| Connectivity → SPI3 | DMA Settings → Data Width | Byte |
| Connectivity → SPI3 | NVIC Settings → SPI3 global interrupt | Enabled |
| Connectivity → SPI3 | NVIC Settings → Preemption Priority | 6 |

- [ ] **Step 7: 重新生成代码**

CubeMX → GENERATE CODE

- [ ] **Step 8: 验证编译**

```bash
cmake --preset Debug
cmake --build --preset Debug
```

预期：编译通过。可能有未使用变量/函数的 warning，后续任务逐步消除。

- [ ] **Step 9: 烧录验证（确认系统仍能启动）**

```bash
tools/start_gdb_server.bat  # 另开终端
timeout 5 arm-none-eabi-gdb build/Debug/STM32G431Demo.elf -x tools/flash.gdb
```

预期：听到初始化提示音（蜂鸣器 2000Hz），说明 FreeRTOS 正常启动。

- [ ] **Step 10: 提交**

```bash
git add -A
git commit -m "feat: CubeMX 重新配置 — TIM3/4 中心对齐同步 PWM, ADC2 TRGO 触发, SPI3 8MHz"
```

---

## 任务 3: 基础模块

**新建文件:**
- `Core/Inc/foc.h`
- `Core/Inc/motor_hal.h`
- `Core/Src/motor_hal.c`
- `Core/Inc/transform.h`
- `Core/Src/transform.c`
- `Core/Inc/pi.h`
- `Core/Src/pi.c`
- `Core/Inc/svpwm.h`
- `Core/Src/svpwm.c`

**修改文件:**
- `CMakeLists.txt` — 添加新 .c/.h 文件

- [ ] **Step 1: 创建 `Core/Inc/foc.h` — FOC 主结构体**

```c
#ifndef FOC_H
#define FOC_H

#include <stdint.h>
#include "tim.h"
#include "ina240.h"
#include "pi.h"

#define MOTOR_COUNT     2U
#define FOC_PWM_FREQ    10000U    // 10kHz
#define FOC_DT          (1.0f / FOC_PWM_FREQ)  // 100us
#define MOTOR_POLE_PAIRS 7U

// 电机工作模式
typedef enum {
    MOTOR_MODE_OFF = 0,
    MOTOR_MODE_SIX_STEP,
    MOTOR_MODE_VOLTAGE_SINE,
    MOTOR_MODE_CURRENT_LOOP
} MotorMode_t;

// PWM 三相通道映射（一个定时器上的三路 PWM）
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t ch_a;   // TIM_CHANNEL_x — A 相
    uint32_t ch_b;   // TIM_CHANNEL_x — B 相
    uint32_t ch_c;   // TIM_CHANNEL_x — C 相
    uint32_t arr;    // AutoReload 值（用于占空比计算）
} PWM_Channels_t;

typedef struct {
    uint8_t id;                    // 0 = M1, 1 = M2
    PWM_Channels_t pwm;            // PWM 输出通道
    INA240_Channel_t ch_u;         // 电流采样第一相
    INA240_Channel_t ch_v;         // 电流采样第二相
    MotorMode_t mode;              // 当前工作模式
    float elec_angle;              // 电角度 (rad)，由编码器 DMA ISR 更新
    float dt;                      // 控制周期 (s)
    float voltage_mag;             // 开环电压幅值 (V)
    float iq_ref;                  // q 轴电流给定 (A)，上层任务写入
    float id_ref;                  // d 轴电流给定 (A)，通常为 0
    PI_t id_pi;                    // d 轴 PI 控制器
    PI_t iq_pi;                    // q 轴 PI 控制器
    float ia, ib, ic;              // 三相电流（调试用）
    float id, iq;                  // dq 轴电流（调试用）
    float vd, vq;                  // dq 轴电压输出（调试用）
    float duty_a, duty_b, duty_c;  // 三相占空比 [0, 1]（调试用）
} Motor_t;

extern Motor_t g_motor[2];

void FOC_Init(void);

#endif
```

- [ ] **Step 2: 创建 `Core/Inc/motor_hal.h`**

```c
#ifndef MOTOR_HAL_H
#define MOTOR_HAL_H

#include <stdint.h>
#include "foc.h"

// 使能所有电机（拉高 PC14 → MP6536 SHDNB）
void Motor_Enable(void);
// 禁能所有电机（拉低 PC14）
void Motor_Disable(void);
// 启动指定电机的 PWM 输出
void Motor_StartPWM(Motor_t *motor);
// 停止指定电机的 PWM 输出
void Motor_StopPWM(Motor_t *motor);
// 设置三相占空比 [0, 1]
void Motor_SetDuty(Motor_t *motor, float duty_a, float duty_b, float duty_c);

#endif
```

- [ ] **Step 3: 创建 `Core/Src/motor_hal.c`**

```c
#include "motor_hal.h"
#include "main.h"

// PC14 在 CubeMX 中未设标签，直接使用 GPIOC + GPIO_PIN_14
#define MOTOR_ENABLE_PORT   GPIOC
#define MOTOR_ENABLE_PIN    GPIO_PIN_14

void Motor_Enable(void)
{
    HAL_GPIO_WritePin(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN, GPIO_PIN_SET);
}

void Motor_Disable(void)
{
    HAL_GPIO_WritePin(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN, GPIO_PIN_RESET);
}

void Motor_StartPWM(Motor_t *motor)
{
    if (motor == NULL) return;
    HAL_TIM_PWM_Start(motor->pwm.htim, motor->pwm.ch_a);
    HAL_TIM_PWM_Start(motor->pwm.htim, motor->pwm.ch_b);
    HAL_TIM_PWM_Start(motor->pwm.htim, motor->pwm.ch_c);
}

void Motor_StopPWM(Motor_t *motor)
{
    if (motor == NULL) return;
    HAL_TIM_PWM_Stop(motor->pwm.htim, motor->pwm.ch_a);
    HAL_TIM_PWM_Stop(motor->pwm.htim, motor->pwm.ch_b);
    HAL_TIM_PWM_Stop(motor->pwm.htim, motor->pwm.ch_c);
}

void Motor_SetDuty(Motor_t *motor, float duty_a, float duty_b, float duty_c)
{
    if (motor == NULL) return;
    // 限幅
    if (duty_a > 1.0f) duty_a = 1.0f;
    if (duty_a < 0.0f) duty_a = 0.0f;
    if (duty_b > 1.0f) duty_b = 1.0f;
    if (duty_b < 0.0f) duty_b = 0.0f;
    if (duty_c > 1.0f) duty_c = 1.0f;
    if (duty_c < 0.0f) duty_c = 0.0f;

    uint32_t arr = motor->pwm.arr;
    __HAL_TIM_SET_COMPARE(motor->pwm.htim, motor->pwm.ch_a, (uint32_t)(duty_a * arr));
    __HAL_TIM_SET_COMPARE(motor->pwm.htim, motor->pwm.ch_b, (uint32_t)(duty_b * arr));
    __HAL_TIM_SET_COMPARE(motor->pwm.htim, motor->pwm.ch_c, (uint32_t)(duty_c * arr));
}
```

- [ ] **Step 4: 创建 `Core/Inc/transform.h`**

```c
#ifndef TRANSFORM_H
#define TRANSFORM_H

// Clarke 变换：Ia, Ib, Ic → Iα, Iβ（幅值不变形式）
void Clarke(float Ia, float Ib, float Ic, float *I_alpha, float *I_beta);

// Park 变换：Iα, Iβ → Id, Iq
void Park(float I_alpha, float I_beta, float theta, float *I_d, float *I_q);

// 逆 Park 变换：Vd, Vq → Vα, Vβ
void InvPark(float V_d, float V_q, float theta, float *V_alpha, float *V_beta);

#endif
```

- [ ] **Step 5: 创建 `Core/Src/transform.c`**

```c
#include "transform.h"
#include <math.h>

#define ONE_OVER_SQRT3  0.5773502692f  // 1/sqrt(3)
#define SQRT3_OVER_2     0.8660254038f  // sqrt(3)/2

void Clarke(float Ia, float Ib, float Ic, float *I_alpha, float *I_beta)
{
    *I_alpha = Ia;
    *I_beta = (Ia + 2.0f * Ib) * ONE_OVER_SQRT3;
    (void)Ic;
}

void Park(float I_alpha, float I_beta, float theta, float *I_d, float *I_q)
{
    float c = cosf(theta);
    float s = sinf(theta);
    *I_d =  I_alpha * c + I_beta * s;
    *I_q = -I_alpha * s + I_beta * c;
}

void InvPark(float V_d, float V_q, float theta, float *V_alpha, float *V_beta)
{
    float c = cosf(theta);
    float s = sinf(theta);
    *V_alpha = V_d * c - V_q * s;
    *V_beta = V_d * s + V_q * c;
}
```

- [ ] **Step 6: 创建 `Core/Inc/pi.h`**

```c
#ifndef PI_H
#define PI_H

typedef struct {
    float kp;
    float ki;
    float integral;
    float out_max;
    float out_min;
} PI_t;

// 初始化 PI 控制器
void PI_Init(PI_t *pi, float kp, float ki, float out_max, float out_min);
// 执行一步控制计算
float PI_Step(PI_t *pi, float error, float dt);
// 重置积分项
void PI_Reset(PI_t *pi);

#endif
```

- [ ] **Step 7: 创建 `Core/Src/pi.c`**

```c
#include "pi.h"

void PI_Init(PI_t *pi, float kp, float ki, float out_max, float out_min)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->integral = 0.0f;
    pi->out_max = out_max;
    pi->out_min = out_min;
}

float PI_Step(PI_t *pi, float error, float dt)
{
    float p_term = pi->kp * error;
    pi->integral += pi->ki * error * dt;

    // 积分限幅（抗饱和）
    if (pi->integral > pi->out_max)  pi->integral = pi->out_max;
    if (pi->integral < pi->out_min)  pi->integral = pi->out_min;

    float output = p_term + pi->integral;

    // 输出限幅
    if (output > pi->out_max)  output = pi->out_max;
    if (output < pi->out_min)  output = pi->out_min;

    return output;
}

void PI_Reset(PI_t *pi)
{
    pi->integral = 0.0f;
}
```

- [ ] **Step 8: 创建 `Core/Inc/svpwm.h`**

```c
#ifndef SVPWM_H
#define SVPWM_H

#include "foc.h"

// SVPWM 计算：Vα, Vβ → 三相占空比 + 写 PWM 寄存器
void SVPWM_SetVab(float v_alpha, float v_beta, Motor_t *motor);

#endif
```

- [ ] **Step 9: 创建 `Core/Src/svpwm.c`** — 中线钳位 SVPWM

```c
#include "svpwm.h"
#include "motor_hal.h"
#include <math.h>

void SVPWM_SetVab(float v_alpha, float v_beta, Motor_t *motor)
{
    // 逆 Clarke（Vα,Vβ → 三相电压）
    float v_a = v_alpha;
    float v_b = -0.5f * v_alpha + 0.8660254038f * v_beta;
    float v_c = -0.5f * v_alpha - 0.8660254038f * v_beta;

    // 注入三次谐波（中线钳位 = SVPWM 等效）
    float v_max = v_a;
    if (v_b > v_max) v_max = v_b;
    if (v_c > v_max) v_max = v_c;

    float v_min = v_a;
    if (v_b < v_min) v_min = v_b;
    if (v_c < v_min) v_min = v_c;

    float v_offset = (v_max + v_min) * 0.5f;
    v_a -= v_offset;
    v_b -= v_offset;
    v_c -= v_offset;

    // 归一化到 [0, 1]，中心为 0.5
    float duty_a = v_a + 0.5f;
    float duty_b = v_b + 0.5f;
    float duty_c = v_c + 0.5f;

    Motor_SetDuty(motor, duty_a, duty_b, duty_c);

    // 保存调试变量
    motor->duty_a = duty_a;
    motor->duty_b = duty_b;
    motor->duty_c = duty_c;
}
```

- [ ] **Step 10: 创建 `Core/Src/foc.c`** — 全局变量和初始化

```c
#include "foc.h"
#include "motor_hal.h"
#include "main.h"

// 全局电机对象
Motor_t g_motor[2];

void FOC_Init(void)
{
    // --- M1 (TIM4) ---
    g_motor[0].id = 0;
    g_motor[0].pwm.htim = &htim4;
    g_motor[0].pwm.ch_a = TIM_CHANNEL_1;  // PB6
    g_motor[0].pwm.ch_b = TIM_CHANNEL_2;  // PB7
    g_motor[0].pwm.ch_c = TIM_CHANNEL_4;  // PB9
    g_motor[0].pwm.arr = 8499;
    g_motor[0].ch_u = INA240_MOTOR1_U;    // PA5, ADC2_IN13
    g_motor[0].ch_v = INA240_MOTOR1_W;    // PA6, ADC2_IN3
    g_motor[0].mode = MOTOR_MODE_OFF;
    g_motor[0].dt = FOC_DT;
    g_motor[0].voltage_mag = 0.0f;
    g_motor[0].iq_ref = 0.0f;
    g_motor[0].id_ref = 0.0f;

    // --- M2 (TIM3) ---
    g_motor[1].id = 1;
    g_motor[1].pwm.htim = &htim3;
    g_motor[1].pwm.ch_a = TIM_CHANNEL_2;  // PA7
    g_motor[1].pwm.ch_b = TIM_CHANNEL_3;  // PB0
    g_motor[1].pwm.ch_c = TIM_CHANNEL_4;  // PB1
    g_motor[1].pwm.arr = 8499;
    g_motor[1].ch_u = INA240_MOTOR2_U;    // PC4, ADC2_IN5
    g_motor[1].ch_v = INA240_MOTOR2_W;    // PB2, ADC2_IN12
    g_motor[1].mode = MOTOR_MODE_OFF;
    g_motor[1].dt = FOC_DT;
    g_motor[1].voltage_mag = 0.0f;
    g_motor[1].iq_ref = 0.0f;
    g_motor[1].id_ref = 0.0f;
}
```

- [ ] **Step 11: 更新 `CMakeLists.txt`** — 添加新源文件到 `add_executable`

在 `add_executable(${CMAKE_PROJECT_NAME}` 块末尾（`Core/Src/comm_protocol.c)` 之后）添加：

```cmake
        Core/Inc/foc.h
        Core/Src/foc.c
        Core/Inc/motor_hal.h
        Core/Src/motor_hal.c
        Core/Inc/transform.h
        Core/Src/transform.c
        Core/Inc/pi.h
        Core/Src/pi.c
        Core/Inc/svpwm.h
        Core/Src/svpwm.c
```

- [ ] **Step 12: 编译验证**

```bash
cmake --preset Debug && cmake --build --preset Debug
```

预期：编译通过。如果有 PC14 引脚 define 不匹配的 error，在 `gpio.h` / `main.h` 中查找实际名称后修正。

- [ ] **Step 13: 提交**

```bash
git add Core/Inc/foc.h Core/Src/foc.c Core/Inc/motor_hal.h Core/Src/motor_hal.c \
        Core/Inc/transform.h Core/Src/transform.c Core/Inc/pi.h Core/Src/pi.c \
        Core/Inc/svpwm.h Core/Src/svpwm.c CMakeLists.txt
git commit -m "feat: 添加 FOC 基础模块 — Motor HAL, Transform, PI, SVPWM"
```

---

## 任务 4: 层 0 — 6 步换相

**新建文件:**
- `Core/Inc/six_step.h`
- `Core/Src/six_step.c`

**修改文件:**
- `Core/Src/foc.c` — 初始化 PI 参数
- `Core/Src/app_freertos.c` — 添加 6 步换相任务
- `CMakeLists.txt` — 添加新文件
- `Core/Src/main.c` — 调用 `FOC_Init()`, `Motor_Enable()`

- [ ] **Step 1: 创建 `Core/Inc/six_step.h`**

```c
#ifndef SIX_STEP_H
#define SIX_STEP_H

#include "foc.h"

void SixStep_Init(Motor_t *motor, float voltage_mag);
void SixStep_Run(Motor_t *motor);

#endif
```

- [ ] **Step 2: 创建 `Core/Src/six_step.c`**

```c
#include "six_step.h"
#include "motor_hal.h"
#include "mt6701.h"

// 6 扇区对应三相开关表
//   A+/B-/C- = 1/0/0 表示 A 上桥通, B 下桥通, C 下桥通
//   duty 用于限制电流（等效电压）
typedef struct {
    float a, b, c;
} StepTable_t;

static const StepTable_t step_table[6] = {
    { 1, 0, 0 },   // 扇区 0: A+ B-  (C 关断)
    { 1, 1, 0 },   // 扇区 1: A+ C-  (B 关断)
    { 0, 1, 0 },   // 扇区 2: B+ C-  (A 关断)
    { 0, 1, 1 },   // 扇区 3: B+ A-  (C 关断)
    { 0, 0, 1 },   // 扇区 4: C+ A-  (B 关断)
    { 1, 0, 1 },   // 扇区 5: C+ B-  (A 关断)
};

void SixStep_Init(Motor_t *motor, float voltage_mag)
{
    motor->mode = MOTOR_MODE_SIX_STEP;
    motor->voltage_mag = voltage_mag;
}

void SixStep_Run(Motor_t *motor)
{
    // 读机械角度 (0-16383) → 电气角度 (0-360) → 扇区 (0-5)
    uint16_t raw_angle = MT6701_ReadAngle(motor->id);
    uint16_t elec_raw = (raw_angle * MOTOR_POLE_PAIRS) % MT6701_RESOLUTION;
    uint8_t sector = (uint8_t)((uint32_t)elec_raw * 6 / MT6701_RESOLUTION);

    const StepTable_t *step = &step_table[sector];
    float mag = motor->voltage_mag;

    Motor_SetDuty(motor, step->a * mag, step->b * mag, step->c * mag);
}
```

- [ ] **Step 3: 更新 `Core/Src/foc.c`** — 添加 PI 初始化和编码器初始化

在 `FOC_Init()` 末尾（`g_motor[1]` 初始化之后）添加：

```c
    // 初始化 PI 控制器（电流环参数，后续调优）
    for (int i = 0; i < 2; i++)
    {
        PI_Init(&g_motor[i].id_pi, 0.1f, 10.0f, 3.7f, -3.7f);
        PI_Init(&g_motor[i].iq_pi, 0.1f, 10.0f, 3.7f, -3.7f);
    }
```

- [ ] **Step 4: 修改 `Core/Src/main.c`** — 在初始化阶段添加 FOC 初始化

在 `/* USER CODE BEGIN 2 */` 区域内，`INA240_Calibrate()` 之后、`MPU6050_` 调用之前：

```c
  // FOC 初始化（MOVED: 移到此处）
  FOC_Init();
  Motor_Enable();
```

同时在文件顶部 `/* USER CODE BEGIN Includes */` 添加：

```c
#include "foc.h"
#include "motor_hal.h"
```

- [ ] **Step 5: 修改 `Core/Src/app_freertos.c`** — 添加 6 步换相任务

在 `/* USER CODE BEGIN Includes */` 添加：

```c
#include "foc.h"
#include "six_step.h"
```

在 `/* USER CODE BEGIN FunctionPrototypes */` 添加：

```c
void TaskSixStep(void const *argument);
```

在 `/* USER CODE BEGIN Application */` 添加：

```c
void TaskSixStep(void const *argument)
{
    (void)argument;
    SixStep_Init(&g_motor[0], 0.3f);  // M1, 30% 占空比起步
    Motor_StartPWM(&g_motor[0]);

    TickType_t last_wake = xTaskGetTickCount();
    for (;;)
    {
        SixStep_Run(&g_motor[0]);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));  // 1ms 周期（~1kHz 换相）
    }
}
```

在 `MX_FREERTOS_Init()` 的 `/* USER CODE BEGIN RTOS_THREADS */` 内添加任务注册：

```c
  osThreadDef(sixStepTask, TaskSixStep, osPriorityNormal, 0, 128);
  osThreadCreate(osThread(sixStepTask), NULL);
```

- [ ] **Step 6: 更新 CMakeLists.txt** — 添加 six_step 文件

```cmake
        Core/Inc/six_step.h
        Core/Src/six_step.c
```

- [ ] **Step 7: 编译并烧录验证**

```bash
cmake --preset Debug && cmake --build --preset Debug
```

烧录后验证：
1. 听到蜂鸣器初始化提示音
2. M1 电机能否转动（即使抖动）
3. 串口可读取编码器角度（使用现有 `reporterTask`）

- [ ] **Step 8: 提交**

```bash
git add Core/Inc/six_step.h Core/Src/six_step.c Core/Src/foc.c \
        Core/Src/main.c Core/Src/app_freertos.c CMakeLists.txt
git commit -m "feat: 层 0 — 6 步换相驱动 M1"
```

---

## 任务 5: 层 1 — 电压模式正弦波（SVPWM 开环）

**修改文件:**
- `Core/Src/app_freertos.c` — 修改任务为电压正弦模式

- [ ] **Step 1: 修改 `Core/Src/app_freertos.c`**

修改 `TaskSixStep` 实现（或新增 `TaskVoltageSine`）。建议在任务实现中切换 `MOTOR_MODE_VOLTAGE_SINE`：

在 `TaskSixStep` 函数基础上修改（或新建 `TaskVoltageSine`）：

```c
void TaskVoltageSine(void const *argument)
{
    (void)argument;
    g_motor[0].mode = MOTOR_MODE_VOLTAGE_SINE;
    g_motor[0].voltage_mag = 0.5f;  // 0.5V 起转
    Motor_StartPWM(&g_motor[0]);

    TickType_t last_wake = xTaskGetTickCount();
    for (;;)
    {
        // 读编码器机械角度 → 电气角度
        uint16_t raw = MT6701_ReadAngle(g_motor[0].id);
        float mech_angle = (float)raw * 6.283185307f / 16384.0f;
        float elec_angle = mech_angle * MOTOR_POLE_PAIRS;
        g_motor[0].elec_angle = elec_angle;

        // Vα = Vm * cos(θ), Vβ = Vm * sin(θ)
        float vm = g_motor[0].voltage_mag;
        float v_alpha = vm * cosf(elec_angle);
        float v_beta  = vm * sinf(elec_angle);

        SVPWM_SetVab(v_alpha, v_beta, &g_motor[0]);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}
```

> 注意：层 1 仍使用 SPI 阻塞读取编码器。DMA 乒乓读取在层 2 才实现。

- [ ] **Step 2: 注册任务**

如果新建了任务函数，在 `MX_FREERTOS_Init()` 中注册，并移除旧的 6 步任务。

- [ ] **Step 3: 编译烧录验证**

```bash
cmake --build --preset Debug && bash tools/flash_gdb.sh
```

验证：
1. 电机平稳旋转（扭矩脉动明显低于 6 步）
2. 调整 `voltage_mag`（0.3f - 1.0f）可改变转速
3. 串口看编码器角度平滑递增

- [ ] **Step 4: 提交**

```bash
git add Core/Src/app_freertos.c
git commit -m "feat: 层 1 — SVPWM 电压模式正弦波开环驱动"
```

---

## 任务 6: MT6701 DMA 乒乓编码器读取

**新建文件:**
- `Core/Inc/encoder_cache.h`

**修改文件:**
- `Core/Inc/mt6701.h` — 添加 DMA 接口
- `Core/Src/mt6701.c` — 实现 DMA 乒乓 + 寄存器级单次读
- `Core/Src/spi.c` — 配置 SPI3 DMA RX
- `Core/Src/stm32g4xx_it.c` — SPI3 DMA 完成回调
- `CMakeLists.txt`

- [ ] **Step 1: 创建 `Core/Inc/encoder_cache.h`**

```c
#ifndef ENCODER_CACHE_H
#define ENCODER_CACHE_H

#include <stdint.h>
#include "mt6701.h"

// 编码器角度缓存（SPI DMA ISR 写入，FOC ISR / FreeRTOS 任务读取）
typedef struct {
    uint16_t raw_angle;               // 14-bit 原始角度
    float    mech_angle;              // 机械角度 (rad)
    float    elec_angle;              // 电角度 (rad) = mech * 极对数
    uint8_t  status;                  // MT6701 状态字
    volatile uint8_t fresh;           // 新数据标志（1=本轮有新数据）
} EncoderCache_t;

extern EncoderCache_t g_enc[MT6701_NUM_ENCODERS];

#endif
```

- [ ] **Step 2: 修改 `Core/Inc/mt6701.h`** — 添加 DMA 函数声明

在原有接口之下添加：

```c
#include "encoder_cache.h"

// DMA 乒乓读取（ISR 中调用，不阻塞）
// 启动指定编码器的单次 DMA 读取。DMA 完成后在回调中：
//   1. 解析角度 → 更新 g_enc[index]
//   2. 启动另一个编码器的 DMA，形成乒乓循环
void MT6701_StartDMA(uint8_t index);

// DMA 完成回调中调用的解析/启下一轮函数
void MT6701_OnDMAComplete(uint8_t index);

// 计算电角度（极对数 × 机械角度）
// mech_angle_rad: 机械角度弧度 [0, 2π]
static inline float MT6701_ToElecAngle(float mech_angle_rad)
{
    return mech_angle_rad * MOTOR_POLE_PAIRS;
}
```

- [ ] **Step 3: 修改 `Core/Src/mt6701.c`** — 实现 DMA 乒乓

保留原有 `MT6701_Init`, `MT6701_ReadRaw` 等阻塞接口。在文件末尾 `/* USER CODE END */` 前追加：

```c
// ===== DMA 乒乓读取 =====
#include "encoder_cache.h"

EncoderCache_t g_enc[MT6701_NUM_ENCODERS] = {0};

// 3 字节 DMA 传输缓冲（每编码器独立，避免 DMA 竞争）
static uint8_t dma_tx[MT6701_NUM_ENCODERS][3];
static uint8_t dma_rx[MT6701_NUM_ENCODERS][3];

// 当前正在 DMA 传输的编码器索引（在 StartDMA 中记录，回调中读取）
static volatile uint8_t dma_current_index = 0;

// CS=0 → 启动一次 SPI DMA RX（3 字节）
void MT6701_StartDMA(uint8_t index)
{
    dma_current_index = index;
    HAL_GPIO_WritePin((GPIO_TypeDef *)cs_port[index], cs_pin[index], GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(&hspi3, dma_tx[index], dma_rx[index], 3);
}

// SPI TX/RX 完成回调
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi3)
    {
        MT6701_OnDMAComplete(dma_current_index);
    }
}

void MT6701_OnDMAComplete(uint8_t index)
{
    // CS 拉高
    HAL_GPIO_WritePin((GPIO_TypeDef *)cs_port[index], cs_pin[index], GPIO_PIN_SET);

    // 解析 24-bit SSI 帧
    uint32_t raw = ((uint32_t)dma_rx[index][0] << 16)
                 | ((uint32_t)dma_rx[index][1] << 8)
                 |  dma_rx[index][2];

    uint16_t raw_angle = ((uint16_t)(raw >> 10)) & 0x3FFF;
    uint8_t  status    = (uint8_t)((raw >> 4) & 0x0F);

    // 更新缓存（32-bit float 原子写入，ISR 安全）
    g_enc[index].raw_angle  = raw_angle;
    g_enc[index].mech_angle = (float)raw_angle * 6.283185307f / 16384.0f;
    g_enc[index].elec_angle = g_enc[index].mech_angle * 7.0f;  // 7 极对（硬件常数）
    g_enc[index].status     = status;
    g_enc[index].fresh      = 1;

    // 启动另一个编码器的 DMA（乒乓）
    MT6701_StartDMA(1 - index);
}
```

- [ ] **Step 4: 配置 SPI3 DMA**

在 CubeMX 生成代码后，确认 `Core/Src/spi.c` 中 SPI3 的 DMA 已正确配置：

```c
// 在 MX_SPI3_Init() 中应包含：
// HAL_SPI_RegisterCallback(&hspi3, HAL_SPI_TX_RX_COMPLETE_CB_ID, HAL_SPI_TxRxCpltCallback);
```

如果 CubeMX 未生成此回调注册，需在 `/* USER CODE BEGIN SPI3_Init 2 */` 区域手动添加。

- [ ] **Step 5: 在 `Core/Src/main.c` 中启动 DMA 乒乓**

在 `/* USER CODE BEGIN 2 */` 区域，`FOC_Init()` 之后：

```c
  // 启动编码器 DMA 乒乓读取（从编码器 0 开始）
  MT6701_StartDMA(0);
```

- [ ] **Step 6: 更新 CMakeLists.txt** — 添加 `encoder_cache.h`

```cmake
        Core/Inc/encoder_cache.h
```

- [ ] **Step 7: 编译验证**

```bash
cmake --build --preset Debug
```

预期：编译通过。如果有 `MOTOR_POLE_PAIRS` 或 `hspi3` 交叉引用问题，调整 include 顺序。

- [ ] **Step 8: 提交**

```bash
git add Core/Inc/encoder_cache.h Core/Inc/mt6701.h Core/Src/mt6701.c \
        Core/Src/spi.c Core/Src/main.c CMakeLists.txt
git commit -m "feat: MT6701 DMA 乒乓编码器读取 — SPI3 DMA 自动循环"
```

---

## 任务 7: INA240 改为 ISR 友好读取

**修改文件:**
- `Core/Inc/ina240.h` — 添加 ADC DMA 缓冲直接访问接口
- `Core/Src/ina240.c` — 去除累积滤波，改为直接 ADC→电流转换

- [ ] **Step 1: 修改 `Core/Inc/ina240.h`** — 添加 ISR 接口

在原有接口声明后添加：

```c
// ISR 安全：直接从 ADC DMA 环形缓冲区读原始值并转为电流
// ADC 由 TIM3 TRGO 触发，adc_buffer 由 DMA 自动更新
float INA240_GetCurrentFast(INA240_Channel_t channel);

// 获取 ADC 原始缓冲指针（FOC ISR 中直接索引）
extern volatile uint16_t adc_buffer[INA240_NUM_CHANNELS];
```

- [ ] **Step 2: 修改 `Core/Src/ina240.c`** — 简化 ADC 回调 + 添加快速读取

将 `adc_buffer` 改为非 static（供外部 ISR 直接引用）：

```c
// ADC2 DMA 缓冲区（extern 声明在 ina240.h）
volatile uint16_t adc_buffer[INA240_NUM_CHANNELS];
```

修改 `HAL_ADC_ConvCpltCallback`，去除累积滤波（10kHz FOC ISR 自己处理）：

```c
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc2)
    {
        // FOC ISR 在 ADC 中断中直接读取 adc_buffer
        // 此处不做任何处理，仅标记转换完成
        __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_EOC);
    }
}
```

添加快速读取函数：

```c
float INA240_GetCurrentFast(INA240_Channel_t channel)
{
    if (channel >= INA240_NUM_CHANNELS) return 0.0f;
    return adc_to_current(adc_buffer[channel], zero_offset[channel]);
}
```

- [ ] **Step 3: 编译验证**

```bash
cmake --build --preset Debug
```

- [ ] **Step 4: 提交**

```bash
git add Core/Inc/ina240.h Core/Src/ina240.c
git commit -m "feat: INA240 快速读取接口 — 去除累积滤波，ISR 直接访问 ADC DMA 缓冲"
```

---

## 任务 8: 电流闭环（层 2）— 核心 FOC ISR

**新建文件:**
- `Core/Inc/current_ctrl.h`
- `Core/Src/current_ctrl.c`

**修改文件:**
- `Core/Src/stm32g4xx_it.c` — ADC ISR 中实现 FOC
- `Core/Src/app_freertos.c` — 上层速度环任务预留
- `CMakeLists.txt`

- [ ] **Step 1: 创建 `Core/Inc/current_ctrl.h`**

```c
#ifndef CURRENT_CTRL_H
#define CURRENT_CTRL_H

#include "foc.h"

// 一次电流环控制（在 ADC EOC ISR 中调用）
void CurrentCtrl_Run(Motor_t *motor);

#endif
```

- [ ] **Step 2: 创建 `Core/Src/current_ctrl.c`**

```c
#include "current_ctrl.h"
#include "transform.h"
#include "svpwm.h"
#include "ina240.h"

void CurrentCtrl_Run(Motor_t *motor)
{
    // 1. 读电流（直接从 ADC DMA 缓冲）
    float Ia = INA240_GetCurrentFast(motor->ch_u);
    float Ib = INA240_GetCurrentFast(motor->ch_v);
    float Ic = -(Ia + Ib);

    // 保存调试变量
    motor->ia = Ia;
    motor->ib = Ib;
    motor->ic = Ic;

    // 2. Clarke + Park
    float I_alpha, I_beta;
    Clarke(Ia, Ib, Ic, &I_alpha, &I_beta);

    float I_d, I_q;
    Park(I_alpha, I_beta, motor->elec_angle, &I_d, &I_q);

    motor->id = I_d;
    motor->iq = I_q;

    // 3. PI 控制
    float V_d = PI_Step(&motor->id_pi, motor->id_ref - I_d, motor->dt);
    float V_q = PI_Step(&motor->iq_pi, motor->iq_ref - I_q, motor->dt);

    motor->vd = V_d;
    motor->vq = V_q;

    // 4. 逆 Park + SVPWM
    float V_alpha, V_beta;
    InvPark(V_d, V_q, motor->elec_angle, &V_alpha, &V_beta);

    SVPWM_SetVab(V_alpha, V_beta, motor);
}
```

- [ ] **Step 3: 修改 `Core/Src/stm32g4xx_it.c`** — 添加 ADC EOC FOC ISR

ADC 的中断由 `DMA1_Channel4_IRQHandler` 转发 `HAL_DMA_IRQHandler(&hdma_adc2)`，然后 HAL 回调 `HAL_ADC_ConvCpltCallback`。但我们需要在 EOC 中直接执行 FOC。

方案：在 `DMA1_Channel4_IRQHandler` 中（`HAL_DMA_IRQHandler` 之后）直接调用 FOC。

在 `/* USER CODE BEGIN Includes */` 添加：

```c
#include "foc.h"
#include "current_ctrl.h"
#include "encoder_cache.h"
```

在 `DMA1_Channel4_IRQHandler` 的 `/* USER CODE BEGIN DMA1_Channel4_IRQn 1 */` 区域：

```c
  // FOC 电流环：处理两个电机
  // 电角度从编码器 DMA 缓存读取（SPI ISR 维护）
  for (int i = 0; i < 2; i++)
  {
      g_motor[i].elec_angle = g_enc[i].elec_angle;
      if (g_motor[i].mode == MOTOR_MODE_CURRENT_LOOP)
      {
          CurrentCtrl_Run(&g_motor[i]);
      }
  }
```

- [ ] **Step 4: 修改 `Core/Src/app_freertos.c`** — 添加电流闭环模式任务

新增/修改为电流闭环任务。在 `/* USER CODE BEGIN Application */` 添加：

```c
void TaskCurrentLoop(void const *argument)
{
    (void)argument;
    Motor_t *motor = &g_motor[0];  // M1

    motor->mode = MOTOR_MODE_CURRENT_LOOP;
    motor->id_ref = 0.0f;
    motor->iq_ref = 0.1f;  // 0.1A Iq 启动（小电流试转）
    Motor_StartPWM(motor);

    // 此任务只做监控，实际 FOC 由 ADC ISR 驱动
    for (;;)
    {
        osDelay(100);
    }
}
```

在 `MX_FREERTOS_Init()` 的 `/* USER CODE BEGIN RTOS_THREADS */` 注册：

```c
  osThreadDef(currentLoopTask, TaskCurrentLoop, osPriorityNormal, 0, 128);
  osThreadCreate(osThread(currentLoopTask), NULL);
```

- [ ] **Step 5: 更新 CMakeLists.txt** — 添加 current_ctrl

```cmake
        Core/Inc/current_ctrl.h
        Core/Src/current_ctrl.c
```

- [ ] **Step 6: 编译验证**

```bash
cmake --build --preset Debug
```

- [ ] **Step 7: 提交**

```bash
git add Core/Inc/current_ctrl.h Core/Src/current_ctrl.c \
        Core/Src/stm32g4xx_it.c Core/Src/app_freertos.c CMakeLists.txt
git commit -m "feat: 层 2 — 电流闭环 FOC (Clarke/Park + PI + SVPWM) 在 ADC ISR 中 10kHz 执行"
```

---

## 任务 9: 中断优先级 + 编码器 DMA 启动

**修改文件:**
- `Core/Src/main.c` — NVIC 优先级覆盖 + DMA 乒乓启动

- [ ] **Step 1: 修改 `Core/Src/main.c`** — 添加 NVIC 优先级覆盖

在 `MX_FREERTOS_Init()` 之后、`osKernelStart()` 之前：

```c
  /* USER CODE BEGIN 2 */
  // ...
  MX_FREERTOS_Init();

  // 手动覆盖中断优先级（CubeMX 在 FreeRTOS 下限制 ≥5）
  NVIC_SetPriority(ADC1_2_IRQn, 4);      // FOC 电流环 10kHz，最高优先级
  NVIC_SetPriority(SPI3_IRQn, 6);        // 编码器 DMA 乒乓

  // SPI3 DMA 中断与 SPI3 全局中断共享同一向量（SPI3_IRQn）
  // DMA TC 中断通过 HAL_SPI_TxRxCpltCallback 回调处理

  /* Start scheduler */
  osKernelStart();
```

> 确认：在文件顶部 `/* USER CODE BEGIN Includes */` 已有 `#include "main.h"`（提供 NVIC 宏）。

- [ ] **Step 2: 确认 DMA 乒乓已启动**

任务 6 Step 5 中已在 `FOC_Init()` 后调用 `MT6701_StartDMA(0)`，确认此行存在。

- [ ] **Step 3: 确认 TIM3 主定时器已启动**

TIM3 的 PWM 需要启动才能产生 TRGO 触发 ADC。在 `Motor_StartPWM()` 中启动 TIM 通道时就启动了计数器。TIM4 从模式会自动跟随。

确认 `Motor_StartPWM()` 在电流环任务（`TaskCurrentLoop`）启动前调用，且 TIM3（M2 的定时器）也有通道启动才能产生 TRGO。

补充：如果 M2 不需要运行但需要 TIM3 TRGO，可在 `FOC_Init()` 或初始化阶段专门启动 TIM3 的任一 PWM 通道：

```c
  // 确保 TIM3 计数器运行（即使 M2 未用，TRGO 仍需要）
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  // 启动至少一个通道
```

将此行添加到 `Motor_Enable()` 或 `FOC_Init()` 中。

- [ ] **Step 4: 编译并烧录测试**

```bash
cmake --build --preset Debug
```

烧录后验证：
1. FreeRTOS 正常启动（蜂鸣器提示音）
2. 不触发 HardFault
3. 电流环在小 Iq 给定下电机能否保持转矩

- [ ] **Step 5: 提交**

```bash
git add Core/Src/main.c Core/Src/foc.c Core/Src/motor_hal.c
git commit -m "feat: 中断优先级手动覆盖 + TIM3 TRGO 确保启动 + 编码器乒乓启动"
```

---

## 任务 10: 验证与调试

- [ ] **Step 1: 层 0 验证（6 步换相）**

```bash
# 烧录
timeout 5 arm-none-eabi-gdb build/Debug/STM32G431Demo.elf -x tools/flash.gdb
```

| 检查项 | 方法 | 预期 |
|--------|------|------|
| 系统启动 | 听蜂鸣器 | 2000Hz 提示音 |
| M1 PWM 输出 | 示波器看 PB6/PB7/PB9 | 有 PWM 波形（方波切换） |
| 编码器角度 | 串口查看 reporterTask 数据 | 角度 0-360° 变化 |
| M1 转动 | 直接观察 | 电机转动（可能抖动） |

- [ ] **Step 2: 层 1 验证（电压正弦）**

| 检查项 | 方法 | 预期 |
|--------|------|------|
| M1 平滑转动 | 直接观察 | 比 6 步明显平滑 |
| 电流波形 | 串口数据 | 近似正弦 |
| 调速 | 修改 voltage_mag | 速度变化 |

- [ ] **Step 3: 层 2 验证（电流闭环）**

| 检查项 | 方法 | 预期 |
|--------|------|------|
| Id 趋零 | 串口读 `g_motor[0].id` | 接近 0 |
| Iq 跟踪 | 串口读 `g_motor[0].iq` | 接近 `iq_ref` |
| PI 参数调优 | 先 Kp 后 Ki | 阶跃响应无振荡 |

调试用 GDB 命令：

```bash
# 读取 Id/Iq（当前任务上下文可能不准确，参考值）
arm-none-eabi-gdb build/Debug/STM32G431Demo.elf \
  -ex "target remote localhost:61234" \
  -ex "monitor halt" \
  -ex "print g_motor[0].id" \
  -ex "print g_motor[0].iq" \
  -ex "print g_enc[0].elec_angle"
```

- [ ] **Step 4: 提交最终验证通过的代码**

```bash
git commit -m "chore: 层 0-2 FOC 验证通过，PI 参数初步整定"
```

---

## 任务 11: 清理与合并

- [ ] **Step 1: 最终编译确认**

```bash
cmake --build --preset Debug
```

确认 0 error，0 warning（或仅有非关键的 unused 警告）。

- [ ] **Step 2: 审查改动列表**

```bash
git diff master --stat
```

确认所有改动都是预期的。

- [ ] **Step 3: 合并回 master**

根据项目工作流选择合并方式（merge / rebase / PR）。建议在确认所有层验证通过后合并。

---

## 风险点

| 风险 | 缓解措施 |
|------|----------|
| CubeMX 重新生成覆盖手动代码 | 所有新增代码放在 USER CODE 保护区内 |
| TIM 同步不成功 | 先单独验证 TIM3 输出 10kHz PWM，再配 TIM4 从模式 |
| ADC 中断频率过高导致 HardFault | ISR 代码保持精简（<15μs），避免 float 除法过多 |
| SPI DMA 与 FOC ISR 竞争 g_enc | 32-bit float 原子读写，优先级 ADC(4) > SPI(6)，安全 |
| 电机参数不匹配导致 PI 难调 | 从小 Iq 起调（0.1A），逐步增大 |
