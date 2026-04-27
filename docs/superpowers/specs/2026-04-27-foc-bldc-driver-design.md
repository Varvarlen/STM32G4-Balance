# FOC 无刷电机驱动设计

## 1. 背景与目标

基于当前 STM32G431 电机控制 Demo 项目，为两个无刷电机实现 FOC（磁场定向控制）驱动，为后续平衡车项目做准备。

### 硬件概况

| 项目 | 参数 |
|------|------|
| MCU | STM32G431CBU6 (Cortex-M4F, 170MHz) |
| 电机 | 无刷电机 ×2，极对数 7，KV=330，相电阻 6.5Ω |
| 供电 | 7.4V |
| 驱动芯片 | MP6536 ×2（三相半桥驱动，内置死区保护） |
| 电流采样 | INA240 ×4 通道 |
| 编码器 | MT6701 14-bit 磁编码器 ×2（SSI 接口，SPI3） |
| RTOS | FreeRTOS V10.3.1 |

### 硬件连接

详见 `docs/HARDWARE_CONNECTIONS.md`（待创建）。

## 2. 设计策略：逐层叠加

放弃一步到位的 FOC 实现，改为 4 层增量搭建。每层可独立验证，出问题时只可能是当前层或下一层的 bug。

| 层 | 名称 | 核心功能 | 验证标准 |
|:--:|------|----------|----------|
| 0 | 6 步换相 | 开环方波驱动，PWM+编码器+电流确认 | 电机能转动 |
| 1 | 电压模式正弦波 | SVPWM 开环正弦驱动 | 电机平滑转动，电流近似正弦 |
| 2 | 电流闭环 | Clarke/Park + Id/Iq PI 控制 | Iq 跟踪设定值，Id 趋零 |
| 3 | 速度/位置闭环 | 上层位置/速度 PI + 前馈 | 完整 FOC 性能 |

## 3. 文件结构

```
Core/
├── Inc/
│   ├── motor_hal.h       — 电机 HAL 抽象（PWM 设置/使能/占空比）
│   ├── foc.h             — FOC 主结构体（Motor 状态/参数/接口）
│   ├── svpwm.h           — SVPWM 计算
│   ├── transform.h       — Clarke/Park/InvPark 纯数学变换
│   ├── pi.h              — 通用 PI 控制器（带积分抗饱和）
│   ├── current_ctrl.h    — 电流闭环（层 2）
│   ├── velocity_ctrl.h   — 速度/位置闭环（层 3）
│   └── six_step.h        — 6 步换相（层 0）
├── Src/
│   ├── motor_hal.c
│   ├── foc.c
│   ├── svpwm.c
│   ├── transform.c
│   ├── pi.c
│   ├── current_ctrl.c
│   ├── velocity_ctrl.c
│   └── six_step.c
└── Src/app_freertos.c     — FOC 相关任务注册（USER CODE 保护区）
```

依赖方向严格单向：上层依赖下层，下层不感知上层。`transform` 和 `pi` 为纯计算模块，无硬件依赖。

## 4. 层 0：6 步换相

### 原理

根据编码器角度划分 6 个扇区，每个扇区导通一对上下桥臂。

### 接口

```c
void SixStep_Init(Motor_t *motor);
void SixStep_Run(Motor_t *motor);      // 读角度→查表→PWM 输出
```

### 验证项

- 示波器看 3 路 PWM 波形
- 串口看编码器角度 0-360° 随转动变化
- 电机能转动

## 5. 层 1：电压模式正弦波

### 原理

编码器角度查正弦表，开环给电压，无反馈控制。

```
angle → sin(θ)/sin(θ±120°) → 三相占空比
```

### 接口

```c
void SVPWM_Init(void);
void SVPWM_SetVab(float v_alpha, float v_beta, Motor_t *motor);
```

### 验证项

- 电机平稳旋转，调整 `voltage_magnitude` 可调速
- 电流波形近似正弦
- 比 6 步换相噪音明显降低

## 6. 层 2：电流闭环

### 原理

三相电流 → Clarke → Park → Id/Iq PI → InvPark → SVPWM。

### 数据流

```
INA240(2相) → Ia,Ib
              ↓  Ic = -(Ia+Ib)
           Clarke → Iα,Iβ
              ↓  encoder → electrical angle
            Park → Id,Iq
              ↓
     ┌── Id_ref=0 ──→ [PI] ──→ Vd ──┐
     │                                ├──→ InvPark → Vα,Vβ → SVPWM → PWM
     └── Iq_ref ────→ [PI] ──→ Vq ──┘
```

### 电流环频率：10kHz

10kHz（100μs 周期）无法用 FreeRTOS 任务调度（tick 1kHz），必须在中断中执行。

### 中断链路

```
TIM3 (主) + TIM4 (从) 同步中心对齐 PWM (10kHz)
  │
  ├── TIM3 Update Event (counter=0，两个电机 PWM 中心点重合)
  │     └── TIM3 TRGO ──→ ADC2 触发采样（4 通道，一次采集两电机电流）
  │                         │
  │                         └── ADC EOC ISR ──→ CurrentCtrl_Run(&motor1)
  │                                         ──→ CurrentCtrl_Run(&motor2)
  │
  └── 比较值更新（影子寄存器，下周期生效）
```

### PI 控制器结构体

```c
typedef struct {
    float kp, ki;
    float integral;
    float out_max, out_min;   // 积分限幅抗饱和
} PI_t;

float PI_Step(PI_t *pi, float error, float dt);
```

### 电流控制单步（在 ADC ISR 中执行）

```c
void CurrentCtrl_Run(Motor_t *motor) {
    float Ia = adc_current_buffer[motor->ch_u];     // → 安培
    float Ib = adc_current_buffer[motor->ch_v];
    float Ic = -(Ia + Ib);

    float I_alpha, I_beta;
    Clarke(Ia, Ib, Ic, &I_alpha, &I_beta);

    float I_d, I_q;
    Park(I_alpha, I_beta, motor->elec_angle, &I_d, &I_q);

    float V_d = PI_Step(&motor->id_pi, motor->id_ref - I_d, motor->dt);
    float V_q = PI_Step(&motor->iq_pi, motor->iq_ref - I_q, motor->dt);

    float V_alpha, V_beta;
    InvPark(V_d, V_q, motor->elec_angle, &V_alpha, &V_beta);
    SVPWM_SetVab(V_alpha, V_beta, motor);
}
```

### 验证项

- Id 趋近 0，Iq 跟踪设定值
- 阶跃响应无振荡不饱和
- 参数整定顺序：先 Kp（P 控制能转），再加小 Ki（消除静差）

## 7. 层 3：速度/位置闭环

速度环/位置环在 FreeRTOS 任务中运行（100Hz-1kHz），通过 `Motor_t.iq_ref` 与电流环 ISR 通信。

（详细设计留待层 2 完成后再展开。）

## 8. 编码器 DMA 乒乓读取

### 方案

SPI3 DMA 自动乒乓循环读取两个 MT6701 编码器。FOC ISR 只读内存缓存，不等待 SPI。

```
SPI3 DMA 乒乓:
  ┌──────────┐    DMA完成     ┌──────────┐
  │ 读编码器0 │ ──────────→ │ 读编码器1 │
  │ CS=PB4低  │ ←────────── │ CS=PA4低  │
  └──────────┘    DMA完成     └──────────┘
       ↓                           ↓
  更新 g_enc[0]             更新 g_enc[1]

FOC ISR (ADC EOC):
  elec_angle = g_enc[motor->id].elec_angle  ← 纯内存访问，零等待
```

### SPI 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| SCK 频率 | **8 MHz** | MT6701 极限 15.6 MHz，保守取值留余量 |
| 模式 | **Mode 1** (CPOL=0, CPHA=1) | SSI 要求 |
| 帧长 | 24-bit（3×8-bit） | — |
| 单编码器传输时间 | ~3μs | 24bit ÷ 8MHz |
| 两编码器乒乓周期 | ~12μs（含 CS 开销） | — |
| 缓存更新率 | ~80kHz | 远高于 10kHz FOC |

### SPI3 CubeMX 配置

| 路径 | 值 |
|------|-----|
| Parameter Settings → Clock → Prescaler | 使 SCK = 8 MHz |
| Parameter Settings → Data Size | 8 Bits |
| Parameter Settings → CPOL | Low |
| Parameter Settings → CPHA | 2 Edge |

### 角度缓存安全性

`g_enc[n].elec_angle` 是 32-bit float，Cortex-M4 单周期对齐读写天然原子。SPI ISR 写入，FOC ISR 读取，无临界区。

## 9. 中断优先级设计

STM32G4 NVIC 优先级分组 4（4-bit 全抢占），`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`。

### 完整优先级表

| 优先级 | 中断源 | 频率 | 耗时 | 职责 | 调用 OS API |
|:------:|--------|:----:|:----:|------|:-----------:|
| 0-3 | System Faults | — | — | HardFault/BusFault/UsageFault | — |
| **4** | **ADC1_2** | **10kHz** | ~10μs | FOC 电流环全集（读缓存→变换→PI→SVPWM） | **否** |
| — | `MAX_SYSCALL = 5` | — | — | FreeRTOS 临界区分界线 | — |
| **6** | SPI3 DMA TC | ~80kHz | <3μs | 乒乓：解析编码器帧→更新 `g_enc[]`→切 CS→启下次 DMA | **否** |
| 7 | TIM3/4 | 预留 | — | PWM 同步（当前不用） | — |
| 8-10 | USART1 DMA | 按需 | <3μs | 串口收发 | 是 |
| 15 | SysTick/PendSV | 1kHz | <2μs | FreeRTOS 心跳/任务切换 | — |

### 关键设计点

**ADC 优先级 4** 高于 FreeRTOS 临界区上限（5），确保 10kHz FOC 在任何情况下不被延迟。FOC ISR 不调用任何 FreeRTOS API，安全。

**CubeMX 限制**：启用 FreeRTOS 后 CubeMX 不允许设置 `<5` 的优先级。在 CubeMX 中暂时设为 5，然后在代码中手动覆盖：

```c
// 在 MX_FREERTOS_Init() 之后、osKernelStart() 之前
NVIC_SetPriority(ADC1_2_IRQn, 4);
```

STM32 电机控制项目中这是标准做法。

**SPI DMA 不调 API**：也设 <5，但优先级 6 低于 ADC 4，FOC 先跑，角度最多旧一个 DMA 周期（~12μs），对 10kHz 电流环无影响。

## 10. CubeMX 改动总览

以下改动需要在 CubeMX 中完成（提醒形式，不直接改代码），重新生成代码后适配。

### TIM3 / TIM4 同步

两台电机的 PWM 分别由 TIM4（M1）和 TIM3（M2）驱动。为确保电流采样对齐到各自 PWM 中心点，两个定时器必须同步运行：

- **TIM3 为主**（Master），输出 TRGO 触发 ADC2
- **TIM4 为从**（Slave），由 TIM3 的 TRGO 触发同步启动

CubeMX 配置：

| 定时器 | 路径 | 值 | 说明 |
|--------|------|-----|------|
| TIM3 | Parameter Settings → Counter Settings → Prescaler | 0 | 170MHz 不分频 |
| TIM3 | Parameter Settings → Counter Settings → AutoReload | 8499 | (170M / 10k / 2) - 1 |
| TIM3 | Parameter Settings → Counter Settings → Center-Aligned Mode | Center Aligned mode 1 | 中心对齐 |
| TIM3 | Parameter Settings → Trigger Output (TRGO) → Trigger Event | Update Event | 触发 ADC2 + 同步 TIM4 |
| TIM3 | Parameter Settings → PWM Generation CH2/3/4 | PWM mode 1 | M2 三相 PWM |
| TIM3 | Parameter Settings → PWM Generation → Pulse | 0 | 初始占空比 0 |
| TIM4 | Parameter Settings → Slave Mode → Slave Mode | Trigger Mode | 由 TIM3 TRGO 启动 |
| TIM4 | Parameter Settings → Slave Mode → Trigger Source | ITRx (TIM3) | 跟随 TIM3 |
| TIM4 | Parameter Settings → Counter Settings → Prescaler | 0 | 同 TIM3 |
| TIM4 | Parameter Settings → Counter Settings → AutoReload | 8499 | 同 TIM3 |
| TIM4 | Parameter Settings → Counter Settings → Center-Aligned Mode | Center Aligned mode 1 | 同 TIM3 |
| TIM4 | Parameter Settings → PWM Generation CH1/2/4 | PWM mode 1 | M1 三相 PWM |
| TIM4 | Parameter Settings → PWM Generation → Pulse | 0 | 初始占空比 0 |

两个计数器同步后，TIM3 的 Update Event 同时对应两个电机 PWM 的中心点，ADC2 一次采样所有 4 个电流通道。FOC ISR 中依次处理两个电机。

### ADC2

| 路径 | 值 | 说明 |
|------|-----|------|
| Parameter Settings → ADC_Settings → External Trigger Source | Timer 3 Trigger Out event | TIM3 TRGO 触发 |
| Parameter Settings → ADC_Settings → External Trigger → Trigger Detection | Rising edge | — |
| NVIC Settings → ADC1 and ADC2 interrupts | Enabled | — |
| NVIC Settings → ADC1 and ADC2 → Preemption Priority | 5 | CubeMX 中暂设 5，运行时覆盖为 4 |

### SPI3

| 路径 | 值 | 说明 |
|------|-----|------|
| Parameter Settings → Clock → Prescaler | SCK = 8 MHz | 保守值，极限 15.6 MHz |
| Parameter Settings → Data Size | 8 Bits | — |
| Parameter Settings → CPOL | Low | Mode 1 |
| Parameter Settings → CPHA | 2 Edge | Mode 1 |

## 11. 与 FreeRTOS 的协作

| 层级 | 运行位置 | 周期 | 通信方式 |
|------|----------|:----:|----------|
| 电流环 | ADC ISR（优先级 4） | 10kHz | 全局变量 `Motor_t` |
| 速度/位置环 | FreeRTOS 任务 | 100Hz-1kHz | 读写 `Motor_t.iq_ref` 设定值 |
| 数据上报 | `reporterTask`（50ms） | 20Hz | 读全局变量 → 串口 |
| 编码器更新 | SPI3 DMA ISR（优先级 6） | ~80kHz | 写 `g_enc[]` 全局缓存 |

所有 ISR ↔ FreeRTOS 通信均通过 32-bit 全局变量（Cortex-M4 原子读写），无需信号量/队列。

## 12. 实施顺序

1. 编写 `HARDWARE_CONNECTIONS.md` 硬件连接文档
2. CubeMX 重新配置 TIM3/4、ADC2、SPI3
3. 层 0：6 步换相 + 验证
4. 层 1：SVPWM 电压模式 + 验证
5. 层 2：电流闭环 + 编码器 DMA 乒乓 + 中断优先级 + 验证
6. 层 3：速度/位置闭环（设计待展开）
