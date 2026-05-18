# 平衡车设计规格

日期：2026-05-19 | 状态：已确认

## 1. 系统架构

### 任务布局（正常模式）

| 任务 | 频率 | 优先级 | 栈 | 核心操作 |
|------|:----:|:------:|:--:|------|
| TaskCLI | 1ms 轮询 | Normal | 256 | USART1/USART2 双串口命令解析、遥测开关、参数在线调优 |
| TaskBalanceLoop | 1kHz TIM17 触发 | High | 768 | IMU SPI 读取 → Kalman 倾角 → 平衡 PID → 速度环（双电机交替）→ iq_ref |

### 控制流

```
IMU SPI → Kalman倾角 → 平衡PID → 速度指令(RPM) → EKF速度估计 → 速度PI → iq_ref → 电流PI(ADC ISR 20kHz) → SVPWM
              ↑                          ↑
         角速度阻尼(Kd)           遥控速度偏置
```

### 平衡 PID 公式

```
tilt_err = target_angle - measured_angle    // 目标倾角通常 0°

balance_output = Kp_angle × tilt_err           // 角度比例
               + Kd_gyro   × gyro_rate         // 角速度阻尼（陀螺仪原始值）
               + target_speed                   // 遥控前向速度偏置
```

输出电压在 ±balance_output_max (RPM) 限幅后，经差速混合分配到左右轮 `speed_ref`。

### 差速转向

```
speed_ref_L = balance_output + steer_offset
speed_ref_R = balance_output - steer_offset
```

遥控转向指令 `T<deg/s>` 映射为 `steer_offset`。

## 2. 清理清单

### 删除

| 文件/代码 | 理由 |
|-----------|------|
| `Core/Src/six_step.c` / `Core/Inc/six_step.h` | 六步换相，已被 FOC 替代 |
| `Core/Src/debug_capture.c` / `Core/Inc/debug_capture.h` | 20kHz 电流阶跃采集，仅调试用 |
| `Core/Src/speed_capture.c` / `Core/Inc/speed_capture.h` | 1kHz 速度 burst 采集，仅调试用 |
| `foc.h` 中 `MOTOR_MODE_SIX_STEP` / `MOTOR_MODE_VOLTAGE_SINE` 枚举值 | 未使用 |
| `cli.c` 中 `CMD_Step()` / `CMD_Load()` | 阶跃/负载测试命令 |
| `cli.c` 中 `g_step_test` / `g_load_test` 状态机 | 阶跃/负载测试 |
| `cli.c` 中 `CMD_Position()` / `CMD_Current()` / `CMD_Speed()` | 旧指令格式 |
| `app_freertos.c` 中 `TaskDebugCapture` / `TaskTelemetry` / `StartTaskIMU` | 旧任务 |
| `app_freertos.c` 中 `g_test_mode` 启动分支 | 阶跃测试模式 |
| `app_freertos.c` 中遥测任务创建 | 迁移至平衡任务内 |
| `main.c` 中 3 秒倒计时启动选择器 | 简化启动 |
| `main.c` 中 `g_test_mode` | 阶跃模式标志 |

### 重命名

| 当前 | 改为 |
|------|------|
| `mpu6050.c/h` | `mpu6500.c/h` |

`kf_angle.c/h` 保留原名（卡尔曼倾角估计器，职责不变）。

### 重构

| 组件 | 当前 | 整理后 |
|------|------|--------|
| 启动流程 | 3 模式分支 | 统一初始化 → 校准仅 CLI `CAL` 触发 |
| CLI 指令 | 20+ 条 (R/L/RS/LS/RT/LT/RE/LE/RP/LP 前缀) | 精简 8 条 (S/T/STOP/PK/PS/PC/P/T/CAL/?) |
| 遥测 | 独立任务 200Hz 11ch | 平衡任务内按需 7ch |
| 平衡/速度环 | 分离任务 | 合并为 TaskBalanceLoop |

## 3. CLI 指令

### 运行指令

| 指令 | 功能 | 例 |
|------|------|-----|
| `S<RPM>` | 前进速度 | `S100` |
| `T<val>` | 转向（deg/s 或 RPM 差速） | `T30` |
| `STOP` | 紧急停止（电机中性化） | |

### 调参指令

| 指令 | 功能 | 例 |
|------|------|-----|
| `PK` | 查询平衡参数 | |
| `PK ANG=<val>` | 设置角度 Kp | `PK ANG=15` |
| `PK GYR=<val>` | 设置角速度 Kd | `PK GYR=2.0` |
| `PK SP=<val>` | 设置速度前馈 | `PK SP=1.5` |
| `PK MAX=<val>` | 设置平衡输出限幅 | `PK MAX=500` |
| `PK ANG0=<val>` | 设置目标倾角 (°) | `PK ANG0=0.5` |
| `PS P=X I=Y` | 设置速度 PI | `PS P=0.044 I=1.221` |
| `PC P=X I=Y` | 设置电流 PI | `PC P=15 I=5295` |
| `P` | 查询全部参数 | |
| `T` | 开关遥测 | |
| `CAL` | 进入校准模式 | |
| `?` | 帮助 | |

### 双串口

- **USART1**：调试口，全部指令接收，printf 输出
- **USART2**：蓝牙口，只接收运行指令 (S/T/STOP)，无 printf 输出
- 共用 CLI 解析核心，通过上下文区分输出目标

## 4. 遥测格式

平衡任务内按需发送，7 通道 float 帧：

```
[Tilt_angle(°)] [Gyro(°/s)] [Speed_ref(RPM)] [Speed_fb_L(RPM)] [Speed_fb_R(RPM)] [iq_L(A)] [iq_R(A)]
总长: 7×4 + 4 = 32 字节，200Hz 为 6.4KB/s
```

## 5. 保留的控制栈

| 层级 | 频率 | 说明 |
|------|:----:|------|
| FOC 电流环 | 20kHz | ADC ISR，id/iq PI + SVPWM |
| 编码器读取 | SPI DMA | MT6701，双编码器快照 |
| 速度环 | 1kHz | EKF 3-state + PI + 斜坡 |
| 校准模式 | 按需 | 5 步校准 (电流偏置/相线/编码器方向/零位/电机参数) |
| INA240 电流检测 | 20kHz | 双通道同步采样 |

## 6. 新增文件

| 文件 | 职责 |
|------|------|
| `Core/Inc/balance_ctrl.h` | `BalanceCtrl_t` 结构体 + 参数定义 |
| `Core/Src/balance_ctrl.c` | 平衡 PID 实现 + 差速混合 |
