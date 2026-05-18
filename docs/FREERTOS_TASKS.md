# FreeRTOS 任务列表

## 正常模式

| 任务 | 函数 | 周期 | 栈 (words) | 优先级 | 核心操作 |
|------|------|:----:|:---------:|:------:|------|
| TaskCLI | StartCLITask | 1ms | 256 | Normal | 串口 R/L 前缀指令解析 + PI 参数 |
| TaskSpeedLoop | StartTaskSpeedLoop | 1ms | 512 | High | EKF + 速度 PI + 位置 P (级联) + 斜坡 (TIM17 触发) |
| TaskTelemetry | StartTaskTelemetry | 5ms | 320 | Low | 遥测帧上报 11 通道 (200Hz) |
| TaskIMU | StartTaskIMU | 10ms | 384 | Normal | SPI 读 MPU6500 + 卡尔曼滤波 |

### 遥测帧格式

```
频率: 200Hz (5ms)    帧尾: 0x7F800000 (+inf)
位置模式: f0=pos_ref, 速度模式: f0=speed_ref_ramp
[f0] M1 ref (rad或RPM)    [f1] M1 pos_est (rad)      [f2] M1 speed_fb (RPM)
[f3] M1 iq (A)            [f4] M1 speed_ref (RPM)
[f5] M2 ref (rad或RPM)    [f6] M2 pos_est (rad)      [f7] M2 speed_fb (RPM)
[f8] M2 iq (A)            [f9] M2 speed_ref (RPM)
[f10] M1 编码器机械角 (rad, 方向校正)
```

### 指令

| 指令 | 功能 | 例 |
|------|------|-----|
| `RS<RPM>` | M1 进入速度模式 [ramp] | `RS500` |
| `LS<RPM>` | M2 进入速度模式 | `LS-300` |
| `R<A>` | M1 电流模式 iq_ref=A | `R0.3` |
| `L<A>` | M2 电流模式 | `L-0.5` |
| `R<A> L<B>` | 两电机堆叠 | `R0.2 L0.3` |
| `RT<RPM>` | M1 阶跃 当前→RPM [no ramp] [burst] | `RT100` |
| `RT<A> <B>` | M1 阶跃 A→B | `RT50 100` |
| `LT<RPM>` | M2 阶跃 | `LT200` |
| `LT<A> <B>` | M2 阶跃 | `LT-50 50` |
| `RE<RPM>` | M1 负载实验 | `RE50` |
| `LE<RPM>` | M2 负载实验 | `LE-100` |
| `PRS` / `PLS` | 查询速度 PI | |
| `PRS P=X I=Y` | 设置速度 PI | |
| `PRC P=X I=Y` | 设置电流 PI | |
| `RP<deg>` | M1 位置模式 (累计角度) | `RP1000` |
| `LP<deg>` | M2 位置模式 | `LP-500` |
| `PRP` / `PLP` / `PP` | 查询/设置位置环 Kp | `PP P=210` |
| `T` | 开关遥测输出 | |
| `?` | 帮助+状态 | |

位置模式: P 控制器 + EMA (τ≈5ms) → 级联速度 PI, 速度环斜坡 (20000 RPM/s) 接管平滑。反馈为编码器增量展开位置 (meas_cont)，非 EKF 估计值。速度/阶跃/负载命令自动退出位置模式。
RS/LS 与 R/L 互斥：RS/LS 进入速度模式，R/L 退出速度模式切回电流模式。RE/LE 自动执行完整实验流程。

## 阶跃测试模式

| 任务 | 函数 | 周期 | 栈 (words) | 优先级 | 核心操作 |
|------|------|:----:|:---------:|:------:|------|
| TaskCLI | StartCLITask | 1ms | 384 | Normal | 阶跃 CLI (R/L/r 命令) |
| debugCaptureTask | TaskDebugCapture | 100Hz | 512 | Normal | 轮询采集完成 + 500帧 id/iq 下传 |

阶跃测试模式下 TaskSpeedLoop/TaskTelemetry/TaskIMU 不创建。

## 校准模式

| 任务 | 函数 | 周期 | 栈 (words) | 优先级 | 核心操作 |
|------|------|:----:|:---------:|:------:|------|
| TaskCLI | StartCLITask | 1ms | 1024 | Normal | 校准 CLI (R1-5, L1-5, RS, q) |

校准模式下仅创建 TaskCLI，其他任务不创建。printf 浮点格式化需大栈 (newlib ~800B)。

## 栈用量

| 模式 | TaskCLI | TaskSpeedLoop | TaskTelemetry | TaskIMU | debugCapture | 合计 |
|------|:------:|:------------:|:------------:|:------:|:------------:|:----:|
| 正常 | 256 | 512 | 320 | 384 | — | 1472 words (5.9KB) |
| 校准 | 1024 | — | — | — | — | 1024 words (4KB) |
| 阶跃测试 | 384 | — | — | — | 512 | 896 words (3.5KB) |

FreeRTOS 堆: 16384 字节 (heap_4)，正常模式堆使用约 7KB (43%)，剩余约 9KB。
校准模式动态分配 8KB (pvPortMalloc)，堆剩余约 4KB。
