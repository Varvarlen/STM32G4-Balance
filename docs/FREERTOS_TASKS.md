# FreeRTOS 任务列表

## 正常模式

| 任务 | 函数 | 周期 | 栈 (words) | 优先级 | 核心操作 |
|------|------|:----:|:---------:|:------:|------|
| TaskCLI | StartCLITask | 1ms | 256 | Normal | 串口 R/L 前缀指令解析 + PI 参数 |
| TaskSpeedLoop | StartTaskSpeedLoop | 1ms | 512 | High | 速度 PI + 自适应窗口 RPM + 斜坡 (TIM17 触发) |
| TaskTelemetry | StartTaskTelemetry | 5ms | 320 | Low | 遥测帧上报 10 通道 (200Hz) |
| TaskIMU | StartTaskIMU | 10ms | 384 | Normal | SPI 读 MPU6500 + 卡尔曼滤波 |

### 遥测帧格式

```
频率: 100Hz (10ms)    帧尾: 0x7F800000 (+inf)
[f0] M1 speed_ref (RPM)   [f1] M1 speed_fb (RPM)   [f2] M1 iq_ref (A)
[f3] M1 iq (A)            [f4] M1 mech_angle (rad)
[f5] M2 speed_ref (RPM)   [f6] M2 speed_fb (RPM)   [f7] M2 iq_ref (A)
[f8] M2 iq (A)            [f9] M2 mech_angle (rad)
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
| `?` | 帮助+状态 | |

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
