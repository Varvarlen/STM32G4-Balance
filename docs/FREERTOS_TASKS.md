# FreeRTOS 任务列表

## 正常模式

| 任务 | 函数 | 周期 | 栈 (words) | 优先级 | 核心操作 |
|------|------|:----:|:---------:|:------:|------|
| TaskCLI | StartCLITask | 1ms | 192 | Normal | 串口 R/L/V/W 指令解析 |
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
| `E<值>` | M1 负载实验: 斜坡2s→蜂鸣器3s→恢复3s | `E50` |
| `F<值>` | M2 负载实验 | `F-100` |
| `V<值>` | 设置 M1 目标转速 (RPM)，进入速度模式 | `V500` |
| `W<值>` | 设置 M2 目标转速 (RPM)，进入速度模式 | `W-300` |
| `R<值>` | 设置 M1 iq_ref (限幅 ±2A)，切回电流模式 | `R0.3` |
| `L<值>` | 设置 M2 iq_ref，切回电流模式 | `L-0.5` |
| `R<值>L<值>` | 同时设置两电机电流模式 | `R0.2L0.3` |

V/W 与 R/L 互斥：V/W 进入速度模式，R/L 退出速度模式切回电流模式。
E/F 自动执行完整实验流程，期间蜂鸣器提示用户施加/解除负载。

## 阶跃测试模式

| 任务 | 函数 | 周期 | 栈 (words) | 优先级 | 核心操作 |
|------|------|:----:|:---------:|:------:|------|
| TaskCLI | StartCLITask | 1ms | 384 | Normal | 阶跃 CLI (SR/SL/r 命令) |
| debugCaptureTask | TaskDebugCapture | 100Hz | 512 | Normal | 轮询采集完成 + 500帧 id/iq 下传 |

阶跃测试模式下 TaskSpeedLoop/TaskTelemetry/TaskIMU 不创建。

## 校准模式

| 任务 | 函数 | 周期 | 栈 (words) | 优先级 | 核心操作 |
|------|------|:----:|:---------:|:------:|------|
| TaskCLI | StartCLITask | 1ms | 1024 | Normal | 校准 CLI (c1-5, d1-5, s, q) |

校准模式下仅创建 TaskCLI，其他任务不创建。printf 浮点格式化需大栈 (newlib ~800B)。

## 栈用量

| 模式 | TaskCLI | TaskSpeedLoop | TaskTelemetry | TaskIMU | debugCapture | 合计 |
|------|:------:|:------------:|:------------:|:------:|:------------:|:----:|
| 正常 | 192 | 512 | 320 | 384 | — | 1408 words (5.6KB) |
| 校准 | 1024 | — | — | — | — | 1024 words (4KB) |
| 阶跃测试 | 384 | — | — | — | 512 | 896 words (3.5KB) |

FreeRTOS 堆: 16384 字节 (heap_4)，正常模式堆使用约 7KB (43%)，剩余约 9KB。
校准模式动态分配 8KB (pvPortMalloc)，堆剩余约 4KB。
