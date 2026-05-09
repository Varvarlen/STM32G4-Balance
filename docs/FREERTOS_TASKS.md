# FreeRTOS 任务列表

## 正常模式

| 任务 | 函数 | 周期 | 栈 (words) | 优先级 | 核心操作 |
|------|------|:----:|:---------:|:------:|------|
| defaultTask | StartDefaultTask | 1ms | 128 | Normal | 串口 R/L iq_ref 指令解析 |
| mpuTask | TaskMPU6500 | 10ms | 384 | Normal | SPI 读 MPU6500 + 卡尔曼滤波 |
| currentLoopTask | TaskCurrentLoop | 10ms | 384 | Normal | FOC 电流闭环（ISR 10kHz 驱动）+ 遥测帧上报 |

### 遥测帧格式

```
频率: 100Hz (10ms)    帧尾: 0x7F800000 (+inf)
[f0] M1 id (A)    [f1] M1 iq (A)    [f2] M1 vd (V)    [f3] M1 vq (V)
[f4] M2 id (A)    [f5] M2 iq (A)    [f6] M2 vd (V)    [f7] M2 vq (V)
[f8] M1 RPM       [f9] M2 RPM
```

### iq_ref 指令

正常模式下通过串口发送 (230400bps)：

| 指令 | 功能 | 例 |
|------|------|-----|
| `R<值>` | 设置 M1 iq_ref (限幅 ±2A) | `R0.3` |
| `L<值>` | 设置 M2 iq_ref | `L-0.5` |
| `R<值>L<值>` | 同时设置两电机 | `R0.2L0.3` |

## 阶跃测试模式

| 任务 | 函数 | 周期 | 栈 (words) | 优先级 | 核心操作 |
|------|------|:----:|:---------:|:------:|------|
| defaultTask | StartDefaultTask | 1ms | 384 | Normal | 阶跃 CLI (SR/SL 命令) |
| debugCaptureTask | TaskDebugCapture | 100Hz | 512 | Normal | 轮询采集完成 + 500帧 id/iq 下传 |

阶跃测试模式下 `mpuTask` 和 `currentLoopTask` 不创建，无遥测帧输出。

测试命令：`SR<值>` = M1 阶跃，`SL<值>` = M2 阶跃，`r` = 重发上次数据，例 `SR0.5`、`SL-0.3`。

ISR 以 10kHz 采集 id/iq，前 50 点 (5ms) 为基线，之后自动施加阶跃，采集 500 点 (50ms) 后停止。下传格式：每帧 3 floats [id, iq, iq_ref] + 帧尾 +inf。iq_ref 在发送时实时计算（前 50 点 = 0，之后 = step），BSS 仅存 id/iq（4000 字节）。

## 校准模式

| 任务 | 函数 | 周期 | 栈 (words) | 优先级 | 核心操作 |
|------|------|:----:|:---------:|:------:|------|
| defaultTask | StartDefaultTask | 1ms | 1024 | Normal | 校准 CLI (c1-5, d1-5, s, q) |

校准模式下 `mpuTask` 和 `currentLoopTask` 不创建，串口无遥测帧输出。

## 历史任务（已注释保留）

| 任务 | 函数 | 说明 |
|------|------|------|
| voltageSineTask | TaskVoltageSine | SVPWM 开环正弦驱动 |
| speedReportTask | TaskSpeedReport | 编码器测速上报 |
| sixStepTask | TaskSixStep | 六步换相驱动 |

## 编码器方向

| 参数 | M1 | M2 | 说明 |
|------|:--:|:--:|------|
| enc_direction | -1 | +1 | 编码器 DMA ISR 中直接处理 |
| 电角度计算 | `mech × 7 × enc_dir` | 同上 | 电流闭环不依赖 `Motor_t.direction` |

`Motor_t.direction` 仅用于开环模式（TaskVoltageSine/TaskSixStep）。

## 栈用量

| 模式 | defaultTask | mpuTask | currentLoopTask | debugTask | 合计 |
|------|:----------:|:------:|:--------------:|:---------:|:----:|
| 正常 | 128 | 384 | 384 | — | 896 words (3.5KB) |
| 校准 | 1024 | — | — | — | 1024 words (4KB) |
| 阶跃测试 | 384 | — | — | 512 | 896 words (3.5KB) |

FreeRTOS 堆: 8192 字节 (heap_4)，采集缓冲 4KB 通过 pvPortMalloc 动态分配。
校准零漂缓冲也改为动态分配 (8KB @ c1)，正常/阶跃测试模式下不占用堆内存。
