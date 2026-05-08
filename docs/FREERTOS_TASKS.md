# FreeRTOS 任务列表

## 正常模式

| 任务 | 函数 | 周期 | 栈 (words) | 优先级 | 核心操作 |
|------|------|:----:|:---------:|:------:|------|
| defaultTask | StartDefaultTask | 1ms | 128 | Normal | 串口 R/L iq_ref 指令解析 |
| mpuTask | TaskMPU6500 | 10ms | 384 | Normal | SPI 读 MPU6500 + 卡尔曼滤波 |
| currentLoopTask | TaskCurrentLoop | 10ms | 384 | Normal | FOC 电流闭环（ISR 20kHz 驱动）+ 遥测帧上报 |

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

| 模式 | defaultTask | mpuTask | currentLoopTask | 合计 |
|------|:----------:|:------:|:--------------:|:----:|
| 正常 | 128 | 384 | 384 | 896 words (3.5KB) |
| 校准 | 1024 | — | — | 1024 words (4KB) |

FreeRTOS 堆: 8192 字节 (heap_4)，校准模式分配约 5.6KB，正常模式约 5.1KB。
