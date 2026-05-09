# 电流环阶跃测试调试指南

## 概述

调试分支 `worktree-feat+current-loop-debug`，基于 master `da76c9a`。
目标：通过阶跃响应测试优化 FOC 电流环 PI 参数。

## 启动模式

上电后 3 秒倒计时内按键选择：

| 按键 | 模式 | 说明 |
|------|------|------|
| `s` | **阶跃测试模式** | CLI 发 SR/SL 命令触发阶跃采集 |
| `c` | 校准模式 | 5 项校准实验，结果存 Flash |
| 超时 | 正常 FOC 模式 | 遥测帧 + R/L 指令控制 iq_ref |

## 阶跃测试命令

进入阶跃测试模式后：

| 命令 | 功能 | 例 |
|------|------|-----|
| `SR<值>` | M1 阶跃测试 | `SR0.2` — 0.2A 阶跃 |
| `SL<值>` | M2 阶跃测试 | `SL-0.5` — -0.5A 阶跃 |
| `r` | 重发上次采集数据 | 纯二进制帧，无 printf |

## 数据格式

每帧 3 个 float（小端 IEEE 754）+ 帧尾 `0x7F800000`（+inf）：

```
[f0] id (A)    [f1] iq (A)    [f2] iq_ref (A)
```

- 帧尾：4 字节 `00 00 80 7F`
- 帧大小：3×4 + 4 = 16 字节
- 波特率：230400
- 总共 500 帧（50ms @ 10kHz）

采集时序：
- 前 50 点 (0~4.9ms)：基线，iq_ref = 0
- 第 50 点起施加阶跃，iq_ref = step
- 后 450 点 (5~50ms)：阶跃响应

## ISR 架构

```
DMA1_Channel4_IRQHandler (10kHz):
  for each motor:
    DebugCapture_PreCtrl   ← 在指定样本点施加阶跃
    if mode == CURRENT_LOOP:
      CurrentCtrl_Run      ← Clarke → Park → PI → SVPWM
    DebugCapture_PostCtrl  ← 采集 id/iq 写入缓冲
```

## PI 参数调优经验

### 最终固化的参数

| 参数 | 值 | 备注 |
|------|-----|------|
| Kp | 12 V/A | |
| Ki | 2400 V/(A·s) | 零点 = Ki/Kp = 200 rad/s ≈ 32 Hz |
| 输出限幅 | ±7.4V | FOC_VBUS |
| 控制周期 | 100µs | 10kHz |
| EMA_SHIFT | 3 | τ ≈ 0.8ms |

### 0.2A 阶跃性能指标

| 指标 | 值 |
|------|-----|
| 上升时间 | 0.9ms |
| 收敛时间 | 6ms |
| 过冲 | <4% |
| 震荡次数 | 1 |

### 调参方法论

1. **零点锁定**：Ki/Kp = 200 (32Hz)，匹配电机 RL 极点实现零极点对消
2. **同比缩放**：调快就 Kp/Ki 同比例放大，不建议单独改 Ki
3. **EMA 先行**：ADC 滤波器时间常数是相位滞后的主因，EMA_SHIFT 从 4→3 后带宽上限翻倍
4. **抗饱和**：条件积分（clamping anti-windup）在输出饱和时冻结积分项

### 调参历程

| 迭代 | Kp | Ki | EMA | 上升 | 收敛 | 过冲 | 备注 |
|------|-----|------|------|------|------|------|------|
| 默认 | 0.5 | 20 | 4 | ~50ms | — | — | 原始参数，响应极慢 |
| 1 | 12 | 2400 | 4 | 1.3ms | 10.5ms | 3% | 零点锁定，但 EMA 拖慢 |
| 2 | 12 | 4800 | 4 | 1.2ms | 14.9ms | 7% | Ki 单独翻倍→过冲恶化 |
| 3 | 14 | 2800 | 4 | 0.7ms | 5.5ms | 5.4% | 同比放大，过冲偏高 |
| **最终** | **12** | **2400** | **3** | **0.9ms** | **6ms** | **4%** | EMA 提速后回到原始 Kp/Ki |

核心发现：**EMA 滤波器是真正的瓶颈**。不改 PI 参数，仅把 EMA 时间常数减半，上升和收敛时间就缩短了 30-40%。

## 代码结构

| 文件 | 职责 |
|------|------|
| `debug_capture.c/h` | 阶跃采集核心：缓冲、ISR 钩子、数据下传 |
| `app_freertos.c` | 测试模式 CLI（SR/SL/r）、TaskDebugCapture |
| `main.c` | g_test_mode 启动分支 |
| `stm32g4xx_it.c` | DMA ISR 中 PreCtrl/PostCtrl 调用 |
| `motor_hal.c/h` | Motor_Neutralize — 单电机失能 |
| `pi.c` | 条件积分抗饱和 |
| `foc.c` | Kp=12, Ki=2400 固化 |
| `ina240.c` | EMA_SHIFT 3 |

## 已知陷阱

1. **全片擦除后校准丢失**：`c1~c5` 和 `d1~d5` 需全重做，Flash 保存的 enc_direction/phase_comp/zero_offset 都会被擦除
2. **Flash 写保护（WRP/PCROP）**：擦除失败时检查 Option Bytes，`STRT > END` 表示解除保护
3. **Flash 控制器锁死**：异常断电后可能锁死，拔电等 10 秒放电后可恢复
4. **FreeRTOS 堆 OOM**：校准缓冲 8KB（`CALIB_OFFSET_SAMPLES=1000 × 4ch × 2B`）和采集缓冲 4KB 必须放在 BSS（static），不能 pvPortMalloc
5. **printf + DMA RX 冲突**：`__io_putchar` 不能用阻塞 TX，必须走 DMA TX（`COMM_SendByte`）
6. **MP6536 门驱浮空**：`Motor_Enable()` 前必须先把 PWM 设为 50% 中性值

## PC 端数据分析

Python 示例：
```python
import struct, serial

ser = serial.Serial('COMx', 230400)
frames = []
while len(frames) < 500:
    data = ser.read(16)  # 3 floats + footer
    id_, iq, iq_ref = struct.unpack('<3f', data[:12])
    frames.append((id_, iq, iq_ref))
```
