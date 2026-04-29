# FreeRTOS 任务列表

| 任务 | 函数 | 周期 | 栈 (words) | 优先级 | 核心操作 |
|------|------|:----:|:---------:|:------:|------|
| defaultTask | StartDefaultTask | 1ms | 128 | Normal | 串口回显 + 't' 触发高速上报 |
| voltageSineTask | TaskVoltageSine | 1ms | 384 | Normal | SVPWM 开环正弦双电机驱动 M1+M2（已注释保留） |
| speedReportTask | TaskSpeedReport | 10ms | 256 | Normal | 读编码器 → RPM+电压浮点帧上报 |
| mpuTask | TaskMPU6500 | 10ms | 384 | Normal | SPI 读 MPU6500 + 卡尔曼滤波 |
| sixStepTask | TaskSixStep | 1ms | 384 | Normal | 六步换相驱动 M1+M2（已注释保留） |
| currentLoopTask | TaskCurrentLoop | 100ms | 384 | Normal | FOC 电流闭环 — Iq=0.1A 起步（当前启用，FOC 由 ADC ISR 10kHz 驱动） |

---

## 任务参数速查（当前）

| 任务 | 周期 | 栈(words) | 优先级 | 核心操作 |
|------|:----:|:---------:|:------:|------|
| defaultTask | 1ms | 128 | Normal | 串口回显 + 't' 触发高速上报 |
| voltageSineTask | 1ms | 384 | Normal | SVPWM 开环正弦双电机驱动 M1+M2（已注释保留） |
| speedReportTask | 10ms | 256 | Normal | 读编码器 → RPM+电压浮点帧上报 |
| mpuTask | 10ms | 384 | Normal | SPI 读 MPU6500 + 卡尔曼滤波 |
| sixStepTask | 1ms | 384 | Normal | 六步换相驱动 M1+M2（已注释保留） |
| currentLoopTask | 100ms | 384 | Normal | FOC 电流闭环 — Iq=0.1A 起步（当前启用，FOC 由 ADC ISR 10kHz 驱动） |

---

## FOC 调试经验

### 编码器映射 — 耦合根因

**硬件文档和实际接线不一致**。文档说 M1→PB4, M2→PA4，但实际物理接线相反。

| 电机 | 编码器 CS | MT6701 index | motor_id |
|------|-----------|:------------:|:--------:|
| M1 (TIM4) | PB4 | **0** | 0 |
| M2 (TIM3) | PA4 | **1** | 1 |

**错误映射的症状**：两个电机转速完全同步，用手转动一个电机会带动另一个。因为每个电机读的是对方编码器的角度，形成交叉反馈。

**验证方法**：停掉一个电机的 PWM，手动转动另一个电机轴，观察串口上报的编码器角度变化来确定对应关系。

### 编码器方向补偿

两个 MT6701 编码器物理安装方向均与电机正转方向相反，且 M1/M2 编码器彼此反向安装。

- `direction` 字段用于 SVPWM 电角度补偿：`elec_rad = mech_rad * 7 * direction`
- 两电机均需 `direction = -1`
- 测速时 M2 需额外符号翻转（编码器安装方向与 M1 相反）

### SVPWM 旋转方向效率

正反转效率不对称是编码器方向导致的——电压矢量旋转方向需与编码器递增方向一致才高效。

**高效方向**（已验证）：
```c
v_alpha = (vm * rev) * sin(elec_rad);
v_beta  = -(vm * rev) * cos(elec_rad);
// rev=+1 → 正转, rev=-1 → 反转
// direction=-1 已在 elec_rad 中补偿
```

**性能对比**（0.5V，两电机）：

| 方向 | 转速 | 总线电流 | 效率 |
|------|:----:|:--------:|:----:|
| 正转（高效） | ~1600 RPM | 0.38A | ✓ |
| 反转（低效） | ~638 RPM | ~1A | ✗ |

如需反转也高效，对调电机三相线中的任意两相。

### INA240 采样校准

**型号确认**：芯片丝印 INA240**A2**，增益 50V/V（非代码初始假设的 A1/20V/V）。读数偏差 2.5 倍。

**校准必须在零电流下进行**：
- 正确顺序：启动 TIM3（产生 ADC 触发）→ **保持 PC14 LOW（MP6536 禁能）**→ INA240_Calibrate() → 再拉高 PC14 使能电机
- 错误做法：先 Motor_Enable() 再校准，此时 PWM 引脚默认低电平导致 MP6536 下桥导通，有电流流过

**EMA 滤波**：在 `HAL_ADC_ConvCpltCallback`（10kHz）中执行 `filtered += (adc - filtered) >> 4`，等效约 31 倍过采样。

**参数速查**：

| 参数 | 值 |
|------|-----|
| 采样电阻 | 20 mΩ |
| INA240 增益 | 50 V/V (A2) |
| 电流分辨率 | ~0.8 mA/LSB |
| 测量范围 | ±1.65 A |
| EMA 后有效精度 | ~0.14 mA |

### CubeMX 生成代码的已知陷阱

全量重新生成代码会覆盖以下手动修改，**必须在 CubeMX 中同步配置**：

| 文件 | 问题 | CubeMX 修正路径 |
|------|------|----------------|
| `tim.c` | TIM4 从触发 ITR3 应为 ITR2 | Timers→TIM4→Slave Mode→Trigger Source→**ITR2** |
| `FreeRTOSConfig.h` | `configENABLE_FPU` 应为 1 | Middleware→FREERTOS→Config parameters→ENABLE_FPU |
| `usart.c` | USART1 RX DMA 应为 CIRCULAR | Connectivity→USART1→DMA Settings→RX→Mode→**Circular** |
| `main.c` | NVIC 优先级覆盖不在保护区 | 需在 USER CODE 中手动添加 |

### 14-bit 编码器测速要点

- 编码器范围 0-16383（14-bit），wrap 发生在 16384 而非 65536
- `int16_t` 差值法需要手动处理 14-bit 边界：`if (diff > 8192) diff -= 16384`
- 采样周期 10ms 时最大可测转速 ≈ 3000 RPM（8192/16384/0.01*60）
- 如果测量值在 ±180 之间跳变，说明采样周期太长或 wrap 处理有误

### 串口高速上报带宽

| 波特率 | 有效字节率 | 16B/帧上限 |
|--------|:---------:|:--------:|
| 115200 | 11520 B/s | 720 Hz |
| 230400 | 23040 B/s | 1440 Hz |

当前使用 230400，1000Hz 上报（3 通道电流）占 16KB/s，远在带宽内。
