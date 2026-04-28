# 联合测试调试笔记

## 1. 分支隔离

- 测试工作必须在独立分支上进行，保留 master 的可工作基线
- `git stash` 不可靠（未跟踪文件会被留下），用 `git reset --hard <commit>` + `git clean -fd` 彻底回退

## 2. 增量测试策略

从简到繁，每一步确认后再叠加：

| 步骤 | 内容 | 关键验证 |
|:----:|------|------|
| 1 | `HAL_UART_Transmit` 打印字符串 | UART TX 硬件通路 |
| 2 | `COMM_SendByte` 回显 | 环形缓冲区 + DMA TX + DMA RX |
| 3 | `COMM_SendData` 批量发送 | 批量 DMA TX 路径 |
| 4 | 电流传感器 INA240 | ADC DMA + 任务调度 |
| 5 | 编码器 MT6701 | SPI 读取 + 多任务共存 |
| 6 | IMU MPU6500 | SPI + 卡尔曼滤波 + 三任务 |
| 7 | COMM_SendFloatFrame 协议 | 二进制帧打包/解析 |

## 3. GDB 烧录流程

**错误的做法**（MCU 不会运行）：
```
monitor reset → quit
```

**正确的做法**（MCU 复位后继续运行）：
```
monitor halt → load → monitor reset → continue → (kill gdb)
```

使用脚本 `tools/flash.gdb` + `timeout` 命令：
```bash
timeout 5 arm-none-eabi-gdb build/Debug/STM32G431Demo.elf -x tools/flash.gdb
```

## 4. 栈溢出（HardFault）排查

### 典型症状
- FreeRTOS 启动后短时间内 HardFault
- SysTick_Handler → xTaskIncrementTick → vListInsertEnd 崩溃
- CFSR = 0x8200（PRECISERR + BFARVALID），BFAR 指向非法地址

### 定位方法
```
arm-none-eabi-gdb -batch -ex "target remote localhost:61234" -ex "monitor halt" \
  build/Debug/STM32G431Demo.elf -ex "bt" -ex "print/x SCB->CFSR" -ex "quit"
```

### 栈估算
- `char buf[128]` + `float arr[4]` + `snprintf` 调用栈 ≈ 350+ 字节
- 默认 `osThreadDef(task, func, prio, 0, 64)` = 64 words = 256 字节 → **不够**
- 有格式化输出的任务栈至少设为 **256 words (1024 字节)**

## 5. FreeRTOS 配置陷阱

### INCLUDE_uxTaskGetStackHighWaterMark
- **不能单独启用**，必须同时启用 `configCHECK_FOR_STACK_OVERFLOW > 0` 或 `configUSE_TRACE_FACILITY = 1`
- 单独启用会导致 `xTaskIncrementTick` 中访问未初始化的链表结构，触发 HardFault

### 堆大小
- 6 个任务（含 Idle）约需 7KB 堆（栈 + TCB）
- 默认 `configTOTAL_HEAP_SIZE = 6144` 不够
- 建议 **8192** 或更大

## 6. 全局变量通信（单写者单读者）

```c
// 传感器任务写入（唯一的写入者）
static float g_currents[4];    // adcTask 写入

// 上报任务读取（唯一的读取者）
float frame[10];
frame[3] = g_currents[0];      // reporterTask 读取
```

- Cortex-M4 32-bit 对齐读写天然原子，无需临界区
- 传感器任务周期必须 ≥ 上报任务周期（避免丢数据）
- 本方案：传感器 5ms ≥ 上报 50ms ✓

## 7. MPU6500 阻塞问题

MPU6500 的 SPI 读写使用 `HAL_SPI_TransmitReceive(..., HAL_MAX_DELAY)`，若芯片未连接：
- 在 FreeRTOS 启动前调用 → **系统永远无法启动**（无蜂鸣器声、无串口数据）
- 在 FreeRTOS 任务中调用 → 仅该任务阻塞，其他任务正常运行

排除故障时先注释掉 `MPU6050_Init()` 调用确认不阻塞。

## 8. COMM 协议注意点

### COMM_SendFloatFrame
```c
void COMM_SendFloatFrame(const float *data, uint8_t count);
// count = 数据通道数（不含帧尾）
// 最大: PROTOCOL_MAX_CHANNELS = 10
// 帧尾自动附加: 0x7F800000 (+inf)
```

### 帧解析（Python）
```python
FOOTER = b'\x00\x00\x80\x7F'
idx = buf.find(FOOTER)       # 查找帧尾
floats = struct.unpack(f'<{n}f', buf[:idx])
```

## 9. 单帧通道分配（10 通道上限）

```
[1.0, angle0, angle1, M1_U, M1_W, M2_U, M2_W, kf_angle, gyro_x, gyro_z]
 类型   编码器×2      电流×4                  角度     角速度X   角速度Z
```

取舍理由：
- 编码器 status（磁场强弱）不频繁变化，可牺牲
- 加速度计 3 轴数据量太大，保留 KF 融合角即可
- 陀螺仪 X 用于平衡 D 项，Z 用于转向，Y 基本不用可牺牲

## 10. 任务参数速查（当前）

| 任务 | 周期 | 栈(words) | 优先级 | 核心操作 |
|------|:----:|:---------:|:------:|------|
| defaultTask | 1ms | 128 | Normal | 串口回显 + 't' 触发高速上报 |
| voltageSineTask | 1ms | 384 | Normal | SVPWM 开环正弦驱动 M1+M2 |
| speedReportTask | 10ms | 256 | Normal | 读编码器 → RPM+电压浮点帧上报 |
| mpuTask | 10ms | 384 | Normal | SPI 读 MPU6500 + KF |
| sixStepTask | 1ms | 384 | — | 六步换相（已注释，层0测试用） |
| currentLoopTask | 100ms | 384 | — | FOC 电流闭环（已注释，层2待启用） |

---

## 11. FOC 调试经验

### 11.1 编码器映射 — 耦合根因

**硬件文档和实际接线不一致**。文档说 M1→PB4, M2→PA4，但实际物理接线相反。

| 电机 | 编码器 CS | MT6701 index | motor_id |
|------|-----------|:------------:|:--------:|
| M1 (TIM4) | PB4 | **0** | 0 |
| M2 (TIM3) | PA4 | **1** | 1 |

**错误映射的症状**：两个电机转速完全同步，用手转动一个电机会带动另一个。因为每个电机读的是对方编码器的角度，形成交叉反馈。

**验证方法**：停掉一个电机的 PWM，手动转动另一个电机轴，观察串口上报的编码器角度变化来确定对应关系。

### 11.2 编码器方向补偿

两个 MT6701 编码器物理安装方向均与电机正转方向相反，且 M1/M2 编码器彼此反向安装。

- `direction` 字段用于 SVPWM 电角度补偿：`elec_rad = mech_rad * 7 * direction`
- 两电机均需 `direction = -1`
- 测速时 M2 需额外符号翻转（编码器安装方向与 M1 相反）

### 11.3 SVPWM 旋转方向效率

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

### 11.4 INA240 采样校准

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

### 11.5 CubeMX 生成代码的已知陷阱

全量重新生成代码会覆盖以下手动修改，**必须在 CubeMX 中同步配置**：

| 文件 | 问题 | CubeMX 修正路径 |
|------|------|----------------|
| `tim.c` | TIM4 从触发 ITR3 应为 ITR2 | Timers→TIM4→Slave Mode→Trigger Source→**ITR2** |
| `FreeRTOSConfig.h` | `configENABLE_FPU` 应为 1 | Middleware→FREERTOS→Config parameters→ENABLE_FPU |
| `usart.c` | USART1 RX DMA 应为 CIRCULAR | Connectivity→USART1→DMA Settings→RX→Mode→**Circular** |
| `main.c` | NVIC 优先级覆盖不在保护区 | 需在 USER CODE 中手动添加 |

### 11.6 14-bit 编码器测速要点

- 编码器范围 0-16383（14-bit），wrap 发生在 16384 而非 65536
- `int16_t` 差值法需要手动处理 14-bit 边界：`if (diff > 8192) diff -= 16384`
- 采样周期 10ms 时最大可测转速 ≈ 3000 RPM（8192/16384/0.01*60）
- 如果测量值在 ±180 之间跳变，说明采样周期太长或 wrap 处理有误

### 11.7 串口高速上报带宽

| 波特率 | 有效字节率 | 16B/帧上限 |
|--------|:---------:|:--------:|
| 115200 | 11520 B/s | 720 Hz |
| 230400 | 23040 B/s | 1440 Hz |

当前使用 230400，1000Hz 上报（3 通道电流）占 16KB/s，远在带宽内。

