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

## 10. 任务参数速查

| 任务 | 周期 | 栈(words) | 优先级 | 核心操作 |
|------|:----:|:---------:|:------:|------|
| defaultTask | 1ms | 128 | Normal | 串口回显（保留不动） |
| encoderTask | 5ms | 256 | Normal | SPI 读 MT6701 → 全局变量 |
| adcTask | 5ms | 256 | Normal | DMA 读 INA240 → 全局变量 |
| mpuTask | 5ms | 512 | Normal | SPI 读 MPU6500 + KF → 全局变量 |
| reporterTask | 50ms | 256 | Normal | 收集全局变量 → COMM_SendFloatFrame |
| healthTask | 5s | 128 | Low | xPortGetFreeHeapSize → COMM_SendData |
