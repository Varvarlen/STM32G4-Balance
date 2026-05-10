# 串口通信协议

## 物理层

| 参数 | 值 |
|------|-----|
| 外设 | USART1 |
| 波特率 | 230400 |
| 数据位 | 8 |
| 校验 | 无 |
| 停止位 | 1 |
| 流控 | 无 |

## 发送 (TX)

- 方式：**DMA 非阻塞发送**
- 缓冲：256 字节环形缓冲区 → DMA 缓冲区 → USART TX
- 线程安全：任务与 ISR 并发访问通过临界区保护 (PRIMASK)

## 接收 (RX)

- 方式：**CIRCULAR DMA + 空闲中断**
- 缓冲：256 字节 DMA 循环缓冲区 → 256 字节环形缓冲区
- 解码：`StartDefaultTask` 以 1ms 周期轮询 `COMM_Available()` / `COMM_ReadByte()`

## 数据帧协议 (二进制遥测)

帧采用**小端 IEEE 754 单精度浮点**编码，帧尾为 `+infinity` 的单精度表示。

### 帧结构

```
[f0: float32 LE] [f1: float32 LE] ... [fN: float32 LE] [00 00 80 7F]
                                                    ↑ 帧尾 = +inf (0x7F800000)
```

- 帧长：`count × 4 + 4` 字节
- 最大通道数：`PROTOCOL_MAX_CHANNELS = 10`
- Cortex-M4 原生小端，float 按字节直接复制

### 发射函数

```c
void COMM_SendFloatFrame(const float *data, uint8_t count);
```

### 正常模式遥测帧 (100Hz, 10ms 周期)

| 通道 | 字段 | 单位 | 说明 |
|:----:|------|:----:|------|
| f0 | M1 id | A | M1 d 轴电流 |
| f1 | M1 iq | A | M1 q 轴电流 |
| f2 | M1 vd | V | M1 d 轴电压 |
| f3 | M1 vq | V | M1 q 轴电压 |
| f4 | M2 id | A | M2 d 轴电流 |
| f5 | M2 iq | A | M2 q 轴电流 |
| f6 | M2 vd | V | M2 d 轴电压 |
| f7 | M2 vq | V | M2 q 轴电压 |
| f8 | M1 RPM | rpm | M1 转速 |
| f9 | M2 RPM | rpm | M2 转速 |

共 10 通道，帧长 44 字节。

### 阶跃测试模式帧 (按需)

| 通道 | 字段 | 单位 | 说明 |
|:----:|------|:----:|------|
| f0 | id | A | d 轴电流 |
| f1 | iq | A | q 轴电流 |
| f2 | iq_ref | A | q 轴电流给定 |

共 3 通道，帧长 16 字节，共 500 帧。

### 历史帧格式 (已弃用)

| 模式 | 通道数 | 内容 |
|------|:------:|------|
| TaskSixStep | 3 | Ia1, Ib1, Ic1 (三相电流) |
| TaskSpeedReport | 4 | M1_RPM, M2_RPM, M1_Vmag, M2_Vmag |

## 文本指令 (CLI)

所有命令通过同一串口接收，以 ASCII 字符 + 可选数值组成。

### 正常模式

| 指令 | 功能 | 例 |
|------|------|-----|
| `R<值>` | 设置 M1 iq_ref (±2A) | `R0.3` |
| `L<值>` | 设置 M2 iq_ref (±2A) | `L-0.5` |
| `R<值>L<值>` | 同时设置两电机 | `R0.2L0.3` |

### 校准模式

| 指令 | 功能 |
|------|------|
| `c1`~`c5` | M1 校准实验 1~5 |
| `d1`~`d5` | M2 校准实验 1~5 |
| `s` | 打印当前校准参数 |
| `q` | 中止当前校准 |

### 阶跃测试模式

| 指令 | 功能 | 例 |
|------|------|-----|
| `SR<值>` | M1 阶跃 | `SR0.5` |
| `SL<值>` | M2 阶跃 | `SL-0.3` |
| `r` | 重发上次采集数据 |

## 启动模式选择

上电 3 秒内按键选择：

| 按键 | 模式 |
|:----:|------|
| `c` | 校准模式 |
| `s` | 阶跃测试模式 |
| 超时 | 正常 FOC 模式 |

## PC 端解析参考

```python
import struct, serial

ser = serial.Serial('COMx', 230400)
while True:
    # 读取 4 字节检查是否为帧尾
    word = ser.read(4)
    if word == b'\x00\x00\x80\x7F':
        break  # 帧尾，重置解析器

# 读取 N 通道帧
data = ser.read(40)  # 10 floats × 4 bytes
floats = list(struct.unpack('<10f', data))
```
