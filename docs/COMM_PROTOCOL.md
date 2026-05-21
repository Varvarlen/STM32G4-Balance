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
- 最大通道数：`PROTOCOL_MAX_CHANNELS = 12`
- Cortex-M4 原生小端，float 按字节直接复制

### 发射函数

```c
void COMM_SendFloatFrame(const float *data, uint8_t count);
```

### 正常模式遥测帧 (200Hz, 5ms 周期)

| 通道 | 字段 | 单位 | 说明 |
|:----:|------|:----:|------|
| f0 | M1 ref | rad / RPM | 位置模式=pos_ref(rad), 速度模式=speed_ref_ramp(RPM) |
| f1 | M1 pos_est | rad | M1 EKF 连续位置估计 |
| f2 | M1 speed_fb | RPM | M1 EKF+EMA 速度反馈 |
| f3 | M1 iq | A | M1 q 轴电流实测 |
| f4 | M1 speed_ref | RPM | M1 当前速度指令 |
| f5 | M2 ref | rad / RPM | 同上 |
| f6 | M2 pos_est | rad | M2 EKF 连续位置估计 |
| f7 | M2 speed_fb | RPM | M2 EKF+EMA 速度反馈 |
| f8 | M2 iq | A | M2 q 轴电流实测 |
| f9 | M2 speed_ref | RPM | M2 当前速度指令 |
| f10 | M1 enc_angle | rad | M1 编码器机械角 (方向校正) |

共 11 通道，帧长 48 字节。发送周期 5ms → 200Hz。遥测默认关闭，通过 `T` 命令手动开启。
位置模式下 f0/f5 为 pos_ref (rad)，速度模式下为 speed_ref_ramp (RPM)。

### 阶跃 burst 采集帧 (二进制 dump, 按需)

阶跃测试完成时通过 `COMM_SendData` 二进制下传：

```
[MAGIC:4B 0x53554252 "RBUS" LE] [COUNT:2B uint16 LE] [FIELDS:1B] [RESV:1B] [DATA:N×4B×FIELDS float32 LE]
```

| 字段 | 索引 | 单位 | 说明 |
|:----:|:----:|:----:|------|
| speed_fb | 0 | RPM | EKF 速度反馈 |
| iq | 1 | A | q 轴电流实测 |
| speed_ref | 2 | RPM | 阶跃给定 (瞬时切换) |
| T_load_est | 3 | N·m | EKF 负载转矩估计 |

默认 256 帧 × 4 字段 = 4096 字节，分批 200B/块发送。

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

## 平衡车遥测帧

频率: 200Hz (每5次平衡循环1帧), 仅在遥测开启时发送
帧格式: 10 通道小端 float + footer

| 通道 | 内容 | 单位 |
|:----:|------|:----:|
| 0 | tilt_angle — 倾角 | ° |
| 1 | gyro_rate — 角速度 | °/s |
| 2 | balance_out — 平衡PID输出 | RPM |
| 3 | speed_fb_R — 右轮速度反馈 | RPM |
| 4 | speed_fb_L — 左轮速度反馈 | RPM |
| 5 | iq_R — 右轮q轴电流 | A |
| 6 | iq_L — 左轮q轴电流 | A |
| 7 | target_angle — 目标倾角 | ° |
| 8-9 | 预留 (0.0) | — |

## 文本指令 (CLI)

所有命令通过同一串口接收，以 ASCII 字符 + 可选数值组成。命令前缀 `R`/`L` 分别表示 M1/M2。

### 正常模式命令

| 指令 | 功能 | 例 |
|------|------|-----|
| `R<A>` | M1 电流模式 iq_ref=A | `R0.3` |
| `L<A>` | M2 电流模式 | `L-0.5` |
| `R<A> L<B>` | 两电机堆叠 | `R0.2 L0.3` |
| `RS<RPM>` | M1 速度模式 [ramp] | `RS500` |
| `LS<RPM>` | M2 速度模式 | `LS-300` |
| `RP<deg>` | M1 位置模式 (累计绝对角度) | `RP1000` |
| `LP<deg>` | M2 位置模式 | `LP-500` |
| `RT<RPM>` | M1 阶跃 当前→RPM [no ramp] [burst] | `RT100` |
| `RT<A> <B>` | M1 阶跃 A→B | `RT50 100` |
| `LT<RPM>` | M2 阶跃 | `LT200` |
| `LT<A> <B>` | M2 阶跃 | `LT-50 50` |
| `RE<RPM>` | M1 负载实验 | `RE50` |
| `LE<RPM>` | M2 负载实验 | `LE-100` |
| 再次 RT/LT/RE/LE | 停止当前测试 | `RT` |

RS/LS 与 R/L 互斥：RS/LS 进入速度模式，R/L 退出速度模式切回电流模式。
RP/LP 进入位置模式 (级联 P+速度 PI)，反馈为编码器增量展开位置。
速度/阶跃/负载命令自动退出位置模式。RE/LE 自动执行完整实验流程。再次执行同一命令可中途停止测试。

### PI 参数命令

| 指令 | 功能 | 例 |
|------|------|-----|
| `PRS` | 查询 M1 速度 PI | 回显 Kp/Ki/ω₀/Out |
| `PLS` | 查询 M2 速度 PI | |
| `PS` | 查询两电机速度 PI | 同时回显 M1+M2 |
| `PRC` | 查询 M1 电流 PI | |
| `PLC` | 查询 M2 电流 PI | |
| `PC` | 查询两电机电流 PI | |
| `PR` / `PL` | 查询单电机全部 PI + Pos | |
| `P` | 查询两电机全部 PI + Pos | |
| `PRP` | 查询 M1 位置 Kp | |
| `PLP` | 查询 M2 位置 Kp | |
| `PP` | 查询/设置两电机位置 Kp | `PP P=210` |
| `PRP P=210` | 设置 M1 位置 Kp | |
| `PLP 150` | 设置 M2 位置 Kp (直接数值) | |
| `PRS P=0.015 I=0.1` | 设置 M1 速度 PI（标签） | |
| `PRS 0.015 0.1` | 设置 M1 速度 PI（位置 Kp Ki） | |
| `PRC P=12 I=2400` | 设置 M1 电流 PI | |
| `PS P=0.015 I=0.1` | 两电机同时设置速度 PI | |
| `PS 0.020 0.150` | 两电机同时设置速度 PI（位置） | |
| `PC P=5 I=1765` | 两电机同时设置电流 PI | |

### 其他命令

| 指令 | 功能 |
|------|------|
| `?` | 打印当前状态 + 命令列表 |
| 校准模式 `R1-5` | M1 校准项 1-5 |
| 校准模式 `L1-5` | M2 校准项 1-5 |
| 校准模式 `RS` | 查看校准参数 |
| 校准模式 `q` | 中止校准 |
| 测试模式 `R<A>` | M1 电流阶跃 (A) |
| 测试模式 `L<A>` | M2 电流阶跃 (A) |
| 测试模式 `r` | 重发上次采集 |

## 平衡车指令协议

### 运行指令

| 指令 | 方向 | 功能 |
|------|:----:|------|
| `S<RPM>\n` | → | 前进速度指令 |
| `T<val>\n` | → | 转向差速指令 |
| `STOP\n` | → | 紧急停止 |
| `B\n` | → | 激活平衡控制 |

### 参数指令

| 指令 | 方向 | 功能 |
|------|:----:|------|
| `P\n` | → | 查询全部参数 (响应为文本) |
| `PK\n` | → | 查询平衡参数 |
| `PK <KEY>=<VAL>\n` | → | 设置平衡参数 (KEY: ANG, GYR, ANG0, MAX) |
| `PS P=<kp> I=<ki>\n` | → | 设置速度 PI |
| `PC P=<kp> I=<ki>\n` | → | 设置电流 PI |
| `T\n` | → | 开关遥测 |
| `VCAL <V>\n` | → | 母线电压校准 (两次不同电压后自动计算) |

> 参数设置的响应均为文本，不含浮点帧。

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

# 读取 11 通道遥测帧
data = ser.read(44)  # 11 floats × 4 bytes
f = list(struct.unpack('<11f', data))
# f[0]=M1.ref(pos或speed)  f[1]=M1.pos_est  f[2]=M1.speed_fb
# f[3]=M1.iq  f[4]=M1.speed_ref
# f[5]=M2.ref(pos或speed)  f[6]=M2.pos_est  f[7]=M2.speed_fb
# f[8]=M2.iq  f[9]=M2.speed_ref
# f[10]=M1 编码器机械角(方向校正)
```
