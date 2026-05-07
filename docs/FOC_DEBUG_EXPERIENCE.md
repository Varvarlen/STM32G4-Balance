# FOC 调试经验

## 当前工作配置

| 参数 | M1 | M2 |
|------|:--:|:--:|
| enc_direction | -1 | +1 |
| PWM 映射 | 原始 A-B-C | A/C 交换 (ch_a↔ch_c) |
| 电流传感器映射 | ch_u=MOTOR1_W, ch_v=MOTOR1_U | ch_u=MOTOR2_W, ch_v=MOTOR2_U |
| phase_comp | 0° | 0° |
| PI (Kp/Ki) | 0.5/20 | 0.5/20 |

等效效果：M2 经 PWM 交换 + 传感器交换后，Clarke 输入端恰好为标准 A/B 相序，与 M1 一致。

## 架构概览

```
ADC DMA ISR (DMA1_Channel4, ~10kHz)
  └─ 同步 g_motor[i].elec_angle = g_enc[i].elec_angle
  └─ CurrentCtrl_Run(&g_motor[i])   // 电流闭环时
       ├─ INA240_GetCurrentFast()   → Ia, Ib, Ic=-(Ia+Ib)
       ├─ Clarke()                  → Iα, Iβ
       ├─ fast_sincos(elec+comp)    → Park: Id, Iq
       ├─ PI_Step(id_pi) / PI_Step(iq_pi) → Vd, Vq
       ├─ 逆 Park                   → Vα, Vβ
       └─ SVPWM_SetVab()            → 三相占空比 → TIM CCR

编码器 SPI DMA ISR (HAL_SPI_TxRxCpltCallback)
  └─ 解析 24-bit SSI 帧 → raw_angle
  └─ g_enc[].elec_angle = mech * 7 * enc_direction  // 方向已在编码器层处理
```

## 校准体系

项目使用独立的校准模块 (`Core/Src/calibration.c`)，包含 5 个校准实验，可在硬件更换时独立调用：

| 编号 | 实验 | 校准对象 | CLI 命令 |
|:----:|------|----------|:--------:|
| 1 | 电流采样零漂校准 | INA240 zero_offset[4] | `c1` / `d1` |
| 2 | 相线及电流传感器映射 | ch_u/ch_v 映射 | `c2` / `d2` |
| 3 | 编码器方向匹配 | enc_direction | `c3` / `d3` |
| 4 | 编码器零位校准 | phase_comp | `c4` / `d4` |
| 5 | 电机参数辨识 | 相电阻 | `c5` / `d5` |

> 校准结果保存在 MCU 内部 Flash（最后一页 0x0801F800），上电自动加载。

## INA240 关键陷阱

**枚举顺序必须匹配 ADC 扫描顺序。** ADC 扫描由 CubeMX 配置的 Rank 决定，INA240 枚举的整数值直接用作 `adc_buffer` 和 `filtered_buffer` 的索引。两侧不一致会导致每个电机读到对方的电流传感器。

### 验证方法

查看 `adc.c` 中 `MX_ADC2_Init()` 的 Rank 1-4 对应哪个 `ADC_CHANNEL`，确保枚举顺序与此一致。本项目实际扫描顺序为 IN13, IN3, IN5, IN12，对应枚举：

```c
typedef enum {
    INA240_MOTOR1_U = 0,    // M1 V/B相 — PA5  — ADC2_IN13 — rank1
    INA240_MOTOR1_W = 1,    // M1 U/A相 — PA6  — ADC2_IN3  — rank2
    INA240_MOTOR2_U = 2,    // M2 V/B相 — PC4  — ADC2_IN5  — rank3
    INA240_MOTOR2_W = 3,    // M2 W/C相 — PB2  — ADC2_IN12 — rank4
} INA240_Channel_t;
```

> **命名陷阱**：枚举名用 INA240 芯片引脚标签（U/W），不是电机相名。`INA240_MOTOR1_U` 实际连接 M1 V/B 相。

**教训**：不要假设扫描顺序，必须从 `adc.c` 代码中确认 Rank 分配。

## 调试实验记录

以下实验是初始调试时手动执行的，已完成其使命。**现在应使用校准模块替代。**

### 实验 A：编码器方向校准

**目的**：确定 `enc_direction`（+1 或 -1）。

**方法**：用手转动电机，观察 `g_enc[i].raw_angle` 变化。正转时 raw 递增 → +1，递减 → -1。

**当前值**：M1=-1, M2=+1（已写入 `mt6701.c:161`）。

### 实验 B：开环相序标定

**目的**：判断三相线是否交叉。

**方法**：施加相同的旋转电压矢量给两台电机，观察 RPM 方向。同向 = 相序一致，反向 = 其中一台有交叉。

**结论**：M2 有相序交叉，经实验 C/E 确定为 A/C 交换。

### 实验 C：电流传感器-相线映射

**目的**：确认每个 INA240 通道连接在哪个电机相上。

**方法**：逐相 DC 激励（占空比 58%，~0.6V），观察 INA240 通道的电流响应。

**结论**：M1 为标准 A/B 映射，M2 为 W/U 交换。已写入 `foc.c:19-20,38-39`。

### 实验 D：DC 锁轴 phase_comp 标定（已由校准模块替代）

**目的**：测量编码器零位与电机 d 轴的偏移角。

**方法**：施加 DC 电压锁轴，读取编码器角度。

**当前值**：phase_comp = 0°（两个电机）。注意：此值为电压锁方法测得，可能不够精确。推荐使用校准模块的 Iq 旋转 + Id 精锁方法重新校准。

**经验**：
- 7 极对电机在 1 个机械圆周内有 7 个稳定点，电气角度相同 (mod 2π)。
- 电压锁方法对静摩擦敏感，Iq 旋转 + Id 精锁方法更可靠。

### 实验 E：PWM 交换全遍历

**目的**：遍历 PWM 交换组合确定正确映射。

**方法**：开环 q 轴电压驱动，遍历 4 种 PWM 交换。

**结论**：M2 需要 A/C 交换。已写入 `foc.c:33-36`。

## PI 参数调优

电机电气时间常数极小 (R≈6.5Ω, L≈50μH → L/R≈7.7μs)，远快于 PWM 周期 (100μs)。Ki 过大时积分快速积累导致 Vq 饱和。

**当前值**：Kp=0.5, Ki=20（两电机相同）。

## FOC 电流环验证

用手抓住旋转中的电机轴：
- 电机被迫减速 → 反电动势下降 → Vq 自动退饱和
- Iq 仍稳定跟踪设定值、Id≈0 → 电流环正常
- Vq 在高速时饱和 (≈Vbus) 是反电动势补偿的**正常行为**，不是故障

## 常见故障症状速查

| 症状 | 可能原因 |
|------|------|
| 静止时 Iq 跟踪，转动后振荡/饱和 | enc_direction 符号错误 |
| Id/Iq 以电频率 2ω 振荡 | Park 参考系反转 (encoder 方向错) |
| Id 很大，电机锁死 | phase_comp 偏移约 90° |
| Iq 跟踪，电机不转 | d/q 轴交换 (传感器相序错) |
| 所有 phase_comp 下电机均不转 | PWM 通道映射有误 或 电流传感器错位 |
| 两电机中只有一台正常 | 检查该电机的 enc_dir / PWM / 传感器 三条独立路径 |
