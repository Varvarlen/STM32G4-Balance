# 硬件连接

> **约定**: 软件中使用 ABC 表示三相，硬件 PCB 丝印使用 UVW。对应关系: **U=A, V=B, W=C**。

## 电机驱动 PWM

| 电机 | 驱动芯片 | PWM1 (A/U) | PWM2 (B/V) | PWM3 (C/W) | SHDNB |
|------|---------|------------|------------|------------|-------|
| M1 | MP6536 (#1) | PB9 (TIM4_CH4) | PB7 (TIM4_CH2) | PB6 (TIM4_CH1) | PC14 |
| M2 | MP6536 (#2) | PB1 (TIM3_CH4) | PB0 (TIM3_CH3) | PA7 (TIM3_CH2) | PC14 |

## 电流采样 INA240

| 通道 | 枚举名 | 引脚 | ADC 通道 | 对应相位 |
|------|--------|------|----------|-------------|
| 0 | INA240_MOTOR1_U | PA5 | ADC2_IN13 | M1 V/B 相 |
| 1 | INA240_MOTOR1_W | PA6 | ADC2_IN3 | M1 U/A 相 |
| 2 | INA240_MOTOR2_U | PC4 | ADC2_IN5 | M2 V/B 相 |
| 3 | INA240_MOTOR2_W | PB2 | ADC2_IN12 | M2 W/C 相 |

> **ADC 扫描顺序**: Rank1=IN13, Rank2=IN3, Rank3=IN5, Rank4=IN12。INA240 枚举必须与此顺序一致。
> 验证方法: 查看 `Core/Src/adc.c` 中 `MX_ADC2_Init()` 的 Rank 配置。

## 编码器 MT6701

| 编码器 | 片选引脚 | SPI | enc_direction |
|--------|---------|-----|:---:|
| M1 | PB4 | SPI3 (SCK=PC10, MISO=PC11) | -1 |
| M2 | PA4 | SPI3 (SCK=PC10, MISO=PC11) | +1 |

## 电源

| 项目 | 值 |
|------|-----|
| 电机供电 | 7.4V |
| MCU 供电 | 3.3V |
| INA240 偏置 | 1.65V (Vs/2) |
| INA240 增益 | 50 V/V |
| 采样电阻 | 20 mΩ |

## FOC 参数

| 参数 | M1 | M2 |
|------|:--:|:--:|
| enc_direction | -1 | +1 |
| PWM 通道映射 | ch_a/b/c 原始 | ch_a↔ch_c 交换 |
| 电流传感器映射 | 原始 | ch_u↔ch_v 交换 |
| phase_comp | 0° | 0° |
| PI Kp/Ki | 0.5/20 | 0.5/20 |
| iq_ref 正转方向 | 负值 | 正值 |

> M1: 标准 A-B-C 相序, A/B 相电流采样, 无特殊补偿。
> M2: A/C PWM 交换 + 传感器交换 → 等效为标准 A/B 相序。调试过程详见 `docs/FOC_DEBUG_EXPERIENCE.md`。
