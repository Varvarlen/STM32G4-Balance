# 校准模块文档

## 概述

校准模块 (`Core/Src/calibration.c` / `Core/Inc/calibration.h`) 提供 5 项独立的校准实验，用于在硬件更换或初始调试时系统性地测定 FOC 控制所需的各项参数。校准结果自动保存到 MCU 内部 Flash，上电后自动加载。

## 快速开始

1. 烧录固件，通过串口 (USART1, 115200) 连接
2. 上电后若首次使用，串口输出 `No valid calibration in flash, using defaults`
3. 按顺序执行校准实验（推荐从 1 到 5）：
   ```
   c1 → 零漂校准
   c2 → M1 相线映射
   c3 → M1 编码器方向
   c4 → M1 编码器零位
   d1-d4 → 对 M2 重复
   c5 → M1 相电阻（可选）
   ```
4. 任何时候输入 `s` 查看已保存的参数
5. 输入 `q` 中止正在进行的校准

## CLI 命令参考

| 命令 | 功能 | 电机 | 耗时 |
|:----:|------|:----:|:----:|
| `c1` | 电流采样零漂校准 | 全局 | ~1s |
| `c2` | 相线及电流传感器映射 | M1 | ~1.5s |
| `c3` | 编码器方向匹配 | M1 | ~3s |
| `c4` | 编码器零位校准 | M1 | ~4s |
| `c5` | 电机参数辨识 | M1 | ~2.5s |
| `d1`~`d5` | 同上 | M2 | 同上 |
| `s` | 打印当前校准参数 | — | — |
| `q` | 中止正在进行的校准 | — | — |

## 5 项校准实验

### 实验 1 — 电流采样零漂校准 (`c1` / `d1`)

**目的**：测定 INA240 各通道在零电流时的 ADC 读数（零偏值）。

**原理**：在电机未使能（PC14=LOW）的状态下，采集 1000 次 ADC 原始值，使用 **trimmed mean**（剔除首尾各 10%）消除离群噪声。同时计算噪声标准差作为质量指标。

**输出**：
```
CH0: offset=2047 noise=1.23 LSB
CH1: offset=2049 noise=1.15 LSB
CH2: offset=2046 noise=1.31 LSB
CH3: offset=2050 noise=1.08 LSB
```

**质量判断**：noise > 5 LSB 表示该通道异常（可能是接地不良或干扰）。

**注意**：零偏随温度有轻微漂移（INA240 典型 ~2.5µV/°C，折合约 0.15 LSB/°C），但在正常工作温度范围内影响可忽略。

---

### 实验 2 — 相线及电流传感器对应关系辨识 (`c2` / `d2`)

**目的**：确定每个 INA240 通道实际连接在电机的哪一相上。

**原理**：逐相注入小电压直流激励（占空比 58%，约 0.6V），观察 4 个 INA240 通道的电流响应。有显著读数的通道即为该相的传感器。

**方法**：
1. 禁用另一台电机
2. 对 A 相施加 58% 占空比，其余 50% → 读取 4 通道 → 最大响应通道 = A 相传感器
3. 对 B 相施加 58% 占空比，其余 50% → 读取 4 通道 → 最大响应通道 = B 相传感器
4. C 相由基尔霍夫定律推算，无需传感器

**安全**：DC 注入电流约 100mA（6Ω 绕组 + 0.6V 有效电压），远小于额定电流。

**输出**：
```
Phase A: resp CH1 (0.095A)
Phase B: resp CH0 (0.088A)
=== 相线映射 M1 OK: U→CH1, V→CH0 ===
```

**注意**：
- PC14 由两个 MP6536 共用，校准 M1 时 M2 会被强停。
- 此实验依赖 INA240 枚举顺序与 ADC 扫描顺序一致（见 `docs/FOC_DEBUG_EXPERIENCE.md` INA240 陷阱章节）。

---

### 实验 3 — 编码器方向匹配 (`c3` / `d3`)

**目的**：确定 `enc_direction` 值（+1 或 -1），使编码器读数递增方向与电机正转方向一致。

**原理**：施加 0.5Hz 开环旋转电压矢量（Vm=0.5V），读取 `g_enc[].mech_angle`（纯机械角，无方向因子）。比较机械角累积变化方向：
- 累积增加 → enc_direction = +1
- 累积减少 → enc_direction = -1

**输出**：
```
accum_delta=+5.234 rad → enc_direction=+1
=== 编码器方向 M1 OK: enc_dir=+1 (saved) ===
```

**注意**：此实验使用 `g_enc[].mech_angle`（而非 `elec_angle`），因为后者已包含当前的 `enc_direction` 修正，会形成循环依赖。

---

### 实验 4 — 编码器零位校准 (`c4` / `d4`)

**目的**：测定 `phase_comp`，使电角度 0° 精确对应电机 A 相绕组的 d 轴方向。

**方法 — Iq 旋转 + Id 精锁**（对比传统电压锁方法的改进）：

```
Phase 1 — Iq 旋转 (~2s)
  电流闭环, Iq=0.3A, virtual_angle 从 0 → 2π (0.5Hz)
  电机被 Iq 转矩驱动旋转，克服静摩擦
  ISR 读取 virtual_angle 执行电流闭环，Cortex-M4 32-bit 原子安全

Phase 2 — Id 精锁 (500ms)
  切换 Id=0.8A, Iq=0, virtual_angle=0
  d 轴强锁到 A 相绕组方向 (0° 电角)
  PI 控制器持续输出 Vd 维持 Id 电流，自动补偿扰动

Phase 3 — 读取编码器偏移
  读取 5 次 g_enc[].elec_angle，取平均
  phase_comp = normalize(-avg_elec)

Phase 4 — 验证
  切回编码器, 跑 Iq=0.3A/Id=0, 200ms 内测量角度漂移
  |RPM| < 30 → PASS, 否则 WARN
```

**输出**：
```
Phase 1: Iq rotation @0.5Hz, Iq=0.3A (ISR-driven)
Phase 2: Id lock @0.8A, 500ms
Phase 3: read encoder offset
  sample 1: enc_elec=1.234 → offset=5.049 rad
  sample 2: enc_elec=1.230 → offset=5.053 rad
  ...
Phase 4: verification
  drift: 0.012 rad (3.4 RPM)
=== 编码器零位 M1 OK: phase_comp=5.051 rad (289.4 deg) (saved) ===
```

**与电压锁方法的对比**：

| 维度 | 电压 DC 锁轴（旧） | Iq 旋转 + Id 精锁（新） |
|------|--------------------|-------------------------|
| 对齐力矩 | 弱，依赖永磁体 | 强，PI 闭环持续输出 |
| 静摩擦处理 | 无 | Iq 旋转克服 stiction |
| 可重复性 | 低，受初始位置影响 | 高，每次锁定路径一致 |
| 精度 | 中 | 高（5 次采样 + 验证） |

**安全说明**：
- ISR 交互：任务写入 `virtual_angle` (float, 32-bit)，ISR 读取。Cortex-M4 硬件保证对齐 32-bit 读写原子性，无需临界区。
- 电流限制：Iq=0.3A / Id=0.8A，在电机额定范围内。PI 输出限幅 5V，远低于 7.4V 母线。
- `CALIB_IsAborted()` 在每个循环迭代中检查，输入 `q` 可随时中止。

---

### 实验 5 — 电机参数辨识 (`c5` / `d5`)

**目的**：测量电机相电阻。

**原理**：两级 Id 电流注入（0.3A、0.6A），测量稳态 Vd 电压。利用差值消除 IGBT 管压降：

```
R_d = (Vd2 - Vd1) / (Id2 - Id1)
R_phase = (2/3) × R_d     ← Clarke 幅值不变形式修正
```

**输出**：
```
Id=0.3A → Vd=0.452V
Id=0.6A → Vd=0.870V
=== 电机参数 M1 OK: R_phase=1.393 ohm (saved) ===
```

**注意**：
- 测量的是 d 轴等效电阻，已修正为 Y 接每相电阻。
- 如需测量电感，需要更高频率的交流注入（当前未实现）。

---

## Flash 持久化

校准参数存储在 `CalibParams_t` 结构中，写入 MCU 内部 Flash 最后一页（Page 63, `0x0801F800`, 2KB）。

### 存储布局

```
0x0801F800 ┌──────────────────┐
           │ magic (0xCA1B0001)│
           │ version (1)       │
           │ enc_direction[2]  │  ← int8_t × 2
           │ phase_comp[2]     │  ← float × 2 (rad)
           │ zero_offset[4]    │  ← uint16_t × 4
           │ ch_u_mapping[2]   │  ← uint8_t × 2
           │ ch_v_mapping[2]   │  ← uint8_t × 2
           │ phase_resistance[2]│ ← float × 2 (Ω)
           │ crc32             │  ← 完整性校验
0x0801FA00 └──────────────────┘ (实际只占用约 50 字节)
```

### 加载流程

```
上电 → FOC_Init() 后
     → CALIB_FlashLoad(&g_calib)
     → CALIB_FlashIsValid() 检查 magic + CRC32
     → 有效: 应用 enc_direction → MT6701_SetEncDirection
              应用 phase_comp → motor->phase_comp
              应用 zero_offset → INA240_SetAllZeroOffsets
              应用 ch_u/ch_v → motor->ch_u / motor->ch_v
     → 无效: 使用代码默认值，串口提示 "No valid calibration"
```

### 安全性

- CRC32 校验防止 bit-rot
- STM32G4 硬件 ECC（每 64-bit 8 位纠错码）
- Flash 写入仅在电机停转时进行（校准函数末尾）
- Flash 擦除/编程期间 CPU 暂停（~20ms），不影响后续运行

---

## 并发安全

### 锁机制

```c
static volatile uint8_t calib_busy = 0;

uint8_t CALIB_TryLock(void);   // 原子检查并置位，返回 0=已被占用
void    CALIB_Unlock(void);    // 释放锁
```

### ISR 交互安全性

| 校准类型 | 模式 | ISR 行为 | 并发风险 |
|----------|------|----------|:--------:|
| 零漂校准 | — | EMA 滤波继续运行 | 无（仅读取 ADC buffer） |
| 相线映射 | VOLTAGE_SINE | ISR 跳过 CurrentCtrl_Run | 无（任务独占 PWM 控制） |
| 编码器方向 | VOLTAGE_SINE | ISR 跳过 CurrentCtrl_Run | 无 |
| 编码器零位 | CURRENT_LOOP | ISR 读取 virtual_angle 执行 PI | 无（32-bit 原子） |
| 电机参数 | CURRENT_LOOP | ISR 读取 id_ref 执行 PI | 无（32-bit 原子） |

**关键保证**：Cortex-M4 保证对齐的 32-bit 内存访问是原子的。`float` 类型（32-bit）和 `uint16_t` 类型（16-bit）的读写在此架构上天然安全，无需临界区。

### PC14 共用问题

两个 MP6536 驱动芯片共用 GPIO PC14 作为使能引脚。校准任一电机时：

1. `Motor_Disable()` → PC14 = LOW → **两个电机同时禁能**
2. 目标电机重新 `Motor_StartPWM()` → PC14 由该电机的校准流程重新拉高
3. 校准完成后 `calib_safe_stop()` → PC14 再次拉低

串口输出会提示：`=== 相线映射校准 M1 START (M2 will be stopped) ===`

---

## 校准参数结构

```c
typedef struct {
    uint32_t magic;              // 0xCA1B0001
    uint16_t version;            // 1

    int8_t   enc_direction[2];   // M1, M2: +1 或 -1
    float    phase_comp[2];      // M1, M2: 编码器零位偏移 (rad)
    uint16_t zero_offset[4];     // INA240 四通道零偏 ADC 值
    uint8_t  ch_u_mapping[2];    // 电机 i 的 U 相对应 INA240 通道号
    uint8_t  ch_v_mapping[2];    // 电机 i 的 V 相对应 INA240 通道号
    float    phase_resistance[2]; // M1, M2: 相电阻 (Ω)

    uint32_t crc32;              // 完整性校验
} CalibParams_t;
```

可通过串口命令 `s` 随时查看当前值：
```
=== CALIB PARAMS ===
enc_dir:   M1=-1 M2=+1
phase_comp: M1=0.000 M2=0.000 rad
zero_offs: 2048 2047 2046 2050
ch_u_map:  M1=1 M2=3
ch_v_map:  M1=0 M2=2
R_phase:   M1=1.393 M2=1.401 ohm
valid: YES
```

---

## 相关文件

| 文件 | 说明 |
|------|------|
| `Core/Inc/calibration.h` | 校准模块 API 声明 |
| `Core/Src/calibration.c` | 校准模块实现 |
| `Core/Src/foc.c` | FOC_Init() 中加载校准参数 |
| `Core/Src/current_ctrl.c` | Park 角度 virtual/encoder 分叉 |
| `Core/Src/main.c` | 启动序列中加载 Flash 校准 |
| `Core/Src/app_freertos.c` | StartDefaultTask CLI 命令处理 |
| `docs/FOC_DEBUG_EXPERIENCE.md` | 调试经验与故障速查 |
| `docs/HARDWARE_CONNECTIONS.md` | 硬件连接参考 |
