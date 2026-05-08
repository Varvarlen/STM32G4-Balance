# 校准模块使用指南

## 快速开始

1. 烧录固件，串口连接 USART1 (230400)
2. 上电后首次使用，串口输出 `No valid calibration in flash, using defaults`
3. 按顺序执行（**顺序不可颠倒**）：
   ```
   c1 → 全局零漂校准      (~1.2s, 电机停转)
   c2 → M1 相线映射       (~1.5s, 电机轻微嗡嗡声)
   c3 → M1 编码器方向     (~3s,   电机低压慢转)
   c4 → M1 编码器零位     (~4.5s, 电机旋转后锁死)
   d2 → M2 相线映射       (重复 M1 流程)
   d3 → M2 编码器方向
   d4 → M2 编码器零位
   c5 → M1 相电阻(可选)   (~2.5s, 电机锁死)
   ```
4. `s` 随时查看已保存参数，`q` 中止校准

## CLI 命令

| 命令 | 功能 | 电机 | 耗时 | 电机状态 |
|:----:|------|:----:|:----:|:-----:|
| `c1` | 电流零漂校准 | 全局 | ~1.2s | 停转 |
| `c2` | 相线及传感器映射 | M1 | ~1.5s | 轻微振动 |
| `c3` | 编码器方向匹配 | M1 | ~3s | 低压慢转 |
| `c4` | 编码器零位校准 | M1 | ~4.5s | 旋转→锁死 |
| `c5` | 电机参数辨识 | M1 | ~2.5s | 锁死 |
| `d1`~`d5` | 同上 | M2 | 同上 | 同上 |
| `s` | 打印校准参数 | — | — | — |
| `q` | 中止当前校准 | — | — | — |

> `c1`/`d1` 无论输入哪个都执行全局校准（4 通道同时测量），只需执行一次。

## 校准执行顺序

```
c1 (零漂,全局) ──────────────────────────────────────────────┐
    │                                                         │
    ├── c2 (映射,M1) ── c3 (方向,M1) ── c4 (零位,M1) ── c5 (电阻,M1,可选)
    │                                                         │
    └── d2 (映射,M2) ── d3 (方向,M2) ── d4 (零位,M2) ── d5 (电阻,M2,可选)

依赖关系:
  c1 ← 无依赖, 最先执行
  c2 ← 依赖 c1 (零漂影响电流读数准确性)
  c3 ← 依赖 c2 (正确的相线映射保证旋转方向判断正确)
  c4 ← 依赖 c1+c2+c3 (零漂+映射+方向 = 电流环才能正确运行)
  c5 ← 依赖 c1+c2 (零漂+映射 = Id 电流准确)
```

---

## 各实验详细操作指南

### 实验 1 — 电流零漂校准 (`c1`)

**电机状态**：完全停转（PC14 保持 LOW）  
**风险**：无  
**串口输出示例**：

```
=== 零漂校准 START ===
 CH0: offset=2047 noise=1.23 LSB
 CH1: offset=2049 noise=1.15 LSB
 CH2: offset=2046 noise=1.31 LSB
 CH3: offset=2050 noise=1.08 LSB
=== 零漂校准 OK (saved) ===
```

**监控要点**：

| 指标 | 正常范围 | 异常处理 |
|------|:--------:|------|
| offset | 2048 ± 10 | > 2060 或 < 2036 → 检查 INA240 供电 (3.3V / 1.65V 偏置) |
| noise | < 5 LSB | ≥ 5 → 检查该通道接地、屏蔽、电源纹波 |
| 各通道一致性 | 4 通道 offset 相差 < 5 | 差异大 → INA240 个体差异或 PCB 走线问题 |

**物理验证**：用万用表测量电机端子电压，应为 0V（±10mV）。

---

### 实验 2 — 相线及传感器映射 (`c2` / `d2`)

**电机状态**：DC 注入时轻微"嗡嗡"声，电流 ~100mA，电机不转  
**风险**：无（100mA 远小于额定电流）  
**串口输出示例**：

```
=== 相线映射校准 M1 START (M2 will be stopped) ===
 Phase A: resp CH1 (0.095A)
 Phase B: resp CH0 (0.088A)
=== 相线映射 M1 OK: U→CH1, V→CH0 (saved) ===
```

**监控要点**：

| 指标 | 正常范围 | 异常处理 |
|------|:--------:|------|
| 响应电流 | 50-150mA | < 10mA → 电机相线未连接或断线 |
| | | > 500mA → 可能有短路 |
| A/B 响应不同通道 | 必须不同 | 相同 → 两个传感器接在同一相上 |
| 电机声音 | 轻微嗡嗡声 | 无声 → 检查 MP6536 是否使能、PWM 是否正确 |

**物理验证**：手指触摸电机轴能感受到轻微振动，确认有电流流过。

**注意**：两电机共用 PC14，校准 M1 时 M2 被强停。串口会提示 `M2 will be stopped`。

---

### 实验 3 — 编码器方向匹配 (`c3` / `d3`)

**电机状态**：低压慢转（Vm=0.5V, 0.5Hz），约半圈/秒  
**风险**：无（电压极低）  
**串口输出示例**：

```
=== 编码器方向 M1 START ===
 accum_delta=+5.234 rad → enc_direction=+1
=== 编码器方向 M1 OK: enc_dir=+1 (saved) ===
```

**异常输出**：

```
=== 编码器方向 M1 FAILED: 电机未转动 (|delta|=0.003) ===
 检查: Vm 是否过低, 电机是否卡死, 编码器是否正常
```

**监控要点**：

| 观察项 | 正常 | 异常 |
|--------|------|------|
| 电机旋转 | 平稳慢转 | 不转 → 检查 c2 是否完成、PWM 线是否接对 |
| | | 抖动 → 电压太低或相序错误 |
| accum_delta | 远离 0 的显著值 | ≈ 0 → 回报 FAIL，电机没转 |
| 旋转方向 | 正转（顺时针，面对轴端） | 反转 → enc_direction 会判为 -1，这是正确的 |

**物理验证**：目视确认电机轴在缓慢旋转。听声音：应有平稳低频嗡嗡声。

---

### 实验 4 — 编码器零位校准 (`c4` / `d4`)

**电机状态**：Phase 1 旋转 → Phase 2 突然锁死 → Phase 3-4 保持锁死  
**风险**：低（Iq=0.3A, Id=0.8A 在额定范围内）  
**串口输出示例**：

```
=== 编码器零位校准 M1 START (M2 stopped) ===
 Phase 1: Iq rotation @0.5Hz, Iq=0.3A (ISR-driven)
 Phase 2: Id lock @0.8A, 500ms
 Phase 3: read encoder offset
  sample 1: enc_elec=1.234 → offset=5.049 rad
  sample 2: enc_elec=1.230 → offset=5.053 rad
  sample 3: enc_elec=1.232 → offset=5.051 rad
  sample 4: enc_elec=1.234 → offset=5.049 rad
  sample 5: enc_elec=1.231 → offset=5.052 rad
 Phase 4: verification (Id=0.3A hold)
  drift: 0.012 rad (3.4 RPM)
=== 编码器零位 M1 OK: phase_comp=5.051 rad (289.4 deg) (saved) ===
```

**分阶段监控**：

| 阶段 | 电机状态 | 正常 | 异常及处理 |
|------|:--------:|------|------|
| Phase 1 (Iq 旋转) | 以 0.5Hz 平稳旋转 | 匀速不卡顿 | 不转/急跳 → c2/c3 未正确完成 |
| | | | 抖动剧烈 → 降低 CALIB_ZERO_IQ 或提高 PI |
| Phase 2 (Id 精锁) | 突然停止, 伴随轻微"咔嗒" | **不要触碰电机！** | 继续旋转 → Id 电流不够, 检查零漂 c1 |
| Phase 3 (读偏移) | 保持锁死 | 5 个 offset 相差 < 0.02 rad | 差异 > 0.05 rad → 锁轴不稳定, 加大 Id 或延长锁定时长 |
| Phase 4 (验证) | 保持锁死 | drift < 30 RPM → PASS | drift ≥ 30 RPM → **WARN**，phase_comp 可能不准 |

**Phase 4 验证说明**：用 `Id=0.3A, Iq=0`（纯 d 轴电流）锁轴。如果 `phase_comp` 正确，d 轴对齐到 A 相绕组，电机应保持静止。如果 drift ≥ 30 RPM，建议重新执行 c4。

**为什么用 Id 而非 Iq 验证**：Iq 是转矩电流，注入后电机必然旋转，无法验证零位正确性。Id 是对齐电流——相位正确时仅产生对齐力矩（无旋转），相位错误时部分泄漏到 q 轴产生旋转。

---

### 实验 5 — 电机参数辨识 (`c5` / `d5`)

**电机状态**：锁死（d 轴对齐，两次不同电流注入）  
**风险**：无（Id ≤ 0.6A）  
**串口输出示例**：

```
=== 电机参数辨识 M1 START ===
 Id=0.3A → Vd=1.953V
 Id=0.6A → Vd=3.895V
=== 电机参数 M1 OK: R_phase=6.473 ohm (saved) ===
```

**监控要点**：

| 指标 | 说明 |
|------|------|
| 线性关系 | Vd2 ≈ 2 × Vd1（因 I2 = 2 × I1）——验证测量一致性 |
| R_phase 范围 | 典型无刷电机 1-20 Ω |
| R_phase < 0.5 Ω | 可能有短路或测量错误 |
| R_phase > 50 Ω | 可能有断线或接触不良 |
| 数据手册对比 | 与电机规格书中的线电阻对比，应为线电阻/2 |

**原理**：两级电流差值法消除 IGBT 管压降：
```
R_d = (Vd2 - Vd1) / (I2 - I1)
R_phase = R_d  (幅值不变 Clarke 下 d 轴等效电阻 = 相电阻)
```

---

## 故障速查

| 症状 | 可能原因 | 检查步骤 |
|------|----------|------|
| c1 噪声 > 5 LSB | 电源纹波/接地不良 | 用示波器看 INA240 OUT 引脚 |
| c2 全部通道电流 ≈ 0 | PC14 未拉高, MP6536 禁能 | 万用表量 PC14 是否为 3.3V |
| c2 A/B 响应相同 | 两个 INA240 接在同一相 | 对照 `docs/HARDWARE_CONNECTIONS.md` 检查接线 |
| c3 电机不转 | c2 未先执行, 或 PC14 未拉高 | 先执行 c2 确认映射正确 |
| c3 电机抖动 | Vm 太低或相序错 | 调高 Vm (当前 0.5V) 或重做 c2 |
| c4 Phase 1 不转 | c2/c3 未完成, 或电流环不工作 | 确认零漂(c1) → 映射(c2) → 方向(c3) 都已完成 |
| c4 Phase 3 样本差异大 | 锁轴不稳定, 摩擦或齿槽转矩大 | 增大 CALIB_ZERO_ID_LOCK 到 1.0A |
| c4 Phase 4 PASS 但电机实际偏角 | 7 极对电机有 7 个平衡点 | 正常 — 电角度一致, 不影响 FOC |
| 任何时候按 q 无效 | 当前正在 osDelay 盲区 | 最长 100ms 后响应 (已拆分为短延时) |
| 校准后 FOC 仍不正常 | 参数未被加载 | 输入 `s` 确认 `valid: YES`，确认执行了正确的 c/d 命令 |
| Flash save FAILED | Flash 页已损坏 | 检查 0x0801F800 是否被其他代码占用 |

---

## Flash 持久化

参数存储在 MCU Flash 最后一页 (Page 63, `0x0801F800`, 2KB)：

```
offset  内容
0x00    magic   0xCA1B0001  (4B)
0x04    version 1            (2B)
0x06    reserved             (2B)
0x08    enc_direction[2]     (2B, M1+M2)
0x0A    phase_comp[2]        (8B, float×2)
0x12    zero_offset[4]       (8B, uint16_t×4)
0x1A    ch_u_mapping[2]      (2B)
0x1C    ch_v_mapping[2]      (2B)
0x1E    phase_resistance[2]  (8B, float×2)
0x26    crc32                (4B)
        总计约 50 字节
```

每次校准完成后自动调用 `CALIB_FlashSave()` 写入。上电时 `main.c` 中加载：

```
CALIB_FlashLoad → 检查 magic + CRC32
  → 有效: 应用 enc_direction/phase_comp/zero_offset/ch_u/ch_v
  → 无效: 使用代码默认值，串口打印 "No valid calibration in flash"
```

**安全保证**：CRC32 + STM32G4 硬件 ECC 双重校验。Flash 写入仅在电机停转时进行。

---

## 并发安全

| 机制 | 实现 |
|------|------|
| **busy 锁** | `CALIB_TryLock()` — 同一时刻只允许一个校准实验运行 |
| **遥测抑制** | `CALIB_IsBusy()` — TaskCurrentLoop 检查后跳过串口发送，避免二进制帧干扰 printf |
| **abort 链** | 所有长延时（≥50ms）拆分为短延时 + `CALIB_IsAborted()` 检查 |
| **abort 清理** | `calib_abort_cleanup()` — 统一执行：停 PWM → 禁能 → 恢复 PI → 解锁 |
| **ISR 安全** | 校准写入 `virtual_angle`/`id_ref` (float, 32-bit)，ISR 读取——Cortex-M4 原子保证 |

---

## 校准参数 (`s` 命令输出)

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

`valid: NO` 表示 Flash 中没有有效校准数据（首次使用或 Flash 被擦除）。

---

## 相关文件

| 文件 | 说明 |
|------|------|
| `Core/Inc/calibration.h` | API 声明, CalibParams_t 结构 |
| `Core/Src/calibration.c` | 5 个校准函数 + Flash 持久化 + 并发安全 |
| `Core/Src/foc.c` | FOC_Init() 中加载校准参数 |
| `Core/Src/current_ctrl.c` | Park 角度 virtual/encoder 分叉 |
| `Core/Src/main.c` | 启动序列中加载 Flash 校准 |
| `Core/Src/app_freertos.c` | StartDefaultTask CLI 命令处理 + 遥测抑制 |
| `Core/Src/mt6701.c` | enc_direction 读写接口 |
| `Core/Src/ina240.c` | zero_offset 读写接口 |
| `STM32G431XX_FLASH.ld` | Flash 最后一页保留为 CALIB_FLASH 区域 |
| `docs/FOC_DEBUG_EXPERIENCE.md` | FOC 调试经验与故障速查 |
| `docs/HARDWARE_CONNECTIONS.md` | 硬件连接参考 |
