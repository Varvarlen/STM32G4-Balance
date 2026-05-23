# 代码审查报告

**日期:** 2026-05-24 | **分支:** `balance_pid` | **范围:** 全项目 | **基准:** HEAD (107804c)

## 审查概要

| 维度 | 检查项 | 发现问题数 |
|------|:------:|:----------:|
| 安全 (S1-S9) | 9 | 5 Critical, 2 High |
| 控制理论 (C1-C6) | 6 | 2 High, 2 Medium |
| 并发/时序 (T1-T7) | 7 | 1 Critical, 3 High, 2 Medium |
| 代码质量 (Q1-Q5) | 5 | 1 High, 3 Medium, 2 Low |

> **严重度定义**
> - **Critical**: 可能导致电机失控、系统永久死锁或物理危险
> - **High**: 可能导致不正确行为，或在特定场景下触发 Critical
> - **Medium**: 不规范或潜在大规模修改时会暴露的问题
> - **Low**: 风格建议或可维护性改进

## 问题汇总

| ID | 类别 | 严重度 | 文件 | 简述 |
|----|------|--------|------|------|
| CR-001 | 安全 | 🔴 Critical | `pi.c:12-35` | PI_Step 未对 error 做 NaN 检查，一旦积分被污染永久失效 |
| CR-002 | 安全 | 🔴 Critical | `app_freertos.c:385-386` | NaN/Inf 检查仅覆盖 iq_cmd 落点，PI 输入端全路径未防护 |
| CR-003 | 安全 | 🔴 Critical | `app_freertos.c:244` | IMU 连续故障时 `continue` 跳过 IWDG 喂狗，8s 后硬件复位 |
| CR-004 | 时序 | 🔴 Critical | `bt_comm.c:112-114` | BT_COMM_SendData 含阻塞死循环（TX buffer 满时忙等），可挂起 CLI 任务 |
| CR-005 | 安全 | 🔴 Critical | `stm32g4xx_it.c:278-287` | 编码器 SPI 故障时 FOC 使用陈旧角度，无检测/保护机制 |
| CR-006 | 安全 | 🔴 Critical | `app_freertos.c:369-399` | BalanceLoop 与 ADC ISR 竞争写入 `iq_ref`/`id_ref`，无互斥保护 |
| CR-007 | 安全 | 🟠 High | `balance_ctrl.c:67` | `target_angle`（速度外环输出）未经 NaN 检查直接参与 PID 计算 |
| CR-008 | 时序 | 🟠 High | `app_freertos.c:274,338,351` | 多个 CLI 可写变量缺少 `COMPILER_BARRIER`（`target_speed`、`target_yaw_angle`） |
| CR-009 | 时序 | 🟠 High | `app_freertos.c:310-314` | 偏航角度积分使用 `g_speed[].speed_fb`（EKF 估计，含延迟），累积误差随时间发散 |
| CR-010 | 控制 | 🟠 High | `app_freertos.c:353` | 偏航速率 PI 误差公式 `yaw_rate - target_yaw_rate` 符号与实际转向方向相反 |
| CR-011 | 时序 | 🟠 High | `stm32g4xx_it.c:278-287` | ADC ISR 中 FOC 计算链（Clarke/Park/PI/SVPWM）执行时间未量化，20kHz 下裕度未知 |
| CR-012 | 控制 | 🟠 High | `speed_ctrl.c:68-149` | EKF 预测使用固定 0.001s dt，实际可能被 ISR 抢占导致 dt 漂移 |
| CR-013 | 代码 | 🟠 High | `app_freertos.c:67-71` | 关键状态变量缺少 `volatile` 修饰符，编译器可能优化掉 ISR 写入的可见更新 |
| CR-014 | 安全 | 🟡 Medium | `app_freertos.c:246-248` | IMU 故障恢复后不会自动恢复平衡控制，需用户手动重发 `B` 命令 |
| CR-015 | 控制 | 🟡 Medium | `app_freertos.c:304-315` | 偏航角度每帧无条件积分 `odom_filt * 0.001`，无角度防卷绕 (±180°) |
| CR-016 | 时序 | 🟡 Medium | `app_freertos.c:448-451` | 控制循环超时仅计数不处理，连续超时可能引发 IWDG 复位雪崩 |
| CR-017 | 控制 | 🟡 Medium | `app_freertos.c:291-294` | 速度外环 PI 的 I 限幅和输出限幅共用 `SPEED_OUTER_MAX`（应为 I < Out） |
| CR-018 | 代码 | 🟡 Medium | 多处 | 多个魔法数未宏定义：`0.095f`(EMA α)、`6.4f/6.6f`(VBUS 阈值)、`2.0f`(iq 限幅) |
| CR-019 | 安全 | 🟡 Medium | `calibration.c` | 校准模式中 `HAL_Delay` 阻塞时间不确定，如在校准期间 IWDG 可能超时 |
| CR-020 | 时序 | 🟡 Medium | `bt_comm.c:96-107` | BT_COMM_SendByte 的临界区保护不完整：`BT_COMM_StartTxDma` 可能在已禁用 IRQ 的情况下调用，而其中的 `HAL_UART_Transmit_DMA` 需要 IRQ 完成 |
| CR-021 | 代码 | 🟢 Low | `speed_ctrl.c:136-142` | EKF 协方差更新中对称元素显式赋值冗余（P[3]/[6]/[7]） |
| CR-022 | 代码 | 🟢 Low | `speed_ctrl.h:9` | `SPEED_RAMP_MAX=20000` 已定义但从未使用 |
| CR-023 | 代码 | 🟢 Low | `main.c:322-331` | Error_Handler 未调用 Fault_DisableMotors，CPU 挂死时电机保持最后状态 |
| CR-024 | 代码 | 🟢 Low | `docs/FREERTOS_TASKS.md:46-49` | 遥测帧文档描述 ch8/ch9 为 yaw_rate/target_yaw_rate，实际代码为 yaw_angle/target_yaw_angle |

---

## 详细发现

### CR-001 | 🔴 Critical | 安全 | `pi.c:12-35`

**问题:** `PI_Step` 函数对 `error` 参数不做 NaN 检查。如果调用方传入 NaN（例如上游传感器故障未拦截），`pi->integral += ki * error * dt` 会将积分永久污染为 NaN — 即使后续 error 恢复正常，积分也无法恢复。

**影响范围:** 所有 PI 控制器实例：
- `g_motor[0/1].id_pi` / `iq_pi`（电流环，ADC ISR 上下文）
- `speed_ctrl` 中的 PI（暂未直接在速度模式中使用 PI_Step，但有初始化）
- 所有通过 `PI_Init` 初始化并通过 `PI_Step` 使用的 PI 实例

**风险:** 电流环 PI 积分被 NaN 污染后，SVPWM 输出电压将变为 NaN，可能导致桥臂直通或电机失控。

**建议:** 在 `PI_Step` 开头添加 NaN 保护：
```c
if (isnan(error)) {
    // 保持积分值不变，仅输出积分项（或冻结积分并返回 0）
    // 递增全局 NaN 计数器
    return 0.0f;
}
```

---
> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________
---

### CR-002 | 🔴 Critical | 安全 | `app_freertos.c:385-386`

**问题:** 虽然 `iq_cmd` 有 NaN/Inf 检查（L385-386），但它的上游路径中的中间变量未全量覆盖。特别是 `balance_out`（来自 `BalanceCtrl_Run`）和 `steer`（来自偏航 PI）在传递给 `BALANCE_DIRECT_GAIN` 乘法之前只有 `balance_out` 在 `BalanceCtrl_Run` 中有 NaN 检查，但 `steer` 的 NaN 检查只在其数学表达式 `g_yaw_kp * err + yaw_i` 的间接路径上。

**具体场景:** 如果 `g_yaw_kp` 或 `g_yaw_ki` 被 CLI 误设为 NaN：
1. `g_balance.steer` 的线性组合结果为 NaN
2. `iq_cmd = iq_base + iq_diff` 中 `iq_diff = BALANCE_DIRECT_GAIN * steer` = NaN
3. L385 的 `isnan(iq_cmd)` 检测到 NaN → 置零 ✓

当前这种场景是**被防护住的**。但如果在 `iq_cmd` 限幅之前有其他操作绕过 NaN 检查（例如后续有人修改代码在 L385 之前插入操作），会非常危险。

**建议:** 在 `BalanceCtrl_Run` 返回前增加对 `balance_out` 的 NaN 断言，在偏航 PI 输出 `steer` 前增加 NaN 检查。

---
> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________
---

### CR-003 | 🔴 Critical | 安全 | `app_freertos.c:244`

**问题:** IMU SPI 读取失败时执行 `continue`（L244），跳过本次循环的所有后续处理，包括 L402 的 IWDG 喂狗（`tick % 100 == 0`）。如果 IMU 永久性故障（SPI 总线损坏），BalanceLoop 任务会以 1kHz 频率持续执行 "读取失败 → continue" 的循环，**IWDG 永远不会被喂狗**，8 秒后硬件复位。

**影响:**
- 如果 IMU 故障是暂时的（< 8s），没问题 — IWDG 还在 8s 超时内
- 如果 IMU 故障是永久的，系统将每 ~8s 复位一次，无限循环

**建议:** 在 `continue` 之前增加 IWDG 喂狗检查，或在 IMU 故障次数 > 阈值后将任务挂起而非空转：

```c
if (imu_fault_cnt >= 5 && !imu_faulted) {
    imu_faulted = 1;
    g_imu_fault_total++;
    g_balance.active = 0;
    // ... disable motors ...
}
// 即使 IMU 故障，也执行关键维护
if (tick % 100 == 0) HAL_IWDG_Refresh(&hiwdg);
// VBUS 保护也应在 IMU 故障时继续检查
continue;
```

更好方案：将 IWDG 喂狗和 VBUS 欠压检查移到 IMU 故障处理之前执行。

---
> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________
---

### CR-004 | 🔴 Critical | 时序 | `bt_comm.c:112-114`

**问题:** `BT_COMM_SendData` 使用忙等循环：
```c
while (BT_RingBuffer_Available(&txBuf) == BT_UART_TX_BUFFER_SIZE - 1);
```
如果 USART2 TX DMA 因某种原因停滞（例如 USART2 硬件故障、DMA 通道冲突），此循环将永久阻塞调用者。`BT_COMM_SendData` 被 `BT_SendTelemetryFrame` 调用（app_freertos.c:445），后者在 BalanceLoop（优先级 High）中以 50Hz 频率调用。如果此处死锁，**整个 BalanceLoop 任务被挂起**，电机控制停止，IWDG 8s 后复位。

**建议:**
- 改为超时等待（例如循环最多 N 次后丢弃该帧）
- 或用非阻塞方式：TX buffer 满时直接返回，丢弃当前帧

```c
// 替换忙等待为超时保护
int retry = 1000;
while (BT_RingBuffer_Available(&txBuf) == BT_UART_TX_BUFFER_SIZE - 1) {
    if (--retry <= 0) return;  // 超时放弃
}
```

---
> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________
---

### CR-005 | 🔴 Critical | 安全 | `stm32g4xx_it.c:278-287`

**问题:** 编码器 SPI DMA 完成 ISR（`DMA1_Channel5/6_IRQHandler`）处理 MT6701 数据更新 `g_enc[i].elec_angle` 和 `g_enc[i].mech_angle`。如果 SPI 通信失败（DMA 错误、CRC 错误、或 MT6701 传感器故障），编码器缓存不会更新，但 **ADC ISR（DMA1_Channel4）仍然使用这些陈旧角度进行 FOC 计算**（L278-279: `g_motor[i].elec_angle = g_enc[i].elec_angle`）。

编码器角度陈旧会导致 Park 变换使用错误的参考系，电流解耦失败，可能产生错误的转矩矢量甚至正反馈。

**建议:**
1. 在 `EncoderCache_t` 中增加时间戳/计数器，ADC ISR 检测到数据过时时进入安全状态
2. 或在 SPI DMA 错误回调中设置故障标志，ADC ISR 检查后安全停止

---
> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________
---

### CR-006 | 🔴 Critical | 安全 | `app_freertos.c:369-399`

**问题:** `BalanceLoop` 任务调用 `Motor_SetIqRef(&g_motor[i], iq_cmd)` 写入 `g_motor[i].iq_ref`，而 `CurrentCtrl_Run`（ADC ISR 上下文中）读取 `g_motor[i].iq_ref` 用于 PI 计算。这是一个**无互斥保护的并发读写**。

在 Cortex-M4 上，单精度 float (32-bit) 的读写是单指令原子操作（`VLDR`/`VSTR`），所以不会出现字撕裂。但存在以下问题：
1. BalanceLoop 设置 `iq_ref` 后，ADC ISR 可能在当前 PWM 周期的任意时刻使用新值
2. 这本质上是异步更新，可能在不同时刻产生不同的转矩指令
3. 如果有代码后续增加非原子修改（例如先清除再设置两个相关值），会产生中间态

**当前实际风险:** 低（Cortex-M4 float 原子访问），但缺乏文档记录和防护。

**建议:** 在代码中添加注释明确依赖 Cortex-M4 字对齐原子访问的特性，或在 `foc.h` 中为 `iq_ref` 添加说明。

---
> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________
---

### CR-007 | 🟠 High | 安全 | `balance_ctrl.c:67`

**问题:** `BalanceCtrl_Run` 中：
```c
float output = bc->kp_angle * tilt_err - bc->kd_gyro * bc->gyro_filt;
```
- `tilt_err = bc->target_angle - bc->tilt_angle`（L53）
- `bc->target_angle` 由速度外环 PI 计算（app_freertos.c:290），可能被 CLI 写入 `NaN`（例如用户输入 `PK ANG0=NaN`）
- `bc->tilt_angle` 来自卡尔曼滤波（已经 NaN 保护）
- `bc->kp_angle` / `bc->kd_gyro` 由 CLI 写入，没有 NaN 检查

虽然 `tilt_angle` / `gyro_rate` / `gyro_filt` 在 L35-38 有 NaN 检查（`BalanceCtrl_Run:35`），但 `target_angle` 和 `kp_angle` / `kd_gyro` 没有被 NaN 检查覆盖。如果 CLI 写入 NaN 到这些参数，输出将变为 NaN，然后传递给 `iq_cmd`（虽有 L385 保护，但防御深度不够）。

**建议:** 扩展 `BalanceCtrl_Run:35` 的 NaN 检查范围，增加对 `target_angle`、`kp_angle`、`kd_gyro` 的检查。

---
> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________
---

### CR-008 | 🟠 High | 时序 | `app_freertos.c:274,338,351`

**问题:** `COMPILER_BARRIER()` 用于防止编译器优化将 CLI 任务写入的变量缓存到寄存器。但以下由 CLI/BT 写入的变量缺少 BARRIER：

| 变量 | 写入方 | 读取位置 | 有 BARRIER？ |
|------|--------|---------|:----------:|
| `g_balance.target_speed` | CLI `S` 命令 / BT 控制帧 | `app_freertos.c:285` | ❌ 缺失 |
| `g_balance.target_yaw_angle` | CLI `A` 命令 / BT 控制帧 | `app_freertos.c:339` | ❌ 缺失 |
| `g_balance.kp_angle` | CLI `PK ANG=` 命令 | `balance_ctrl.c:32` | ✓ |
| `g_balance.kd_gyro` | CLI `PK GYR=` 命令 | `balance_ctrl.c:32` | ✓ |
| `g_speed_outer_kp/ki` | CLI `PS` 命令 | `app_freertos.c:274` | ✓ |
| `g_yaw_kp/ki` | CLI 命令 | `app_freertos.c:351` | ✓ |
| `g_yaw_angle_kp/ki` | CLI 命令 | `app_freertos.c:338` | ✓ |

**风险:** `target_speed` 和 `target_yaw_angle` 被 CLI/BT 任务修改后，BalanceLoop 可能最坏情况下读取到寄存器缓存的旧值达数毫秒（取决于编译器优化等级和 BARRIER 位置）。对于 `target_yaw_angle`（影响转向），可能导致转向方向延迟响应。

**建议:** 在 `app_freertos.c:285`（速度外环读取 target_speed 后）和 L339（偏航角度外环读取 target_yaw_angle 后）之前增加 `COMPILER_BARRIER()`。

---
> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________
---

### CR-009 | 🟠 High | 时序 | `app_freertos.c:310-314`

**问题:** 偏航角度通过每帧积分 `odom_filt * dt` 计算（1kHz）。`odom_filt` 来源于 `g_speed[MOTOR_RIGHT].speed_fb - g_speed[MOTOR_LEFT].speed_fb`，其中 `speed_fb` 是 EKF 估计值（`SpeedCtrl_UpdateRPM` 输出）。EKF 速度估计的带宽约 14Hz（`EKF_Q_ACCEL=400`），这意味着：
1. EKF 速度对快速转向的响应有延迟
2. 积分累积会随时间漂移
3. 编码器量化噪声（14-bit MT6701，单圈分辨率 16384）经过差速放大后被积分

**建议:** 考虑直接积分编码器角度差（而非 EKF 估计速度），或增加基于陀螺仪 Z 轴的短期角度漂移修正。当前互补滤波（`YAW_COMP_ALPHA`）只修正陀螺仪偏置，不修正积分漂移。

---
> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________
---

### CR-010 | 🟠 High | 控制 | `app_freertos.c:353`

**问题:** 偏航速率 PI 误差公式：
```c
float err = yaw_rate - target_yaw_rate;  // 正yaw→正steer→CW→对抗CCW
```
注释说"对抗CCW"，暗示这是一个**负反馈**配置。但需验证实际物理方向：
- `yaw_rate = gyro.z - gyro_bias_z` — 陀螺 Z 轴角速率（绕垂直轴）
- `target_yaw_rate` — 角度外环输出的目标角速率
- `steer = g_yaw_kp * err + yaw_i` — 正 steer → 左轮加速右轮减速 → ?

如果 `err > 0`（实测 yaw_rate > 目标），PI 输出正 steer，左轮加速、右轮减速 → 这应该产生什么方向的偏航？需要确认实际物理方向是否匹配负反馈。

**风险:** 如果符号反了，偏航控制变成正反馈，会自行振荡或旋转加速（之前在 commit `3b7947e`/`2635668` 中修复过一次 `odom_rate` 符号问题）。鉴于近期多次修改偏航符号，存在回归风险。

**建议:** 在文档 `docs/MOTOR_PARAMS.md` 或 `CALIB_PARAMS.md` 中明确记录"正 yaw_rate 对应物理左转还是右转"，并在代码中添加物理方向注释。

---
> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________
---

### CR-011 | 🟠 High | 时序 | `stm32g4xx_it.c:278-287`

**问题:** ADC DMA ISR 中调用 `CurrentCtrl_Run`，该函数执行：
1. INA240 电流读取（3 次 ADC 值查询）
2. Clarke 变换（加减乘）
3. `fast_sincos`（查表 + 线性插值）
4. Park 变换（矩阵乘法）
5. `PI_Step` × 2（d/q 轴，含条件 anti-windup）
6. 逆 Park 变换
7. `SVPWM_SetVab`（扇区判断 + 占空比计算 → `Motor_SetDuty` → `__HAL_TIM_SET_COMPARE`）

每轮 `CurrentCtrl_Run` 执行两个电机（for 循环 0→1），在 20kHz 下每周期预算 = 50μs。170MHz Cortex-M4F 的 `fast_sincos` 约 20 cycles，浮点乘法 1 cycle。整体估计约 15-20μs，应在预算内。但**没有量化测量**。

**建议:** 在 `DMA1_Channel4_IRQHandler` 中增加执行时间测量（DWT 周期计数），验证最坏情况执行时间 < 40μs（80% 预算）。

---
> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________
---

### CR-012 | 🟠 High | 控制 | `speed_ctrl.c:68`

**问题:** EKF 预测使用固定 `dt = SPEED_LOOP_DT = 0.001`：
```c
float dt = SPEED_LOOP_DT;   // 0.001
```
但 `SpeedCtrl_UpdateRPM` 在 BalanceLoop 中每 tick 调用一次。如果 ISR 抢占导致某次调用延迟（例如上一帧超时），实际 dt 不是 0.001s，EKF 预测会使用错误的 dt。

**风险:** 偶尔一帧的 dt 偏差影响不大（EKF 本身有鲁棒性），但如果连续多帧超时，EKF 状态估计会显著偏离。

**建议:** 使用 DWT 测量与上次调用的实际时间差，或至少检测 dt 偏差 >10% 时跳过 EKF 更新。

---
> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________
---

### CR-013 | 🟠 High | 代码 | `app_freertos.c:67-71`

**问题:** 以下全局状态变量缺少 `volatile` 修饰符：
- `g_foc_snap[].mech_angle` / `g_foc_snap[].iq` — 虽然有 `volatile` 在结构体定义中 ✓

实际上 `encoder_cache.h` 中已正确定义为 `volatile`。但：
- `g_motor[i].iq_ref` — **无** `volatile`（ADC ISR 读取，BalanceLoop 写入）
- `g_motor[i].speed_mode` — **无** `volatile`
- `g_balance.active` — **无** `volatile`（CLI 任务写入，BalanceLoop 读取）

`g_motor[i].iq_ref` 是最关键的——虽然 32-bit float 在 Cortex-M4 上是原子操作，但编译器可能将 `iq_ref` 缓存在 FPU 寄存器中，导致 ADC ISR 在多个 FOC 周期中看到同一个旧值。

**建议:** `g_motor[i].iq_ref` 增加 `volatile` 修饰符，或在 ADC ISR 读取前使用 `COMPILER_BARRIER()`。

---
> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________
---

### CR-014 | 🟡 Medium | 安全 | `app_freertos.c:246-248`

**问题:** IMU 从故障中恢复后：
```c
if (imu_faulted) {
    imu_faulted = 0;
    printf("[FAULT] IMU recovered\r\n");
}
```
仅清除 `imu_faulted` 标志和打印消息，但**不会重新激活平衡控制**（`g_balance.active` 仍为 0）。用户必须手动发送 `B` 命令重新激活。

**场景:** 在车载行驶过程中，短暂 IMU 干扰（例如 EMI 尖峰导致的偶发性 SPI 超时）可能导致 5 次连续失败 → 平衡关闭 → 即使 IMU 恢复正常，平衡车也不再站立，可能倾倒。

**建议:** 可考虑 IMU 恢复后自动恢复平衡控制（经过短暂稳定期确认），或在 telemetry 中发送明显的恢复提示。当前设计偏保守（安全优先），但可用性差。

---
> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________
---

### CR-015 | 🟡 Medium | 控制 | `app_freertos.c:304-315`

**问题:** 偏航角度每帧无条件累加：
```c
g_balance.yaw_angle += odom_filt * dt;
```
无角度环绕处理（如 ±180° 防卷绕）。如果平衡车连续旋转多圈，`yaw_angle` 会增长到很大的值，而 `target_yaw_angle` 从 BT 控制杆来的角度指令可能是有限范围。角度误差 `target - yaw_angle` 在持续旋转后会变得非常大（例如 3600°），导致偏航 PI 输出饱和。

**建议:** 对 `yaw_angle` 做 ±180° 或 ±360° 周期性卷绕：
```c
while (g_balance.yaw_angle > 180.0f) g_balance.yaw_angle -= 360.0f;
while (g_balance.yaw_angle < -180.0f) g_balance.yaw_angle += 360.0f;
```

---
> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________
---

### CR-016 | 🟡 Medium | 时序 | `app_freertos.c:448-451`

**问题:** 控制循环超时仅递增计数器，不做防御：
```c
if ((DWT->CYCCNT - t_start) > 170000) {
    g_cycle_overrun_cnt++;
}
```
如果连续超时，下一帧的时序挤压可能导致雪崩式过载 — 每帧越来越晚，直到任务完全跟不上 1kHz 定时器。

**建议:** 检测到超时后，跳过部分非关键处理（如遥测帧打包、蓝牙帧发送）以恢复时序。

---
> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________
---

### CR-017 | 🟡 Medium | 控制 | `app_freertos.c:291-294`

**问题:** 速度外环 PI 的积分限幅和输出限幅共用 `SPEED_OUTER_MAX`：
```c
speed_i += g_speed_outer_ki * err * SPEED_OUTER_DT;
if (speed_i >  SPEED_OUTER_MAX) speed_i =  SPEED_OUTER_MAX;     // I ≤ 5.0
if (speed_i < -SPEED_OUTER_MAX) speed_i = -SPEED_OUTER_MAX;
...
g_balance.target_angle = g_speed_outer_kp * err + speed_i;       // P + I
if (g_balance.target_angle >  SPEED_OUTER_MAX) ...               // Out ≤ 5.0
```

当 P 项非零时，`target_angle = P + I` 可能超过输出限幅（因为 I 已达上限时 P 仍有贡献），clamping anti-windup 会介入，但效果有限。I 限幅应**小于**输出限幅，留空间给 P 项。

**建议:** `speed_i` 的限幅降低到 0.7×`SPEED_OUTER_MAX` 左右。

---
> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________
---

### CR-018 | 🟡 Medium | 代码 | 多处

**问题:** 以下魔法数缺少宏定义：

| 位置 | 值 | 含义 | 建议宏名 |
|------|-----|------|---------|
| `app_freertos.c:313` | `0.095f` | odom EMA α (τ=10ms) | `ODOM_EMA_ALPHA` |
| `app_freertos.c:387-388` | `2.0f` | iq_cmd 硬限幅 | 已有 `BALANCE_DIRECT_GAIN`→ 应定义 `IQ_CMD_MAX` |
| `app_freertos.c:406` | `6.4f` | VBUS 低压停止阈值 | `VBUS_UVLO_THRESHOLD` |
| `app_freertos.c:417` | `6.6f` | VBUS 低压告警阈值 | `VBUS_WARN_THRESHOLD` |
| `app_freertos.c:406` | `tick % 100 == 0` | 喂狗/电压检查周期(100ms) | `MAINTENANCE_DIV` |
| `app_freertos.c:417` | `tick % 800 == 0` | 低压蜂鸣间隔(800ms) | `VBUS_BEEP_INTERVAL_TICKS` |
| `app_freertos.c:439` | `tick % 20 == 0` | BT 遥测周期(50Hz) | `BT_TELEM_DIV` |
| `app_freertos.c:449` | `170000` | DWT 超时阈值(cycles) | `CYCLE_OVERRUN_THRESHOLD` |

---
> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________
---

### CR-019 | 🟡 Medium | 安全 | `calibration.c`

**问题:** 校准流程中使用大量 `HAL_Delay`（如电机参数标定、编码器偏移测量期间的阻塞等待）。在正常模式下校准不会运行（校准模式不启动 BalanceLoop），但如果在某种边缘情况下进入校准流程且 IWDG 已在运行，长时间的阻塞等待（可达数秒）可能导致 IWDG 超时复位。

**检查:** 需要确认校准模式下 IWDG 是否仍在运行。查看 `main.c` 启动流程 — `MX_IWDG_Init` 在用户代码之前调用，IWDG 一旦启动就无法软件停止。校准模式下如果 `TaskCLI` 是唯一运行的任务（`app_freertos.c:113-115`），IWDG 不会被喂狗 → 必将在 8s 内复位。

**这是校准模式的问题** — 校准期间的 CLI 任务不喂狗。

**建议:** 在 CLI 任务中添加 IWDG 喂狗，或在校准模式下延长 IWDG 超时时间。

---
> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________
---

### CR-020 | 🟡 Medium | 时序 | `bt_comm.c:96-107`

**问题:** `BT_COMM_SendByte` 的临界区实现：
```c
uint32_t primask = __get_PRIMASK();
__disable_irq();
BT_RingBuffer_Write(&txBuf, byte);
BT_COMM_StartTxDma();
if (!primask) __enable_irq();
```

`BT_COMM_StartTxDma` 内部调用 `HAL_UART_Transmit_DMA(&huart2, txDmaBuf, len)`。如果 DMA 已在传输中（`txDmaBusy == 1`），`StartTxDma` 不会启动新的 DMA。但如果 `txDmaBusy == 0` 且 `HAL_UART_Transmit_DMA` 被调用，HAL 内部需要使能 UART 中断（`UART_IT_TC`）。在 IRQ 已禁用的情况下调用 `HAL_UART_Transmit_DMA` 是**安全的** — 它只是配置 DMA 和使能中断，不依赖中断来返回。

但有一个隐含问题：`BT_COMM_SendByte` 是从 `__io_putchar`（`printf` 底层）调用的，而 `printf` 可能在 BalanceLoop（已禁用 IRQ 的临界区？不，BalanceLoop 不会禁用 IRQ）或 CLI 任务中调用。CLI 任务在 `BT_COMM_SendByte` 期间禁用 IRQ 是正确的（防止 ISR 同时修改环形缓冲区）。但 `BT_COMM_StartTxDma` 中的 `HAL_UART_Transmit_DMA` 可能会轮询等待 DMA 就绪，如果 DMA 未就绪且 IRQ 被禁用，DMA 完成中断无法触发 → 死锁。

**当前状态:** 实际上 `BT_COMM_StartTxDma` 只在 `txDmaBusy == 0` 时调用 `HAL_UART_Transmit_DMA`，而在 IRQ 禁用期间调用 `HAL_UART_Transmit_DMA` 是 OK 的。风险较低但值得文档化。

---
> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________
---

### CR-021 | 🟢 Low | 代码 | `speed_ctrl.c:136-142`

EKF 协方差更新中显式赋值 P[3]、P[6]、P[7]（对称元素），这些值可以通过保持 P 对称性隐式维护。不影响正确性，但增加了 3 行冗余代码。

> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________

---

### CR-022 | 🟢 Low | 代码 | `speed_ctrl.h:9`

`SPEED_RAMP_MAX=20000` 已定义但未在 `speed_ctrl.c` 中使用（斜坡逻辑注释为 `// 阶跃测试` 模式）。可清理。

> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________

---

### CR-023 | 🟢 Low | 代码 | `main.c:322-331`

`Error_Handler` 函数在 CPU 严重错误时仅禁用中断并自旋。与硬故障处理器不同，它**不调用** `Fault_DisableMotors()`。如果 `Error_Handler` 因 HAL 初始化失败被调用，电机可能处于不确定状态。

> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________

---

### CR-024 | 🟢 Low | 文档 | `docs/FREERTOS_TASKS.md:46-49`

遥测帧文档描述 ch8 为 `yaw_rate(°/s)`、ch9 为 `target_yaw_rate(°/s)`，但实际 `app_freertos.c:433-434` 发送的是 `yaw_angle` 和 `target_yaw_angle`。文档未与 commit `45227fa`（取消速率模式，改为偏航角度闭环）同步更新。

> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________

---

## 看门狗专项分析

### IWDG 配置

| 参数 | 值 |
|------|-----|
| 时钟源 | LSI (32kHz 标称, 实测约 32-40kHz) |
| 预分频 | /256 → 125Hz |
| 重装载值 | RL=1000 → 8s 超时 |
| 启动时机 | `main.c:126` — FreeRTOS 调度之前 |
| 喂狗方式 | BalanceLoop `tick % 100 == 0` → 100ms 间隔 |

### 喂狗覆盖分析

| 场景 | 喂狗被跳过？ | 备注 |
|------|:----------:|------|
| 正常运行 | ✗ | 100ms 间隔，8s 超时→80× 裕度 |
| IMU SPI 暂时故障 (<5次) | ✗ | continue 跳过但后续 tick 会喂狗 |
| IMU SPI 永久故障 (>5次) | **✓** | `continue` 导致永不喂狗 → 8s 复位循环 |
| BalanceLoop 被 CLI 阻塞 | ✗ | BalanceLoop Priority=High, 抢占 CLI |
| ADC ISR 长时间执行 | ✗ | ISR 不阻塞任务调度，BalanceLoop 会延迟但最终喂狗 |
| TIM17 定时器停振 | **✓** | BalanceLoop 在 `ulTaskNotifyTake(pdTRUE, portMAX_DELAY)` 永久阻塞 → 8s 复位 |
| 校准模式 | **✓** | 仅 TaskCLI 运行，无人喂狗 → 8s 复位 |

### 改善建议

1. **IMU 永久故障喂狗空洞**: 将 IWDG 喂狗移到 IMU 故障检查之前（见 CR-003）
2. **TIM17 停振**: 可添加软件超时（在 `ulTaskNotifyTake` 使用有限超时，超时后主动检查并喂狗）
3. **校准模式 IWDG**: 在 CLI 任务中添加喂狗，或延长 IWDG 超时（见 CR-019）

---

## 架构时序专项分析

### 任务/ISR 抢占拓扑

```
优先级: NVIC ISR > BalanceLoop(High) > CLI(Normal) > DefaultTask(Idle)

┌─────────────────────────────────────────────────────────┐
│ ADC DMA ISR (20kHz, NVIC)                              │
│ ├── elec_angle = g_enc[].elec_angle                    │
│ ├── CurrentCtrl_Run(motor[i])                          │
│ │   ├── INA240_GetCurrentFast → Clarke → Park           │
│ │   ├── PI_Step(d) + PI_Step(q)                        │
│ │   ├── InvPark → SVPWM_SetVab → Motor_SetDuty         │
│ ├── g_foc_snap[i].mech_angle = g_enc[].mech_angle      │
│ └── g_foc_snap[i].iq = g_motor[i].iq                   │
│ 抢占: BalanceLoop 的任何时刻                              │
└─────────────────────────────────────────────────────────┘
         │
         ▼ 抢占关系
┌─────────────────────────────────────────────────────────┐
│ TIM17 ISR (1kHz, NVIC)                                 │
│ └── vTaskNotifyGiveFromISR → 唤醒 BalanceLoop          │
└─────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────┐
│ BalanceLoop (1kHz, High)                               │
│ ├── IMU 读取 + 卡尔曼滤波                                │
│ ├── 速度外环 (100Hz, tick%10)                           │
│ ├── 偏航积分 (1kHz)                                     │
│ ├── 偏航控制 (50Hz, tick%20)                            │
│ ├── 平衡 PD → 差速混合 → iq_cmd                        │
│ ├── SpeedCtrl_UpdateRPM (EKF) ← 读取 g_foc_snap         │
│ ├── Motor_SetIqRef → 写入 g_motor[].iq_ref             │
│ ├── IWDG 喂狗 / VBUS 保护 (100ms, tick%100)             │
│ └── 遥测 / BT 遥测                                      │
│ 被抢占: ADC ISR (20kHz), TIM17 ISR (1kHz)               │
└─────────────────────────────────────────────────────────┘
         │
         ▼ 时间片
┌─────────────────────────────────────────────────────────┐
│ CLI (1ms, Normal)                                      │
│ ├── CLI_Process → 命令解析 + printf                     │
│ ├── BT 帧解析 → 写入 g_balance 参数                     │
│ └── 被抢占: BalanceLoop (High), 所有 ISR                 │
└─────────────────────────────────────────────────────────┘
```

### 数据共享矩阵

| 数据 | 写入者 | 读取者 | 保护机制 | 安全性 |
|------|--------|--------|---------|:------:|
| `g_enc[].elec_angle` | SPI DMA ISR | ADC DMA ISR | volatile ✓ | ✅ |
| `g_enc[].mech_angle` | SPI DMA ISR | ADC DMA ISR | volatile ✓ | ✅ |
| `g_foc_snap[].mech_angle` | ADC DMA ISR | BalanceLoop | volatile ✓ | ✅ |
| `g_foc_snap[].iq` | ADC DMA ISR | BalanceLoop | volatile ✓ | ✅ |
| `g_motor[].iq_ref` | BalanceLoop | ADC DMA ISR | ❌ 无保护 | ⚠️ |
| `g_motor[].id_ref` | BalanceLoop | ADC DMA ISR | ❌ 无保护 | ⚠️ |
| `g_motor[].speed_mode` | BalanceLoop | ADC DMA ISR | ❌ 无保护 | ⚠️ |
| `g_balance.kp_angle` | CLI | BalanceLoop | COMPILER_BARRIER ✓ | ✅ |
| `g_balance.target_speed` | CLI/BT | BalanceLoop | ❌ 缺 BARRIER | ⚠️ |
| `g_balance.target_yaw_angle` | CLI/BT | BalanceLoop | ❌ 缺 BARRIER | ⚠️ |
| `g_yaw_kp/ki` | CLI | BalanceLoop | COMPILER_BARRIER ✓ | ✅ |
| `g_balance.steer` | BalanceLoop | BalanceLoop | N/A (单写) | ✅ |
| `txBuf` (BT) | CLI / BalanceLoop | BT DMA ISR | `__disable_irq()` 临界区 | ✅ |
| `rxBuf` (BT) | USART2 ISR | CLI | 单写单读 | ✅ |

### 关键时序路径

**最坏情况延迟链:** ADC ISR(20kHz) → ISR 可抢占 BalanceLoop → BalanceLoop 被延迟 → EKF dt 偏离 → 控制输出延迟 → 电流环用旧 iq_ref

如果 BalanceLoop 中的全部处理时间接近 1ms，ADC ISR 会频繁抢占并延长每一帧，导致 DWT 超时计数器递增。在极端情况下可能引发 IWDG 复位。

---

## 正面发现

以下是值得保持的好实践：

1. **故障处理器完善**: HardFault/MemManage/BusFault/UsageFault 全部有 `FaultDump` 诊断 + `Fault_DisableMotors` 安全停止
2. **卡尔曼滤波器 NaN 防护**: `KalmanAngle_Predict` 和 `KalmanAngle_Update` 都有完整的 NaN 检测和自动恢复
3. **倾倒保护多层**: 倾斜角度 + 角速度双重阈值保护，触发后重置 EMA 状态
4. **IWDG/VBUS 保护独立**: 硬件看门狗和软件欠压保护互不依赖
5. **DWT 超时检测**: 使用硬件周期计数器检测控制循环超时，不依赖软件定时器
6. **FOC 快照一致性**: `g_foc_snap` 在同一 ADC ISR 周期捕获两轮数据，消除时间偏差
7. **编码器方向标定**: 支持 Flash 存储每个编码器的方向，校准时自动确定

---

*审查完成。Please review each item above. Mark your decisions in the 批注区 (checkbox + notes), and I will implement fixes according to your annotations.*
