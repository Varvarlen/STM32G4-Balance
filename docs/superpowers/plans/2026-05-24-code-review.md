# 代码审查实施计划

> **审查目标:** 全项目深度审查（安全 + 控制理论 + 并发时序 + 代码质量），产出 `docs/CODE_REVIEW.md`

**技术栈:** STM32G431 + FreeRTOS + FOC + 卡尔曼滤波 + 级联 PID

---

### Task 0: 更新 GitNexus 索引

- [ ] **Step 1: 重建索引**

```bash
npx gitnexus analyze
```

### Task 1: Phase 1 — 影响分析

- [ ] **Step 1: 查看近期提交改动的符号**

用 GitNexus detect_changes 和 git diff 查看 balance_pid 分支与 main 的差异文件清单：

```bash
git diff main...HEAD --stat
```

- [ ] **Step 2: 对核心控制函数做上游 impact 分析**

对以下符号逐个调用 `gitnexus_impact({target, direction: "upstream"})`：
- `BalanceCtrl_Run`
- `SpeedCtrl_UpdateRPM`
- `SpeedCtrl_EnterMode`
- `StartBalanceLoopTask`
- `CurrentCtrl_Run`

记录每个符号的直接调用者和受影响的任务/ISR。

- [ ] **Step 3: 对全局状态变量做上游 impact**

对 `g_balance`、`g_speed`、`g_motor`、`g_foc_snap` 查询所有写入点和读取点。

- [ ] **Step 4: 标出改动密集 + 依赖密集的热区**

汇总 Step 1-3 结果，产出热区 Top-N 列表，指导 Phase 2 的资源倾斜。

### Task 2: Phase 2.1 — 安全审查 (S1-S9)

**检查文件:** `app_freertos.c`、`balance_ctrl.c`、`foc.c`、`current_ctrl.c`、`mpu6500.c`、`mt6701.c`、`motor_hal.c`、`stm32g4xx_it.c`、`adc.c`

- [ ] **S1: NaN/Inf 传播路径**

搜索所有 `isnan`/`isinf` 调用，确认：
1. 每个浮点运算路径在喂给 `Motor_SetIqRef` 前都有 NaN 检查
2. 所有 PID 积分累加前不对 error 做 NaN 检查
3. `atan2f`/`KalmanAngle_GetAngle` 返回值是否可能为 NaN

执行：
```
Grep: isnan|isinf  → 列出所有保护点的位置和覆盖变量
Grep: Motor_SetIqRef  → 逆向追踪 iq_cmd 计算过程中每个浮点运算是否有 NaN 防护
Grep: "\\+=" 在 app_freertos.c balance_ctrl.c  → 检查每个累加操作前的 NaN 检查
```

- [ ] **S2: 传感器故障降级状态机**

追踪 IMU 故障时 `g_balance.active` 和 `g_motor[i].speed_mode` 的完整状态转换：
```
Read: app_freertos.c:222-250 (IMU 故障处理块)
Read: app_freertos.c:296-316 (active 检查和安全停止)
```

检查：
1. IMU 恢复后 `imu_faulted` 标志清除 → 但 `g_balance.active` 是否仍为 0？
2. 编码器故障（MT6701 SPI 失败）是否有类似故障处理？
3. `g_foc_snap` 数据如果陈旧（编码器未更新）是否会被检测？

- [ ] **S3: 保护机制完备性**

检查三条独立保护路径：
1. 倾倒保护：`balance_ctrl.c:54-63` — `BALANCE_TILT_MAX=45°` 和 `BALANCE_GYRO_MAX=350°/s`
2. VBUS 欠压：`app_freertos.c:406-419` — `vbus <= 6.4f` 停止，`6.4 < vbus <= 6.6` 告警
3. IWDG：`app_freertos.c:402` — `tick % 100 == 0` 喂狗，8s 超时

检查：
1. 三条路径是否完全独立（一条失败不影响另外两条）
2. 倾倒保护触发后 `gyro_filt = 0.0f` 重置，再次 B 命令是否会因旧值误触
3. VBUS 保护中 `tick % 100 == 0` 嵌套判断是否合理

- [ ] **S4: ISR 安全性**

```
Read: stm32g4xx_it.c (ADC ISR / TIM ISR)
Read: current_ctrl.c (CurrentCtrl_Run — ADC ISR 中调用)
Read: foc.c (FOC 计算)
```

检查：
1. ADC ISR 中是否调用 `printf` 或任何阻塞函数
2. `CurrentCtrl_Run` 执行时间（FOC Clark/Park/PI/SVPWM）最坏情况
3. TIM17 ISR 是否只做 `portYIELD_FROM_ISR` / 信号量释放

- [ ] **S5: 栈溢出风险**

根据 `FREERTOS_TASKS.md`：
- TaskCLI: 512 words（正常模式）
- TaskBalanceLoop: 896 words

检查：
1. `StartBalanceLoopTask` 中最深的局部变量栈帧（`KalmanAngle_t kf` + 多个 static 数组）
2. `StartCLITask` 中 `CLI_Process()` 递归/嵌套深度
3. 是否有大局部 buffer（如 `char buf[N]`）

- [ ] **S6: 初始化顺序**

```
Read: main.c (SystemClock_Config → MX_GPIO_Init → MX_DMA_Init → ... → MX_FREERTOS_Init)
Read: app_freertos.c:90-121 (MX_FREERTOS_Init → 任务创建)
Read: app_freertos.c:157-210 (StartBalanceLoopTask 初始化)
```

检查：
1. MPU6500 SPI 在 RTOS 调度前是否已初始化
2. IWDG 在 `MX_IWDG_Init` 中启动 vs 任务创建时序
3. `g_foc_snap` 在 BalanceLoop 首次读取前是否已被 ADC ISR 填充

- [ ] **S7: IWDG 喂狗覆盖**

逐行追踪 `StartBalanceLoopTask` 中的所有退出路径：

```
Read: app_freertos.c:222-456 (完整主循环)
```

标注每个可能绕过 `tick % 100 == 0` 的路径：
1. `ulTaskNotifyTake(pdTRUE, portMAX_DELAY)` — 阻塞等待，如果 TIM17 停止 → IWDG 复位
2. IMU SPI timeout → `continue` — 到达 tick=100 时仍会喂狗 ✓
3. 倾倒保护 `g_balance.active = 0` — 循环继续运行 ✓
4. CLI 任务抢占 — BalanceLoop 被延迟但不会永久阻塞

- [ ] **S8: IWDG 喂狗周期合理性**

分析：
1. 喂狗间隔 = 100ms，IWDG 超时 = 8s，安全裕度 = 80x
2. 最坏情况：CLI 任务阻塞 + ADC ISR 抢占 + BalanceLoop 一周期超时
   - 单周期超时 ≈ 2ms（DWT 检测到超时但不会 stop）
   - CLI 最坏阻塞：`printf` 通过 230400 baud USART 输出长字符串
   - 100ms 间隔在最坏情况下 > 8s？需要量化计算

- [ ] **S9: IWDG 启动时机**

```
Grep: MX_IWDG_Init
Grep: hiwdg
```

检查：
1. `MX_IWDG_Init` 是在 `main()` 中哪个位置被调用
2. IMU 预热 `osDelay(10) × 20 = 200ms` 在 RTOS 任务中，此时 IWDG 已运行（需要被 BalanceLoop 任务喂狗）
3. BalanceLoop 任务创建后到首次喂狗之间：卡尔曼初始化(约 200ms) + 等待 TIM17 触发

### Task 3: Phase 2.2 — 控制理论审查 (C1-C6)

- [ ] **C1: 误差符号一致性**

逐个检查所有 PI/PID 误差公式：

| 位置 | 公式 | 预期符号 |
|------|------|---------|
| `balance_ctrl.c:53` | `tilt_err = target_angle - tilt_angle` | 正角度 → 正轮速（向前追） |
| `app_freertos.c:285` | `err = avg_speed - target_speed` | `target_angle` 输出符号（正 err→正 target_angle→前倾→加速 ✓）|
| `app_freertos.c:339` | `angle_err = target_yaw_angle - yaw_angle` | 正误差 → 正 rate 指令 |
| `app_freertos.c:353` | `err = yaw_rate - target_yaw_rate` | 正 rate → 正 steer → CW |

需要验证：PI 误差公式 `err = actual - target` 还是 `err = target - actual`，以及 PI 内部实现。

```
Read: pi.c  → 检查 PI_Process() 的误差处理
```

- [ ] **C2: 离散化精度**

验证各环路 dt 与实际调用频率一致：

| 宏 | 声明值 | 实际调用 | 频率 |
|----|--------|---------|------|
| `SPEED_LOOP_DT` | 0.001 | `speed_ctrl.c`（每 tick） | 1kHz ✓ |
| `SPEED_OUTER_DT` | 0.01 | `tick % 10 == 0` | 100Hz ✓ |
| `YAW_OUTER_DT` | 0.01 | `tick % 10 == 0` | 100Hz ✓ |
| `YAW_ANGLE_OUTER_DT` | 0.02 | `tick % 20 == 0` | 50Hz ✓ |

检查 `SPEED_LOOP_DT` 在 EKF 预测中的使用（speed_ctrl.c:68）—— 假设 1kHz，实际是否可能被其他 ISR 延迟？

- [ ] **C3: 级联带宽比**

计算各环路带宽并验证级联规则（内环 ≥ 2× 外环）：

| 环 | 频率 | 近似带宽 | 外环比 |
|----|------|----------|--------|
| FOC 电流 | ~20kHz PWM | ~2kHz | — |
| 平衡 PD | 1kHz | ~100Hz (Kd=1.2) | 内环 ✓ |
| 速度 PI (轮) | 1kHz (EKF) | ~4.4Hz (ωz) | 内/外 |
| 速度外环 | 100Hz | ~0.5Hz? | 外环 |
| 偏航速率 PI | 100Hz | ~? | 内环 |
| 偏航角度外环 | 50Hz | ~? | 外环 |

计算 PI 带宽：`ωz = Ki/Kp`，带宽度 ≈ 交叉频率。

- [ ] **C4: EMA/滤波器参数一致性**

所有 EMA 的 α 值：

| 位置 | α | dt | τ |
|------|---|----|---|
| `BALANCE_GYRO_EMA_ALPHA` | 0.181 | 1ms | 5ms |
| `odom_filt` (app_freertos.c:313) | 0.095 | 1ms | 10ms |
| `SPEED_EMA_ALPHA` | 0.393 | 1ms | 2ms |
| `YAW_COMP_ALPHA` | 0.01 | 10ms | 1s |

验证公式 `α = 1 - exp(-dt/τ)` 与每个声明值一致，检查是否有隐式硬编码 α 值。

- [ ] **C5: 积分限幅一致性**

| PI | I 限幅 | 输出限幅 | 一致性 |
|----|--------|---------|--------|
| 速度外环 I | `SPEED_OUTER_MAX=5.0` | `SPEED_OUTER_MAX=5.0` | I ≤ Out ✓ |
| 偏航角度 I | `YAW_ANGLE_MAX_I=30` | `YAW_ANGLE_MAX_RATE=150` | I ≤ Out ✓ |
| 偏航速率 I | `YAW_PI_MAX=50` | `YAW_PI_MAX=50` | I = Out — 应 I < Out |

- [ ] **C6: 单位转换系数**

| 系数 | 值 | 推导 | 正确性 |
|------|-----|------|--------|
| `RPM_PER_RADPS` | 9.5493 | 60/2π | ✓ |
| `BALANCE_DIRECT_GAIN` | 0.01 A/RPM | 经验值 | 需验证物理含义 |
| `YAW_RPM_TO_DPS` | 1.65 | 6×r/W | 需验证 r=2.75cm, W=10cm |
| `RAD_TO_DEG` | 57.2958 | 180/π | ✓ |
| `RPM_FROM_RADPS` | 9.5493 | 同 RPM_PER_RADPS | ✓ |

### Task 4: Phase 2.3 — 并发/时序审查 (T1-T7)

- [ ] **T1: 参数竞态 — COMPILER_BARRIER 覆盖**

列出 CLI 写入 + BalanceLoop 读取的所有共享变量对：

```
Grep: COMPILER_BARRIER  → 检查每个使用点的保护对象
Grep: extern.*g_  → 列出所有全局变量
```

核对：
- `g_balance.kp_angle/kd_gyro/target_angle/output_max` → `BalanceCtrl_Run:32` ✓
- `g_speed_outer_kp/g_speed_outer_ki` → `app_freertos.c:274` ✓
- `g_yaw_angle_kp/g_yaw_angle_ki` → `app_freertos.c:338` ✓
- `g_yaw_kp/g_yaw_ki` → `app_freertos.c:351` ✓
- `g_balance.target_speed` → 有 BARRIER 吗？
- `g_balance.target_yaw_angle` → 有 BARRIER 吗？

- [ ] **T2: 循环超时级联影响**

```
Read: app_freertos.c:448-451 (DWT 超时检测)
```

分析：
1. 检测到超时后只是 `g_cycle_overrun_cnt++`，没有其他处理
2. 下一周期是否因为上一周期超时而被挤压？
3. `ulTaskNotifyTake(pdTRUE, portMAX_DELAY)` — 如果处理时间 > 1ms，它会立即返回（因为 TIM17 已经发了通知）

- [ ] **T3: ADC ISR 抢占窗口**

```
Read: foc.c (FOC 计算链 — ADC ISR 上下文)
Read: app_freertos.c:369-399 (平衡输出 + Motor_SetIqRef)
```

分析：
1. ADC ISR 更新 `g_foc_snap[i].mech_angle` — BalanceLoop 读取同一位置
2. ADC ISR 调用 `CurrentCtrl_Run` → 修改 `g_motor[i].iq`、`g_motor[i].id`
3. BalanceLoop 在 step 5 设置 `Motor_SetIqRef` 后，ADC ISR 可能在下一次触发时覆盖

关键问题：BalanceLoop 的 set_iq 和 ADC ISR 的 current_ctrl 之间是否有 double-buffering？

- [ ] **T4: 定时器触发抖动**

```
Grep: htim17
Grep: htim1 (ADC 触发源)
```

分析：
1. TIM1 触发 ADC，ADC 完成触发 ISR — TIM17 触发 BalanceLoop
2. TIM1 和 TIM17 的时钟源和预分频，两者相位关系
3. TIM17 周期 = 1ms，TIM1 周期 = 1/PWM_freq

检查 CubeMX 配置中两者的同步关系。

- [ ] **T5: 编码器快照一致性**

```
Grep: g_foc_snap
```

分析 `g_foc_snap[0]` 和 `g_foc_snap[1]` 是否在同一 ADC 序列转换中填充：
- 如果是同一个 ADC 规则组，两个通道在同一个 ADC ISR 中读取 → ✓ 同步
- 如果是分开的 ADC（ADC1/ADC2 各读一个），两者的 ISR 可能有时序差

- [ ] **T6: BT/CLI 并发写入**

```
Read: bt_comm.c (BT 帧解析 → 修改 g_balance 参数)
Read: cli.c (CLI 命令处理 → 修改 g_balance 参数)
```

分析：
1. BT 解析在 CLI 任务的 `CLI_Process` 中调用 → 与 CLI 命令处理同任务，串行 ✓
2. BT 写入 `g_balance.target_yaw_angle` 是否有 COMPILER_BARRIER？

- [ ] **T7: IWDG vs RTOS 调度时序**

分析最坏情况喂狗延迟：
1. CLI 任务 `printf` 长字符串阻塞时间（230400 baud × N chars）
2. BalanceLoop 任务优先级 High vs CLI Normal — BalanceLoop 总是抢占 CLI ✓
3. BUT：BalanceLoop 等待 TIM17 信号量时被阻塞，CLI 运行
4. 如果 CLI 的 `printf` 在 BalanceLoop 被唤醒前占用 USART 超过 100ms？
   - USART TX 用 DMA 还是轮询？
   - 230400 baud ≈ 28.8KB/s，printf 通常 < 100 字节 → < 3.5ms ✓

### Task 5: Phase 2.4 — 代码质量审查 (Q1-Q5)

- [ ] **Q1: 魔法数**

扫描所有未命名的数值常量：

```
Grep: "\d+\.\d*f" → 列出所有 float 字面量
Grep: "\d+\s*[/\*]" → 检查数值计算是否应有宏定义
```

重点关注：
- `app_freertos.c:313` — `0.095f`（odom EMA α）
- `app_freertos.c:387` — `2.0f`（iq 限幅）
- `app_freertos.c:406` — `6.4f` / `6.6f`（VBUS 阈值）
- `balance_ctrl.c` — 直接写死的限幅值
- `app_freertos.c:439` — `tick % 20 == 0`（BT 遥测频率）

- [ ] **Q2: 死代码/冗余**

```
Grep: "//.*[Rr]emoved|//.*[Oo]ld|//.*[Uu]nused" → 注释掉的旧代码
Grep: "static.*=\s*0.*//" → 注释掉的变量初始化
```

检查：
1. `speed_ctrl.c` 中 EKF 的 `ekf_P[3]` / `ekf_P[6]` / `ekf_P[7]` 显式赋值是否必要（对称矩阵）
2. `speed_ctrl.h` 中 `SPEED_RAMP_MAX=20000` 是否被使用
3. `speed_ctrl.h` 中 `no_ramp` 字段是否被使用
4. 已移除功能的残留注释（如 `SPEED_DRIFT_KP 已移除`）

- [ ] **Q3: 错误处理缺口**

```
Grep: "HAL_" → 检查返回值检查
Grep: "pvPortMalloc|malloc" → 检查 NULL 检查
Grep: "MPU6500_ReadAll|MT6701_Read" → 检查错误返回值处理
```

- [ ] **Q4: 文档同步**

检查 `docs/` 下文档与代码的一致性：

| 文档 | 需核实的关键点 |
|------|--------------|
| `FREERTOS_TASKS.md` | 遥测通道定义（ch8=偏航角度 而非 yaw_rate？）|
| `COMM_PROTOCOL.md` | Y→A 命令变更是否反映 |
| `CALIB_PARAMS.md` | 参数值是否与头文件默认值一致 |
| `MOTOR_PARAMS.md` | Kt/J 等是否与 speed_ctrl.h 一致 |

- [ ] **Q5: 架构边界 — CubeMX 生成 vs 手写**

```
Read: app_freertos.c — 验证 USER CODE BEGIN/END 块
Read: main.c — 验证 USER CODE 块
Read: stm32g4xx_it.c — 验证 USER CODE 块
```

检查是否有手写代码落在 CubeMX 生成区域（会在下次重新生成时丢失）。

### Task 6: Phase 3 — 执行流交叉验证

对每个 Critical/High 问题，用 GitNexus 沿执行流验证：

- [ ] **Step 1: 对每个 Critical 问题查询相关执行流**

```bash
# 示例：如果发现 NaN 传播问题
npx gitnexus context BalanceCtrl_Run
npx gitnexus query "balance control NaN fault"
```

- [ ] **Step 2: 确认问题在实际运行时路径上而非死代码**

用 `gitnexus_impact({target: "<受影响的符号>", direction: "downstream"})` 确认问题代码确实在活跃执行流中。

- [ ] **Step 3: 滤除误报**

如果某个问题只在 dead path 上出现，降级为 Low 或移除。

### Task 7: Phase 4 — 产出审查报告

- [ ] **Step 1: 创建报告框架**

写入 `docs/CODE_REVIEW.md`，使用确定好的模板格式。

- [ ] **Step 2: 逐项填充**

按 Task 2-6 的发现逐项写入，标注 ID、类别、严重度、位置、问题描述、风险、修复建议。

- [ ] **Step 3: 写入汇总表和统计**

整理问题总数、分严重度统计、分类统计。

- [ ] **Step 4: 写入专项分析**

- 看门狗专项分析（S7-S9 综合结论）
- 架构时序专项分析（T1-T7 综合结论，含任务/ISR 抢占图）

- [ ] **Step 5: 写入正面发现**

列出值得保留的好实践。

- [ ] **Step 6: 为每个问题添加批注区**

```markdown
---
> **批注区**
> - [ ] 修复  [ ] 不修复  [ ] 推迟
> 说明：_____________
---
```

- [ ] **Step 7: 提交报告**
