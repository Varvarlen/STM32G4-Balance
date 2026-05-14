# 速度环开发调试报告

## 时间线

| 阶段 | 内容 |
|------|------|
| 前期 | α-β 滤波器 + 速度 PI 基础实现 |
| 优化 | α-β → EKF 3-state 替换（Python 离线验证 → MCU C 实现） |
| 实验 | 负载扰动测试 + 速度扫描 + 1kHz 阶跃响应 |
| 调试 | 串口、printf 栈溢出、内存分配、TX 缓冲区、电源过流等一系列问题 |

## 最终架构

```
TIM17 (1kHz ISR) → TaskSpeedLoop (High)
  ├── SpeedCtrl_UpdateRPM()   EKF 3-state [θ, ω, T_load]
  └── SpeedCtrl_Run()         速度 PI + 斜坡 + 死区

ADC DMA ISR (10kHz) → CurrentCtrl_Run()  电流 PI (零极点对消)

CLI: V/W(速度) R/L(电流) E/F(负载实验) S/T(阶跃测试)
遥测: 200Hz, 10 通道 float 帧
Burst: 1kHz, 256 帧 × 4 字段, 事后二进制 dump
```

## α-β vs EKF 对比

| 指标 | α-β | EKF | 改善 |
|------|:--:|:--:|:--:|
| 负载扰动 RMSE | 18-31 RPM | 2.3-3.2 RPM | ~87% |
| 稳态纹波 | ~4 RPM | ~1.5 RPM | ~63% |
| 低速(<100RPM) | 估算滞后明显 | 自适应增益 | 根本解决 |

**EKF 参数**（125 组 Python 扫描最优）：
- q_accel = 1000, q_tload = 10, R = 0.01
- 3-state 标量展开，~80 FPU ops/step @ 1kHz → <5μs
- 静态 BSS 协方差矩阵，无 malloc 依赖

## 阶跃响应性能 (1kHz 实测)

| 阶跃 | 上升时间 | 超调 | 稳定时间 | 峰值电流 |
|------|:--:|:--:|:--:|:--:|
| 0→30 RPM | 7ms | 7.7% | 31ms | 0.46A |
| 0→100 RPM | 7ms | 4.0% | 19ms | 1.20A |
| 0→200 RPM | 11ms | 3.7% | 13ms | 1.24A |
| 0→300 RPM | 15ms | 2.4% | 17ms | 1.33A |
| 50→-50 RPM | 8ms | 4.3% | 9ms | 1.31A |

M1/M2 一致性优秀，差异 <0.5 RPM。

## Bug 记录

### 1. printf 浮点格式化 → CLI 任务栈溢出

**现象**: S/T/E/F 命令 printf 无输出，任务静默失败  
**根因**: `printf("%.0f", float_val)` 在 newlib-nano 中需要 ~800B 栈，CLI 任务栈仅 192 words (768B)  
**修复**: 改用 `printf("%d", (int)val)` 整数格式化  
**教训**: STM32 项目 CLI 任务栈小，**禁止在 Normal 优先级任务中使用浮点 printf**

### 2. pvPortMalloc 静默失败

**现象**: burst capture 无数据，状态机卡死等 2s 超时  
**根因**: FreeRTOS heap_4 16KB，正常模式剩 ~9KB，分配 6.4KB 可能因碎片化失败  
**修复**: 改用静态 BSS 缓冲区 `static float sc_buf[256*4]` (4KB)  
**教训**: 嵌入式实时系统中，**大于 1KB 的堆分配不可靠**，优先用静态分配

### 3. SpeedCapture_IsBusy() 含 ready 状态

**现象**: 状态机永不等不到 `!IsBusy()`，一直卡到 2s 超时  
**根因**: `IsBusy = active || ready || dumping`，capture 完成后 ready=1 → IsBusy=true  
**修复**: `IsBusy` 只含 `active` 和 `dumping`，`ready` 用独立 `IsReady()` 查询  
**教训**: 状态机中"忙"和"就绪"是不同语义，不能混用

### 4. TX 环形缓冲区死锁

**现象**: 前几个 burst dump 正常，后续全部卡死  
**根因**: UART TX 缓冲区仅 256B，burst 数据 4104B 一次性写入。COMM_SendData 先写满缓冲区再启动 DMA，缓冲区满后 DMA 尚未启动 → 死锁  
**修复**: 分包发送，每 200 字节 `osDelay(5)` 等 DMA 排空  
**教训**: **大块数据写入前要确保 DMA 已启动**，或者分包+延迟

### 5. 退出速度模式 iq_ref 残留

**现象**: E/F 负载实验结束后电机暴走  
**根因**: `SpeedCtrl_ExitMode` 只清速度环状态，未清零 `g_motor.iq_ref`，电流环继续执行残留值  
**修复**: 退出后显式 `Motor_SetIqRef(&motor, 0.0f)`  
**教训**: 级联控制器切换模式时，**下游给定值必须显式清零**

### 6. ST-Link VCP 自动复位

**现象**: 脚本连接后 MCU 启动消息出现在命令之后，命令被丢弃  
**根因**: PySerial 打开串口时 DTR 拉低，ST-Link VCP 通过电容耦合到 NRST，触发 MCU 复位  
**修复**: 打开串口后等 3 秒，等 MCU 启动完成再接受命令  
**教训**: ST-Link 虚拟串口有自动复位特性，脚本需加启动等待

### 7. 电源过流保护

**现象**: S500 (0→500 RPM 瞬时阶跃) 触发电源保护  
**根因**: 无斜坡瞬时阶跃 → PI 瞬间输出 2A → 7.4V×2A=14.8W 瞬时功率  
**修复**: 取消 S500/T500，改用 S300/T300  
**教训**: 阶跃测试需考虑电源能力，**大阶跃应保留斜坡或限制阶跃幅度**

### 8. Python 脚本数据丢失

**现象**: 自动 sweep 只记录到少量帧  
**根因**: `time.sleep()` 阻塞主线程 → 串口 OS 缓冲区 4096B → 0.47s 满 → 数据丢失  
**修复**: 后台 serial_reader 线程持续读串口，主线程只负责发命令  
**教训**: **Python 串口采集必须用独立线程读数据**，不能在主线程 sleep

## 经验总结

1. **离线验证→MCU 实现的路径有效**: Python 参数扫描找到最优 EKF 参数，直接移植到 C 代码，一次烧录即达到预期性能
2. **1kHz burst 比高频遥测更有价值**: 100Hz 遥测对阶跃分析精度不足（±10ms），1kHz burst 给 ±1ms 精度且不占串口带宽
3. **静态分配优于动态分配**: 嵌入式系统中的 malloc 不可靠，优先 BSS 静态缓冲
4. **串口 TX/RX 分离是假象**: TX 阻塞会间接影响系统响应（printf 卡死 → CLI 无输出 → 不知道命令是否执行）
5. **CLI 任务栈要留够余量**: 192 words 对纯整数打印够用，任何浮点格式化都是定时炸弹
6. **状态机设计要区分 busy/ready/done**: 三种状态语义不同，不能合并判断
