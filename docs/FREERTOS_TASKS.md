# FreeRTOS 任务列表

## 正常模式（平衡车）

| 任务 | 函数 | 周期 | 栈 (words) | 优先级 | 核心操作 |
|------|------|:----:|:---------:|:------:|------|
| TaskCLI | StartCLITask | 1ms | 256 | Normal | USART1/USART2 串口命令 + 遥测开关 + 参数调优 |
| TaskBalanceLoop | StartBalanceLoopTask | 1kHz | 768 | High | IMU→卡尔曼→平衡PID→速度环(双电机交替)→差速 (TIM17 触发) |

## 校准模式

| 任务 | 函数 | 周期 | 栈 (words) | 优先级 | 核心操作 |
|------|------|:----:|:---------:|:------:|------|
| TaskCLI | StartCLITask | 1ms | 1024 | Normal | 校准 CLI (R1-5, L1-5, RS, q) |

## 栈用量

| 模式 | TaskCLI | TaskBalanceLoop | 合计 |
|------|:------:|:---------------:|:----:|
| 正常 | 256 | 768 | 1024 words (4.1KB) |
| 校准 | 1024 | — | 1024 words (4KB) |

## 指令

| 指令 | 功能 | 例 |
|------|------|-----|
| `B` | 激活平衡控制 | |
| `STOP` | 紧急停止 | |
| `S<RPM>` | 前进速度指令 | `S100` |
| `T<val>` | 转向指令 | `T30` |
| `PK` | 查询平衡参数 | |
| `PK ANG=<val>` | 设置角度 Kp | `PK ANG=15` |
| `PK GYR=<val>` | 设置角速度 Kd | `PK GYR=2.0` |
| `PK ANG0=<val>` | 设置目标倾角 (°) | `PK ANG0=0.5` |
| `PK MAX=<val>` | 设置输出限幅 (RPM) | `PK MAX=500` |
| `PS P=X I=Y` | 设置速度 PI | `PS P=0.044 I=1.221` |
| `PC P=X I=Y` | 设置电流 PI | `PC P=15 I=5295` |
| `P` | 查询全部参数 | |
| `T` | 开关遥测 | |
| `CAL` | 进入校准模式 | |
| `?` | 帮助 | |

## 遥测帧格式（按需, 200Hz 子采样）

```
[Tilt_angle(°)] [Gyro(°/s)] [Balance_out(RPM)] [Speed_fb_L(RPM)] [Speed_fb_R(RPM)] [iq_L(A)] [iq_R(A)]
```
