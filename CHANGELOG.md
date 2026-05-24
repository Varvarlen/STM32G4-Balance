# 变更记录

## v3.2 (2026-05-24)

### 新增
- 拿起检测：轮速 >500 RPM 且陀螺 <30°/s 持续 300ms 自动关闭平衡
- BT 偏航速率直连模式：摇杆直接控制转向速率，跳过角度外环，响应更快
- BT 偏航松杆 EMA 平滑：速率模式切回 heading hold 时无冲击
- CLI 任务补充 IWDG 喂狗（校准模式下防止看门狗复位）
- IMU 故障恢复后自动重激活平衡控制（倾角 ≤10° 时）
- 偏航角度 ±360° 防卷绕

### 安全修复
- PI_Step NaN 保护：error 为 NaN 时冻结积分，防止永久污染
- balance_out / steer NaN 检查：中间输出异常归零，防止传导向电机
- IWDG/VBUS 维护移至 IMU 读取之前：IMU 故障时仍能按时喂狗
- BT_COMM_SendData 非阻塞：TX buffer 满时丢弃帧，避免阻塞 BalanceLoop 任务
- Error_Handler 增加 Fault_DisableMotors：CPU 严重错误时安全停止电机

### 控制优化
- BT 遥控速度上限独立宏 `BT_SPEED_MAX_RPM=200`，与平衡输出限幅解耦
- 偏航参数整体提灵敏：外环 KP 5→10，内环 KP 0.5→1.0，Steer 限幅 50→80 RPM
- 速度外环 I 限幅 (3.5°) < 输出限幅 (5.0°)

### 修复
- CLI A 命令因 AT 检测吞字符导致无法执行
- 偏航方向注释，确认负反馈逻辑正确

### 代码质量
- 魔法数宏化：VBUS 阈值、EMA α、IWDG 周期等 8 处
- 清理废弃宏 `SPEED_RAMP_MAX`
- 遥测文档修正（ch8/ch9 实际为 yaw_angle 而非 yaw_rate）

## v3.1 (2026-05-19)

### 新增
- 偏航绝对角度闭环控制（级联：角度外环 + 速率内环 + 互补滤波）
- 蓝牙遥控完整支持（控制帧 + 遥测帧 + AT 桥模式）
- 速度外环 PI（100Hz，位置差分测速 → target_angle）
- EKF 3-state 速度估计（替代 α-β 滤波器）
- odom_rate EMA 滤波抑制编码器量化噪声

### 修复
- IWDG 看门狗集成，CubeMX 配置 8s 超时
- 电机序号魔数化 MOTOR_LEFT/MOTOR_RIGHT
- Fault_DisableMotors 去重，提取到 motor_hal.c
- 蜂鸣器音效优化，音调语义分层

## v3.0 (2026-05-10)

- 项目改名 STM32G431Demo → STM32G4-Balance
- FOC 电流闭环（Clarke/Park/PI/SVPWM，20kHz）
- 平衡 PD 控制（1kHz）
- 卡尔曼滤波倾角估计（含陀螺零偏）
- CLI 串口命令系统 + 遥测帧
- 电机参数自动标定流程
- 开源准备：MIT LICENSE + README
