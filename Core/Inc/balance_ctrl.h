#ifndef BALANCE_CTRL_H
#define BALANCE_CTRL_H

#include <stdint.h>

// 平衡 PID 默认参数
#define BALANCE_KP_DEFAULT      20.0f   // 角度比例增益 (RPM/°), 倾角→轮速
#define BALANCE_KD_DEFAULT       1.2f   // 角速度阻尼增益 (RPM per °/s)

#define BALANCE_OUTPUT_MAX     800.0f   // 平衡输出限幅 (RPM), 电机物理能力保护
#define BT_SPEED_MAX_RPM       200.0f   // BT 遥控速度上限 (RPM), 限制激烈程度
#define BALANCE_STEER_MAX      100.0f   // 转向差速限幅 (RPM)
#define BALANCE_TILT_MAX        45.0f   // 倾角超限自动急停 (°)
#define BALANCE_GYRO_MAX       350.0f   // 角速度超限辅助急停 (°/s)
#define BALANCE_GYRO_EMA_ALPHA  0.181f  // 陀螺仪EMA α (τ≈5ms, dt=1ms), 快速响应扰动
#define BALANCE_DIRECT_GAIN    0.01f   // 直接力矩增益 (A/RPM): balance_out → iq_ref 转换系数
#define IQ_CMD_MAX             2.0f    // iq_cmd 硬限幅 (A), CR-018 魔法数消除
// SPEED_DRIFT_KP 已移除, 由速度外环 (speed_ctrl.h: SPEED_OUTER_KP/KI) 替代

// 偏航角度外环 (级联: 角度误差 → 目标角速度 → 速率PI → steer)
#define YAW_ANGLE_OUTER_DIV   20       // 1kHz/20 = 50Hz (速率环 2:1 级联)
#define YAW_ANGLE_OUTER_DT    0.02f    // 20ms
#define YAW_ANGLE_KP          5.0f     // 偏航角度 P gain (°/s per °)
#define YAW_ANGLE_KI          60.0f    // 偏航角度 I gain (°/s per °·s)
#define YAW_ANGLE_MAX_I       30.0f    // 角度环积分限幅 (°/s), 防卷绕
#define YAW_ANGLE_MAX_RATE    150.0f   // 角度环输出限幅 (°/s), 避免剧烈转向
#define YAW_BT_ANGLE_STEP     4.0f     // BT满杆每帧角度增量 (°), 50Hz满杆→200°/s
// 偏航速率内环 (互补滤波 + PI, 100Hz)
#define YAW_OUTER_DIV         10       // 1kHz/10 = 100Hz
#define YAW_OUTER_DT          0.01f    // 10ms
#define YAW_PI_KP             0.5f     // 偏航 P (RPM per °/s)
#define YAW_PI_KI             0.4f     // 偏航 I (RPM per °/s²)
#define YAW_PI_MAX            50.0f   // steer 输出限幅 (RPM)
#define YAW_COMP_ALPHA        0.01f    // 互补滤波 α @100Hz (τ≈1s: 1/(0.01×100)=1.0s)
#define YAW_RPM_TO_DPS        1.65f    // RPM差速 → °/s 偏航率 (6*r/W = 6*2.75/10.0, r=2.75cm轮径 W=10cm轮距)

// 注: kp_angle / kd_gyro / target_angle / output_max 由 CLI 任务运行时写入,
// 平衡任务 (1kHz) 读取. Cortex-M4 32-bit 对齐字访问原子, 无需互斥锁.
typedef struct {
    // 参数（可通过 CLI 在线调整）
    float   kp_angle;       // 角度比例增益 (RPM/°)
    float   kd_gyro;        // 角速度阻尼增益 (RPM per °/s)

    float   target_angle;   // 目标倾角 (°), 通常 0
    float   target_speed;   // 遥控前向速度指令 (RPM)
    float   target_yaw_angle;// 目标偏航角度 (°), CLI/BT写入
    float   steer;          // 转向指令 (RPM 差速量), 由偏航PI输出
    float   output_max;     // 平衡输出限幅 (RPM)

    // 运行状态（只读）
    float   tilt_angle;     // 当前倾角 (°), 卡尔曼滤波估计值
    float   gyro_rate;      // 当前陀螺仪角速度 (°/s, 原始)
    float   gyro_filt;      // 陀螺仪EMA滤波值 (°/s), 喂给PID D项
    float   yaw_angle;       // 当前偏航角度 (°), 编码器差速积分(只读)
    float   balance_out;    // 平衡 PID 输出 (RPM)
    float   speed_ref_l;    // 左轮速度指令 (RPM)
    float   speed_ref_r;    // 右轮速度指令 (RPM)

    uint8_t active;         // 平衡控制激活标志
    uint8_t yaw_mode;       // 偏航模式: 0=heading hold (target_yaw_angle不变), 1=BT主动转向
} BalanceCtrl_t;

void BalanceCtrl_Init(BalanceCtrl_t *bc);
void BalanceCtrl_Run(BalanceCtrl_t *bc);

#endif
