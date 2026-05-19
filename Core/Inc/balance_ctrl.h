#ifndef BALANCE_CTRL_H
#define BALANCE_CTRL_H

#include <stdint.h>

// 平衡 PID 默认参数（第一版保守值，后续根据实验调参）
#define BALANCE_KP_DEFAULT      15.0f   // 角度比例增益 (RPM/°)
#define BALANCE_KD_DEFAULT       2.0f   // 角速度阻尼增益 (RPM per °/s)
#define BALANCE_OUTPUT_MAX     500.0f   // 平衡输出限幅 (RPM)
#define BALANCE_STEER_MAX      100.0f   // 转向差速限幅 (RPM)
#define BALANCE_TILT_MAX        45.0f   // 倾角超限自动急停 (°)

typedef struct {
    // 参数（可通过 CLI 在线调整）
    float   kp_angle;       // 角度比例增益 (RPM/°)
    float   kd_gyro;        // 角速度阻尼增益 (RPM per °/s)
    float   kff_speed;      // 速度前馈增益 (暂未使用, 预留)
    float   target_angle;   // 目标倾角 (°), 通常 0
    float   target_speed;   // 遥控前向速度指令 (RPM)
    float   steer;          // 转向指令 (RPM 差速量)
    float   output_max;     // 平衡输出限幅 (RPM)

    // 运行状态（只读）
    float   tilt_angle;     // 当前倾角 (°), 卡尔曼估计值
    float   gyro_rate;      // 当前陀螺仪角速度 (°/s)
    float   balance_out;    // 平衡 PID 输出 (RPM)
    float   speed_ref_l;    // 左轮速度指令 (RPM)
    float   speed_ref_r;    // 右轮速度指令 (RPM)

    uint8_t active;         // 平衡控制激活标志
} BalanceCtrl_t;

void BalanceCtrl_Init(BalanceCtrl_t *bc);
void BalanceCtrl_Run(BalanceCtrl_t *bc);

#endif
