#ifndef SPEED_CTRL_H
#define SPEED_CTRL_H

#include "pi.h"
#include <stdint.h>

#define SPEED_LOOP_FREQ       1000.0f
#define SPEED_LOOP_DT         (1.0f / SPEED_LOOP_FREQ)
#define SPEED_RAMP_MAX        5000.0f

// 速度 PI 参数 (ωn≈7Hz ζ≈0.8, BW≈8Hz, 零点=R/L=355)
#define SPEED_PI_DEFAULT_KP   0.044f
#define SPEED_PI_DEFAULT_KI   1.221f

// EKF 3-state 过程噪声 (离线遥测扫描最优: qa=200 qt=1 R=0.1)
// 离散化: Q_d[vel] = q_accel * dt²,  Q_d[t_load] = q_tload * dt
#define EKF_Q_ACCEL          50.0f     // 加速度过程噪声 (rad/s²)² (压低共振基频<20Hz)
#define EKF_Q_TLOAD           1.0f     // 负载转矩过程噪声 (N·m)²/s
#define EKF_R_MEAS             0.10f   // 测量噪声 (rad²)

// 电机参数
#define MOTOR_KT            0.0290f    // 转矩常数 (N·m/A)
#define MOTOR_J             1.83e-5f   // 转子惯量 (kg·m²)

typedef struct {
    PI_t    pi;
    float   kp, ki;
    float   speed_ref;
    float   speed_ref_ramp;
    float   speed_fb;          // EKF 速度估计 (RPM)
    float   pos_est;           // EKF 连续位置估计 (rad)
    float   vel_est;           // EKF 速度估计 (rad/s)
    float   t_load_est;        // EKF 负载转矩估计 (N·m)
    float   ekf_P[9];          // 状态协方差 3x3 行优先
    float   kt;                // 转矩常数 (N·m/A)
    float   j;                 // 转子惯量 (kg·m²)
    float   meas_cont;         // 展开后连续测量位置 (rad)
    float   last_meas_raw;     // 上一帧原始测量 (rad, 用于展开)
    float   raw_rpm;
    uint8_t speed_mode;
    uint8_t no_ramp;         // 阶跃测试: 跳过斜坡, 瞬时切换给定
    int8_t  enc_dir;
    uint8_t first_run;
} SpeedCtrl_t;

// 初始化速度控制器
void SpeedCtrl_Init(SpeedCtrl_t *sc, float kp, float ki,
                    float out_max, float out_min, int8_t enc_dir,
                    float kt, float j);
// 每 1ms 调用：EKF 速度估计 (替代 α-β 滤波器)
void SpeedCtrl_UpdateRPM(SpeedCtrl_t *sc, float mech_angle, float iq);
// 每 1ms 调用：斜坡 + 速度 PI，返回 iq_ref；电流模式返回 0
float SpeedCtrl_Run(SpeedCtrl_t *sc);
// 进入速度模式
void SpeedCtrl_EnterMode(SpeedCtrl_t *sc, float speed_ref);
// 退出速度模式
void SpeedCtrl_ExitMode(SpeedCtrl_t *sc);
// 运行时修改速度 PI 参数（同时更新 kp/ki 成员和 pi 对象）
void SpeedCtrl_SetGains(SpeedCtrl_t *sc, float kp, float ki);

#endif
