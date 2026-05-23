#ifndef SPEED_CTRL_H
#define SPEED_CTRL_H

#include "pi.h"
#include <stdint.h>

#define SPEED_LOOP_FREQ       1000.0f
#define SPEED_LOOP_DT         (1.0f / SPEED_LOOP_FREQ)
#define SPEED_RAMP_MAX        20000.0f

// 速度外环 (100Hz, 差分测速 + PI → target_angle)
#define SPEED_OUTER_DIV       10       // 1kHz/10 = 100Hz
#define SPEED_OUTER_DT        0.01f    // 10ms
#define SPEED_OUTER_KP        0.12f    // 速度外环 P (°/RPM)
#define SPEED_OUTER_KI        0.08f    // 速度外环 I (°/RPM/s)
#define SPEED_OUTER_MAX       5.0f     // target_angle 输出限幅 (°)

// 速度 PI 参数 (ωz=Ki/Kp=4.4Hz, 交叉频率~76Hz)
#define SPEED_PI_DEFAULT_KP   0.044f
#define SPEED_PI_DEFAULT_KI   1.221f

// EKF 3-state 过程噪声
// 离散化: Q_d[vel] = q_accel * dt²,  Q_d[t_load] = q_tload * dt
#define EKF_Q_ACCEL         400.0f     // 加速度过程噪声 (rad/s²)², BW~14Hz
#define EKF_Q_TLOAD           1.0f     // 负载转矩过程噪声 (N·m)²/s
#define EKF_R_MEAS             0.10f   // 测量噪声 (rad²)

// EMA 滤波系数: α = 1 - exp(-dt/τ), dt=1ms
#define SPEED_EMA_ALPHA     0.393f  // τ≈2ms, 送给速度 PI

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
    float   speed_fb_raw;       // EKF 原始速度 (RPM), 用于斜坡初始化
    float   speed_fb_filt;      // EMA τ≈2ms 滤波后速度 (RPM), 送给速度 PI
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

// 进入速度模式
void SpeedCtrl_EnterMode(SpeedCtrl_t *sc, float speed_ref);
// 退出速度模式
void SpeedCtrl_ExitMode(SpeedCtrl_t *sc);



#endif
