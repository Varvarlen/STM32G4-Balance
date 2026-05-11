#ifndef SPEED_CTRL_H
#define SPEED_CTRL_H

#include "pi.h"
#include <stdint.h>

#define SPEED_LOOP_FREQ       1000.0f
#define SPEED_LOOP_DT         (1.0f / SPEED_LOOP_FREQ)
#define SPEED_RAMP_MAX        5000.0f     // 默认加速度限制 (RPM/s)
#define SPEED_DEADBAND_RPM    10.0f       // 低速死区 — |ref|<10 时切断输出

// 速度 PI 默认参数 (Kp=0.02, Ki=0.04 — 零点 2rad/s≈0.32Hz)
#define SPEED_PI_DEFAULT_KP   0.02f
#define SPEED_PI_DEFAULT_KI   0.04f

// α-β 滤波器默认参数 (1kHz, 临界阻尼 Benedict-Bordner)
#define SPEED_ALPHA_DEFAULT     0.15f    // 位置增益 — τ≈6.7ms
#define SPEED_BETA_DEFAULT      0.012f   // 速度增益 — β≈α²/(2-α)
#define SPEED_KT_OVER_J_DEFAULT 0.0f     // Kt/J 比值 (rad/s²/A), 0=纯运动学, 待标定

typedef struct {
    PI_t    pi;                // 速度 PI，输出 iq_ref
    float   kp, ki;
    float   speed_ref;         // 串口设定的目标转速 (RPM)
    float   speed_ref_ramp;    // 斜坡后目标转速 (RPM)
    float   speed_fb;          // α-β 滤波器速度估计值 (RPM)
    float   pos_est;           // α-β 连续位置估计 (rad, 无缠绕)
    float   vel_est;           // α-β 速度估计 (rad/s)
    float   alpha, beta;       // α-β 滤波器增益
    float   kt_over_j;         // Kt/J 比值 (rad/s²/A), 0=纯运动学模式
    float   raw_rpm;           // 速度估计值 (RPM)，与 speed_fb 相同保留兼容
    uint8_t speed_mode;        // 0=电流模式, 1=速度模式
    int8_t  enc_dir;           // 编码器方向 (±1)
    uint8_t first_run;         // 首帧标志 — 快照初始化位置
} SpeedCtrl_t;

// 初始化速度控制器
void SpeedCtrl_Init(SpeedCtrl_t *sc, float kp, float ki,
                    float out_max, float out_min, int8_t enc_dir);
// 每 1ms 调用：α-β 滤波器（位置+转矩模型）速度估计
void SpeedCtrl_UpdateRPM(SpeedCtrl_t *sc, float mech_angle, float iq);
// 每 1ms 调用：斜坡 + 速度 PI，返回 iq_ref；电流模式返回 0
float SpeedCtrl_Run(SpeedCtrl_t *sc);
// 进入速度模式
void SpeedCtrl_EnterMode(SpeedCtrl_t *sc, float speed_ref);
// 退出速度模式，切回电流模式
void SpeedCtrl_ExitMode(SpeedCtrl_t *sc);

#endif
