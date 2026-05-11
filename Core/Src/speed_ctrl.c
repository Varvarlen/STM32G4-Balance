#include "speed_ctrl.h"
#include <math.h>

#define TWO_PI          6.283185307f
#define RPM_PER_RADPS   9.549296586f   // 60/(2π)

void SpeedCtrl_Init(SpeedCtrl_t *sc, float kp, float ki,
                    float out_max, float out_min, int8_t enc_dir)
{
    PI_Init(&sc->pi, kp, ki, out_max, out_min);
    sc->kp = kp;
    sc->ki = ki;
    sc->speed_ref = 0.0f;
    sc->speed_ref_ramp = 0.0f;
    sc->speed_fb = 0.0f;
    sc->pos_est = 0.0f;
    sc->vel_est = 0.0f;
    sc->alpha = SPEED_ALPHA_DEFAULT;
    sc->beta = SPEED_BETA_DEFAULT;
    sc->kt_over_j = SPEED_KT_OVER_J_DEFAULT;
    sc->raw_rpm = 0.0f;
    sc->speed_mode = 0;
    sc->enc_dir = enc_dir;
}

// α-β 滤波器 — 2 状态常速运动学模型 + 转矩前馈
// 每 1ms 调用：预测(模型) → 测量残差(编码器) → 更新(增益)
void SpeedCtrl_UpdateRPM(SpeedCtrl_t *sc, float mech_angle, float iq)
{
    // 1. 预测（常速模型 + 转矩加速度）
    float accel = sc->kt_over_j * iq;
    sc->pos_est += sc->vel_est * SPEED_LOOP_DT
                 + 0.5f * accel * SPEED_LOOP_DT * SPEED_LOOP_DT;
    sc->vel_est += accel * SPEED_LOOP_DT;

    // 2. 测量残差 — 编码器位置展开到预测位置附近（最短路径折返）
    float meas = mech_angle * (float)sc->enc_dir;
    float residual = meas - sc->pos_est;
    if (residual > 3.14159265f)       residual -= TWO_PI;
    else if (residual < -3.14159265f) residual += TWO_PI;

    // 3. α-β 更新
    sc->pos_est += sc->alpha * residual;
    sc->vel_est += sc->beta  * residual / SPEED_LOOP_DT;

    // 4. 输出
    sc->speed_fb = sc->vel_est * RPM_PER_RADPS;
    sc->raw_rpm  = sc->speed_fb;
}

float SpeedCtrl_Run(SpeedCtrl_t *sc)
{
    if (!sc->speed_mode) return 0.0f;

    // 斜坡
    float error = sc->speed_ref - sc->speed_ref_ramp;
    float step = SPEED_RAMP_MAX * SPEED_LOOP_DT;
    if (error > step) {
        sc->speed_ref_ramp += step;
    } else if (error < -step) {
        sc->speed_ref_ramp -= step;
    } else {
        sc->speed_ref_ramp = sc->speed_ref;
    }

    float speed_error = sc->speed_ref_ramp - sc->speed_fb;

    // 低速死区 — 原始指令低于阈值时直接切断输出
    // 14-bit@1kHz 量化噪声 ~3.7 RPM, <10 RPM 时信噪比不足以闭环
    // 用 speed_ref(原始指令)而非 speed_ref_ramp(斜坡值), 确保斜坡启动不被拦截
    if (fabsf(sc->speed_ref) < SPEED_DEADBAND_RPM) {
        PI_Reset(&sc->pi);
        return 0.0f;
    }

    return PI_Step(&sc->pi, speed_error, SPEED_LOOP_DT);
}

void SpeedCtrl_EnterMode(SpeedCtrl_t *sc, float speed_ref)
{
    sc->speed_ref = speed_ref;
    sc->speed_ref_ramp = sc->speed_fb;
    sc->speed_mode = 1;
    PI_Reset(&sc->pi);
}

void SpeedCtrl_ExitMode(SpeedCtrl_t *sc)
{
    sc->speed_mode = 0;
    sc->speed_ref = 0.0f;
    sc->speed_ref_ramp = 0.0f;
    PI_Reset(&sc->pi);
}
