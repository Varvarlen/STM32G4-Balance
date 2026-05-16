#include "speed_ctrl.h"
#include <math.h>

#define TWO_PI          6.283185307f
#define RPM_PER_RADPS   9.549296586f   // 60/(2π)

// 速度反馈陷波器系数 (f0=24.609Hz, BW=3Hz, fs=1kHz)
// H(z) = (1 + a·z⁻¹ + z⁻²) / (1 + a·r·z⁻¹ + r²·z⁻²)
#define NOTCH_B0         1.0f
#define NOTCH_B1        -1.97618f     // a = -2·cos(2π·f0/fs)
#define NOTCH_B2         1.0f
#define NOTCH_A1        -1.95761f     // a·r
#define NOTCH_A2         0.98133f     // r²

void SpeedCtrl_Init(SpeedCtrl_t *sc, float kp, float ki,
                    float out_max, float out_min, int8_t enc_dir,
                    float kt, float j)
{
    PI_Init(&sc->pi, kp, ki, out_max, out_min);
    sc->kp = kp;
    sc->ki = ki;
    sc->speed_ref = 0.0f;
    sc->speed_ref_ramp = 0.0f;
    sc->speed_fb = 0.0f;
    sc->pos_est = 0.0f;
    sc->vel_est = 0.0f;
    sc->t_load_est = 0.0f;
    sc->kt = kt;
    sc->j = j;
    // EKF 初始协方差 — 对角 0.1 (python 同参数)
    for (int i = 0; i < 9; i++) sc->ekf_P[i] = 0.0f;
    sc->ekf_P[0] = 0.1f;  // P[0,0] — 位置不确定度
    sc->ekf_P[4] = 0.1f;  // P[1,1] — 速度不确定度
    sc->ekf_P[8] = 0.1f;  // P[2,2] — 负载不确定度
    sc->meas_cont = 0.0f;
    sc->last_meas_raw = 0.0f;
    sc->raw_rpm = 0.0f;
    sc->speed_fb_raw = 0.0f;
    sc->speed_fb_filt = 0.0f;
    sc->notch_x1 = 0.0f;
    sc->notch_x2 = 0.0f;
    sc->notch_y1 = 0.0f;
    sc->notch_y2 = 0.0f;
    sc->speed_mode = 0;
    sc->no_ramp = 0;
    sc->enc_dir = enc_dir;
    sc->first_run = 1;
}

// EKF 3-state: [pos, vel, T_load]
// 预测时输入 iq(实际电流) → Kt*iq/J = 电磁加速度
// 观测: 编码器连续位置 (增量法展开)
void SpeedCtrl_UpdateRPM(SpeedCtrl_t *sc, float mech_angle, float iq)
{
    float meas_raw = mech_angle * (float)sc->enc_dir;

    // 首帧快照
    if (sc->first_run) {
        sc->pos_est       = meas_raw;
        sc->vel_est       = 0.0f;
        sc->t_load_est    = 0.0f;
        sc->meas_cont     = meas_raw;
        sc->last_meas_raw = meas_raw;
        sc->speed_fb      = 0.0f;
        sc->speed_fb_raw  = 0.0f;
        sc->speed_fb_filt = 0.0f;
        sc->notch_x1 = 0.0f;
        sc->notch_x2 = 0.0f;
        sc->notch_y1 = 0.0f;
        sc->notch_y2 = 0.0f;
        sc->raw_rpm       = 0.0f;
        sc->first_run     = 0;
        return;
    }

    // 1. 测量展开 — 增量法
    float diff = meas_raw - sc->last_meas_raw;
    if (diff > 3.14159265f)       diff -= TWO_PI;
    else if (diff < -3.14159265f) diff += TWO_PI;
    sc->meas_cont     += diff;
    sc->last_meas_raw  = meas_raw;

    // ---- EKF: 预测 + 更新 ----
    float dt   = SPEED_LOOP_DT;   // 0.001
    float Jinv = 1.0f / sc->j;    // 1/J
    float u    = sc->kt * iq * Jinv;  // 电磁加速度 (rad/s²)

    // 读取当前状态和协方差
    float p0 = sc->pos_est;
    float p1 = sc->vel_est;
    float p2 = sc->t_load_est;

    // 协方差 P[3x3] row-major: [0,1,2; 3,4,5; 6,7,8]
    float P00 = sc->ekf_P[0], P01 = sc->ekf_P[1], P02 = sc->ekf_P[2];
    float           /*P10*/       P11 = sc->ekf_P[4], P12 = sc->ekf_P[5];
    float           /*P20*/       /*P21*/             P22 = sc->ekf_P[8];
    // P 对称, P10=P01, P20=P02, P21=P12

    // ---- 预测 ----
    float Hdt = 0.5f * dt * dt * Jinv;   // 0.5*dt²/J
    float Ddt = dt * Jinv;                // dt/J

    float accel = u - p2 * Jinv;          // (Kt*iq - T_load)/J

    float pos_pred = p0 + p1 * dt + 0.5f * accel * dt * dt;
    float vel_pred = p1 + accel * dt;
    float tl_pred  = p2;

    // 协方差预测: P_pred = F*P*F^T + Q
    // 计算 FP (3x3 中间矩阵)
    float FP00 = P00 + dt * P01 - Hdt * P02;
    float FP01 = P01 + dt * P11 - Hdt * P12;
    float FP02 = P02 + dt * P12 - Hdt * P22;

    float FP11 = P11 - Ddt * P12;
    float FP12 = P12 - Ddt * P22;

    // P_pred = FP @ F^T
    float pp00 = FP00 + dt * FP01 - Hdt * FP02;
    float pp01 = FP01 - Ddt * FP02;
    float pp02 = FP02;

    float pp11 = FP11 - Ddt * FP12;
    float pp12 = FP12;
    float pp22 = P22;       // FP[2,2] = P[2,2]

    // 加 Q — 仅速度和负载状态有过程噪声
    pp11 += EKF_Q_ACCEL * dt * dt;
    pp22 += EKF_Q_TLOAD * dt;

    // ---- 更新 ----
    float y = sc->meas_cont - pos_pred;   // innovation (连续位置残差)

    float S = pp00 + EKF_R_MEAS;
    float K0 = pp00 / S;
    float K1 = pp01 / S;    // P[1,0]/S (对称: pp10 = pp01)
    float K2 = pp02 / S;    // P[2,0]/S

    // 状态更新
    sc->pos_est    = pos_pred + K0 * y;
    sc->vel_est    = vel_pred + K1 * y;
    sc->t_load_est = tl_pred  + K2 * y;

    // 协方差更新: P_new = (I - K*H) @ P_pred
    // P_new[i][j] = P_pred[i][j] - K[i] * P_pred[0][j]
    float one_minus_K0 = 1.0f - K0;

    sc->ekf_P[0] = one_minus_K0 * pp00;
    sc->ekf_P[1] = one_minus_K0 * pp01;
    sc->ekf_P[2] = one_minus_K0 * pp02;

    sc->ekf_P[3] = pp01 - K1 * pp00;        // P[1,0] = P[0,1] (保持对称)
    sc->ekf_P[4] = pp11 - K1 * pp01;
    sc->ekf_P[5] = pp12 - K1 * pp02;

    sc->ekf_P[6] = pp02 - K2 * pp00;        // P[2,0] = P[0,2]
    sc->ekf_P[7] = pp12 - K2 * pp01;        // P[2,1] = P[1,2]
    sc->ekf_P[8] = pp22 - K2 * pp02;

    // 5. 输出 — rad/s → RPM, 速度反馈独立 EMA 滤波 (τ≈2ms, α=0.393)
    sc->speed_fb_raw = sc->vel_est * RPM_PER_RADPS;
    sc->speed_fb_filt += (sc->speed_fb_raw - sc->speed_fb_filt) * 0.393f;
    // 陷波器 @ 24.6Hz (BW=3Hz): 抑制机械共振, 奇次谐波 74.2Hz 也衰减 ~20dB
    float notch_in = sc->speed_fb_filt;
    float nout = NOTCH_B0 * notch_in + NOTCH_B1 * sc->notch_x1 + NOTCH_B2 * sc->notch_x2
               - NOTCH_A1 * sc->notch_y1 - NOTCH_A2 * sc->notch_y2;
    sc->notch_x2 = sc->notch_x1;
    sc->notch_x1 = notch_in;
    sc->notch_y2 = sc->notch_y1;
    sc->notch_y1 = nout;
    sc->speed_fb = nout;
    sc->raw_rpm = sc->speed_fb_raw;
}

float SpeedCtrl_Run(SpeedCtrl_t *sc)
{
    if (!sc->speed_mode) return 0.0f;

    // 斜坡 — 阶跃测试模式跳过
    if (sc->no_ramp) {
        sc->speed_ref_ramp = sc->speed_ref;
    } else {
        float error = sc->speed_ref - sc->speed_ref_ramp;
        float step = SPEED_RAMP_MAX * SPEED_LOOP_DT;
        if (error > step) {
            sc->speed_ref_ramp += step;
        } else if (error < -step) {
            sc->speed_ref_ramp -= step;
        } else {
            sc->speed_ref_ramp = sc->speed_ref;
        }
    }

    float speed_error = sc->speed_ref_ramp - sc->speed_fb;

    float iq_pi = PI_Step(&sc->pi, speed_error, SPEED_LOOP_DT);

    // 负载转矩前馈: 补偿静摩擦/负载
    float iq_ff = (sc->kt > 0.0001f) ? (sc->t_load_est / sc->kt) : 0.0f;
    float iq_out = iq_pi + iq_ff;
    if (iq_out > sc->pi.out_max) iq_out = sc->pi.out_max;
    else if (iq_out < sc->pi.out_min) iq_out = sc->pi.out_min;
    return iq_out;
}

void SpeedCtrl_EnterMode(SpeedCtrl_t *sc, float speed_ref)
{
    sc->speed_ref = speed_ref;
    sc->speed_ref_ramp = sc->speed_fb_filt;  // 用滤波后速度初始化斜坡, 避免 EKF 毛刺
    sc->speed_mode = 1;
    PI_Reset(&sc->pi);
}

void SpeedCtrl_ExitMode(SpeedCtrl_t *sc)
{
    sc->speed_mode = 0;
    sc->speed_ref = 0.0f;
    sc->speed_ref_ramp = 0.0f;
    sc->speed_fb_raw = 0.0f;
    sc->speed_fb_filt = 0.0f;
    sc->notch_x1 = 0.0f;
    sc->notch_x2 = 0.0f;
    sc->notch_y1 = 0.0f;
    sc->notch_y2 = 0.0f;
    PI_Reset(&sc->pi);
}

float SpeedCtrl_GetPosition(SpeedCtrl_t *sc)
{
    return sc->meas_cont;  // 纯编码器增量展开位置, 无 EKF 滤波
}

void SpeedCtrl_SetGains(SpeedCtrl_t *sc, float kp, float ki)
{
    sc->kp = kp;
    sc->ki = ki;
    sc->pi.kp = kp;
    sc->pi.ki = ki;
}

