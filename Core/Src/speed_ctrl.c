#include "speed_ctrl.h"
#include <math.h>

void SpeedCtrl_Init(SpeedCtrl_t *sc, float kp, float ki,
                    float out_max, float out_min, int8_t enc_dir)
{
    PI_Init(&sc->pi, kp, ki, out_max, out_min);
    sc->kp = kp;
    sc->ki = ki;
    sc->speed_ref = 0.0f;
    sc->speed_ref_ramp = 0.0f;
    sc->speed_fb = 0.0f;
    sc->last_mech = 0.0f;
    sc->accum_delta = 0.0f;
    sc->accum_ms = 0;
    sc->raw_rpm = 0.0f;
    sc->speed_mode = 0;
    sc->enc_dir = enc_dir;
}

void SpeedCtrl_UpdateRPM(SpeedCtrl_t *sc, float mech_angle)
{
    float delta = mech_angle - sc->last_mech;
    sc->last_mech = mech_angle;

    // 最短路径折返修正
    if (delta > 3.14159265f)  delta -= 6.283185307f;
    if (delta < -3.14159265f) delta += 6.283185307f;

    sc->accum_delta += delta;
    sc->accum_ms++;

    // 自适应窗口：≥SPEED_MIN_DELTA counts 或超时
    float accum_counts = fabsf(sc->accum_delta) * 2607.5946f; // rad→counts
    if (accum_counts >= SPEED_MIN_DELTA || sc->accum_ms >= SPEED_MAX_WINDOW_MS) {
        if (sc->accum_ms > 0) {
            float dt_sec = (float)sc->accum_ms / 1000.0f;
            sc->raw_rpm = (sc->accum_delta / 6.283185307f) / dt_sec * 60.0f
                          * (float)sc->enc_dir;
        }
        sc->accum_delta = 0.0f;
        sc->accum_ms = 0;
    }

    // EMA 滤波
    int32_t diff = (int32_t)((sc->raw_rpm - sc->speed_fb) * 1000.0f);
    sc->speed_fb += (float)diff / 1000.0f * (1.0f / (float)(1 << SPEED_EMA_SHIFT));
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
    // 14-bit@1kHz 量化噪声 ~3.7 RPM, <15 RPM 时信噪比不足以闭环
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
