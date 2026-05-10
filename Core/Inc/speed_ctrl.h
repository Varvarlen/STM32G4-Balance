#ifndef SPEED_CTRL_H
#define SPEED_CTRL_H

#include "pi.h"
#include <stdint.h>

#define SPEED_LOOP_FREQ     1000.0f
#define SPEED_LOOP_DT       (1.0f / SPEED_LOOP_FREQ)
#define SPEED_EMA_SHIFT     4           // τ≈3.7ms @1kHz, 抑制量化噪声
#define SPEED_RAMP_MAX      5000.0f     // 默认加速度限制 (RPM/s)
#define SPEED_MIN_DELTA     4.0f        // 自适应窗口最小角度增量 (counts)
#define SPEED_MAX_WINDOW_MS 20
#define SPEED_DEADBAND_RPM  10.0f       // 低速死区 — |ref|<10 时切断输出, 编码器量化噪声区

// 速度 PI 默认参数 (Kp=0.01, Ki=0.04 — 零点 4rad/s≈0.64Hz)
// 低 Ki 防止积分缓慢累积导致的低频极限环振荡
#define SPEED_PI_DEFAULT_KP 0.01f
#define SPEED_PI_DEFAULT_KI 0.04f

typedef struct {
    PI_t    pi;                // 速度 PI，输出 iq_ref
    float   kp, ki;
    float   speed_ref;         // 串口设定的目标转速 (RPM)
    float   speed_ref_ramp;    // 斜坡后目标转速 (RPM)
    float   speed_fb;          // EMA 滤波后实际转速 (RPM)
    float   last_mech;         // 上一时刻机械角度 (rad)
    float   accum_delta;       // 累积角度增量 (rad)
    uint16_t accum_ms;         // 累积毫秒数
    float   raw_rpm;           // 最新原始 RPM 测量值
    uint8_t speed_mode;        // 0=电流模式, 1=速度模式
    int8_t  enc_dir;           // 编码器方向 (±1)
} SpeedCtrl_t;

// 初始化速度控制器
void SpeedCtrl_Init(SpeedCtrl_t *sc, float kp, float ki,
                    float out_max, float out_min, int8_t enc_dir);
// 每 1ms 调用：自适应窗口 RPM 测量 + EMA 滤波
void SpeedCtrl_UpdateRPM(SpeedCtrl_t *sc, float mech_angle);
// 每 1ms 调用：斜坡 + 速度 PI，返回 iq_ref；电流模式返回 0
float SpeedCtrl_Run(SpeedCtrl_t *sc);
// 进入速度模式
void SpeedCtrl_EnterMode(SpeedCtrl_t *sc, float speed_ref);
// 退出速度模式，切回电流模式
void SpeedCtrl_ExitMode(SpeedCtrl_t *sc);

#endif
