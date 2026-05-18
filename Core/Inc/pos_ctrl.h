#ifndef POS_CTRL_H
#define POS_CTRL_H

#include <stdint.h>

// 位置环参数
#define POS_P_DEFAULT_KP     210.0f  // 比例增益 (RPM/rad), BW~3.5Hz (Kp扫描最优)
#define POS_SPEED_MAX        500.0f   // 位置环输出限幅 (RPM)
#define POS_RAMP_MAX         2000.0f  // 位置模式速度斜坡 (RPM/s)

// EMA 滤波系数: α = 1 - exp(-dt/τ), dt=1ms
#define POS_ERR_EMA_ALPHA    0.181f  // τ≈5ms, 抑制量化噪声

typedef struct {
    float   kp;              // 比例增益 (RPM/rad)
    float   pos_ref;         // 目标位置 (rad, 连续展开)
    float   speed_max;       // 输出限幅 (RPM)
    float   pos_err_filt;    // EMA 滤波后位置误差 (抑制量化噪声)
    float   speed_ref_ramp;  // 经斜坡后的速度参考 (RPM)
    uint8_t active;          // 位置模式激活标志
} PosCtrl_t;

// 初始化位置控制器
void PosCtrl_Init(PosCtrl_t *pc, float kp, float speed_max);
// 每 1ms 调用: 计算速度指令, 返回 speed_ref (RPM)
float PosCtrl_Run(PosCtrl_t *pc, float pos_fb);
// 进入位置模式 (cur_speed: 当前速度 (RPM), 用于初始化斜坡)
void PosCtrl_EnterMode(PosCtrl_t *pc, float pos_ref, float cur_speed);
// 退出位置模式
void PosCtrl_ExitMode(PosCtrl_t *pc);

#endif
