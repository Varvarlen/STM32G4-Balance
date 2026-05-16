#ifndef POS_CTRL_H
#define POS_CTRL_H

#include <stdint.h>

// 位置环参数 (BW≈1.0Hz, 速度环参考跟踪 BW≈7.7Hz / 6)
#define POS_P_DEFAULT_KP     210.0f  // 比例增益 (RPM/rad), BW~3.5Hz (Kp扫描最优)
#define POS_SPEED_MAX        500.0f   // 位置环输出限幅 (RPM)

typedef struct {
    float   kp;              // 比例增益 (RPM/rad)
    float   pos_ref;         // 目标位置 (rad, 连续展开)
    float   speed_max;       // 输出限幅 (RPM)
    float   pos_err_filt;    // EMA 滤波后位置误差 (抑制 EKF 噪声)
    uint8_t active;          // 位置模式激活标志
} PosCtrl_t;

// 初始化位置控制器
void PosCtrl_Init(PosCtrl_t *pc, float kp, float speed_max);
// 每 1ms 调用: 计算速度指令, 返回 speed_ref (RPM)
float PosCtrl_Run(PosCtrl_t *pc, float pos_fb);
// 进入位置模式
void PosCtrl_EnterMode(PosCtrl_t *pc, float pos_ref);
// 退出位置模式
void PosCtrl_ExitMode(PosCtrl_t *pc);

#endif
