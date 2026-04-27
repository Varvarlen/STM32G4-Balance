#ifndef PI_H
#define PI_H

typedef struct {
    float kp;
    float ki;
    float integral;
    float out_max;
    float out_min;
} PI_t;

// 初始化 PI 控制器
void PI_Init(PI_t *pi, float kp, float ki, float out_max, float out_min);
// 执行一步控制计算
float PI_Step(PI_t *pi, float error, float dt);
// 重置积分项
void PI_Reset(PI_t *pi);

#endif
