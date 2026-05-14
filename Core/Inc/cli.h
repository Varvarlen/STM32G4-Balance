#ifndef CLI_H
#define CLI_H

#include <stdint.h>

// 负载实验状态
typedef struct {
    uint8_t  active;
    uint8_t  motor_idx;
    uint8_t  phase;
    uint32_t phase_start;
    float    rpm;
} LoadTest_t;

// 阶跃测试状态
typedef struct {
    uint8_t  active;
    uint8_t  motor_idx;
    uint8_t  phase;
    uint32_t phase_start;
    float    from_rpm;
    float    to_rpm;
} StepTest_t;

extern LoadTest_t g_load_test;
extern StepTest_t g_step_test;

void CLI_Init(void);
void CLI_Process(void);
uint8_t CLI_TelemetryEnabled(void);

#endif
