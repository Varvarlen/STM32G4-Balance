#ifndef CLI_H
#define CLI_H

#include <stdint.h>

// 速度外环 PI 参数 (运行时通过 PS 命令调整)
extern float g_speed_outer_kp;
extern float g_speed_outer_ki;

void CLI_Init(void);
void CLI_Process(void);
uint8_t CLI_TelemetryEnabled(void);

#endif
