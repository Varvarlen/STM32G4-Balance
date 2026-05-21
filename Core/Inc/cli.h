#ifndef CLI_H
#define CLI_H

#include <stdint.h>

// 速度外环 PI 参数 (运行时通过 PS 命令调整)
extern float g_speed_outer_kp;
extern float g_speed_outer_ki;

// 偏航 PI 参数 (运行时通过 PY 命令调整)
extern float g_yaw_kp;
extern float g_yaw_ki;

void CLI_Init(void);
void CLI_Process(void);
uint8_t CLI_TelemetryEnabled(void);

/** @brief printf 输出端口切换 (0=USART1, 1=USART2), main.c 实现 */
void CLI_SetOutputPort(uint8_t port);
uint8_t CLI_GetOutputPort(void);

/** @brief 当前活跃的 CLI 输入端口 (0=USART1, 1=USART2)
 *  CLI_Process 在分发字符前设置, CLI_ReadChar 等据此选择端口 */
extern uint8_t g_cli_active_port;

#endif
