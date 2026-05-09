#ifndef DEBUG_CAPTURE_H
#define DEBUG_CAPTURE_H

#include <stdint.h>
#include "foc.h"

#define CAPTURE_TOTAL       500   // 总采样点数 (50ms @ 10kHz)
#define CAPTURE_PRE_TRIGGER 50    // 预触发点数 (5ms 基线)

typedef struct {
    float          *buf;           // pvPortMalloc 分配, [id,iq] 交替
    volatile uint16_t wr;          // 当前写入计数 (ISR 写入, 需 volatile 防编译器重排)
    uint8_t         motor_id;      // 0=M1, 1=M2
    float           step;          // 阶跃幅度 (A)
    volatile uint8_t active;       // ISR: 采集中
    volatile uint8_t ready;        // ISR→Task: 数据就绪待下传
} DebugCapture_t;

void    DebugCapture_Init(void);
void    DebugCapture_PreCtrl(Motor_t *motor);        // ISR: CurrentCtrl_Run 之前 — 施加阶跃
void    DebugCapture_PostCtrl(Motor_t *motor);       // ISR: CurrentCtrl_Run 之后 — 采集 id/iq
uint8_t DebugCapture_IsActive(void);                 // 遥测抑制查询
void    DebugCapture_Start(uint8_t motor_id, float step);  // CLI 触发
void    DebugCapture_Task(void);                     // 数据下传 (在 Task 中循环调用)
uint8_t DebugCapture_HasData(void);                  // 是否有已采集数据可重发
void    DebugCapture_Resend(void);                   // 重发上次采集数据 (纯二进制帧)

#endif
