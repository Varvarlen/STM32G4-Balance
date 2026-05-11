#include "debug_capture.h"
#include "motor_hal.h"
#include "pi.h"
#include "comm_protocol.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdlib.h>

static DebugCapture_t g_cap;
static float           g_cap_buf[CAPTURE_TOTAL * 2];  // [id, iq] × 500, BSS 4000B
static volatile uint8_t g_cap_has_data;

void DebugCapture_Init(void)
{
    g_cap.buf      = g_cap_buf;
    g_cap.wr       = 0;
    g_cap.active   = 0;
    g_cap.ready    = 0;
    g_cap_has_data = 0;
}

void DebugCapture_Start(uint8_t motor_id, float step)
{
    if (g_cap.active || g_cap.ready) return;

    g_cap.motor_id = motor_id;
    g_cap.step     = step;
    g_cap.wr       = 0;
    g_cap.ready    = 0;

    // 先配电机再激活采集 — 确保 ISR 看到完整初始状态
    Motor_t *m = &g_motor[motor_id];
    Motor_t *other = &g_motor[1 - motor_id];

    Motor_SetDuty(m, 0.50f, 0.50f, 0.50f);          // 预装载 50%，下次 UPDATE 生效
    Motor_StartPWM(m);
    m->mode   = MOTOR_MODE_CURRENT_LOOP;
    m->id_ref = 0.0f;
    m->iq_ref = 0.0f;
    PI_Reset(&m->id_pi);
    PI_Reset(&m->iq_pi);

    // 另一电机失能（50% 零电压 PWM，不转）
    Motor_Neutralize(other);

    // 一切就绪后才激活 — ISR 从下一个周期开始采集
    g_cap.active = 1;

    printf("Capture M%d step=%.3fA (%d samples)\r\n", motor_id + 1, step, CAPTURE_TOTAL);
}

// ISR (10kHz) — CurrentCtrl_Run 之前调用，在精确样本点施加阶跃
void DebugCapture_PreCtrl(Motor_t *motor)
{
    if (!g_cap.active) return;
    if (motor->motor_id != g_cap.motor_id) return;

    if (g_cap.wr == CAPTURE_PRE_TRIGGER) {
        Motor_SetIqRef(motor, g_cap.step);
    }
}

// ISR (10kHz) — CurrentCtrl_Run 之后调用，采集 id/iq
void DebugCapture_PostCtrl(Motor_t *motor)
{
    if (!g_cap.active) return;
    if (motor->motor_id != g_cap.motor_id) return;

    uint16_t w = g_cap.wr;

    if (w < CAPTURE_TOTAL) {
        g_cap.buf[w * 2]     = motor->id;
        g_cap.buf[w * 2 + 1] = motor->iq;
        g_cap.wr = w + 1;
    }

    if (g_cap.wr >= CAPTURE_TOTAL) {
        g_cap.active   = 0;
        g_cap.ready    = 1;
        g_cap_has_data = 1;
    }
}

uint8_t DebugCapture_IsActive(void)
{
    return g_cap.active || g_cap.ready;
}

uint8_t DebugCapture_HasData(void)
{
    return g_cap_has_data;
}

// 发送元数据帧 [pre_trigger_count, step, motor_id] + 500 帧 [id, iq]
static void DebugCapture_SendFrames(void)
{
    for (uint16_t i = 0; i < CAPTURE_TOTAL; i++) {
        float frame[3];
        frame[0] = g_cap_buf[i * 2];
        frame[1] = g_cap_buf[i * 2 + 1];
        frame[2] = (i < CAPTURE_PRE_TRIGGER) ? 0.0f : g_cap.step;
        COMM_SendFloatFrame(frame, 3);
        osDelay(1);
    }
}

// 重发上次采集的数据（纯二进制帧，无 printf）
void DebugCapture_Resend(void)
{
    if (!g_cap_has_data) return;
    if (g_cap.active || g_cap.ready) return;

    DebugCapture_SendFrames();
}

void DebugCapture_Task(void)
{
    for (;;) {
        if (g_cap.ready) {
            // 先停机再下传 — 避免电机在阶跃电流下运行 500ms
            Motor_Neutralize(&g_motor[0]);
            Motor_Neutralize(&g_motor[1]);
            // 等待 DMA 发送完成
            osDelay(2);

            DebugCapture_SendFrames();

            // 等最后一帧 DMA 排空，避免与 printf 冲突
            osDelay(2);

            g_cap.ready = 0;
            printf("Capture done (%d frames sent)\r\n", CAPTURE_TOTAL);
        }
        osDelay(10);
    }
}
