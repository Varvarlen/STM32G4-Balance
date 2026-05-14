#include "speed_capture.h"
#include "comm.h"
#include <string.h>
#include "cmsis_os.h"

// 静态 BSS 缓冲区 — 避免 pvPortMalloc 碎片化失败
static float sc_buf[SC_BURST_SAMPLES * SC_BURST_FIELDS];
static SpeedCapture_t g_sc;
volatile uint8_t g_capture_dumping;

void SpeedCapture_Init(void)
{
    g_sc.buf    = sc_buf;
    g_sc.wr     = 0;
    g_sc.total  = 0;
    g_sc.active = 0;
    g_sc.ready  = 0;
    g_capture_dumping = 0;
}

void SpeedCapture_Start(uint16_t samples)
{
    if (g_sc.active || g_sc.ready) return;
    if (samples > SC_BURST_SAMPLES) samples = SC_BURST_SAMPLES;

    g_sc.total  = samples;
    g_sc.wr     = 0;
    g_sc.active = 1;
    g_sc.ready  = 0;
}

void SpeedCapture_Write(float fb, float iq, float ref, float tload)
{
    if (!g_sc.active) return;

    uint16_t w = g_sc.wr;
    if (w < g_sc.total) {
        float *p = g_sc.buf + (uint32_t)w * SC_BURST_FIELDS;
        p[0] = fb;
        p[1] = iq;
        p[2] = ref;
        p[3] = tload;
        g_sc.wr = w + 1;
    }

    if (g_sc.wr >= g_sc.total) {
        g_sc.active = 0;
        g_sc.ready  = 1;
    }
}

void SpeedCapture_Dump(void)
{
    if (!g_sc.ready) return;

    g_capture_dumping = 1;

    uint16_t count  = g_sc.total;
    uint8_t  fields = SC_BURST_FIELDS;

    uint8_t hdr[8];
    uint32_t magic = SC_MAGIC;
    memcpy(hdr,     &magic,  4);
    memcpy(hdr + 4, &count,  2);
    hdr[6] = fields;
    hdr[7] = 0;

    COMM_SendData(hdr, 8);

    // 分批发送 — 每块 200B, 等 DMA 排空再发下一块
    uint16_t data_bytes = count * fields * (uint16_t)sizeof(float);
    uint8_t *p = (uint8_t *)g_sc.buf;
    while (data_bytes > 0) {
        uint16_t chunk = data_bytes > 200 ? 200 : data_bytes;
        COMM_SendData(p, chunk);
        p += chunk;
        data_bytes -= chunk;
        // 等待 DMA 排空环形缓冲区, 避免下一块 COMM_SendData 死等
        while (!COMM_IsTxIdle()) {
            osDelay(1);  // 让出 CPU 给其他任务
        }
    }

    osDelay(50);
    g_sc.ready = 0;
    g_capture_dumping = 0;
}

uint8_t SpeedCapture_IsBusy(void)
{
    return g_sc.active || g_capture_dumping;
}

uint8_t SpeedCapture_IsReady(void)
{
    return g_sc.ready;
}
