#ifndef __BT_COMM_H__
#define __BT_COMM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ===== 缓冲区大小 ===== */
#define BT_UART_RX_BUFFER_SIZE  256
#define BT_UART_TX_BUFFER_SIZE  256
#define BT_DMA_TX_BUFFER_SIZE   256

/* ===== 帧常量 ===== */
#define BT_FRAME_HEADER  0xA5
#define BT_FRAME_FOOTER  0x5A
#define BT_CTRL_FRAME_LEN   8   /**< 上行控制帧: 0xA5 + payload(5B) + csum + 0x5A */
#define BT_TELEM_FRAME_LEN 16   /**< 下行遥测帧: 0xA5 + payload(13B) + csum + 0x5A */

/* ===== 环形缓冲区 ===== */
typedef struct {
    uint8_t buffer[BT_UART_RX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t overflow_cnt;  /**< 满丢弃计数 */
} BT_RingBuffer_t;

/* ===== 初始化 ===== */
void BT_COMM_Init(void);

/* ===== 发送 (非阻塞, DMA) ===== */
void BT_COMM_SendByte(uint8_t byte);
void BT_COMM_SendData(const uint8_t *data, uint16_t length);
uint8_t BT_COMM_IsTxIdle(void);

/* ===== 接收 ===== */
uint8_t BT_COMM_ReadByte(void);
uint16_t BT_COMM_Available(void);

/* ===== 中断处理 (供 stm32g4xx_it.c / comm.c 调用) ===== */
void BT_COMM_HandleIdleInterrupt(void);
void BT_COMM_OnTxComplete(void);
void BT_COMM_ReinitRxDma(void);

/* ===== 二进制帧接口 ===== */
void BT_SendTelemetryFrame(float tilt, int16_t avg_speed, int16_t vbus,
                           int32_t uptime, uint8_t flags);
uint8_t BT_CalcChecksum(const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __BT_COMM_H__ */
