#include "bt_comm.h"
#include <string.h>

/* ===== 外部 USART2/DMA 句柄 (CubeMX 生成) ===== */
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

/* ===== RX 环形缓冲区 ===== */
static BT_RingBuffer_t rxBuf;

/* ===== RX DMA CIRCULAR 缓冲区 ===== */
#define BT_RX_DMA_BUF_SIZE 256
static uint8_t rxDmaBuf[BT_RX_DMA_BUF_SIZE];
static uint16_t lastRxDmaPos;

/* ===== TX 环形缓冲区 ===== */
static BT_RingBuffer_t txBuf;

/* ===== TX DMA 缓冲区 ===== */
static uint8_t txDmaBuf[BT_DMA_TX_BUFFER_SIZE];
static volatile uint8_t txDmaBusy;

/* ===== 环形缓冲区操作 ===== */

static void BT_RingBuffer_Init(BT_RingBuffer_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->overflow_cnt = 0;
}

static void BT_RingBuffer_Write(BT_RingBuffer_t *rb, uint8_t data)
{
    uint16_t next = (rb->head + 1) % BT_UART_RX_BUFFER_SIZE;
    if (next != rb->tail) {
        rb->buffer[rb->head] = data;
        rb->head = next;
    } else {
        rb->overflow_cnt++;
    }
}

static uint8_t BT_RingBuffer_Read(BT_RingBuffer_t *rb)
{
    if (rb->head != rb->tail) {
        uint8_t data = rb->buffer[rb->tail];
        rb->tail = (rb->tail + 1) % BT_UART_RX_BUFFER_SIZE;
        return data;
    }
    return 0;
}

static uint16_t BT_RingBuffer_Available(BT_RingBuffer_t *rb)
{
    return (rb->head - rb->tail + BT_UART_RX_BUFFER_SIZE) % BT_UART_RX_BUFFER_SIZE;
}

/* ===== TX DMA 启动 ===== */

static void BT_COMM_StartTxDma(void)
{
    if (txDmaBusy) return;

    uint16_t len = 0;
    while (BT_RingBuffer_Available(&txBuf) > 0 && len < BT_DMA_TX_BUFFER_SIZE) {
        txDmaBuf[len++] = BT_RingBuffer_Read(&txBuf);
    }

    if (len == 0) return;

    txDmaBusy = 1;
    HAL_UART_Transmit_DMA(&huart2, txDmaBuf, len);
}

/* ===== 公开接口 ===== */

/** @brief 初始化 USART2 通信模块
 *  @note  启动 CIRCULAR DMA 接收 + 空闲中断
 */
void BT_COMM_Init(void)
{
    BT_RingBuffer_Init(&rxBuf);
    BT_RingBuffer_Init(&txBuf);
    lastRxDmaPos = 0;
    txDmaBusy = 0;

    HAL_UART_Receive_DMA(&huart2, rxDmaBuf, BT_RX_DMA_BUF_SIZE);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}

/** @brief 发送一个字节 (非阻塞, DMA)
 *  @note  环形缓冲满时丢弃字节并递增溢出计数, 避免高优先级任务死锁
 */
void BT_COMM_SendByte(uint8_t byte)
{
    if (BT_RingBuffer_Available(&txBuf) >= BT_UART_TX_BUFFER_SIZE - 1) {
        txBuf.overflow_cnt++;
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    BT_RingBuffer_Write(&txBuf, byte);
    BT_COMM_StartTxDma();
    if (!primask) __enable_irq();
}

/** @brief 发送数据 (非阻塞, DMA) */
void BT_COMM_SendData(const uint8_t *data, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++) {
        while (BT_RingBuffer_Available(&txBuf) == BT_UART_TX_BUFFER_SIZE - 1);
        BT_RingBuffer_Write(&txBuf, data[i]);
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    BT_COMM_StartTxDma();
    if (!primask) __enable_irq();
}

/** @brief 查询 TX 是否空闲 */
uint8_t BT_COMM_IsTxIdle(void)
{
    return (txDmaBusy == 0 && BT_RingBuffer_Available(&txBuf) == 0) ? 1 : 0;
}

/** @brief 从 RX 环形缓冲区读取一个字节 */
uint8_t BT_COMM_ReadByte(void)
{
    if (BT_RingBuffer_Available(&rxBuf) > 0)
        return BT_RingBuffer_Read(&rxBuf);
    return 0;
}

/** @brief 查询 RX 环形缓冲区可用数据字节数 */
uint16_t BT_COMM_Available(void)
{
    return BT_RingBuffer_Available(&rxBuf);
}

/* ===== 中断回调 ===== */

/** @brief USART2 空闲中断处理 (由 USART2_IRQHandler 调用) */
void BT_COMM_HandleIdleInterrupt(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);

    uint16_t currentPos = BT_RX_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

    uint16_t received;
    if (currentPos >= lastRxDmaPos) {
        received = currentPos - lastRxDmaPos;
    } else {
        received = (BT_RX_DMA_BUF_SIZE - lastRxDmaPos) + currentPos;
    }

    for (uint16_t i = 0; i < received; i++) {
        uint16_t idx = (lastRxDmaPos + i) % BT_RX_DMA_BUF_SIZE;
        BT_RingBuffer_Write(&rxBuf, rxDmaBuf[idx]);
    }

    lastRxDmaPos = currentPos;
}

/** @brief TX DMA 完成回调 (由 comm.c HAL_UART_TxCpltCallback 调用) */
void BT_COMM_OnTxComplete(void)
{
    txDmaBusy = 0;
    if (BT_RingBuffer_Available(&txBuf) > 0) {
        BT_COMM_StartTxDma();
    }
}

/** @brief RX DMA 错误恢复 (由 comm.c HAL_UART_ErrorCallback 调用) */
void BT_COMM_ReinitRxDma(void)
{
    HAL_UART_Receive_DMA(&huart2, rxDmaBuf, BT_RX_DMA_BUF_SIZE);
    lastRxDmaPos = 0;
}

/* ===== 二进制帧接口 ===== */

/** @brief 打包并发送下行遥测帧 (16字节)
 *  字段顺序: 0xA5 | flags(bool) | avg_speed(short) | vbus(short) | uptime(int) | tilt(float) | csum | 0x5A
 */
void BT_SendTelemetryFrame(float tilt, int16_t avg_speed, int16_t vbus,
                           int32_t uptime, uint8_t flags)
{
    uint8_t buf[BT_TELEM_FRAME_LEN];
    buf[0] = BT_FRAME_HEADER;       // 0xA5
    buf[1] = flags;                 // bool(1B)

    // short avg_speed (2B, 小端)
    buf[2] = (uint8_t)(avg_speed & 0xFF);
    buf[3] = (uint8_t)((avg_speed >> 8) & 0xFF);

    // short vbus (2B, 小端)
    buf[4] = (uint8_t)(vbus & 0xFF);
    buf[5] = (uint8_t)((vbus >> 8) & 0xFF);

    // int uptime (4B, 小端)
    buf[6] = (uint8_t)(uptime & 0xFF);
    buf[7] = (uint8_t)((uptime >> 8) & 0xFF);
    buf[8] = (uint8_t)((uptime >> 16) & 0xFF);
    buf[9] = (uint8_t)((uptime >> 24) & 0xFF);

    // float tilt (4B, 小端)
    memcpy(&buf[10], &tilt, 4);

    // 校验和: 字节1~13 求和低8位
    uint8_t csum = 0;
    for (int i = 1; i < 14; i++) {
        csum += buf[i];
    }
    buf[14] = csum;
    buf[15] = BT_FRAME_FOOTER;      // 0x5A

    BT_COMM_SendData(buf, BT_TELEM_FRAME_LEN);
}

/** @brief 计算校验和 (求和低8位) */
uint8_t BT_CalcChecksum(const uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) sum += data[i];
    return sum;
}
