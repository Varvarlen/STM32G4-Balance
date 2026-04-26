/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    comm_protocol.c
  * @brief   小端浮点数组协议 发送端实现
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "comm_protocol.h"
#include "comm.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/**
  * @brief  发送一帧浮点数组协议数据
  * @param  data   浮点数组（小端 IEEE 754）
  * @param  count  通道数
  * @note   Cortex-M4 原生小端，float 直接按字节拷贝即可
  *         帧结构: [f0][f1]...[fN][00 00 80 7F]
  */
void COMM_SendFloatFrame(const float *data, uint8_t count)
{
    if (count == 0 || count > PROTOCOL_MAX_CHANNELS)
        return;

    uint16_t data_bytes = (uint16_t)count * 4;
    uint8_t buf[sizeof(uint32_t) * (PROTOCOL_MAX_CHANNELS + 1)];

    /* 复制浮点数据（直接字节复制，保持小端序） */
    for (uint16_t i = 0; i < data_bytes; i++)
    {
        buf[i] = ((const uint8_t *)data)[i];
    }

    /* 追加帧尾: +infinity = 0x7F800000 */
    buf[data_bytes]     = 0x00;
    buf[data_bytes + 1] = 0x00;
    buf[data_bytes + 2] = 0x80;
    buf[data_bytes + 3] = 0x7F;

    COMM_SendData(buf, data_bytes + 4);
}

/* USER CODE END 1 */

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/* USER CODE BEGIN 3 */

/* USER CODE END 3 */
