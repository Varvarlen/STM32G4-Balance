/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    comm_protocol.h
  * @brief   小端浮点数组协议 发送端
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __COMM_PROTOCOL_H__
#define __COMM_PROTOCOL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/** @brief 帧尾标识（+infinity 小端表示: 0x7F800000）*/
#define PROTOCOL_FOOTER_WORD  0x7F800000U

/** @brief 单帧最大通道数 */
#define PROTOCOL_MAX_CHANNELS  10

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

/** @brief 发送一帧浮点数组协议数据
  * @param  data   浮点数组
  * @param  count  通道数（1 ~ PROTOCOL_MAX_CHANNELS）
  * @note  帧格式: [float1][float2]...[floatN][00 00 80 7F]
  *         总长度 = count * 4 + 4 字节
  */
void COMM_SendFloatFrame(const float *data, uint8_t count);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __COMM_PROTOCOL_H__ */
