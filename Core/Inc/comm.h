/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    comm.h
  * @brief   This file contains all the function prototypes for
  *          the comm.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMM_H__
#define __COMM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
// 环形缓冲区大小
#define UART_RX_BUFFER_SIZE 256
#define UART_TX_BUFFER_SIZE 256

// DMA 发送缓冲区大小（单次 DMA 最大发送量）
#define DMA_TX_BUFFER_SIZE 256

// 环形缓冲区结构
typedef struct {
    uint8_t buffer[UART_RX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t overflow_cnt;  // 满丢弃计数
} RingBuffer_t;

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */
// 初始化函数
void COMM_Init(void);

// 发送函数（非阻塞，数据会复制到内部缓冲区）
void COMM_SendByte(uint8_t byte);
void COMM_SendData(const uint8_t *data, uint16_t length);
uint8_t COMM_IsTxIdle(void);  // DMA 空闲且环形缓冲区排空

// 接收函数
uint8_t COMM_ReadByte(void);
uint8_t COMM_PeekByte(void);  // 偷看一眼不消耗
uint16_t COMM_Available(void);

// 空闲中断处理（供 stm32g4xx_it.c 调用）
void COMM_HandleIdleInterrupt(void);

// 环形缓冲区操作
void RingBuffer_Init(RingBuffer_t *rb);
void RingBuffer_Write(RingBuffer_t *rb, uint8_t data);
uint8_t RingBuffer_Read(RingBuffer_t *rb);
uint16_t RingBuffer_Available(RingBuffer_t *rb);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __COMM_H__ */