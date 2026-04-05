/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    comm.c
  * @brief   This file provides code for the configuration
  *          of the UART communication with DMA
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

/* Includes ------------------------------------------------------------------*/
#include "comm.h"

/* USER CODE BEGIN 0 */

// 全局变量声明
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

// 环形缓冲区
RingBuffer_t rxBuffer;

// 接收缓冲区
uint8_t rxData;

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */



/* USER CODE BEGIN 2 */

/**
  * @brief 初始化通信模块
  * @retval None
  */
void COMM_Init(void)
{
  // 初始化环形缓冲区
  RingBuffer_Init(&rxBuffer);
  
  // 启动DMA接收
  HAL_UART_Receive_DMA(&huart1, &rxData, 1);
}

/**
  * @brief 发送一个字节
  * @param byte: 要发送的字节
  * @retval None
  */
void COMM_SendByte(uint8_t byte)
{
  HAL_UART_Transmit(&huart1, &byte, 1, HAL_MAX_DELAY);
}

/**
  * @brief 发送数据
  * @param data: 数据缓冲区
  * @param length: 数据长度
  * @retval None
  */
void COMM_SendData(uint8_t *data, uint16_t length)
{
  HAL_UART_Transmit(&huart1, data, length, HAL_MAX_DELAY);
}

/**
  * @brief 读取一个字节
  * @retval 读取的字节
  */
uint8_t COMM_ReadByte(void)
{
  return RingBuffer_Read(&rxBuffer);
}

/**
  * @brief 检查是否有可用数据
  * @retval 可用数据长度
  */
uint16_t COMM_Available(void)
{
  return RingBuffer_Available(&rxBuffer);
}

/**
  * @brief 初始化环形缓冲区
  * @param rb: 环形缓冲区指针
  * @retval None
  */
void RingBuffer_Init(RingBuffer_t *rb)
{
  rb->head = 0;
  rb->tail = 0;
}

/**
  * @brief 向环形缓冲区写入数据
  * @param rb: 环形缓冲区指针
  * @param data: 要写入的数据
  * @retval None
  */
void RingBuffer_Write(RingBuffer_t *rb, uint8_t data)
{
  uint16_t next = (rb->head + 1) % UART_RX_BUFFER_SIZE;
  if (next != rb->tail)
  {
    rb->buffer[rb->head] = data;
    rb->head = next;
  }
}

/**
  * @brief 从环形缓冲区读取数据
  * @param rb: 环形缓冲区指针
  * @retval 读取的数据
  */
uint8_t RingBuffer_Read(RingBuffer_t *rb)
{
  if (rb->head != rb->tail)
  {
    uint8_t data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % UART_RX_BUFFER_SIZE;
    return data;
  }
  return 0;
}

/**
  * @brief 检查环形缓冲区可用数据
  * @param rb: 环形缓冲区指针
  * @retval 可用数据长度
  */
uint16_t RingBuffer_Available(RingBuffer_t *rb)
{
  return (rb->head - rb->tail + UART_RX_BUFFER_SIZE) % UART_RX_BUFFER_SIZE;
}

/**
  * @brief UART接收完成回调函数
  * @param huart: UART句柄
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    // 将接收到的数据写入环形缓冲区
    RingBuffer_Write(&rxBuffer, rxData);
    
    // 重新启动DMA接收
    HAL_UART_Receive_DMA(&huart1, &rxData, 1);
  }
}

/* USER CODE END 2 */

/* USER CODE BEGIN 3 */

/* USER CODE END 3 */