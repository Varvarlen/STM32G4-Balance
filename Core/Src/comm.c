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

// RX 环形缓冲区（供用户读取）
static RingBuffer_t rxBuffer;

// RX DMA 接收缓冲区（CIRCULAR 模式，连续运行）
#define RX_DMA_BUFFER_SIZE 256
static uint8_t rxDmaBuffer[RX_DMA_BUFFER_SIZE];

// RX CIRCULAR DMA 上一轮读取位置
static uint16_t lastRxDmaPos;

// TX 环形缓冲区（暂存待发送数据）
static RingBuffer_t txBuffer;

// TX DMA 发送缓冲区（DMA 从该 buffer 发送，确保内存安全）
static uint8_t txDmaBuffer[DMA_TX_BUFFER_SIZE];

// TX DMA 忙标志
static volatile uint8_t txDmaBusy;

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/**
  * @brief 启动 TX DMA 传输（从环形缓冲区搬运数据到 DMA 缓冲区后发送）
  * @retval None
  */
static void COMM_StartTxDma(void)
{
  uint16_t len = 0;

  if (txDmaBusy)
    return;

  // 从 TX 环形缓冲区搬运数据到 DMA 发送缓冲区
  while (RingBuffer_Available(&txBuffer) > 0 && len < DMA_TX_BUFFER_SIZE)
  {
    txDmaBuffer[len++] = RingBuffer_Read(&txBuffer);
  }

  if (len == 0)
    return;

  txDmaBusy = 1;
  HAL_UART_Transmit_DMA(&huart1, txDmaBuffer, len);
}

/* USER CODE END 1 */



/* USER CODE BEGIN 2 */

/**
  * @brief 初始化通信模块
  * @note 启用 USART1 空闲中断 + CIRCULAR DMA 接收
  * @retval None
  */
void COMM_Init(void)
{
  // 初始化环形缓冲区
  RingBuffer_Init(&rxBuffer);
  RingBuffer_Init(&txBuffer);

  lastRxDmaPos = 0;
  txDmaBusy = 0;

  // 启动 CIRCULAR DMA 接收（连续运行，不再停止）
  HAL_UART_Receive_DMA(&huart1, rxDmaBuffer, RX_DMA_BUFFER_SIZE);

  // 启用空闲中断
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

/**
  * @brief 发送一个字节（非阻塞，复制到内部 TX 缓冲区后由 DMA 异步发送）
  * @param byte: 要发送的字节
  * @retval None
  */
void COMM_SendByte(uint8_t byte)
{
  // 等待 TX 缓冲区有空位
  while (RingBuffer_Available(&txBuffer) == UART_TX_BUFFER_SIZE - 1);

  // 保护临界区 — TX DMA 完成 ISR 也可能读取 txBuffer 并启动 DMA,
  // 此处关闭全局中断防止两个消费者并发操作环形缓冲区
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  RingBuffer_Write(&txBuffer, byte);
  COMM_StartTxDma();
  if (!primask) __enable_irq();
}

/**
  * @brief 发送数据（非阻塞，复制到内部 TX 缓冲区后由 DMA 异步发送）
  * @param data: 待发送数据缓冲区
  * @param length: 数据长度
  * @retval None
  */
void COMM_SendData(const uint8_t *data, uint16_t length)
{
  for (uint16_t i = 0; i < length; i++)
  {
    // 等待 TX 缓冲区有空位
    while (RingBuffer_Available(&txBuffer) == UART_TX_BUFFER_SIZE - 1);
    RingBuffer_Write(&txBuffer, data[i]);
  }
  // 保护临界区 — 防止 TX DMA 完成 ISR 并发读取 txBuffer
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  COMM_StartTxDma();
  if (!primask) __enable_irq();
}

/**
  * @brief UART 发送完成回调
  * @param huart: UART 句柄
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    // DMA 传输完成，清除忙标志
    txDmaBusy = 0;

    // 还有数据待发送则继续
    if (RingBuffer_Available(&txBuffer) > 0)
    {
      COMM_StartTxDma();
    }
  }
}

/**
  * @brief UART 错误回调
  * @param huart: UART 句柄
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    // 发生错误时重新启动 RX DMA
    HAL_UART_Receive_DMA(&huart1, rxDmaBuffer, RX_DMA_BUFFER_SIZE);
    lastRxDmaPos = 0;
  }
}

/**
  * @brief UART DMA 接收半传输回调
  * @param huart: UART 句柄
  * @retval None
  * @note CIRCULAR DMA 模式下，每填满一半缓冲区时调用此函数
  */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    uint16_t half = RX_DMA_BUFFER_SIZE / 2;
    uint16_t received;
    uint16_t i;

    // 计算自上次处理后到半传输点之间接收的字节数
    if (half >= lastRxDmaPos)
    {
      received = half - lastRxDmaPos;
    }
    else
    {
      received = (RX_DMA_BUFFER_SIZE - lastRxDmaPos) + half;
    }

    // 写入环形缓冲区
    for (i = 0; i < received; i++)
    {
      uint16_t idx = (lastRxDmaPos + i) % RX_DMA_BUFFER_SIZE;
      RingBuffer_Write(&rxBuffer, rxDmaBuffer[idx]);
    }

    lastRxDmaPos = half;
  }
}

/**
  * @brief 读取一个字节
  * @retval 读取的字节，无数据时返回 0
  */
uint8_t COMM_ReadByte(void)
{
  if (RingBuffer_Available(&rxBuffer) > 0)
    return RingBuffer_Read(&rxBuffer);
  return 0;
}

/**
  * @brief 检查 RX 缓冲区可用数据长度
  * @retval 可用数据字节数
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
  * @brief 向环形缓冲区写入一个字节
  * @param rb: 环形缓冲区指针
  * @param data: 要写入的数据（缓冲区满时丢弃）
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
  * @brief 从环形缓冲区读取一个字节
  * @param rb: 环形缓冲区指针
  * @retval 读取的字节（缓冲区空时返回 0）
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
  * @brief 查询环形缓冲区中可读取的字节数
  * @param rb: 环形缓冲区指针
  * @retval 可读取字节数
  */
uint16_t RingBuffer_Available(RingBuffer_t *rb)
{
  return (rb->head - rb->tail + UART_RX_BUFFER_SIZE) % UART_RX_BUFFER_SIZE;
}

/**
  * @brief 处理 USART1 空闲中断（CIRCULAR DMA 版）
  * @note 由 stm32g4xx_it.c 的 USART1_IRQHandler 调用
  * @retval None
  */
void COMM_HandleIdleInterrupt(void)
{
  uint16_t currentPos;
  uint16_t received;

  // 清除空闲中断标志
  __HAL_UART_CLEAR_IDLEFLAG(&huart1);

  // 获取 DMA 当前写入位置（CIRCULAR 模式）
  currentPos = RX_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

  // 计算自上次处理以来新接收的字节数（处理 CIRCULAR 回绕）
  if (currentPos >= lastRxDmaPos)
  {
    received = currentPos - lastRxDmaPos;
  }
  else
  {
    received = (RX_DMA_BUFFER_SIZE - lastRxDmaPos) + currentPos;
  }

  // 将新数据写入环形缓冲区
  for (uint16_t i = 0; i < received; i++)
  {
    uint16_t idx = (lastRxDmaPos + i) % RX_DMA_BUFFER_SIZE;
    RingBuffer_Write(&rxBuffer, rxDmaBuffer[idx]);
  }

  lastRxDmaPos = currentPos;

  // CIRCULAR DMA 无需重新启动，持续运行
}

/* USER CODE END 2 */

/* USER CODE BEGIN 3 */

/* USER CODE END 3 */