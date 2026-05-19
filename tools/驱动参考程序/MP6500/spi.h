#ifndef SPI_H_
#define SPI_H_
#include "system.h"

//与从器件通信时要根据其通信协议设置极性与相位
//极性：时钟在空闲时的电平
//相位：第一个时钟源或第二个时钟源采样 （从器件需不需要一个时钟沿启动）
//将检测接收缓冲区是否有数据 由判断接收非空标志改为判断忙标志   判断接收标志会陷入死循环与其他错误
u8 SPI3_Hardware_Init(uint32_t CPOL,uint32_t CPHA);
u8 SPI3_ReadWriteByte(u8 TxData);//启动传输

u8 SPI2_Hardware_Init(uint32_t CPOL,uint32_t CPHA);
u8 SPI2_ReadWriteByte(uint8_t TxData);//启动传输

u8 SPI_ReadWriteByte(void);
#endif
