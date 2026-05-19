#include "SPI.h"

/*函数功能:初始化硬件SPI1
  入口参数:CPOL：极性 0,1
           CPHA：相位 0,1
  返回值:  无
*/
u8 SPI3_Hardware_Init(uint32_t CPOL,uint32_t CPHA)
{
//if((CPOL!=0&&CPOL!=1)||(CPHA!=0&&CPHA!=1))return 1;

  LL_SPI_InitTypeDef SPI_InitStruct;
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  


  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH ;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO ;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH ;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO ;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**SPI1 GPIO Configuration
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  */
//  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH ;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP ;
//  //GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
//  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

//  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH ;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP ;
//  //GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
//  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//   

 /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;//设置 SPI 的单双向模式(双线全双工)
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;//设置 SPI 的主/从机端模式
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;//设置 SPI 的数据帧长度，可选 8/16 位
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;//设置时钟极性 CPOL，可选高/低电平
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;//设置时钟相位，可选奇/偶数边沿采样
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;//设置 NSS 引脚由 SPI 硬件控制还是软件控制
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV64;//设置时钟分频因子
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;//设置 MSB/LSB 顺序
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_ENABLE;
  SPI_InitStruct.CRCPoly = 7;//设置 CRC  CRC 值计算的多项式
  LL_SPI_Init(SPI3, &SPI_InitStruct);

  LL_SPI_SetStandard(SPI3, LL_SPI_PROTOCOL_MOTOROLA);

  LL_SPI_SetNSSMode(SPI3,LL_SPI_NSS_SOFT);
  LL_SPI_DisableNSSPulseMgt(SPI3);//禁用内部NSS

  LL_SPI_SetRxFIFOThreshold(SPI3, LL_SPI_RX_FIFO_TH_QUARTER);
  //LL_SPI_EnableDMAReq_TX(SPI1);
  //LL_SPI_EnableDMAReq_RX(SPI1);
  LL_SPI_Enable(SPI3);

 return 0;
}


u8 SPI2_Hardware_Init(uint32_t CPOL,uint32_t CPHA)
{
//if((CPOL!=0&&CPOL!=1)||(CPHA!=0&&CPHA!=1))return 1;

  LL_SPI_InitTypeDef SPI_InitStruct;
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  
  /**SPI1 GPIO Configuration
//  PB13   ------> SPI2_SCK
//  PB14   ------> SPI2_MISO
//  PB15   ------> SPI2_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH ;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP ;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH ;
  //GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP ;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
   
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH ;
  //GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP ;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;//设置 SPI 的单双向模式(双线全双工)
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;//设置 SPI 的主/从机端模式
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;//设置 SPI 的数据帧长度，可选 8/16 位
  SPI_InitStruct.ClockPolarity = CPOL;//设置时钟极性 CPOL，可选高/低电平
  SPI_InitStruct.ClockPhase = CPHA;//设置时钟相位，可选奇/偶数边沿采样
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;//设置 NSS 引脚由 SPI 硬件控制还是软件控制
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;//设置时钟分频因子
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;//设置 MSB/LSB 顺序
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_ENABLE;
  SPI_InitStruct.CRCPoly = 7;//设置 CRC  CRC 值计算的多项式
  LL_SPI_Init(SPI2, &SPI_InitStruct);

  LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);

  LL_SPI_SetNSSMode(SPI2,LL_SPI_NSS_SOFT);
  LL_SPI_DisableNSSPulseMgt(SPI2);//禁用内部NSS

  LL_SPI_SetRxFIFOThreshold(SPI2, LL_SPI_RX_FIFO_TH_QUARTER);
  //LL_SPI_EnableDMAReq_TX(SPI1);
  //LL_SPI_EnableDMAReq_RX(SPI1);
  LL_SPI_Enable(SPI2);
 return 0;
}
/*函数功能:发送并接收一个字节
  入口参数:要发送的字节
  返回值:  接收的字节
*/

uint8_t SPI3_ReadWriteByte(uint8_t TxData)
{

 //while(LL_I2S_IsActiveFlag_TXE(SPI1) !=1 );//等待发送完成

 //LL_SPI_TransmitData8(SPI1,TxData);
 *((uint8_t*)&(SPI3->DR)) = TxData;

 while (LL_I2S_IsActiveFlag_BSY(SPI3) == 1);//等待忙标志清零 
 //while (LL_I2S_IsActiveFlag_RXNE(SPI1) != 1); //等待接收完成

 //Rxdata = LL_SPI_ReceiveData8

 return  *((uint8_t*)&(SPI3->DR)); //返回接收数据
}

__inline uint8_t SPI2_ReadWriteByte(uint8_t TxData)
{

 //while(LL_I2S_IsActiveFlag_TXE(SPI1) !=1 );//等待发送完成

 //LL_SPI_TransmitData8(SPI1,TxData);
 *((uint8_t*)&(SPI2->DR)) = TxData;

 while (LL_I2S_IsActiveFlag_BSY(SPI2) == 1);//等待忙标志清零 
 //while (LL_I2S_IsActiveFlag_RXNE(SPI1) != 1); //等待接收完成

 //Rxdata = LL_SPI_ReceiveData8

 return  *((uint8_t*)&(SPI2->DR)); //返回接收数据
}

/*
  CPOL=0；CPHA=0；
  函数功能:软件模拟SPI
  入口参数:发送
  返回值:  接收
*/
#define SPI_SCK(x)  (x==0?(GPIOA->BSRR|= (LL_GPIO_PIN_5<<16)):(GPIOA->BSRR|= (LL_GPIO_PIN_5)))

u8 SPI_ReadWriteByte(void)
{
 uint8_t i;
 uint8_t Rxdata=0;
for(i=0;i<8;i++)
  {
//   if(dat&0x80)
//     MOSI=1;
//   else 
//     MOSI=0;
//   dat<<=1;
   
  LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_10); //拉高时钟线锁存数据
__NOP();__NOP();//delay_10ns(1);
LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_10);//拉低时钟线转移数据
__NOP();__NOP();
 Rxdata<<=1;
   if(LL_GPIO_IsInputPinSet(GPIOC,LL_GPIO_PIN_11))
     Rxdata++; //主机读取数据
__NOP();
     //delay_10ns(1);
 
  }
return Rxdata;
}

