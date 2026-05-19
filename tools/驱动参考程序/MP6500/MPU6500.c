#include "mpu6500.h"


#define OFFSET_AV_NUM 	    100					  //校准偏移量时的平均次数。

#define RAD_TO_DEG     			57.2957795f   // 弧度转角度系数
#define GYRO_WEIGHT    0.98f         			// 陀螺仪数据权重
#define ACCEL_WEIGHT   0.02f         			// 加速度计数据权重
#define DT             0.0005f       			// 积分时间步长（陀螺仪 1kHz采样率）
#define ANGLE_THRESH   90.0f              // 大角度判定阈值


//陀螺仪 加速度计原始数据
uint8_t    Data[14];
xyz_s16_t  gyr_data;			  
xyz_s16_t  acc_data;

xyz_f_t    GYR;				//角速度
xyz_f_t    ACC;				//加速度

xyz_f_t    gyr_angle,acc_angle;
float  	   pitch, roll, yaw;	//俯仰、横滚、偏航

//六轴校准值
xyz_f_t    Acc_Offset={0,0,0};
xyz_f_t    Gyr_Offset={0,0,0};

//校准标志
uint8_t    CALIB_FLAG = 1;

//第一次运行角度融合函数标志
uint8_t    IMU_FLAG=0;   

//处理角速度数据
float IMU_tmp[6];                     //校准后数据
float IMU_filter_tmp[6]={0,0,0,0,0,0};//滤波后数据


#define  IMU_READ    0x80
#define  IMU_WRITE   0x00

uint8_t IMU_Read_Reg(uint8_t Reg)
{
uint8_t Data;
LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_6); 

 SPI2_ReadWriteByte(Reg | IMU_READ);
 Data = SPI2_ReadWriteByte(0xff);

LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_6); 
return Data;
}

uint8_t IMU_Write_Reg(uint8_t Reg,uint8_t Data)
{
uint8_t status;
LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_6); 

 status = SPI2_ReadWriteByte(Reg | IMU_WRITE);
 SPI2_ReadWriteByte(Data);

LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_6); 
return status;
}


//unsigned char IMU_Gyroscope_INIT(void ){

//	if(IMU_Write_Reg(GYRO_CONFIG0,0x00 | 0x05)) return 1;//设置陀螺仪量程与数据输出速率(2000dps 2kHz)
//	delay_ms(50);
//	if(IMU_Write_Reg(GYRO_CONFIG1,0x01 | 0x06)) return 2;//调整带宽和滤波次数
//	delay_ms(50);
//	if(IMU_Write_Reg(GYRO_ACCEL_CONFIG0,0x11)) return 3;//调整陀螺仪和加速度计的低通滤波器带宽
//	delay_ms(50);
//	return 0;
//}

//unsigned char IMU_ACC_INIT(void){

//	if(IMU_Write_Reg(ACCEL_CONFIG0,0x20 | 0x06)) return 1;//设置加速度计量程与数据输出速率(8g 1kHz)
//	delay_ms(50);
//	if(IMU_Write_Reg(ACCEL_CONFIG1,0x0D)) return 2;//调整带宽和滤波次数
//	delay_ms(50);
//	return 0;
//	
//}

uint8_t IMU_Init(void)
{
u8 x;
  LL_GPIO_InitTypeDef GPIO_InitStruct;

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  SPI2_Hardware_Init(LL_SPI_POLARITY_HIGH ,LL_SPI_PHASE_2EDGE);
   x  =IMU_Read_Reg(MPU6500_WHO_AM_I);

		delay_ms(10);
	  IMU_Write_Reg(MPU6500_PWR_MGMT_1,0X80);   		
		delay_ms(10);
		IMU_Write_Reg(MPU6500_PWR_MGMT_1,0X01);   		
		delay_ms(10);
		IMU_Write_Reg(MPU6500_SIGNAL_PATH_RESET,0X07);
		delay_ms(10);
		IMU_Write_Reg(MPU6500_CONFIG,0X0);					
		delay_ms(10);
		IMU_Write_Reg(MPU6500_GYRO_CONFIG,0x18);  		
		delay_ms(10);
		IMU_Write_Reg(MPU6500_ACCEL_CONFIG,0x10); 		
		delay_ms(10);

	return x;
}

void GYRO_ACC_TEMP_GET(uint8_t * Data)
{
uint8_t i;
LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_6); 

SPI2_ReadWriteByte(MPU6500_ACCEL_XOUT_H | IMU_READ);

for(i=0;i<14;i++)
  {
   Data[i] = SPI2_ReadWriteByte(0xff);
  }
LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_6); 

}


int sum_temp[6]={0,0,0,0,0,0};


void MPU6050_Data_Offset(void)
{
u16 sum_cnt = 0;

do{
   sum_cnt++;
    //获取原始数据
  GYRO_ACC_TEMP_GET(Data);
  
	acc_data.x = Data[0]<<8 | Data[1];
	acc_data.y = Data[2]<<8 | Data[3];
	acc_data.z = Data[4]<<8 | Data[5];

	gyr_data.x = Data[8]<<8 | Data[9];
	gyr_data.y = Data[10]<<8 | Data[11];
	gyr_data.z = Data[12]<<8 | Data[13];


//		sum_temp[0] += acc_data.x;
//		sum_temp[1] += acc_data.y;
//		sum_temp[2] += acc_data.z;   // +-8G
		sum_temp[3] += gyr_data.x;
		sum_temp[4] += gyr_data.y;
		sum_temp[5] += gyr_data.z;

    if(sum_cnt >= OFFSET_AV_NUM)
    {
			Acc_Offset.x = 0;
			Acc_Offset.y = 0;
			Acc_Offset.z = 0;

			Gyr_Offset.x = (float)sum_temp[3]/OFFSET_AV_NUM;
			Gyr_Offset.y = (float)sum_temp[4]/OFFSET_AV_NUM;
			Gyr_Offset.z = (float)sum_temp[5]/OFFSET_AV_NUM;
			
			//sum_temp[0] = sum_temp[1] = sum_temp[2] = 
      sum_temp[3] = sum_temp[4] = sum_temp[5] = 0;

      // CALIB_FLAG =0;
      data_save();
     }

   }while(sum_cnt < OFFSET_AV_NUM);
}



void Gyr_Data_handle(void)
{
   //获取原始数据
  GYRO_ACC_TEMP_GET(Data);
  
	acc_data.x = Data[0]<<8 | Data[1];
	acc_data.y = Data[2]<<8 | Data[3];
	acc_data.z = Data[4]<<8 | Data[5];

	gyr_data.x = Data[8]<<8 | Data[9];
	gyr_data.y = Data[10]<<8 | Data[11];
	gyr_data.z = Data[12]<<8 | Data[13];

		/* 得出校准后的数据 */

	IMU_tmp[0] = acc_data.x  * 0.000244140625f;
	IMU_tmp[1] = acc_data.y  * 0.000244140625f;
	IMU_tmp[2] = acc_data.z  * 0.000244140625f;
	
	IMU_tmp[3] = (gyr_data.x  - Gyr_Offset.x) * 0.060975609756f;
	IMU_tmp[4] = (gyr_data.y  - Gyr_Offset.y) * 0.060975609756f;
	IMU_tmp[5] = (gyr_data.z  - Gyr_Offset.z) * 0.060975609756f;



#define ACC_ALPHA_STATIC  0.01f  // 静态时加速度滤波系数
#define GYR_ALPHA_STATIC  0.05f // 静态时陀螺仪滤波系数

    float acc_alpha = ACC_ALPHA_STATIC;
    float gyr_alpha = GYR_ALPHA_STATIC;


//// z轴加速度动态检测（基于重力变化）z轴角速度原始值在0度时约为4096 
//    float gravity_dev = IMU_tmp[2] - ACC_THRESH;
//  
//    if(gravity_dev < 0) {
//        acc_alpha = ACC_ALPHA_DYNAMIC;
//    }

//    // 陀螺仪动态检测（基于角速度幅度）
//    float gyro_mag = fabsf(IMU_tmp[3]) + fabsf(IMU_tmp[4]) + fabsf(IMU_tmp[5]);

//    if(gyro_mag > GYRO_THRESH) {
//        gyr_alpha = GYR_ALPHA_DYNAMIC;
//    }

    // 3. 应用动态滤波
    // 加速度滤波（XYZ）
    IMU_filter_tmp[0] += (IMU_tmp[0] - IMU_filter_tmp[0]) * acc_alpha;
    IMU_filter_tmp[1] += (IMU_tmp[1] - IMU_filter_tmp[1]) * acc_alpha;
    IMU_filter_tmp[2] += (IMU_tmp[2] - IMU_filter_tmp[2]) * acc_alpha;
    
    // 陀螺仪滤波（XYZ）
    IMU_filter_tmp[3] += (IMU_tmp[3] - IMU_filter_tmp[3]) * gyr_alpha;
    IMU_filter_tmp[4] += (IMU_tmp[4] - IMU_filter_tmp[4]) * gyr_alpha;
    IMU_filter_tmp[5] += (IMU_tmp[5] - IMU_filter_tmp[5]) * gyr_alpha;

	/*坐标转换*/

	ACC.x = IMU_filter_tmp[0];
	ACC.y = IMU_filter_tmp[1];
	ACC.z = IMU_filter_tmp[2];

	GYR.x = IMU_filter_tmp[3] ;
	GYR.y = IMU_filter_tmp[4];
	GYR.z = IMU_filter_tmp[5] + 0.3f;;


  acc_angle.y = -atan2f(IMU_filter_tmp[0], sqrtf(IMU_filter_tmp[1]*IMU_filter_tmp[1] + IMU_filter_tmp[2]*IMU_filter_tmp[2])) * RAD_TO_DEG;  // 俯仰角[2,5](@ref)
  acc_angle.x = -atan2f(IMU_filter_tmp[1], sqrtf(IMU_filter_tmp[0]*IMU_filter_tmp[0] + IMU_filter_tmp[2]*IMU_filter_tmp[2])) * RAD_TO_DEG;  // 横滚角[2,5](@ref)

	if(IMU_FLAG == 0) // 使用加速度计算的姿态角赋初值
	{
		IMU_FLAG = 1;
		gyr_angle.y = acc_angle.y;
    gyr_angle.x = acc_angle.x;
	}
	
	if(fabsf(acc_angle.y) > 90.0f) 
	{
		gyr_angle.y = acc_angle.y;
    gyr_angle.x = acc_angle.x;
	}
	else
	{
		gyr_angle.y += IMU_filter_tmp[4] * DT; // 角速度积分
	  gyr_angle.y = gyr_angle.y * 0.999f + acc_angle.y * 0.001f; // 陀螺积分角度与加速度倾角进行融合
		pitch = gyr_angle.y;

		gyr_angle.x += IMU_filter_tmp[3] * DT; // 角速度积分
	  gyr_angle.x = gyr_angle.x * 0.98f + acc_angle.x * 0.02f; // 陀螺积分角度与加速度倾角进行融合
		roll = gyr_angle.x;
	}
}
