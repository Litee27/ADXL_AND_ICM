#ifndef __ICM_H
#define __ICM_H

#include "usart.h"
#include "stm32l1xx_hal.h"
#include <stdio.h>
#include <spi.h>
#include "i2c.h"
#define ICM_CS_PORT 					GPIOC
#define ICM_INT1_PORT 				GPIOB
#define ICM_CS_PIN 						GPIO_PIN_13
#define ICM_INT1_PIN 					GPIO_PIN_3

#define PWR_MGMT0 		0x1F
#define BLK_SEL_W 		0x79 //区块设置
#define MADDR_W 			0x7A //要写入的区块内寄存器地址
#define M_W						0x7B //寄存器写入值
#define BLK_SEL_R 		0x7C //区块设置
#define MADDR_R 			0x7D //要写入的区块内寄存器地址
#define M_R						0x7E //寄存器写入值


#define INT_CONFIG			0x06
#define GYRO_CONFIG0 		0x20
#define ACCEL_CONFIG0 	0x21
#define ACCEL_CONFIG1 	0x24
#define WOM_CONFIG 			0x27
#define INT_SOURCE0	 		0x2B
#define INT_SOURCE1	 		0x2C
#define INT_SOURCE3			0x2D
#define WHO_AM_I				0x75
#define TEMP_CONFIG0		0x22
#define ACCEL_WOM_X_THR	0x4B
#define ACCEL_WOM_Y_THR	0x4C
#define ACCEL_WOM_Z_THR	0x4D
#define APEX_CONFIG0		0x25
#define APEX_CONFIG1		0x26
/*MREG1*/
#define SENSOR_CONFIG3	0x06 
#define APEX_CONFIG2    0x44
#define APEX_CONFIG3    0x45
#define APEX_CONFIG4    0x46
#define APEX_CONFIG5    0x47
#define APEX_CONFIG9    0x48
#define APEX_CONFIG10   0x49
#define INT_SOURCE6	 		0x2F
#define INT_SOURCE7			0x30

#define INT_STATUS3			0x3C
#define ACCELER_SENSITIVITY_2 16384.0
#define ACCELER_SENSITIVITY_4 8192.0
#define ACCELER_SENSITIVITY_8 4096.0
#define ACCELER_SENSITIVITY_16 2048.0

#define GYRO_SENSITIVITY_250 	131.0 
#define DEG_TO_RAD              0.0174532925f  // 精确的弧度转换系数

#define ICM_IIC_ADD (0x68<<1) //MISO(PA6)下拉

#define IIC 0
#define SPI 1
#define IIC_OR_SPI IIC	//选择使用IIC还是SPI


typedef struct ICM_DATA
{
	//加速度值
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	//角度值
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
}ICM_DATA;

extern float accler_sensitivity;
//初始化
void ICM_Init(void);

void ICM_CS_Select(void);
void ICM_CS_Deselect(void);
uint8_t SPI_TransmitReceive(uint8_t data);
uint8_t ICM_ReadRegister(uint8_t reg);
void ICM_WriteRegister(uint8_t reg, uint8_t value);
void ICM_MREG1_WriteRegister(uint8_t Maddr_w,uint8_t M_w);
uint8_t ICM_MREG1_ReadRegister(uint8_t Maddr_r);

//模式与功能
void ICM_MODE_WoM(uint8_t x_thr,uint8_t y_thr,uint8_t z_thr);//Wake on Motion
void ICM_MODE_Sleep(void);
void ICM_Gyro_open(void);//打开陀螺仪和加速度计
void ICM_Read_Data(ICM_DATA* data);//读完后需要重新配置WoM模式
#endif
