#ifndef __ICM_H
#define __ICM_H

#include "usart.h"
#include "stm32l1xx_hal.h"
#include <stdio.h>
#include "adxl362.h"

#define ICM_CS_PORT 					GPIOC
#define ICM_CS_PIN 						GPIO_PIN_13

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
#define WOM_CONFIG 			0x27
#define INT_SOURCE1	 		0x2C
#define WHO_AM_I				0x75
#define TEMP_CONFIG0		0x22
#define ACCEL_WOM_X_THR	0x4B
#define ACCEL_WOM_Y_THR	0x4C
#define ACCEL_WOM_Z_THR	0x4D

#define ACCELER_SENSITIVITY_2 16384.0
#define ACCELER_SENSITIVITY_4 8192.0
#define ACCELER_SENSITIVITY_8 4096.0
#define ACCELER_SENSITIVITY_16 2048.0

#define GYRO_SENSITIVITY_250 	131.0 

extern float accler_sensitivity;
void ICM_Init(void);
void ICM_CS_Select(void);
void ICM_CS_Deselect(void);
uint8_t ICM_ReadRegister(uint8_t reg);
void ICM_WriteRegister(uint8_t reg, uint8_t value);
void ICM_MREG1_WriteRegister(uint8_t Maddr_w,uint8_t M_w);
uint8_t ICM_MREG1_ReadRegister(uint8_t Maddr_r);
void ICM_MODE_WoM(void);
#endif
