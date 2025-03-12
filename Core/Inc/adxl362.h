#ifndef ADXL_H
#define ADXL_H
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

#define ADXL362_CS_PIN   GPIO_PIN_4  // 片选引脚
#define ADXL362_CS_PORT  GPIOA       // 片选端口

#define THRESH_ACT_L			0x20
#define THRESH_ACT_H			0x21
#define TIME_ACT					0x22
#define THRESH_INACT_L		0x23
#define THRESH_INACT_H		0x24
#define TIME_INACT_L			0x25
#define TIME_INACT_H			0x26
#define ACT_INACT_CTL   	0x27
#define INT1							0x2A
#define INT2							0x2B
#define FILTER_CTL				0x2C
#define POWER_CTL					0x2D

#define range_2g 					1.0
#define range_4g 					2.0
#define range_8g 					4.255
	
extern float scale_factor;

void ADXL362_Init(void);
//uint8_t SPI_TransmitReceive(uint8_t data);
uint8_t ADXL362_ReadRegister(uint8_t reg);
void ADXL362_WriteRegister(uint8_t reg, uint8_t value);
void ADXL362_CS_Select(void);
void ADXL362_CS_Deselect(void);
void ADXL362_mode_measure(uint16_t threshold_ACT,uint8_t time_ACT,uint8_t INT_num);
void ADXL362_mode_wakeup(uint16_t threshold_ACT,uint8_t time_ACT,uint16_t threshold_INACT,uint8_t TIME_INACT,uint8_t INT_num);
#endif
