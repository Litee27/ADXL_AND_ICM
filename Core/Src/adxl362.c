#include "adxl362.h"

float scale_factor;
void ADXL362_Init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStruct ;

	__HAL_RCC_GPIOA_CLK_ENABLE();  // 启用 GPIOA 时钟

	GPIO_InitStruct.Pin = GPIO_PIN_4;  // 配置 PA4 引脚
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // 配置为复用推挽输出
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull=GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  // 初始化 GPIOA 引脚
	
    // 写入软复位命令
    ADXL362_WriteRegister(0x1F, 0x52);
    HAL_Delay(10);

    // 配置测量模式，量程 ±2g
    ADXL362_WriteRegister(0x2D, 0x02); // 开启测量模式
}
// ADXL362 片选控制
void ADXL362_CS_Select(void)
{
    HAL_GPIO_WritePin(ADXL362_CS_PORT, ADXL362_CS_PIN, GPIO_PIN_RESET);
}

void ADXL362_CS_Deselect(void)
{
    HAL_GPIO_WritePin(ADXL362_CS_PORT, ADXL362_CS_PIN, GPIO_PIN_SET);
}

uint8_t SPI_TransmitReceive(uint8_t data)
{
    uint8_t rxData = 0;
    if(HAL_SPI_TransmitReceive(&hspi1, &data, &rxData, 1, HAL_MAX_DELAY)!=HAL_OK)
			printf("SPI传输错误,传输值为:%02x\r\n",data);
    return rxData;
}
uint8_t ADXL362_ReadRegister(uint8_t reg)
{
    uint8_t value;
    ADXL362_CS_Select();
    SPI_TransmitReceive(0x0B); // 读寄存器命令
    SPI_TransmitReceive(reg); // 寄存器地址
    value = SPI_TransmitReceive(0x00); // 读取值
    ADXL362_CS_Deselect();
    return value;
}
void ADXL362_WriteRegister(uint8_t reg, uint8_t value)
{
    ADXL362_CS_Select();
    SPI_TransmitReceive(0x0A); // 写寄存器命令
    SPI_TransmitReceive(reg); // 寄存器地址
    SPI_TransmitReceive(value); // 写入值
    ADXL362_CS_Deselect();
}
uint16_t ADXL362_ReadFIFO(uint8_t size)
{
    uint16_t value;
    ADXL362_CS_Select();
    SPI_TransmitReceive(0x0D); // 读取FIFO
		value = (SPI_TransmitReceive(0x00)<<8)|SPI_TransmitReceive(0x00);
    ADXL362_CS_Deselect();
    return value;
}

/*测量模式
threshold_ACT 绝对模式下活动阈值越小越灵敏
TIME_ACT			活动检测时间，在时间内超出阈值则判定为运动，触发中断引脚
INT_NUM				INT1、INT2
*/
void ADXL362_mode_measure(uint16_t threshold_ACT,uint8_t time_ACT,uint8_t INT_num)
{
	printf("0xFF|threshold_ACT:%04x\r\n",0xFF&threshold_ACT);
	printf("0xFF|threshold_ACT>>8:%02x\r\n",0xFF&(threshold_ACT>>8));
		ADXL362_WriteRegister(THRESH_ACT_L, 0xFF&threshold_ACT);//活动阈值
		ADXL362_WriteRegister(THRESH_ACT_H, 0xFF&(threshold_ACT>>8));//活动阈值
		ADXL362_WriteRegister(TIME_ACT, time_ACT);//活动测量10ms，10ms内加速度满足设定阈值，则判断为活动
		ADXL362_WriteRegister(ACT_INACT_CTL, 0x03);//活动检查使能，绝对模式
		if(INT_num==INT1||INT_num==INT2)
			ADXL362_WriteRegister(INT_num, 0x10);//映射到INT1或INT2，活动中断
		else
			printf("INT_NUM只能为INT1或INT2!\r\n");
		ADXL362_WriteRegister(FILTER_CTL, 0x83);//+-8g范围
		ADXL362_WriteRegister(POWER_CTL, 0x02);//测量模式
		
		uint8_t range = (ADXL362_ReadRegister(FILTER_CTL) & 0xC0)>>6;
		printf("range=%d\r\n",range);
		switch(range){
			case(0):
				scale_factor=range_2g;
			case(1):
				scale_factor=range_4g;
			default:
				scale_factor=range_8g;
		}
}

/*唤醒模式
threshold_ACT 	相对模式下活动阈值越小越灵敏
TIME_ACT				活动检测时间，在时间内超出阈值则判定为运动，触发中断引脚
threshold_INACT 相对模式下非活动阈值越大越灵敏
TIME_INACT			非活动检测时间，在时间内超出阈值则判定为运动，触发中断引脚
INT_NUM					INT1、INT2
*/
void ADXL362_mode_wakeup(uint16_t threshold_ACT,uint8_t time_ACT,uint16_t threshold_INACT,uint8_t time_INACT,uint8_t INT_num)
{

		ADXL362_WriteRegister(THRESH_ACT_L, 0xFF&threshold_ACT);//活动阈值
		ADXL362_WriteRegister(THRESH_ACT_H, 0xFF&(threshold_ACT>>8));//活动阈值
		ADXL362_WriteRegister(TIME_ACT, time_ACT);//活动判定时间

		ADXL362_WriteRegister(THRESH_INACT_L, 0xFF&threshold_INACT);//静止阈值
		ADXL362_WriteRegister(THRESH_INACT_H, 0xFF&(threshold_INACT>>8));//静止阈值
		ADXL362_WriteRegister(TIME_INACT_L, time_INACT & 0xFF);//静止判定时间
		ADXL362_WriteRegister(TIME_INACT_H, (time_INACT >> 8) & 0xFF);
		ADXL362_WriteRegister(ACT_INACT_CTL, 0x3F);//环路模式、相对模式、活动静止使能
		if(INT_num==INT1||INT_num==INT2)
			ADXL362_WriteRegister(INT_num, 0x40);//映射到INT1或INT2，活动中断
		else
			printf("INT_NUM只能为INT1或INT2!\r\n");
		ADXL362_WriteRegister(FILTER_CTL, 0x83);//+-8g范围
		ADXL362_WriteRegister(POWER_CTL, 0x1A);//在唤醒模式下进行测量
		
		uint8_t range = ADXL362_ReadRegister(FILTER_CTL) & 0xC0;
		switch(range){
			case(0):
				scale_factor=range_2g;
			case(1):
				scale_factor=range_4g;
			default:
				scale_factor=range_8g;
		}
}
