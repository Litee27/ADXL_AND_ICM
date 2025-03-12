#include "ICM.h"
#include <stdint.h>
#include <stdio.h>
#include "i2c.h"

float accler_sensitivity;
void ICM_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStruct ;

	__HAL_RCC_GPIOC_CLK_ENABLE();  
  __HAL_RCC_GPIOB_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = ICM_CS_PIN;   //CS引脚
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // 配置为复用推挽输出
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull=GPIO_PULLUP;
	HAL_GPIO_Init(ICM_CS_PORT, &GPIO_InitStruct);  
	
	GPIO_InitStruct.Pin = ICM_INT1_PIN;//INT1引脚
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ICM_INT1_PORT, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 1);//INT1中断
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	
}

void ICM_CS_Select(void)
{
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_RESET);
}

void ICM_CS_Deselect(void)
{
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);
}
uint8_t SPI_TransmitReceive(uint8_t data)
{
    uint8_t rxData = 0;
    if(HAL_SPI_TransmitReceive(&hspi1, &data, &rxData, 1, HAL_MAX_DELAY)!=HAL_OK)
			printf("SPI传输错误,传输值为:%02x\r\n",data);
    return rxData;
}
uint8_t ICM_ReadRegister(uint8_t reg)
{
	if(IIC_OR_SPI==SPI)
	{
		uint8_t value;
    ICM_CS_Select();
    SPI_TransmitReceive(0x80|reg); // 读寄存器命令,最高位置1

    value = SPI_TransmitReceive(0x00); // 读取值
    ICM_CS_Deselect();
    return value;
	}
		else if(IIC_OR_SPI==IIC)
	{
		return ICM_IIC_Read(reg);
	}
	
}

void ICM_WriteRegister(uint8_t reg, uint8_t value)
{
	if(IIC_OR_SPI==SPI)
	{
	  ICM_CS_Select();
    SPI_TransmitReceive(reg); // 写寄存器命令，最高位置0
    SPI_TransmitReceive(value); // 写入值
    ICM_CS_Deselect();
	}
	else if(IIC_OR_SPI==IIC)
	{
		ICM_IIC_Write(reg,value);
	}

}

void ICM_MREG1_WriteRegister(uint8_t Maddr_w,uint8_t M_w)
{
		//// 切换到MREG1 Bank
		ICM_WriteRegister(BLK_SEL_W, 0x00);  // BLK_SEL_W = 0x00 (MREG1)
		ICM_WriteRegister(MADDR_W, Maddr_w);  
		ICM_WriteRegister(M_W, M_w);  

}
uint8_t ICM_MREG1_ReadRegister(uint8_t Maddr_r)
{
		//// 切换到MREG1 Bank
		ICM_WriteRegister(BLK_SEL_R, 0x00);  
		ICM_WriteRegister(MADDR_R, Maddr_r);  
		return ICM_ReadRegister(M_R);  

}

void ICM_MODE_WoM(uint8_t x_thr,uint8_t y_thr,uint8_t z_thr)
{
		printf("配置ICM为WoM模式\r\n");
		ICM_WriteRegister(PWR_MGMT0, 0x12);  // PWR_MGMT0: ACCEL_MODE = LP Mode,低功耗模式
		HAL_Delay(50);                  // 等待启动
		
		//阈值设置寄存器在MREG1 Bank，需要使用ICM_MREG1_WriteRegister
		// 配置X轴阈值（ACCEL_WOM_X_THR = 0x4B）
		ICM_MREG1_WriteRegister(ACCEL_WOM_X_THR,x_thr);

		// 配置Y轴阈值（ACCEL_WOM_Y_THR = 0x4C）
		ICM_MREG1_WriteRegister(ACCEL_WOM_Y_THR,y_thr);

		// 配置Z轴阈值（ACCEL_WOM_Z_THR = 0x4D）
		ICM_MREG1_WriteRegister(ACCEL_WOM_Z_THR,z_thr);


//		ICM_WriteRegister(GYRO_CONFIG0, 0x69);//陀螺仪配置
		ICM_WriteRegister(ACCEL_CONFIG0, 0x4A);  // ACCEL_CONFIG0: FS=±4 g, ODR
		ICM_WriteRegister(WOM_CONFIG, 0x03);  // WOM_CONFIG: 启用WoM，比较最初样本，OR模式,第一次过阈值事件时断言WoM中断
		ICM_WriteRegister(INT_CONFIG,0x03);		//INT1脉冲模式
		ICM_WriteRegister(INT_SOURCE1, 0x07);  // INT_SOURCE1: 使能XYZ轴中断到INT1

		uint8_t who_am_i = ICM_ReadRegister(WHO_AM_I);
		if(who_am_i != 0x67) {
				// 处理错误：检查硬件连接、电源、SPI/I2C配置
			printf("who_am_i != 0x67\r\n");

		}
		
		uint8_t icm_range = (ICM_ReadRegister(ACCEL_CONFIG0)&0x60)>>5;
		switch(icm_range){
			case(0):
				accler_sensitivity=ACCELER_SENSITIVITY_16;
				break;
			case(1):
				accler_sensitivity=ACCELER_SENSITIVITY_8;
				break;
			case(2):
				accler_sensitivity=ACCELER_SENSITIVITY_4;
				break;
			case(3):
				accler_sensitivity=ACCELER_SENSITIVITY_2;
				break;
		}
		
		ICM_WriteRegister(PWR_MGMT0, 0x02); 
}
void ICM_MODE_Sleep(void)
{
	ICM_WriteRegister(PWR_MGMT0, 0x00);  // PWR_MGMT0: ACCEL_MODE
	
}
void ICM_Gyro_open(void)
{
		ICM_WriteRegister(PWR_MGMT0, 0x1A);  // 打开加速度计和陀螺仪
		HAL_Delay(50);                  // 等待启动
		ICM_WriteRegister(ACCEL_CONFIG0, 0x4A);  // ACCEL_CONFIG0: FS=±4 g, ODR
		ICM_WriteRegister(GYRO_CONFIG0, 0x69);//陀螺仪配置
	
		uint8_t icm_range = (ICM_ReadRegister(ACCEL_CONFIG0)&0x60)>>5;
	printf("icm_range=%d\n",icm_range);
		switch(icm_range){
			case(0):
				accler_sensitivity=ACCELER_SENSITIVITY_16;
				break;
			case(1):
				accler_sensitivity=ACCELER_SENSITIVITY_8;
				break;
			case(2):
				accler_sensitivity=ACCELER_SENSITIVITY_4;
				break;
			case(3):
				accler_sensitivity=ACCELER_SENSITIVITY_2;
				break;
		}

}
void IMU_Update2(float gx, float gy, float gz, float ax, float ay, float az);
void ICM_Read_Data(ICM_DATA* data)
{
			ICM_Gyro_open();
	HAL_Delay(1);  
	
	//			// 读取 X 轴加速度
			uint8_t accel_x_h = ICM_ReadRegister(0x0B);  // 读取高8位
			uint8_t accel_x_l = ICM_ReadRegister(0x0C);  // 读取低8位
			int16_t accel_x = (int16_t)((accel_x_h << 8) | accel_x_l);  // 合并为 16 位有符号整数

			float accel_x_s=(float)accel_x/accler_sensitivity;

			// 读取 Y 轴加速度
			uint8_t accel_y_h = ICM_ReadRegister(0x0D);  // 读取高8位
			uint8_t accel_y_l = ICM_ReadRegister(0x0E);  // 读取低8位
			int16_t accel_y = (int16_t)((accel_y_h << 8) | accel_y_l);  // 合并为 16 位有符号整数
			float accel_y_s=(float)accel_y/accler_sensitivity;
			// 读取 Z 轴加速度
			uint8_t accel_z_h = ICM_ReadRegister(0x0F);  // 读取高8位
			uint8_t accel_z_l = ICM_ReadRegister(0x10);  // 读取低8位
			int16_t accel_z = (int16_t)((accel_z_h << 8) | accel_z_l);  // 合并为 16 位有符号整数
			float accel_z_s=(float)accel_z/accler_sensitivity;
			
			//角速度
			uint8_t gyro_x_h =ICM_ReadRegister(0x11);
			uint8_t gyro_x_l =ICM_ReadRegister(0x12);
			int16_t gyro_x = (int16_t)((gyro_x_h << 8) | gyro_x_l);  // 合并为 16 位有符号整数
			float gyro_x_DEG = (float)gyro_x / GYRO_SENSITIVITY_250;//单位：°/s
			float gyro_x_RAD = (float)gyro_x / GYRO_SENSITIVITY_250*DEG_TO_RAD;//转换成弧度，单位：弧度/s
			
			uint8_t gyro_y_h =ICM_ReadRegister(0x13);
			uint8_t gyro_y_l =ICM_ReadRegister(0x14);
			int16_t gyro_y = (int16_t)((gyro_y_h << 8) | gyro_y_l);  // 合并为 16 位有符号整数
			float gyro_y_DEG = (float)gyro_y / GYRO_SENSITIVITY_250;
			float gyro_y_RAD = (float)gyro_y / GYRO_SENSITIVITY_250*DEG_TO_RAD;
			
			uint8_t gyro_z_h =ICM_ReadRegister(0x15);
			uint8_t gyro_z_l =ICM_ReadRegister(0x16);
			int16_t gyro_z = (int16_t)((gyro_z_h << 8) | gyro_z_l);  // 合并为 16 位有符号整数
			float gyro_z_DEG = (float)gyro_z / GYRO_SENSITIVITY_250;
			float gyro_z_RAD = (float)gyro_z / GYRO_SENSITIVITY_250*DEG_TO_RAD;
			
				printf("gyro_x=%d,gyro_x_DEG=%f\n,gyro_x_RAD=%f\n",gyro_x,gyro_x_DEG,gyro_x_RAD);
			
			printf("ICM加速度X：%f\nICM加速度Y：%06f\nICM加速度Z：%f\r\n",accel_x_s,accel_y_s,accel_z_s);
			printf("ICM角速度X：%f\nICM角速度Y：%f\nICM角速度Z：%f\r\n",gyro_x_RAD,gyro_y_RAD,gyro_z_RAD);
			
			uint8_t ICM_status = ICM_ReadRegister(0x3A); // 读取中断状态寄存器
			ICM_status = ICM_ReadRegister(0x3B); // 读取中断状态寄存器
			ICM_status = ICM_ReadRegister(0x3C); // 读取Tilt状态寄存器
			
			data->acc_x=accel_x;
			data->acc_y=accel_y;
			data->acc_z=accel_z;
			data->gyro_x=gyro_x;
			data->gyro_y=gyro_y;
			data->gyro_z=gyro_z;
			
}


