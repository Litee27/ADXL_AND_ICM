#include "ICM.h"


float accler_sensitivity;
void ICM_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStruct ;

	__HAL_RCC_GPIOC_CLK_ENABLE();  

	GPIO_InitStruct.Pin = GPIO_PIN_13;   
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // 配置为复用推挽输出
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull=GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);  
	
}

void ICM_CS_Select(void)
{
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_RESET);
}

void ICM_CS_Deselect(void)
{
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);
}
uint8_t ICM_ReadRegister(uint8_t reg)
{
		uint8_t value;
    ICM_CS_Select();
    SPI_TransmitReceive(0x80|reg); // 读寄存器命令,最高位置1

    value = SPI_TransmitReceive(0x00); // 读取值
    ICM_CS_Deselect();
    return value;
}

void ICM_WriteRegister(uint8_t reg, uint8_t value)
{
    ICM_CS_Select();
    SPI_TransmitReceive(reg); // 写寄存器命令，最高位置0
    SPI_TransmitReceive(value); // 写入值
    ICM_CS_Deselect();
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

void ICM_MODE_WoM()
{
	printf("配置ICM为WoM模式\r\n");
		ICM_WriteRegister(PWR_MGMT0, 0x0A);  // PWR_MGMT0: ACCEL_MODE = LP Mode,低功耗模式
		HAL_Delay(50);                  // 等待启动
		
		
		// 配置X轴阈值（ACCEL_WOM_X_THR = 0x4B）
		ICM_MREG1_WriteRegister(ACCEL_WOM_X_THR,0x32);

		// 配置Y轴阈值（ACCEL_WOM_Y_THR = 0x4C）
		ICM_MREG1_WriteRegister(ACCEL_WOM_Y_THR,0x32);

		// 配置Z轴阈值（ACCEL_WOM_Z_THR = 0x4D）
		ICM_MREG1_WriteRegister(ACCEL_WOM_Z_THR,0x32);


		ICM_WriteRegister(GYRO_CONFIG0, 0x69);//陀螺仪配置
		ICM_WriteRegister(ACCEL_CONFIG0, 0x6A);  // ACCEL_CONFIG0: FS=±2 g, ODR
		ICM_WriteRegister(WOM_CONFIG, 0x03);  // WOM_CONFIG: 启用WoM，比较最初样本，OR模式,第一次过阈值事件时断言WoM中断
		ICM_WriteRegister(INT_CONFIG,0x03);		//INT1脉冲模式
		ICM_WriteRegister(INT_SOURCE1, 0x07);  // INT_SOURCE1: 使能XYZ轴中断到INT1

		
		
		uint8_t who_am_i = ICM_ReadRegister(WHO_AM_I);
		while(who_am_i != 0x67) {
				// 处理错误：检查硬件连接、电源、SPI/I2C配置
			printf("错误，复位设备\r\n");
			ICM_WriteRegister(PWR_MGMT0, 0x00);  // 复位设备
			HAL_Delay(1000);
			ICM_WriteRegister(PWR_MGMT0, 0x0A);  // 启用加速度计,低功耗模式
			HAL_Delay(1000);
		}
		
		uint8_t icm_range = (ICM_ReadRegister(ACCEL_CONFIG0)&0x60)>>5;
		switch(icm_range){
			case(0):
				accler_sensitivity=ACCELER_SENSITIVITY_16;
			case(1):
				accler_sensitivity=ACCELER_SENSITIVITY_8;
			case(2):
				accler_sensitivity=ACCELER_SENSITIVITY_4;
			case(3):
				accler_sensitivity=ACCELER_SENSITIVITY_2;
		}
}
