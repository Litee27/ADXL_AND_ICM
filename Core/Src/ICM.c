#include "ICM.h"


float accler_sensitivity;
void ICM_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStruct ;

	__HAL_RCC_GPIOC_CLK_ENABLE();  

	GPIO_InitStruct.Pin = GPIO_PIN_13;   
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // ����Ϊ�����������
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
    SPI_TransmitReceive(0x80|reg); // ���Ĵ�������,���λ��1

    value = SPI_TransmitReceive(0x00); // ��ȡֵ
    ICM_CS_Deselect();
    return value;
}

void ICM_WriteRegister(uint8_t reg, uint8_t value)
{
    ICM_CS_Select();
    SPI_TransmitReceive(reg); // д�Ĵ���������λ��0
    SPI_TransmitReceive(value); // д��ֵ
    ICM_CS_Deselect();
}

void ICM_MREG1_WriteRegister(uint8_t Maddr_w,uint8_t M_w)
{
		//// �л���MREG1 Bank
		ICM_WriteRegister(BLK_SEL_W, 0x00);  // BLK_SEL_W = 0x00 (MREG1)
		ICM_WriteRegister(MADDR_W, Maddr_w);  
		ICM_WriteRegister(M_W, M_w);  

}
uint8_t ICM_MREG1_ReadRegister(uint8_t Maddr_r)
{
		//// �л���MREG1 Bank
		ICM_WriteRegister(BLK_SEL_R, 0x00);  
		ICM_WriteRegister(MADDR_R, Maddr_r);  
		return ICM_ReadRegister(M_R);  

}

void ICM_MODE_WoM()
{
	printf("����ICMΪWoMģʽ\r\n");
		ICM_WriteRegister(PWR_MGMT0, 0x0A);  // PWR_MGMT0: ACCEL_MODE = LP Mode,�͹���ģʽ
		HAL_Delay(50);                  // �ȴ�����
		
		
		// ����X����ֵ��ACCEL_WOM_X_THR = 0x4B��
		ICM_MREG1_WriteRegister(ACCEL_WOM_X_THR,0x32);

		// ����Y����ֵ��ACCEL_WOM_Y_THR = 0x4C��
		ICM_MREG1_WriteRegister(ACCEL_WOM_Y_THR,0x32);

		// ����Z����ֵ��ACCEL_WOM_Z_THR = 0x4D��
		ICM_MREG1_WriteRegister(ACCEL_WOM_Z_THR,0x32);


		ICM_WriteRegister(GYRO_CONFIG0, 0x69);//����������
		ICM_WriteRegister(ACCEL_CONFIG0, 0x6A);  // ACCEL_CONFIG0: FS=��2 g, ODR
		ICM_WriteRegister(WOM_CONFIG, 0x03);  // WOM_CONFIG: ����WoM���Ƚ����������ORģʽ,��һ�ι���ֵ�¼�ʱ����WoM�ж�
		ICM_WriteRegister(INT_CONFIG,0x03);		//INT1����ģʽ
		ICM_WriteRegister(INT_SOURCE1, 0x07);  // INT_SOURCE1: ʹ��XYZ���жϵ�INT1

		
		
		uint8_t who_am_i = ICM_ReadRegister(WHO_AM_I);
		while(who_am_i != 0x67) {
				// ������󣺼��Ӳ�����ӡ���Դ��SPI/I2C����
			printf("���󣬸�λ�豸\r\n");
			ICM_WriteRegister(PWR_MGMT0, 0x00);  // ��λ�豸
			HAL_Delay(1000);
			ICM_WriteRegister(PWR_MGMT0, 0x0A);  // ���ü��ٶȼ�,�͹���ģʽ
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
