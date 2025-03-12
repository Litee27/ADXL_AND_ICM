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
	
	GPIO_InitStruct.Pin = ICM_CS_PIN;   //CS����
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // ����Ϊ�����������
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull=GPIO_PULLUP;
	HAL_GPIO_Init(ICM_CS_PORT, &GPIO_InitStruct);  
	
	GPIO_InitStruct.Pin = ICM_INT1_PIN;//INT1����
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ICM_INT1_PORT, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 1);//INT1�ж�
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
			printf("SPI�������,����ֵΪ:%02x\r\n",data);
    return rxData;
}
uint8_t ICM_ReadRegister(uint8_t reg)
{
	if(IIC_OR_SPI==SPI)
	{
		uint8_t value;
    ICM_CS_Select();
    SPI_TransmitReceive(0x80|reg); // ���Ĵ�������,���λ��1

    value = SPI_TransmitReceive(0x00); // ��ȡֵ
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
    SPI_TransmitReceive(reg); // д�Ĵ���������λ��0
    SPI_TransmitReceive(value); // д��ֵ
    ICM_CS_Deselect();
	}
	else if(IIC_OR_SPI==IIC)
	{
		ICM_IIC_Write(reg,value);
	}

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

void ICM_MODE_WoM(uint8_t x_thr,uint8_t y_thr,uint8_t z_thr)
{
		printf("����ICMΪWoMģʽ\r\n");
		ICM_WriteRegister(PWR_MGMT0, 0x12);  // PWR_MGMT0: ACCEL_MODE = LP Mode,�͹���ģʽ
		HAL_Delay(50);                  // �ȴ�����
		
		//��ֵ���üĴ�����MREG1 Bank����Ҫʹ��ICM_MREG1_WriteRegister
		// ����X����ֵ��ACCEL_WOM_X_THR = 0x4B��
		ICM_MREG1_WriteRegister(ACCEL_WOM_X_THR,x_thr);

		// ����Y����ֵ��ACCEL_WOM_Y_THR = 0x4C��
		ICM_MREG1_WriteRegister(ACCEL_WOM_Y_THR,y_thr);

		// ����Z����ֵ��ACCEL_WOM_Z_THR = 0x4D��
		ICM_MREG1_WriteRegister(ACCEL_WOM_Z_THR,z_thr);


//		ICM_WriteRegister(GYRO_CONFIG0, 0x69);//����������
		ICM_WriteRegister(ACCEL_CONFIG0, 0x4A);  // ACCEL_CONFIG0: FS=��4 g, ODR
		ICM_WriteRegister(WOM_CONFIG, 0x03);  // WOM_CONFIG: ����WoM���Ƚ����������ORģʽ,��һ�ι���ֵ�¼�ʱ����WoM�ж�
		ICM_WriteRegister(INT_CONFIG,0x03);		//INT1����ģʽ
		ICM_WriteRegister(INT_SOURCE1, 0x07);  // INT_SOURCE1: ʹ��XYZ���жϵ�INT1

		uint8_t who_am_i = ICM_ReadRegister(WHO_AM_I);
		if(who_am_i != 0x67) {
				// ������󣺼��Ӳ�����ӡ���Դ��SPI/I2C����
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
		ICM_WriteRegister(PWR_MGMT0, 0x1A);  // �򿪼��ٶȼƺ�������
		HAL_Delay(50);                  // �ȴ�����
		ICM_WriteRegister(ACCEL_CONFIG0, 0x4A);  // ACCEL_CONFIG0: FS=��4 g, ODR
		ICM_WriteRegister(GYRO_CONFIG0, 0x69);//����������
	
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
	
	//			// ��ȡ X ����ٶ�
			uint8_t accel_x_h = ICM_ReadRegister(0x0B);  // ��ȡ��8λ
			uint8_t accel_x_l = ICM_ReadRegister(0x0C);  // ��ȡ��8λ
			int16_t accel_x = (int16_t)((accel_x_h << 8) | accel_x_l);  // �ϲ�Ϊ 16 λ�з�������

			float accel_x_s=(float)accel_x/accler_sensitivity;

			// ��ȡ Y ����ٶ�
			uint8_t accel_y_h = ICM_ReadRegister(0x0D);  // ��ȡ��8λ
			uint8_t accel_y_l = ICM_ReadRegister(0x0E);  // ��ȡ��8λ
			int16_t accel_y = (int16_t)((accel_y_h << 8) | accel_y_l);  // �ϲ�Ϊ 16 λ�з�������
			float accel_y_s=(float)accel_y/accler_sensitivity;
			// ��ȡ Z ����ٶ�
			uint8_t accel_z_h = ICM_ReadRegister(0x0F);  // ��ȡ��8λ
			uint8_t accel_z_l = ICM_ReadRegister(0x10);  // ��ȡ��8λ
			int16_t accel_z = (int16_t)((accel_z_h << 8) | accel_z_l);  // �ϲ�Ϊ 16 λ�з�������
			float accel_z_s=(float)accel_z/accler_sensitivity;
			
			//���ٶ�
			uint8_t gyro_x_h =ICM_ReadRegister(0x11);
			uint8_t gyro_x_l =ICM_ReadRegister(0x12);
			int16_t gyro_x = (int16_t)((gyro_x_h << 8) | gyro_x_l);  // �ϲ�Ϊ 16 λ�з�������
			float gyro_x_DEG = (float)gyro_x / GYRO_SENSITIVITY_250;//��λ����/s
			float gyro_x_RAD = (float)gyro_x / GYRO_SENSITIVITY_250*DEG_TO_RAD;//ת���ɻ��ȣ���λ������/s
			
			uint8_t gyro_y_h =ICM_ReadRegister(0x13);
			uint8_t gyro_y_l =ICM_ReadRegister(0x14);
			int16_t gyro_y = (int16_t)((gyro_y_h << 8) | gyro_y_l);  // �ϲ�Ϊ 16 λ�з�������
			float gyro_y_DEG = (float)gyro_y / GYRO_SENSITIVITY_250;
			float gyro_y_RAD = (float)gyro_y / GYRO_SENSITIVITY_250*DEG_TO_RAD;
			
			uint8_t gyro_z_h =ICM_ReadRegister(0x15);
			uint8_t gyro_z_l =ICM_ReadRegister(0x16);
			int16_t gyro_z = (int16_t)((gyro_z_h << 8) | gyro_z_l);  // �ϲ�Ϊ 16 λ�з�������
			float gyro_z_DEG = (float)gyro_z / GYRO_SENSITIVITY_250;
			float gyro_z_RAD = (float)gyro_z / GYRO_SENSITIVITY_250*DEG_TO_RAD;
			
				printf("gyro_x=%d,gyro_x_DEG=%f\n,gyro_x_RAD=%f\n",gyro_x,gyro_x_DEG,gyro_x_RAD);
			
			printf("ICM���ٶ�X��%f\nICM���ٶ�Y��%06f\nICM���ٶ�Z��%f\r\n",accel_x_s,accel_y_s,accel_z_s);
			printf("ICM���ٶ�X��%f\nICM���ٶ�Y��%f\nICM���ٶ�Z��%f\r\n",gyro_x_RAD,gyro_y_RAD,gyro_z_RAD);
			
			uint8_t ICM_status = ICM_ReadRegister(0x3A); // ��ȡ�ж�״̬�Ĵ���
			ICM_status = ICM_ReadRegister(0x3B); // ��ȡ�ж�״̬�Ĵ���
			ICM_status = ICM_ReadRegister(0x3C); // ��ȡTilt״̬�Ĵ���
			
			data->acc_x=accel_x;
			data->acc_y=accel_y;
			data->acc_z=accel_z;
			data->gyro_x=gyro_x;
			data->gyro_y=gyro_y;
			data->gyro_z=gyro_z;
			
}


