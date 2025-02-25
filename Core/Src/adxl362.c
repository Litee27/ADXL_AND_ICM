#include "adxl362.h"

float scale_factor;
void ADXL362_Init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStruct ;

	__HAL_RCC_GPIOA_CLK_ENABLE();  // ���� GPIOA ʱ��

	GPIO_InitStruct.Pin = GPIO_PIN_4;  // ���� PA4 ����
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // ����Ϊ�����������
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull=GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  // ��ʼ�� GPIOA ����
	
    // д����λ����
    ADXL362_WriteRegister(0x1F, 0x52);
    HAL_Delay(10);

    // ���ò���ģʽ������ ��2g
    ADXL362_WriteRegister(0x2D, 0x02); // ��������ģʽ
}
// ADXL362 Ƭѡ����
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
			printf("SPI�������,����ֵΪ:%02x\r\n",data);
    return rxData;
}
uint8_t ADXL362_ReadRegister(uint8_t reg)
{
    uint8_t value;
    ADXL362_CS_Select();
    SPI_TransmitReceive(0x0B); // ���Ĵ�������
    SPI_TransmitReceive(reg); // �Ĵ�����ַ
    value = SPI_TransmitReceive(0x00); // ��ȡֵ
    ADXL362_CS_Deselect();
    return value;
}
void ADXL362_WriteRegister(uint8_t reg, uint8_t value)
{
    ADXL362_CS_Select();
    SPI_TransmitReceive(0x0A); // д�Ĵ�������
    SPI_TransmitReceive(reg); // �Ĵ�����ַ
    SPI_TransmitReceive(value); // д��ֵ
    ADXL362_CS_Deselect();
}
uint16_t ADXL362_ReadFIFO(uint8_t size)
{
    uint16_t value;
    ADXL362_CS_Select();
    SPI_TransmitReceive(0x0D); // ��ȡFIFO
		value = (SPI_TransmitReceive(0x00)<<8)|SPI_TransmitReceive(0x00);
    ADXL362_CS_Deselect();
    return value;
}

/*����ģʽ
threshold_ACT ����ģʽ�»��ֵԽСԽ����
TIME_ACT			����ʱ�䣬��ʱ���ڳ�����ֵ���ж�Ϊ�˶��������ж�����
INT_NUM				INT1��INT2
*/
void ADXL362_mode_measure(uint16_t threshold_ACT,uint8_t time_ACT,uint8_t INT_num)
{
	printf("0xFF|threshold_ACT:%04x\r\n",0xFF&threshold_ACT);
	printf("0xFF|threshold_ACT>>8:%02x\r\n",0xFF&(threshold_ACT>>8));
		ADXL362_WriteRegister(THRESH_ACT_L, 0xFF&threshold_ACT);//���ֵ
		ADXL362_WriteRegister(THRESH_ACT_H, 0xFF&(threshold_ACT>>8));//���ֵ
		ADXL362_WriteRegister(TIME_ACT, time_ACT);//�����10ms��10ms�ڼ��ٶ������趨��ֵ�����ж�Ϊ�
		ADXL362_WriteRegister(ACT_INACT_CTL, 0x03);//����ʹ�ܣ�����ģʽ
		if(INT_num==INT1||INT_num==INT2)
			ADXL362_WriteRegister(INT_num, 0x10);//ӳ�䵽INT1��INT2����ж�
		else
			printf("INT_NUMֻ��ΪINT1��INT2!\r\n");
		ADXL362_WriteRegister(FILTER_CTL, 0x83);//+-8g��Χ
		ADXL362_WriteRegister(POWER_CTL, 0x02);//����ģʽ
		
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

/*����ģʽ
threshold_ACT 	���ģʽ�»��ֵԽСԽ����
TIME_ACT				����ʱ�䣬��ʱ���ڳ�����ֵ���ж�Ϊ�˶��������ж�����
threshold_INACT ���ģʽ�·ǻ��ֵԽ��Խ����
TIME_INACT			�ǻ���ʱ�䣬��ʱ���ڳ�����ֵ���ж�Ϊ�˶��������ж�����
INT_NUM					INT1��INT2
*/
void ADXL362_mode_wakeup(uint16_t threshold_ACT,uint8_t time_ACT,uint16_t threshold_INACT,uint8_t time_INACT,uint8_t INT_num)
{

		ADXL362_WriteRegister(THRESH_ACT_L, 0xFF&threshold_ACT);//���ֵ
		ADXL362_WriteRegister(THRESH_ACT_H, 0xFF&(threshold_ACT>>8));//���ֵ
		ADXL362_WriteRegister(TIME_ACT, time_ACT);//��ж�ʱ��

		ADXL362_WriteRegister(THRESH_INACT_L, 0xFF&threshold_INACT);//��ֹ��ֵ
		ADXL362_WriteRegister(THRESH_INACT_H, 0xFF&(threshold_INACT>>8));//��ֹ��ֵ
		ADXL362_WriteRegister(TIME_INACT_L, time_INACT & 0xFF);//��ֹ�ж�ʱ��
		ADXL362_WriteRegister(TIME_INACT_H, (time_INACT >> 8) & 0xFF);
		ADXL362_WriteRegister(ACT_INACT_CTL, 0x3F);//��·ģʽ�����ģʽ�����ֹʹ��
		if(INT_num==INT1||INT_num==INT2)
			ADXL362_WriteRegister(INT_num, 0x40);//ӳ�䵽INT1��INT2����ж�
		else
			printf("INT_NUMֻ��ΪINT1��INT2!\r\n");
		ADXL362_WriteRegister(FILTER_CTL, 0x83);//+-8g��Χ
		ADXL362_WriteRegister(POWER_CTL, 0x1A);//�ڻ���ģʽ�½��в���
		
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
