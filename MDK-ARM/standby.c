#include "standby.h"
#include "stm32l1xx_hal.h"  //������Ҫ��ͷ�ļ�
#include <stdio.h>
void SysEnter_Standby(void)
{  
	printf("3s�����͹���\r\n");
		HAL_Delay(3000);
	__HAL_RCC_PWR_CLK_ENABLE();                         //ʹ��PWRʱ��
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);                  //���Wake_UP��־
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);           //����PA0���ڻ���
    HAL_PWREx_EnableUltraLowPower();                    //���ó��͹���
	HAL_PWR_EnterSTANDBYMode();                         //�������ģʽ   
	
}
