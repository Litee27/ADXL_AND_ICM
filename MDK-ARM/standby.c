#include "standby.h"
#include "stm32l1xx_hal.h"  //包含需要的头文件
#include <stdio.h>
void SysEnter_Standby(void)
{  
	printf("3s后进入低功耗\r\n");
		HAL_Delay(3000);
	__HAL_RCC_PWR_CLK_ENABLE();                         //使能PWR时钟
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);                  //清除Wake_UP标志
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);           //设置PA0用于唤醒
    HAL_PWREx_EnableUltraLowPower();                    //设置超低功耗
	HAL_PWR_EnterSTANDBYMode();                         //进入待机模式   
	
}
