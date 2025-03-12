/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adxl362.h"
#include "stdio.h"
#include "ICM.h"
#include "angle2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */
extern uint8_t counti;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
//void EXTI9_5_IRQHandler(void)
//{
//  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
//  /* USER CODE END EXTI9_5_IRQn 0 */
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
//  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
//printf("中断\n");
//  /* USER CODE END EXTI9_5_IRQn 1 */
//}

/* USER CODE BEGIN 1 */

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//	counti=1;
//		if (GPIO_Pin == GPIO_PIN_9){
//			uint8_t counti=0;
//			while(counti<100)
//				{
//					
//					ICM_GET_ANGLE();//获取ICM角度
//					printf("ypl****************:\n");
//					printf("roll:%f\npitch:%f\nyaw:%f\n",imu.roll,imu.pitch,imu.yaw);
//					printf("ypl****************:\n");
//				//	ICM_Read_Data(&Mydata);
//				HAL_Delay(10);	
//					counti++;
//				}
//		}
		
//}
	


//INT1中断读取加速度
void EXTI3_IRQHandler(void)
{
  
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
			
		if (GPIO_Pin == GPIO_PIN_3) {
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);
			// 读取 X 轴加速度
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
			
			//角度
			uint8_t gyro_x_h =ICM_ReadRegister(0x11);
			uint8_t gyro_x_l =ICM_ReadRegister(0x12);
			int16_t gyro_x = (int16_t)((gyro_x_h << 8) | gyro_x_l);  // 合并为 16 位有符号整数
			float gyro_x_s = (float)gyro_x / GYRO_SENSITIVITY_250;
			
			uint8_t gyro_y_h =ICM_ReadRegister(0x13);
			uint8_t gyro_y_l =ICM_ReadRegister(0x14);
			int16_t gyro_y = (int16_t)((gyro_y_h << 8) | gyro_y_l);  // 合并为 16 位有符号整数
			float gyro_y_s = (float)gyro_y / GYRO_SENSITIVITY_250;
			
			uint8_t gyro_z_h =ICM_ReadRegister(0x15);
			uint8_t gyro_z_l =ICM_ReadRegister(0x16);
			int16_t gyro_z = (int16_t)((gyro_z_h << 8) | gyro_z_l);  // 合并为 16 位有符号整数
			float gyro_z_s = (float)gyro_z / GYRO_SENSITIVITY_250;
			
			printf("ICM加速度X：%f\nICM加速度Y：%f\nICM加速度Z：%f\r\n",accel_x_s,accel_y_s,accel_z_s);
			printf("ICM倾角X：%f\nICM倾角Y：%f\nICM倾角Z：%f\r\n",gyro_x_s,gyro_y_s,gyro_z_s);
			
			uint8_t ICM_status = ICM_ReadRegister(0x3A); // 读取中断状态寄存器
			ICM_status = ICM_ReadRegister(0x3B); // 读取中断状态寄存器
			ICM_status = ICM_ReadRegister(0x3C); // 读取Tilt状态寄存器
    }
}


/* USER CODE END 1 */
