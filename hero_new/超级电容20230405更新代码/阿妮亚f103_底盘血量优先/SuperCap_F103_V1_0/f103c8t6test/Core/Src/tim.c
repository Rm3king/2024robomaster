/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "can.h"
#include "referee_system.h"
#include "spi.h"
	extern uint8_t SPI_to_401_Plimit[8];
	extern uint8_t SPI_from_401_Vcap[8];
	
	extern float TX_Plimit;//通过DMA向f401发送功率限制的真实数据，初始值35W
	extern float RX_Vcap;//DMA接收到的来自f401的真实数据，初始值24V
	
	extern f2byte SPI_TXdata;
	extern f2byte SPI_RXdata;


/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}
/* TIM3 init function */
void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 11, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* TIM3 interrupt Init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 10, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* TIM2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /* TIM3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

	/*
	f103通过SPI向F401发送当前的功率限制信息，共8个字节的信息，解析协议如下
	Byte0：0x5A
	Byte1：0xA5
	
	Byte2 ~ Byte5：表示float类型的4个字节
	
	Byte6：sum_data      sum_data=Byte2|Byte3|Byte4|Byte5
	Byte7：0xAA
	
	
	f401通过SPI向F103发送当前的电容电压信息，共8个字节的信息，解析协议如下
	Byte0：0xA5
	Byte1：uint8_t_(data_plimit_float+0.5f)   即f401会将上次接收到的功率限制信息回传，以便f103进行传输数据的校验
	
	Byte2 ~ Byte5：表示float类型的4个字节
	
	Byte6：sum_data      sum_data=Byte2|Byte3|Byte4|Byte5|Byte1
	Byte7：0x5A
	
	*/

static uint8_t uart_send_cnt;

float PLIMIT_TEST;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim3.Instance) 
	{
		if(uart_send_cnt == 0)
		{
			//在进行DMA传输之前需要先将待发送的Plimit数据按照传输协议放入DMA传送数组（8Byte）
			SPI_to_401_Plimit[0] = 0x5A;
			SPI_to_401_Plimit[1] = 0xA5;
			
			SPI_TXdata.f_data = TX_Plimit;
//			SPI_TXdata.f_data = PLIMIT_TEST;
//			SPI_TXdata.f_data = 10;
			SPI_to_401_Plimit[2] = SPI_TXdata.f_byte[0];
			SPI_to_401_Plimit[3] = SPI_TXdata.f_byte[1];
			SPI_to_401_Plimit[4] = SPI_TXdata.f_byte[2];
			SPI_to_401_Plimit[5] = SPI_TXdata.f_byte[3];
			
			SPI_to_401_Plimit[6] = SPI_to_401_Plimit[2] | SPI_to_401_Plimit[3] | SPI_to_401_Plimit[4] | SPI_to_401_Plimit[5];
			
			SPI_to_401_Plimit[7] = 0xAA;
		}
		
		HAL_UART_Transmit(&huart1,&SPI_to_401_Plimit[uart_send_cnt],1,0xffff);
		
		uart_send_cnt++;
		if(uart_send_cnt >= 8)
		{
			uart_send_cnt = 0;
		}

		
		
		
		
		
		
		
		
//		//在SPI接收之前需要清空接收数组(随机设置几个值),由DMA对接收数组进行填充，如果正确接收完毕之后，数组的值应当发送变化
//		SPI_from_401_Vcap[0] = 0xA0;
//		SPI_from_401_Vcap[1] = 0xB0;
//		SPI_from_401_Vcap[2] = 0xC0;
//		SPI_from_401_Vcap[3] = 0xD0;
//		SPI_from_401_Vcap[4] = 0xD1;
//		SPI_from_401_Vcap[5] = 0xC1;
//		SPI_from_401_Vcap[6] = 0xB1;
//		SPI_from_401_Vcap[7] = 0xA1;
		
		
//		//每10ms 触发SPI主机进行传输
//		HAL_SPI_TransmitReceive_DMA(&hspi1,SPI_to_401_Plimit,SPI_from_401_Vcap,8);
	
		
	}
	if(htim->Instance == TIM2){
			CAN1_Send();
			ParseRefereeSystemData();
				//在进行DMA传输之前需要先将待发送的Plimit数据按照传输协议放入DMA传送数组（8Byte）
//		SPI_to_401_Plimit[0] = 0x5A;
//		SPI_to_401_Plimit[1] = 0xA5;
//		
//		SPI_TXdata.f_data = TX_Plimit;
//		SPI_to_401_Plimit[2] = SPI_TXdata.f_byte[0];
//		SPI_to_401_Plimit[3] = SPI_TXdata.f_byte[1];
//		SPI_to_401_Plimit[4] = SPI_TXdata.f_byte[2];
//		SPI_to_401_Plimit[5] = SPI_TXdata.f_byte[3];
//		
//		SPI_to_401_Plimit[6] = SPI_to_401_Plimit[2] | SPI_to_401_Plimit[3] | SPI_to_401_Plimit[4] | SPI_to_401_Plimit[5];
//		
//		SPI_to_401_Plimit[7] = 0xAA;
//		
//		//在SPI接收之前需要清空接收数组(随机设置几个值),由DMA对接收数组进行填充，如果正确接收完毕之后，数组的值应当发送变化
//		SPI_from_401_Vcap[0] = 0xA0;
//		SPI_from_401_Vcap[1] = 0xB0;
//		SPI_from_401_Vcap[2] = 0xC0;
//		SPI_from_401_Vcap[3] = 0xD0;
//		SPI_from_401_Vcap[4] = 0xD1;
//		SPI_from_401_Vcap[5] = 0xC1;
//		SPI_from_401_Vcap[6] = 0xB1;
//		SPI_from_401_Vcap[7] = 0xA1;
		
		
		//每10ms 触发SPI主机进行传输
//		HAL_SPI_TransmitReceive_DMA(&hspi1,SPI_to_401_Plimit,SPI_from_401_Vcap,8);
		  switch(robot_referee_status.game_robot_status.robot_level)
			{
				case 1:
					TX_Plimit = 43.0f;
					break;
				case 2:
					TX_Plimit = 47.0f;
					break;
				case 3:
					TX_Plimit = 53.0f;
					break;
				}
//			HAL_SPI_TransmitReceive_DMA(&hspi1,SPI_to_401_Plimit,SPI_from_401_Vcap,8);
		}
}



/* USER CODE END 1 */
