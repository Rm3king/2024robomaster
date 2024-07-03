/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 12, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PA11     ------> USART6_TX
    PA12     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PA11     ------> USART6_TX
    PA12     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */


extern uint8_t uart_data;

uint8_t uart_history[30];
static uint8_t uart_cnt;
	extern float plimit;


void my_uart_handle(void)
{
	uint8_t i;
	uint8_t sum_data;
	
//	LED_RUN_TOGGLE;
	

	for(i=0;i<=30-8;i++)
	{
		sum_data = uart_history[i+2] | uart_history[i+3] | uart_history[i+4] | uart_history[i+5];
		
		//在这里处理接收到的数据
		if(  (uart_history[i]==0x5A)&&(uart_history[i+1]==0xA5)&&(uart_history[i+6]==sum_data)&&(uart_history[i+7]==0xAA)  )
		{
			SPI_RXdata.f_byte[0] = uart_history[i+2];
			SPI_RXdata.f_byte[1] = uart_history[i+3];
			SPI_RXdata.f_byte[2] = uart_history[i+4];
			SPI_RXdata.f_byte[3] = uart_history[i+5];
			RX_Plimit = SPI_RXdata.f_data;
			
			if((RX_Plimit>1)&&(RX_Plimit<200))//对接收到的功率值进行限幅处理
			{
				plimit = RX_Plimit;
			}
			
			LED_RUN_TOGGLE;
		}
	
	}





}


void no_it_uart(void)
{
	uart_history[uart_cnt] = uart_data;
	
	uart_cnt++;
	if(uart_cnt>=30) //
	{
		uart_cnt = 0;
		
		my_uart_handle();
	}

	
//	HAL_StatusTypeDef return_state;
//	return_state = HAL_UART_Receive_IT(&huart6, &uart_data,1);
//	if(return_state != HAL_OK)
//	{
//		if(return_state == HAL_BUSY)
//		{
//			__HAL_UART_CLEAR_OREFLAG(&huart6);
//			huart6.RxState = HAL_UART_STATE_READY;
//			huart6.Lock = HAL_UNLOCKED;
//			return_state = HAL_UART_Receive_IT(&huart6, &uart_data,1);
//		}
//	
//	}

}








void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)//????
{
	HAL_StatusTypeDef return_state;
	
	if(UartHandle->Instance == USART6)
	{
		__HAL_UART_DISABLE_IT(&huart6,UART_IT_RXNE);
		
		uart_history[uart_cnt] = uart_data;
		
		uart_cnt++;
		if(uart_cnt>=30) //
		{
			uart_cnt = 0;
			
			my_uart_handle();
		}

		
//		HAL_StatusTypeDef return_state;
//		return_state = HAL_UART_Receive_IT(&huart6, &uart_data,1);
//		if(return_state != HAL_OK)
//		{
//			if(return_state == HAL_BUSY)
//			{
//				__HAL_UART_CLEAR_OREFLAG(&huart6);
//				huart6.RxState = HAL_UART_STATE_READY;
//				huart6.Lock = HAL_UNLOCKED;
//				return_state = HAL_UART_Receive_IT(&huart6, &uart_data,1);
//			}
//		}
		__HAL_UART_ENABLE_IT(&huart6,UART_IT_RXNE);
		
		//再次开启中断接收
		return_state = HAL_UART_Receive_IT(&huart6, &uart_data,1);
		//如果返回值有问题，则下次就不会正常进入中断了
		if(return_state != HAL_OK)
		{
			//解除串口忙状态（由ORE导致，需要清零ORE位）
			if(return_state == HAL_BUSY)
			{
				//清除ORE错误
				__HAL_UART_CLEAR_OREFLAG(&huart6);
				huart6.RxState = HAL_UART_STATE_READY;
				huart6.Lock = HAL_UNLOCKED;
				
				return_state = HAL_UART_Receive_IT(&huart6, &uart_data,1);
			}
			
		}
		
		
		
	}
	
	
}





void uart_send_data(void)
{
	//在进行DMA传输之前需要先将待发送的Plimit数据按照传输协议放入DMA传送数组（8Byte）
	SPI_from_401_Vcap[0] = 0xA5;
	
	SPI_from_401_Vcap[1] = (uint8_t)(RX_Plimit + 0.5f);
	
	TX_Vcap = V_C_real;
	SPI_TXdata.f_data = TX_Vcap;
	SPI_from_401_Vcap[2] = SPI_TXdata.f_byte[0];
	SPI_from_401_Vcap[3] = SPI_TXdata.f_byte[1];
	SPI_from_401_Vcap[4] = SPI_TXdata.f_byte[2];
	SPI_from_401_Vcap[5] = SPI_TXdata.f_byte[3];
	
	SPI_from_401_Vcap[6] = SPI_from_401_Vcap[2] | SPI_from_401_Vcap[3] | SPI_from_401_Vcap[4] | SPI_from_401_Vcap[5] | SPI_from_401_Vcap[1];
	
	SPI_from_401_Vcap[7] = 0x5A;

}




/**
  * @brief  UART error callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	HAL_StatusTypeDef return_state;
	
		//再次开启中断接收
		return_state = HAL_UART_Receive_IT(&huart6, &uart_data,1);
		//如果返回值有问题，则下次就不会正常进入中断了
		if(return_state != HAL_OK)
		{
			//解除串口忙状态（由ORE导致，需要清零ORE位）
			if(return_state == HAL_BUSY)
			{
				//清除ORE错误
				__HAL_UART_CLEAR_OREFLAG(&huart6);
				huart6.RxState = HAL_UART_STATE_READY;
				huart6.Lock = HAL_UNLOCKED;
				
				return_state = HAL_UART_Receive_IT(&huart6, &uart_data,1);
			}
		}
  
}










/* USER CODE END 1 */
