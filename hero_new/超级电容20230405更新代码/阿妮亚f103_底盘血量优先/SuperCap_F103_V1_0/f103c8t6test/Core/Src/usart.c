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
DMA_HandleTypeDef hdma_usart1_rx;

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
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
    PB7     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_USART1_ENABLE();

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
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
    PB7     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


//        (+) HAL_UART_RxHalfCpltCallback()
//        (+) HAL_UART_RxCpltCallback()
//        (+) HAL_UART_ErrorCallback()



//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//    static uint8_t Rx_buf_pos;	//本次回调接收的数据在缓冲区的起点
//    static uint8_t Rx_length;	//本次回调接收数据的长度
//    Rx_length = Size - Rx_buf_pos;
//    fifo_s_puts(&uart_rx_fifo, &USART1_Rx_buf[Rx_buf_pos], Rx_length);	//数据填入 FIFO
//    Rx_buf_pos += Rx_length;
//    if (Rx_buf_pos >= RX_BUF_SIZE) Rx_buf_pos = 0;	//缓冲区用完后，返回 0 处重新开始
//}

extern float supercap_voltage;
extern float supercap_energy_percent;


extern uint8_t Uart_f401_data[50];

static uint8_t Uart_CNT =0;
uint8_t Uart_data_rec[50];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t i;
	uint8_t sum_data;
	HAL_StatusTypeDef return_state;
	
	__HAL_UART_DISABLE_IT(&huart1,UART_IT_RXNE);
	
	Uart_f401_data[Uart_CNT] = Uart_data_rec[0];
	
	
//	for(i=0;i<=20-8;i++)
//	{
//		sum_data = Uart_f401_data[i+2] | Uart_f401_data[i+3] | Uart_f401_data[i+4] | Uart_f401_data[i+5] | Uart_f401_data[i+1];
//		
//		//在这里处理接收到的数据
//		if(  (Uart_f401_data[i+0]==0xA5)&&(Uart_f401_data[i+6]==sum_data)&&(Uart_f401_data[i+7]==0x5A)  )
//		{
//			SPI_RXdata.f_byte[0] = Uart_f401_data[i+2];
//			SPI_RXdata.f_byte[1] = Uart_f401_data[i+3];
//			SPI_RXdata.f_byte[2] = Uart_f401_data[i+4];
//			SPI_RXdata.f_byte[3] = Uart_f401_data[i+5];
//			
//			RX_Vcap = SPI_RXdata.f_data;
//			if(RX_Vcap>0&&RX_Vcap<30){
//				supercap_voltage = RX_Vcap;
//			}
//			supercap_energy_percent = (RX_Vcap*RX_Vcap)/613.06f*100.0f;
//			
//			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
//		}
//	
//	}
	
	Uart_CNT++;
	if(Uart_CNT >=19)
	{
		Uart_CNT = 0;

		for(i=0;i<=20-8;i++)
		{
			sum_data = Uart_f401_data[i+2] | Uart_f401_data[i+3] | Uart_f401_data[i+4] | Uart_f401_data[i+5] | Uart_f401_data[i+1];
			
			//在这里处理接收到的数据
			if(  (Uart_f401_data[i+0]==0xA5)&&(Uart_f401_data[i+6]==sum_data)&&(Uart_f401_data[i+7]==0x5A)  )
			{
				SPI_RXdata.f_byte[0] = Uart_f401_data[i+2];
				SPI_RXdata.f_byte[1] = Uart_f401_data[i+3];
				SPI_RXdata.f_byte[2] = Uart_f401_data[i+4];
				SPI_RXdata.f_byte[3] = Uart_f401_data[i+5];
				
				RX_Vcap = SPI_RXdata.f_data;
				if(RX_Vcap>0&&RX_Vcap<30){
					supercap_voltage = RX_Vcap;
				}
//				supercap_energy_percent = (RX_Vcap*RX_Vcap)/613.06f*100.0f;
				supercap_energy_percent = (RX_Vcap*RX_Vcap)*0.142399f;
				
				HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
			}
		
		}		
	
	}
	
	
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
	
//	HAL_UART_Receive_IT(&huart1, Uart_f401_data, 1);
	HAL_UART_Receive_IT(&huart1, Uart_data_rec, 1);
		//再次开启中断接收
		return_state = HAL_UART_Receive_IT(&huart1, Uart_data_rec, 1);
		//如果返回值有问题，则下次就不会正常进入中断了
		if(return_state != HAL_OK)
		{
			//解除串口忙状态（由ORE导致，需要清零ORE位）
			if(return_state == HAL_BUSY)
			{
				//清除ORE错误
				__HAL_UART_CLEAR_OREFLAG(&huart1);
				huart1.RxState = HAL_UART_STATE_READY;
				huart1.Lock = HAL_UNLOCKED;
				
				return_state = HAL_UART_Receive_IT(&huart1, Uart_data_rec, 1);
			}
			
		}	

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
		return_state = HAL_UART_Receive_IT(&huart1, Uart_data_rec, 1);
		//如果返回值有问题，则下次就不会正常进入中断了
		if(return_state != HAL_OK)
		{
			//解除串口忙状态（由ORE导致，需要清零ORE位）
			if(return_state == HAL_BUSY)
			{
				//清除ORE错误
				__HAL_UART_CLEAR_OREFLAG(&huart1);
				huart1.RxState = HAL_UART_STATE_READY;
				huart1.Lock = HAL_UNLOCKED;
				
				return_state = HAL_UART_Receive_IT(&huart1, Uart_data_rec, 1);
			}
		}	
	
  
}








/* USER CODE END 1 */
