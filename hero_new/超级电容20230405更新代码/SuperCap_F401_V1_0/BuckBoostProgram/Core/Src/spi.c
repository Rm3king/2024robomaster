/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
#include "spi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI1 DMA Init */
    /* SPI1_RX Init */
    hdma_spi1_rx.Instance = DMA2_Stream0;
    hdma_spi1_rx.Init.Channel = DMA_CHANNEL_3;
    hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_rx.Init.Mode = DMA_NORMAL;
    hdma_spi1_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_spi1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi1_rx);

    /* SPI1_TX Init */
    hdma_spi1_tx.Instance = DMA2_Stream3;
    hdma_spi1_tx.Init.Channel = DMA_CHANNEL_3;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmatx,hdma_spi1_tx);

    /* SPI1 interrupt Init */
//    HAL_NVIC_SetPriority(SPI1_IRQn, 7, 0);
//    HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

    /* SPI1 DMA DeInit */
    HAL_DMA_DeInit(spiHandle->hdmarx);
    HAL_DMA_DeInit(spiHandle->hdmatx);

    /* SPI1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */


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
	extern float plimit;
//SPI发送和接收的DMA中断函数，在中断里面可以查看DMA接收到的SPI数据
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	uint8_t sum_data;
//	LED_RUN_TOGGLE;//测试使用
	
	sum_data = 0;
	sum_data = SPI_to_401_Plimit[2] | SPI_to_401_Plimit[3] | SPI_to_401_Plimit[4] | SPI_to_401_Plimit[5];
	
	//在这里处理接收到的数据
	if(  (SPI_to_401_Plimit[0]==0x5A)&&(SPI_to_401_Plimit[1]==0xA5)&&(SPI_to_401_Plimit[6]==sum_data)&&(SPI_to_401_Plimit[7]==0xAA)  )
	{
		SPI_RXdata.f_byte[0] = SPI_to_401_Plimit[2];
		SPI_RXdata.f_byte[1] = SPI_to_401_Plimit[3];
		SPI_RXdata.f_byte[2] = SPI_to_401_Plimit[4];
		SPI_RXdata.f_byte[3] = SPI_to_401_Plimit[5];
		RX_Plimit = SPI_RXdata.f_data;
		
		if((RX_Plimit>1)&&(RX_Plimit<200))//对接收到的功率值进行限幅处理
		{
			plimit = RX_Plimit;
		}
	}
	
	
	
	
	
	
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
	
	
	//在SPI接收之前需要清空接收数组(随机设置几个值),由DMA对接收数组进行填充，如果正确接收完毕之后，数组的值应当发送变化
	SPI_to_401_Plimit[0] = 0xA0;
	SPI_to_401_Plimit[1] = 0xB0;
	SPI_to_401_Plimit[2] = 0xC0;
	SPI_to_401_Plimit[3] = 0xD0;
	SPI_to_401_Plimit[4] = 0xD1;
	SPI_to_401_Plimit[5] = 0xC1;
	SPI_to_401_Plimit[6] = 0xB1;
	SPI_to_401_Plimit[7] = 0xA1;



}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{


}


/* USER CODE END 1 */
