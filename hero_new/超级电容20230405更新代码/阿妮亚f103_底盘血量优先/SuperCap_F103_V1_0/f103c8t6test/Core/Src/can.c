/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"
#include "referee_system.h"
/* USER CODE BEGIN 0 */
static CAN_TxHeaderTypeDef TxMessage; //CAN���͵���Ϣ����Ϣͷ
static CAN_RxHeaderTypeDef RxMessage; //CAN���յ���Ϣ����Ϣͷ
float supercap_voltage = 0.0f;
float supercap_energy_percent = 0.0f;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn,12, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
 void CANFilter_Config(void)
{
    CAN_FilterTypeDef  sFilterConfig;
    
    sFilterConfig.FilterBank = 0;                       //CAN��������ţ���Χ0-27
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;   //CAN������ģʽ������ģʽ���б�ģʽ
    sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;  //CAN�������߶ȣ�16λ��32λ
    sFilterConfig.FilterIdHigh = 0x401 << 5;			//32λ�£��洢Ҫ����ID�ĸ�16λ
    sFilterConfig.FilterIdLow = 0x800 << 5;					//32λ�£��洢Ҫ����ID�ĵ�16λ
    sFilterConfig.FilterMaskIdHigh = 0x800 << 5;			//����ģʽ�£��洢��������
    sFilterConfig.FilterMaskIdLow = 0x800 << 5;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;				//����ͨ����������ƥ��󣬴洢���ĸ�FIFO
    sFilterConfig.FilterActivation = ENABLE;    		//���������
    sFilterConfig.SlaveStartFilterBank = 0;
    
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) 
		 {
        Error_Handler();
       }

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t  data[8];
    HAL_StatusTypeDef	status;
    
    if (hcan == &hcan1) 
		{	
        status = HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMessage, data);
        if (HAL_OK == status)
			{                              			
					if (hcan == &hcan1)
					{
						switch (RxMessage.StdId)
						{
							case CAN_SUPERCAP_RECEIVE_ID:
							{
								robot_referee_status.game_robot_status.robot_level = data[0];
								break;
							}

							default:
								break;
						}
					}
       }
    }
}
void CAN1_Send()
{
    uint32_t TxMailbox;
    uint8_t data[8];
	int32_t supercap_vv = supercap_voltage * 10000;
	int32_t supercap_per = supercap_energy_percent * 10000;
	
	volatile uint32_t WaitTime;
	
    TxMessage.IDE = CAN_ID_STD;     //����ID����
	 TxMessage.StdId = 0x301;        //����ID��
    TxMessage.RTR = CAN_RTR_DATA;   //���ô�������֡
	 TxMessage.DLC = 8;              //�������ݳ���
	data[0] = supercap_vv>>24;
	data[1] = supercap_vv>>16;
	data[2] = supercap_vv>>8;
	data[3] = supercap_vv;
	data[4] = supercap_per>>24;
	data[5] = supercap_per>>16;
	data[6] = supercap_per>>8;
	data[7] = supercap_per;
	
	
	WaitTime = 100000;//��������������ʱ�򣬵ȴ�һ�ᣬ���ȴ���Լ10ns*100000=1ms��������仹�������жϳ�ʱ���ȴ���
	while((HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0) && (WaitTime>0) )
	{
		WaitTime--;
	}
	//����Ƿ��п��з������䣬���û���з������䣬���֡���ݺ��Ե���ǰ������ѹ��˳�ʱʱ�䣩
	if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0 )//CAN����ʽ������Ҫ�ȼ�鷢�������Ƿ�ǿ�
	{
		if (HAL_CAN_AddTxMessage(&hcan1, &TxMessage, data, &TxMailbox) != HAL_OK)
		{
			Error_Handler();
		}	
	}	
	
	
	
	
//	if (HAL_CAN_AddTxMessage(&hcan1, &TxMessage, data, &TxMailbox) != HAL_OK)
//		{
//        Error_Handler();
//     }	
}

void CAN_Start_Init()
{
  if (HAL_CAN_Start(&hcan1) != HAL_OK) 
    {
        Error_Handler();
     }

    
    /* 3. Enable CAN RX Interrupt */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) !=  HAL_OK) {
        Error_Handler();
    }
}
/* USER CODE END 1 */
