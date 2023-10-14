/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "iwdg.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

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
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern DMA_HandleTypeDef hdma_adc1;
extern uint16_t ADC_Value_buffer[30];
extern uint8_t State_flag;

uint16_t ADC_1;
float temp_1;


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
	
	//SPI的通信频率为2.25Mbit/s，每10ms f103和f401进行一次数据交换，每次数据交换8个Byte
	uint8_t SPI_to_401_Plimit[8];//通过DMA向f401发送功率限制的原始数据
	uint8_t SPI_from_401_Vcap[8];//DMA接收到的来自f401的原始数据
	float RX_Plimit = 33.0f;//通过DMA向f401发送功率限制的真实数据，初始值33W
	float TX_Vcap = 24.0f;//DMA接收到的来自f401的真实数据，初始值24V
	f2byte SPI_TXdata;
	f2byte SPI_RXdata;
	
	uint8_t uart_data;
	
	extern float plimit;//初始上电限制功率33W




//最终版本应该加上看门狗防止程序跑飞

static uint8_t uart_send_cnt;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	
//	MX_IWDG_Init();
//	HAL_IWDG_Refresh(&hiwdg);//喂狗一次
	
	LED_RUN_OFF;
	LED_CAP_STA_OFF;
	LED_CAP_UV_OFF;
	
	MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
	
  MX_TIM2_Init();
  MX_TIM1_Init();
	MX_TIM3_Init();
	
//	MX_USART1_UART_Init();
	MX_USART6_UART_Init();
	
	HAL_UART_Receive_IT(&huart6, &uart_data,1);
	
  /* USER CODE BEGIN 2 */
	
	
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2000*0.5f-1);
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,2000*0.5f-1);
//	
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
//	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
//	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
//	
//	while (1);
	

	if(HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADC_Value_buffer,3)!= HAL_OK)//启动ADC转换，TIM3每个PWMCLK触发ADC采样，DMA搬运之后进入DMA中断进行处理
	{
    Error_Handler();	
	}	
	__HAL_ADC_DISABLE_IT(&hadc1, (ADC_IT_EOC | ADC_IT_AWD | ADC_IT_OVR | ADC_IT_JEOC));//关闭ADC的中断
	hdma_adc1.Instance->CR &= ~(DMA_IT_HT|DMA_IT_TE|DMA_IT_DME|DMA_IT_FE);//操作寄存器关闭DMA的半满中断，仅启动DMA传输一次完成的中断
	hdma_adc1.Instance->CR &= ~(DMA_IT_HT|DMA_IT_TE|DMA_IT_DME|DMA_IT_FE);//操作寄存器关闭DMA的半满中断，仅启动DMA传输一次完成的中断
	

	
	__HAL_TIM_DISABLE_IT(&htim1, (TIM_IT_UPDATE | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3|TIM_IT_CC4|TIM_IT_COM|TIM_IT_TRIGGER|TIM_IT_BREAK));//关闭tim1的中断
	
//	HAL_Delay(1);

	
	HAL_TIM_Base_Start_IT(&htim2);
	Load_Switch_ON;
	HAL_Delay(300);
	

	HAL_TIM_Base_Start(&htim3);//开启定时器3以42kHz的频率触发ADC采样，ADC对三个通道采样完成之后会产生DMA中断，在DMA中断中处理环路
	
	HAL_Delay(50);
	
	
//	Load_Switch_ON;
	State_flag = 0x00;
	Start();
	
	
	HAL_Delay(500);
	
	
		//在进行DMA传输之前需要先将待发送的Plimit数据按照传输协议放入DMA传送数组（8Byte）
		SPI_from_401_Vcap[0] = 0xA5;
		
		SPI_from_401_Vcap[1] = (uint8_t)(RX_Plimit + 0.5f);
		
		SPI_TXdata.f_data = TX_Vcap;
		SPI_from_401_Vcap[2] = SPI_TXdata.f_byte[0];
		SPI_from_401_Vcap[3] = SPI_TXdata.f_byte[1];
		SPI_from_401_Vcap[4] = SPI_TXdata.f_byte[2];
		SPI_from_401_Vcap[5] = SPI_TXdata.f_byte[3];
		
		SPI_from_401_Vcap[6] = SPI_to_401_Plimit[2] | SPI_to_401_Plimit[3] | SPI_to_401_Plimit[4] | SPI_to_401_Plimit[5] | SPI_to_401_Plimit[1];
		
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

//		HAL_SPI_TransmitReceive_DMA(&hspi1,SPI_from_401_Vcap,SPI_to_401_Plimit,8);//开启SPI传输
	
	
	
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		
	//在进行DMA传输之前需要先将待发送的Plimit数据按照传输协议放入DMA传送数组（8Byte）
	SPI_from_401_Vcap[0] = 0xA5;
	
	SPI_from_401_Vcap[1] = (uint8_t)(RX_Plimit + 0.5f);
	
	TX_Vcap = V_C_real;
//		SPI_TXdata.f_data = plimit;
	SPI_TXdata.f_data = TX_Vcap;
	SPI_from_401_Vcap[2] = SPI_TXdata.f_byte[0];
	SPI_from_401_Vcap[3] = SPI_TXdata.f_byte[1];
	SPI_from_401_Vcap[4] = SPI_TXdata.f_byte[2];
	SPI_from_401_Vcap[5] = SPI_TXdata.f_byte[3];
	
	SPI_from_401_Vcap[6] = SPI_from_401_Vcap[2] | SPI_from_401_Vcap[3] | SPI_from_401_Vcap[4] | SPI_from_401_Vcap[5] | SPI_from_401_Vcap[1];
	
	SPI_from_401_Vcap[7] = 0x5A;
//		LED_RUN_TOGGLE;
		
		
//		HAL_UART_Receive(&huart6, &uart_data,1,0xffff);
//		no_it_uart();
		
		
		for(uart_send_cnt = 0;uart_send_cnt<8;uart_send_cnt++)
		{
			HAL_UART_Transmit(&huart6,&SPI_from_401_Vcap[uart_send_cnt],1,0xffff);
			HAL_Delay(3);//3ms
//			HAL_UART_Receive(&huart6, &uart_data,1,0xffff);
//			no_it_uart();
		}
		
		HAL_Delay(10);
		
		
		
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
//		LED_RUN_TOGGLE;
//		HAL_Delay(500);
		
//		HAL_SPI_TransmitReceive(&hspi1,SPI_from_401_Vcap,SPI_to_401_Plimit,8,0xff);//开启SPI传输
		
		
		
		
		
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
