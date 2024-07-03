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
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

	uint8_t Uart_data[1024];
	
	uint8_t Uart_f401_data[50];
	
	extern uint8_t Uart_data_rec[50];

	
	
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
	
	//SPI的通信频率为2.25Mbit/s，每10ms f103和f401进行一次数据交换，每次数据交换（双工通信）8个Byte
	uint8_t SPI_to_401_Plimit[8];//通过DMA向f401发送功率限制的原始数据
	uint8_t SPI_from_401_Vcap[8];//DMA接收到的来自f401的原始数据
	float TX_Plimit = 40.0f;//通过DMA向f401发送功率限制的真实数据，初始值40W
	float RX_Vcap = 24.0f;//DMA接收到的来自f401的真实数据，初始值24V
	f2byte SPI_TXdata;
	f2byte SPI_RXdata;
	
extern float PLIMIT_TEST;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
//	HAL_UART_Receive_DMA(&huart1, Uart_data, 1024);
	
	HAL_TIM_Base_Start_IT(&htim3);//开启定时器3
	
	SPI_to_401_Plimit[0]=0xA5;

	CANFilter_Config();
  CAN_Start_Init();
	HAL_TIM_Base_Start_IT(&htim2);
	
	HAL_Delay(2000);
//	HAL_UART_Receive_DMA(&huart1, Uart_f401_data, 20);
	HAL_UART_Receive_IT(&huart1, Uart_data_rec, 1);


  /* USER CODE END 2 */



  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
//		PLIMIT_TEST = 30;
//		
//		HAL_Delay(3000);
//		
//		PLIMIT_TEST = 20;
//		
//		HAL_Delay(3000);
				
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
//		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
//		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);
//		HAL_Delay(5000);//用于代码测试，测试动态改变功率参考值的情况
//		TX_Plimit= 40;
//		HAL_Delay(5000);
//		TX_Plimit= 20;
		
		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
