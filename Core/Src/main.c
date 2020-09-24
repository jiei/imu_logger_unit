/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "lpme1_driver.h"
#include "lpme1.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Enable printf
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Lpme1_Data lpme1Data;

TIM_OC_InitTypeDef ConfigOC_Speaker;
_Bool logger_flag=0;
int counter=0;
const int counter_th=400;	//400 * 0.1 = 40[s] logging
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE
{
 HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
 return ch;
}

void setup(void);
void speaker_output(float duty);

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  setup();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1)==GPIO_PIN_RESET) {
		  for(int i=0;i<3;i++){
		  	  speaker_output(0.5);
		  	  HAL_Delay(500);
		  	  speaker_output(0);
		  	  HAL_Delay(500);
		  }
		  logger_flag=1;
	  }
	  if(logger_flag==0 && counter>counter_th){
	  	  speaker_output(0.5);
	  	  counter=0;
		  HAL_Delay(1000);
		  speaker_output(0);
	  }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void setup(){
	ConfigOC_Speaker.OCMode = TIM_OCMODE_PWM1;
	ConfigOC_Speaker.OCPolarity = TIM_OCPOLARITY_HIGH;
	ConfigOC_Speaker.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	ConfigOC_Speaker.OCFastMode = TIM_OCFAST_DISABLE;
	ConfigOC_Speaker.OCIdleState = TIM_OCIDLESTATE_RESET;
	ConfigOC_Speaker.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	HAL_Delay(300);
	printf("Timestamp,Yaw Angle,Yaw Speed\r\n");
	HAL_Delay(100);
	while(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
		HAL_Delay(100);
	}
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);

	//Ready beep
	for(int i=0;i<3;i++){
		speaker_output(0.5);
		HAL_Delay(50);
		speaker_output(0);
		HAL_Delay(50);
	}
}

void speaker_output(float duty){
	if(duty>0){
		ConfigOC_Speaker.Pulse=(int)(duty*htim3.Init.Period);
		HAL_TIM_PWM_ConfigChannel(&htim3,&ConfigOC_Speaker,TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
	} else {
		ConfigOC_Speaker.Pulse=0;
		HAL_TIM_PWM_ConfigChannel(&htim3,&ConfigOC_Speaker,TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2){
	  if(lpme1_get_timestamp(&lpme1Data.time)==LPME1_OK) HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);	//For instant visual debug
	  //lpme1_get_acc(lpme1Data.acc);
	  lpme1_get_gyr(lpme1Data.gyr);
	  //lpme1_get_mag(lpme1Data.mag);
	  lpme1_get_euler(lpme1Data.euler);
	  //lpme1_get_quat(lpme1Data.quat);

	  if(logger_flag==1){
		  printf("%.3f,%f,%.3f\r\n", lpme1Data.time, lpme1Data.euler[2], lpme1Data.gyr[2]);
		  counter++;
	  }
	  if(counter>counter_th){
		  logger_flag=0;
	  }
  }
  HAL_TIM_Base_Start_IT(&htim2);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
