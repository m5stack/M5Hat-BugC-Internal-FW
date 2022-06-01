/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "gpio.h"
#include "sk6812.h"
#include "i2c.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t offset=0;

typedef struct 
{
	GPIO_TypeDef *GPIOx;
	uint16_t pinNum;
	uint8_t pulse;
	uint8_t flip;
} Pin_t;

Pin_t pwm_pin[4];
void MotorReversible(uint8_t motorx, int8_t value)
{
  value = 0 - value;
	if(value>=0)
	{
		pwm_pin[motorx].pulse = value;
		switch(motorx)
		{
			case 0:
			{
				GPIOB->BRR=GPIO_PIN_1;
				pwm_pin[motorx].GPIOx= GPIOA;
				pwm_pin[motorx].pinNum= GPIO_PIN_1;
				break;
			}
			case 1:
			{
				GPIOA->BRR=GPIO_PIN_5;
				pwm_pin[motorx].GPIOx= GPIOA;
				pwm_pin[motorx].pinNum= GPIO_PIN_7;
				break;
			}
      case 2:
			{	
				GPIOA->BRR = GPIO_PIN_4;
				pwm_pin[motorx].GPIOx = GPIOA;
				pwm_pin[motorx].pinNum = GPIO_PIN_3;
				break;
			}
			case 3:
			{
				GPIOA->BRR=GPIO_PIN_6;
				pwm_pin[motorx].GPIOx= GPIOA;
				pwm_pin[motorx].pinNum= GPIO_PIN_2;
				break;
				
			}
		}
	}
	else
	{
		pwm_pin[motorx].pulse= -value;
		switch(motorx)
		{
			case 0:
			{
        GPIOA->BRR=GPIO_PIN_1;
				pwm_pin[motorx].GPIOx= GPIOB;
				pwm_pin[motorx].pinNum= GPIO_PIN_1;
				break;
			}
			case 1:
			{
				GPIOA->BRR=GPIO_PIN_7;
				pwm_pin[motorx].GPIOx= GPIOA;
				pwm_pin[motorx].pinNum= GPIO_PIN_5;
				break;
			}
      case 2:
			{		
				GPIOA->BRR=GPIO_PIN_3;
				pwm_pin[motorx].GPIOx= GPIOA;
				pwm_pin[motorx].pinNum= GPIO_PIN_4;
				break;
			}
			case 3:
			{
        GPIOA->BRR=GPIO_PIN_2;
				pwm_pin[motorx].GPIOx= GPIOA;
				pwm_pin[motorx].pinNum= GPIO_PIN_6;
				break;
			}
		}
	}	
}

void i2c1_receive_callback(uint8_t *rx_data, uint16_t len) {
	offset=*rx_data;
	if(len>1)
	{
		len -= 1;
		if(((offset & 0xf0) == 0x00) && offset<4)//Motor
		{		
			for(uint8_t i=offset; i<(offset + len); i++ )
			{
				MotorReversible(i, (int8_t)rx_data[1 + i - offset]);
			}
		} 
		else if(((offset & 0xf0) == 0x10 && offset<20))
		{
			if(rx_data[1]>1)
			{
				neopixel_set_all_color(rx_data[3]<<16 | rx_data[2]<<8 | rx_data[4]);
			}
			else
			{
				neopixel_set_color(rx_data[1], rx_data[3]<<16 | rx_data[2]<<8 | rx_data[4]);
			}
			neopixel_show();
		}
	}
}


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
  MX_I2C1_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
	HAL_I2C_EnableListen_IT(&hi2c1);
	HAL_TIM_Base_Start_IT(&htim16);
	pwm_pin[0] = (Pin_t){GPIOB, GPIO_PIN_1, 0, 1};
	pwm_pin[1] = (Pin_t){GPIOA, GPIO_PIN_7, 0, 1};
	pwm_pin[2] = (Pin_t){GPIOA, GPIO_PIN_4, 0, 1};
	pwm_pin[3] = (Pin_t){GPIOA, GPIO_PIN_6, 0, 1};
	
	sk6812_init(2);
  /* USER CODE END 2 */
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
	{
    /* USER CODE END WHILE */
			
    /* USER CODE BEGIN 3 */
		if(TIM16->CNT >= 100)				//电平拉高
		{
			TIM16->CNT = 0;
			for(uint8_t i = 0; i < 4; i++) 
			{		
				if(pwm_pin[i].pulse != 0) 
				{
					pwm_pin[i].GPIOx->BSRR = pwm_pin[i].pinNum;
					pwm_pin[i].flip = 1;
				}
			}
		} 
		else 
		{
			for(uint8_t i = 0; i < 4; i++)
			{		
				if(TIM16->CNT >= pwm_pin[i].pulse && pwm_pin[i].flip) 
				{
					pwm_pin[i].GPIOx->BRR = pwm_pin[i].pinNum;
					pwm_pin[i].flip = 0;
				}
			}
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
