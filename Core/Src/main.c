/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flash.h"
#include "sk6812.h"
#include "i2c_ex.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIRMWARE_VERSION 1
#define I2C_ADDRESS 0x38
#define APPLICATION_ADDRESS     ((uint32_t)0x08001000)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t g_charge_ctrl = 0;
uint8_t i2c_address[1] = {0};
volatile uint8_t fm_version = FIRMWARE_VERSION;
volatile int8_t speed_value[4] = {0};
volatile int8_t speed_update_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IAP_Set()
{
	uint8_t i;
 
	uint32_t *pVecTab=(uint32_t *)(0x20000000);
	//????????SRAM???
	for(i = 0; i < 48; i++)
	{
		*(pVecTab++) = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i<<2));
	}
  /* Enable the SYSCFG peripheral clock*/
#if 1 //STM32
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  //??? SRAM ??? 0x00000000
  __HAL_SYSCFG_REMAPMEMORY_SRAM();
#else //AMP32
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SYSCFG);
    /* Remap SRAM at 0x00000000 */
    SYSCFG->CFG1_B.MMSEL = SYSCFG_MemoryRemap_SRAM;
#endif
}

void user_i2c_init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**I2C1 GPIO Configuration
  PA9   ------> I2C1_SCL
  PA10   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* I2C1 interrupt Init */
  NVIC_SetPriority(I2C1_IRQn, 0);
  NVIC_EnableIRQ(I2C1_IRQn);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x0000020B;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = i2c_address[0]<<1;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
  /* USER CODE BEGIN I2C1_Init 2 */
  set_i2c_slave_address(i2c_address[0]);
  /* USER CODE END I2C1_Init 2 */

}

void i2c_address_write_to_flash(void) 
{   
  writeMessageToFlash(i2c_address , 1);   
}

void i2c_address_read_from_flash(void) 
{   
  if (!(readPackedMessageFromFlash(i2c_address, 1))) {
    i2c_address[0] = I2C_ADDRESS;
    i2c_address_write_to_flash();
  }
}

volatile uint8_t offset=0;

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
				GPIOA->BRR=GPIO_PIN_3;
				pwm_pin[motorx].GPIOx= GPIOA;
				pwm_pin[motorx].pinNum= GPIO_PIN_4;
				break;
			}
			case 1:
			{
				GPIOB->BRR=GPIO_PIN_1;
				pwm_pin[motorx].GPIOx= GPIOA;
				pwm_pin[motorx].pinNum= GPIO_PIN_2;
				break;
			}
      case 2:
			{	
				GPIOA->BRR = GPIO_PIN_1;
				pwm_pin[motorx].GPIOx = GPIOA;
				pwm_pin[motorx].pinNum = GPIO_PIN_7;
				break;
			}
			case 3:
			{
				GPIOA->BRR=GPIO_PIN_6;
				pwm_pin[motorx].GPIOx= GPIOA;
				pwm_pin[motorx].pinNum= GPIO_PIN_5;
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
        GPIOA->BRR=GPIO_PIN_4;
				pwm_pin[motorx].GPIOx= GPIOA;
				pwm_pin[motorx].pinNum= GPIO_PIN_3;
				break;
			}
			case 1:
			{
				GPIOA->BRR=GPIO_PIN_2;
				pwm_pin[motorx].GPIOx= GPIOB;
				pwm_pin[motorx].pinNum= GPIO_PIN_1;
				break;
			}
      case 2:
			{		
				GPIOA->BRR=GPIO_PIN_7;
				pwm_pin[motorx].GPIOx= GPIOA;
				pwm_pin[motorx].pinNum= GPIO_PIN_1;
				break;
			}
			case 3:
			{
        GPIOA->BRR=GPIO_PIN_5;
				pwm_pin[motorx].GPIOx= GPIOA;
				pwm_pin[motorx].pinNum= GPIO_PIN_6;
				break;
			}
		}
	}	
}

void switchInputAndOutput(uint16_t pin, uint32_t type, uint32_t pull_type, uint32_t freq_type)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (type == GPIO_MODE_ANALOG)
      __HAL_RCC_ADC1_CLK_ENABLE();    
    /*Configure GPIO pins : PAPin PAPin PAPin PAPin
                            PAPin PAPin PAPin PAPin */
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = type;
    GPIO_InitStruct.Pull = pull_type;
    GPIO_InitStruct.Speed = freq_type;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    if (type == GPIO_MODE_ANALOG) {
      hadc.Instance = ADC1;
      hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
      hadc.Init.Resolution = ADC_RESOLUTION_12B;
      hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
      hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
      hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
      hadc.Init.LowPowerAutoWait = DISABLE;
      hadc.Init.LowPowerAutoPowerOff = DISABLE;
      hadc.Init.ContinuousConvMode = DISABLE;
      hadc.Init.DiscontinuousConvMode = DISABLE;
      hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
      hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
      hadc.Init.DMAContinuousRequests = DISABLE;
      hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
      HAL_ADC_Init(&hadc);      
    }    
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void readADCState(uint32_t channel, uint8_t bit_mode)
{
    uint32_t AD_Value = 0;
    uint32_t Value[22] = {0};
    uint8_t tmp_buff[16] = {0};
    uint16_t tmp_buff_len = 0;
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc.Instance->CHSELR = 0;
    uint32_t max = 0;
    uint32_t min = 0;

    sConfig.Channel = channel;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc, &sConfig);

    HAL_ADCEx_Calibration_Start(&hadc);

    for(int n=0;n<22;n++)
    {
        HAL_ADC_Start(&hadc);
        HAL_ADC_PollForConversion(&hadc, 10);
        if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc), HAL_ADC_STATE_REG_EOC))
        {
        	Value[n] = HAL_ADC_GetValue(&hadc);
          AD_Value += Value[n];
        }
    }
    max=Value[0];
    min=Value[0];

    for(char n=0;n<22;n++)//取最大�?��?�最小�??
    {
        max=(Value[n]<max)?max:Value[n];    
        min=(min<Value[n])?min:Value[n];
    }     

    AD_Value = (AD_Value - max - min) / 20;
    
    if (bit_mode == 12) {
      tmp_buff[0] = (uint8_t)AD_Value;
      tmp_buff[1] = (uint8_t)(AD_Value>>8);
      tmp_buff_len = 2;
    } else if (bit_mode == 8) {
      tmp_buff[0] = map(AD_Value,0,4095,0,255);
      tmp_buff_len = 1;
    }
    i2c1_set_send_data(tmp_buff, tmp_buff_len);
}

void Slave_Complete_Callback(uint8_t *rx_data, uint16_t len) {
	offset=*rx_data;
	if(len>1)
	{
		len -= 1;
		if(((offset & 0xf0) == 0x00) && offset<4)//Motor
		{		
			for(uint8_t i=offset; i<(offset + len); i++ )
			{
        speed_value[i] = (int8_t)rx_data[1 + i - offset];
				MotorReversible(i, speed_value[i]);
			}
		} 
		else if(((offset & 0xf0) == 0x10 && offset<0x14))
		{
			switchInputAndOutput(GPIO_PIN_0, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_MEDIUM);
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
    else if(offset == 0xFD)
    {
      if (rx_data[1] == 1) {
        LL_I2C_DeInit(I2C1);
        LL_I2C_DisableAutoEndMode(I2C1);
        LL_I2C_Disable(I2C1);
        LL_I2C_DisableIT_ADDR(I2C1);
        HAL_TIM_PWM_DeInit(&htim16);  
        NVIC_SystemReset();       
      }
    }    
//		else if(((offset & 0xf0) == 0x40 && offset<0x41))
//		{
//			g_charge_ctrl = rx_data[1];
//			if(g_charge_ctrl)
//			{
//				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);
//			}
//			else
//			{
//				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
//			}
//		}
		else if(offset == 0xFF)
		{
      if (len == 1) {
        if (rx_data[1] < 128) {
          i2c_address[0] = rx_data[1];
          i2c_address_write_to_flash();
          user_i2c_init();
        }
      }
		}	    
	} else if (len == 1) {
		if(((offset & 0xf0) == 0x20 && offset<0x21))
		{
			switchInputAndOutput(GPIO_PIN_0, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
			readADCState(0, 8);
		}
		else if(((offset & 0xf0) == 0x30 && offset<0x32))
		{
			switchInputAndOutput(GPIO_PIN_0, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
			readADCState(0, 12);
		}
//		else if(((offset & 0xf0) == 0x40 && offset<0x41))
//		{
//			i2c1_set_send_data((uint8_t *)&g_charge_ctrl, 1);
//		}	
		else if(((offset & 0xf0) == 0x00) && offset<4)
		{
			i2c1_set_send_data((uint8_t *)&speed_value[offset], 1);
		}	
		else if(offset == 0xFE)
		{
			i2c1_set_send_data((uint8_t *)&fm_version, 1);
		}	
		else if(offset == 0xFF)
		{
			i2c1_set_send_data(i2c_address, 1);
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
  IAP_Set();
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
  // MX_I2C1_Init();
  MX_TIM16_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
  i2c_address_read_from_flash();
  user_i2c_init();    
	HAL_TIM_Base_Start_IT(&htim16);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
	pwm_pin[0] = (Pin_t){GPIOA, GPIO_PIN_3, 0, 1};
	pwm_pin[1] = (Pin_t){GPIOA, GPIO_PIN_2, 0, 1};
	pwm_pin[2] = (Pin_t){GPIOA, GPIO_PIN_1, 0, 1};
	pwm_pin[3] = (Pin_t){GPIOA, GPIO_PIN_6, 0, 1};

  sk6812_init(2);  
  i2c1_it_enable();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    i2c_timeout_counter = 0;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(TIM16->CNT >= 100)				//��ƽ����
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI14_Enable();

   /* Wait till HSI14 is ready */
  while(LL_RCC_HSI14_IsReady() != 1)
  {

  }
  LL_RCC_HSI14_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(48000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_HSI14_EnableADCControl();
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
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
