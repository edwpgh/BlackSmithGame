/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "DFPLAYER_MINI.h"
#include <FLASH_PAGE.h>
#include "ADXL375.h"
#include "math.h"


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
ADC_HandleTypeDef hadc2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t X,Y,Z = 0;
float smoothX, lastX , smoothY, lastY , smoothZ, lastZ;
int dispX , dispY , dispZ, topz;
uint32_t peak = 0;
float alphaX ,  alphaY ,  alphaZ ;
float shock;
uint32_t shock_det_tick;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void Update_LEDs(int score);
void DF_Choose(uint8_t);
//void DF_Init (uint8_t );


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define MAX_LED 100
#define LED_COUNT 20
uint16_t read_adc()
{
	uint16_t ADC_val;
	HAL_ADC_Start(&hadc2);
	while(HAL_ADC_PollForConversion(&hadc2,100) != 0);
	ADC_val = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);
	return ADC_val;
}

// Variables ...........................................
uint8_t LED_Data[MAX_LED][4];
int datasentflag=0;
int comp;
uint32_t tops;
//Shift register variables
uint8_t dig0, dig1, dig2;
uint8_t dig0_map[11] = {0x3F, 0x0C, 0x5B, 0x5E, 0x6C, 0x76, 0xF7, 0x1C, 0xFF, 0xFE, 0x00};
uint8_t dig1_map[11] = {0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0xFF, 0x7B, 0x00};
uint8_t dig2_map[11] = {0x7F, 0x30, 0xEC, 0xF8, 0xB2, 0xDA, 0xDE, 0x70, 0xFF, 0xFA, 0x00};
#define SEG_OFF	10

// End of Variables ....................................
////////////////////////////////////START_WS2811_setup_functions//////////////////////////////
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
	datasentflag=1;
}

void Set_LED (int LEDnum, int Red, int Green, int Blue)
{
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}

uint16_t pwmData[(24*MAX_LED)+50];

void WS2812_Send (void)
{
	uint32_t indx=0;
	uint32_t color;

	for (int i = 0; i < MAX_LED; i++)
	{
		color = ((uint32_t)LED_Data[i][1] << 16) | ((uint32_t)LED_Data[i][2] << 8) | LED_Data[i][3];

		for (int bit = 23; bit >= 0; bit--)
		{
			if (color & (1 << bit))
			{
				pwmData[indx] = 60;  // logic 1
			}
			else
			{
				pwmData[indx] = 30;  // logic 0
			}
			indx++;
		}
	}

	for (int i = 0; i < 50; i++)
	{
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
	while (!datasentflag);
	datasentflag = 0;
}
////////////////////////////////////END_WS2811_setup_functions//////////////////////////////
////////////////////////////////////START_the_seven-segment_data_sending_function//////////////////////////////
void DisplayNumber(uint16_t value)
{
	uint8_t d0, d1, d2, dig0, dig1, dig2;

	// Reset Shift register
	HAL_GPIO_WritePin(MR_GPIO_Port, MR_Pin, 0);
	HAL_Delay(1);
	HAL_GPIO_WritePin(MR_GPIO_Port, MR_Pin, 1);
	//////

	///// Extraction and placement on digits
	if(value != SEG_OFF)
	{
		d0 = value % 10;          //one
		d1 = (value / 10) % 10;  // Villagers
		d2 = value / 100;       //  Centuries
		dig0 = dig0_map[d2];
		dig1 = dig1_map[d1];
		dig2 = dig2_map[d0];
	}
	else // 11 means off
	{
		dig0 = dig0_map[10];
		dig1 = dig1_map[10];
		dig2 = dig2_map[10];
	}

	///Combination of three numbers
	uint32_t digs = dig0 + (dig1 << 8) + (dig2 << 16);


	// insert digits to shift register
	for(int i = 0; i < 24; i++)
	{
		HAL_GPIO_WritePin(SER_GPIO_Port, SER_Pin, ((digs >> i)&0x01));
		__NOP(); //// Delay_12ns
		HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, 1);
		__NOP();
		HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, 0);
	}
	HAL_GPIO_WritePin(Latch_GPIO_Port, Latch_Pin, 1);
	__NOP();
	HAL_GPIO_WritePin(Latch_GPIO_Port, Latch_Pin, 0);


}
////////////////////////////////////END_the_seven-segment_data_sending_function//////////////////////////////
void clear_Segs()
{

}

////////////////////////////////////START_the_7seg_effects_And_Records_functions//////////////////////////////////////////
void show_effect_countup_blocking(int target) {
	if (target <= 0) {
		DisplayNumber(0);
		return;
	}

	int steps = 40;
	int base_delay = 4;
	int slow_max = 180;
	HAL_Delay(1000);
	DF_Choose(4);
	for (int i = 1; i <= steps; i++) {
		int val = (i * target) / steps;
		DisplayNumber(val);

		int delay_ms = base_delay + (val * slow_max) / ( (target>0) ? target : 1 );
		Update_LEDs(val);
		HAL_Delay(delay_ms);


	}

	DisplayNumber(target);
}

void blink_on_target(uint32_t target) {
	Flash_Read_Data (0x08007000, &tops , 1 );/// Records_save
	if( tops < target)
	{
		DF_Choose(1);
		Flash_Write_Data (0x08007000,&target, 1);
	}
	else
	{
		DF_Choose(2);
	}
	for (int i = 0; i < 3; i++) {
		DisplayNumber(target);
		HAL_Delay(500);
		DisplayNumber(SEG_OFF);
		HAL_Delay(500);
		DisplayNumber(target);
		HAL_Delay(500);
		DisplayNumber(SEG_OFF);
		HAL_Delay(500);


	}
	DisplayNumber(target);

}
////////////////////////////////////END_the_7seg_effects_And_Records_functions//////////////////////////////////////////
///////////////////////////////////START_ws2811_effect_functions/////////////////////////////////////////
void Update_LEDs(int score)
{

	uint8_t leds_to_light = score / 50;
	if (leds_to_light > LED_COUNT) leds_to_light = LED_COUNT;


	for (int i = 0; i < LED_COUNT; i++) {
		Set_LED(i, 0, 0, 0);
	}

	for (int i = 0; i < leds_to_light; i++) {
		Set_LED(i, 255, 0, 125);
	}

	WS2812_Send();
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
  MX_DMA_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

    ADXL375 dev;
    int errors = 0;
    errors = ADXL375_Initialise(&dev, &hspi1);
    ADXL375_EnableShockDetection(&dev, 0x07, 25, 10);
    WriteData(&dev, ADXL375_FIFO_CTL, 0x80 | 0x1F, 0);
//    WriteData(&dev, ADXL375_OFFSET_X, 0xFF, 0);

	DisplayNumber(SEG_OFF); ////// For_Start
	for(int i = 0; i < 6; i++) ////// For_debugging_and_error
	{
		HAL_GPIO_TogglePin(MCU_LED_GPIO_Port, MCU_LED_Pin);
		HAL_Delay(50);
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	dig2 = 0x01;
	dig1 = 0x00;
	dig0 = 0x00;
	int THRESHOLD = 100;
	int ADC_Now=0;
	int ADC_Last=0;
	int ADC_MAX = 900;

	DF_Init(50);
	HAL_Delay(100);

	// Just for starting bug
	ADC_Last = read_adc();

	lastX = 0;
	lastZ = 0;
	lastY = 0;

	HAL_InitTick(0);

	while (1)
	{
//		HAL_StatusTypeDef status;
//		status = ADXL375_ReadAcceleration(&dev);
//		if (status == HAL_OK)
//			ADXL375_CleanRawValues(&dev);


//		alphaX = 0.1;
//		smoothX = alphaX * dev.accData[0] + (1 - alphaX) * lastX;
//		lastX = smoothX;
//		dispX = (int) smoothX;
//		/////
//		alphaY = 0.1;
//		smoothY = alphaY * dev.accData[1] + (1 - alphaY) * lastY;
//		lastY = smoothY;
//		dispY = (int) smoothY ;
//		///////
//		alphaZ = 0.1;
//		smoothZ = alphaZ * dev.accData[2] + (1 - alphaZ) * lastZ;
//		lastZ = smoothZ;
//		dispZ = (int) smoothZ + 100 ;
		//			  DisplayNumber(smoothY);

		// Read Data Continuously .............................................
		peak = ADXL375_Read_peak_from_100(&dev);

		// Check Shock ........................................................
		uint8_t shockStat = 0;
		if (ADXL375_CheckShock(&dev, &shockStat))
		{
			shock = 1;
			shock_det_tick = HAL_GetTick();
		}
		else
		{
			if(HAL_GetTick() - shock_det_tick > 5000)
				shock = 0;
		}

		// Prepare peak data ..................................................
		if(shock == 1)
		{
			if(topz < peak)
				topz = peak;
		}
		else
			topz = 0;

//			  HAL_Delay(101);
		//////Hit_detection_commands
//		ADC_Now = read_adc();
//		int diff = ADC_Now - ADC_Last;
//
//		if (diff > THRESHOLD)
//		{
//
//			peak = diff;
//
//
//			int display_val = (peak * 999) / ADC_MAX;
//			if (display_val > 999) display_val = 999;
//
//
//			show_effect_countup_blocking(display_val); //
//			blink_on_target(display_val);
//
//		}
//		ADC_Last = ADC_Now;
//		HAL_ADC_Stop(&hadc2);
//		HAL_Delay(1);
//


		//	  num++;


		//sprintf (msg,"ADC: %u\r\n",read_adc());
		//HAL_UART_Transmit(&huart2,(uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		//HAL_Delay(500);

		HAL_Delay(10);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 89;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
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

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin|MCU_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MR_Pin|CLK_Pin|Latch_Pin|SER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI1_CS_Pin MCU_LED_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|MCU_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_key_Pin */
  GPIO_InitStruct.Pin = MCU_key_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MCU_key_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MR_Pin CLK_Pin Latch_Pin SER_Pin */
  GPIO_InitStruct.Pin = MR_Pin|CLK_Pin|Latch_Pin|SER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
