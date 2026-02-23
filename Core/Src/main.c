/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define ON  1
#define OFF 0

#define OLD 0
#define NOW 1

#define SERVO_PAN_MIN							600
#define SERVO_PAN_MAX							2200
#define SERVO_PAN_GAIN						4

//#define SERVO_PAN_SPEED_LV0				1000
#define SERVO_PAN_SPEED_LV0				800
#define SERVO_PAN_SPEED_LV1				300
#define SERVO_PAN_SPEED_LV2				100
#define SERVO_PAN_SPEED_LV3				50

#define SERVO_TILT_MIN						800
#define SERVO_TILT_MAX						1600
#define SERVO_TILT_GAIN						4

#define SERVO_TILT_SPEED_LV0			800
#define SERVO_TILT_SPEED_LV1			300
#define SERVO_TILT_SPEED_LV2			100
#define SERVO_TILT_SPEED_LV3			50

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

void HextoASCLL(uint8_t *p1, uint8_t *p2, uint8_t size);
void Memcopy(uint8_t *p1, uint8_t *p2, uint8_t size);
void SysTick_Main(void);

uint8_t Pelco_Frame_Chk(uint8_t *buf);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t	recv1[32];
uint8_t	send1[32];

uint8_t	recv2[32];
uint8_t	send2[32];

uint16_t systick_cnt;

// 0:stop 1:right or up 2:left or down
uint8_t servo_sts_pan[2];
uint8_t servo_sts_tilt[2];

uint16_t servo_pwm_pan;
uint16_t servo_pwm_tilt;

// 0 to 63
uint8_t servo_speed_pan;
uint8_t servo_speed_tilt;

uint8_t servo_speed_pan_lv;
uint8_t servo_speed_tilt_lv;

uint16_t servo_pan_cnt;
uint16_t servo_tilt_cnt;

uint16_t servo_pan_cnt_max;
uint16_t servo_tilt_cnt_max;

// pelco cmd
uint8_t pelco_cmd_buf[7];
uint8_t pelco_cmd_now[7];
uint8_t pelco_cmd_old[7];


uint8_t pelco_recv_flg;

uint8_t mode;

uint8_t dir;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	mode = 1;		// manual mode
	
	dir = 0;
	
	servo_pan_cnt_max  = SERVO_PAN_SPEED_LV1;
	servo_tilt_cnt_max = SERVO_PAN_SPEED_LV1;
	
	servo_pwm_pan  = 1400;
	servo_pwm_tilt = 1300;
	
	servo_sts_pan[OLD] = 0;
	servo_sts_pan[NOW] = 0;

	servo_sts_tilt[OLD] = 0;
	servo_sts_tilt[NOW] = 0;
	
	send1[0] = 0x30;
	send1[1] = 0x30;
	send1[2] = 0x30;
	send1[3] = 0x30;
	send1[4] = 0x30;
	send1[5] = 0x0D;
	send1[6] = 0x0A;

	send2[0] = 0x30;
	send2[1] = 0x30;
	send2[2] = 0x30;
	send2[3] = 0x30;
	send2[4] = 0x30;
	send2[5] = 0x0D;
	send2[6] = 0x0A;
	
	systick_cnt = 0;
	
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);

	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,servo_pwm_pan);		
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,servo_pwm_tilt);		


	HAL_UART_Receive_DMA(&huart1,recv1,7);
	HAL_UART_Receive_DMA(&huart2,recv2,7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 49;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 49;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HextoASCLL(uint8_t *p1, uint8_t *p2, uint8_t size)
{
	uint8_t loop1;
	uint8_t dat;
	uint8_t dat_h;
	uint8_t dat_l;
	
	for( loop1 = 0; loop1 < size; loop1++ )
	{
		dat = *(p1+loop1);
		
		dat_h = (dat >> 4) & 0x0F;
		dat_l = (dat >> 0) & 0x0F;
		
		if( dat_h < 0x0A )
		{
			dat_h = dat_h | 0x30;
		}
		else
		{
			dat_h = dat_h + 0x37;
		}
		
		if( dat_l < 0x0A )
		{
			dat_l = dat_l | 0x30;
		}
		else
		{
			dat_l = dat_l + 0x37;
		}
		
		*(p2+(loop1*2)+0) = dat_h;
		*(p2+(loop1*2)+1) = dat_l;
	}
	
}

void Memcopy(uint8_t *p1, uint8_t *p2, uint8_t size)
{
	uint8_t loop1;
	
	for( loop1 = 0; loop1 < size; loop1++ )
	{
		*(p2 + loop1) = *(p1 + loop1);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t ans;

	if( huart == &huart1 )													// SightLine Port
	{
//		if( mode == 0x01 )														// Auto scan mode?
//		{
			ans = Pelco_Frame_Chk(recv1);								// Recv Frame data chk
			if( ans == 0 )															// Recv frame no error?
			{
				send2[14] = 0x0D;
				send2[15] = 0x0A;
			
				HextoASCLL(recv1,send2,7);								//
				HAL_UART_Transmit_DMA(&huart2,send2,16);	//
				
				Memcopy(recv1,pelco_cmd_buf,7);						// Recv data copy to buf
				pelco_recv_flg = ON;											// Recv flg ON
			}
//		}

		HAL_UART_Receive_DMA(&huart1,recv1,7);
	}
	else if( huart == &huart2 )											// USB COM Port
	{
		ans = Pelco_Frame_Chk(recv2);									// Recv Frame data chk
		if( ans == 0 )																// Recv frame no error?
		{			
			send2[14] = 0x0D;
			send2[15] = 0x0A;
		
			HextoASCLL(recv2,send2,7);									//
			HAL_UART_Transmit_DMA(&huart2,send2,16);		//
			
			HAL_UART_Transmit_DMA(&huart1,send2,16);		//
			
			mode = recv2[2] & 0x01;											// Manual Scan or Auto Scan
			
			if( mode == 0x00 )													// Manual Scan?
			{
				Memcopy(recv2,pelco_cmd_buf,7);						// Recv data copy to buf
				pelco_recv_flg = ON;											// Recv flg ON
			}
			
			HAL_UART_Receive_DMA(&huart2,recv2,7);			//
		}
	}
}	

uint8_t Pelco_Frame_Chk(uint8_t *buf)
{
	uint8_t		err_flg;
	uint8_t		sum;
	uint8_t		loop1;
	err_flg = 1;
	
	if( *(buf+0) == 0xFF )
	{
		sum = 0;
    for( loop1 = 0; loop1 < 5; loop1++ )
    {
			sum += *(buf+1+loop1);
    }
		
		if( *(buf+6) == sum )
		{
			err_flg = 0;			
		}
	}
	
	return(err_flg);
}

void SysTick_Main(void)
{
	uint8_t		pan_cmd;
	uint8_t		tilt_cmd;

	uint8_t		pan_speed;
	uint8_t		tilt_speed;
	
	if( pelco_recv_flg == ON )
	{
		pelco_recv_flg = OFF;
		
		Memcopy(pelco_cmd_now,pelco_cmd_old,7);
		Memcopy(pelco_cmd_buf,pelco_cmd_now,7);
		
		pan_cmd  = (pelco_cmd_now[3] >> 1) & 0x03;
		tilt_cmd = (pelco_cmd_now[3] >> 3) & 0x03;
		
		servo_sts_pan[NOW] = pan_cmd;
		servo_sts_tilt[NOW] = tilt_cmd;
		
		pan_speed = pelco_cmd_now[4];
		tilt_speed = pelco_cmd_now[5];
				
		if( pan_speed > 50 )
		{
			servo_pan_cnt_max = 5;
		}
		else if( pan_speed > 32 )
		{
			servo_pan_cnt_max = 15;
		}
		else if( pan_speed > 16 )
		{
			servo_pan_cnt_max = 25;	
		}
		else if( pan_speed > 10 )
		{
			servo_pan_cnt_max = 100;			
		}
		else if( pan_speed > 5 )
		{
			servo_pan_cnt_max = 200;			
		}
		else
		{
			servo_pan_cnt_max = 400;				
		}

		if( tilt_speed > 50 )
		{
			servo_tilt_cnt_max = 5;
		}
		else if( tilt_speed > 32 )
		{
			servo_tilt_cnt_max = 15;
		}
		else if( tilt_speed > 16 )
		{
			servo_tilt_cnt_max = 25;
		}
		else if( tilt_speed > 10 )
		{
			servo_tilt_cnt_max = 100;			
		}
		else if( tilt_speed > 5 )
		{
			servo_tilt_cnt_max = 200;			
		}
		else
		{
			servo_tilt_cnt_max = 400;
		}
	}
	
	//------
	//------ PAN Control
	//------
	
	if( servo_sts_pan[NOW] != 0x00 )
	{
		servo_pan_cnt++;
		if( servo_pan_cnt >= servo_pan_cnt_max )
		{
			servo_pan_cnt = 0;
			
			if( servo_sts_pan[NOW] == 0x02 )
			{
				servo_pwm_pan += SERVO_PAN_GAIN;
				if( servo_pwm_pan > SERVO_PAN_MAX )
				{
					servo_pwm_pan = SERVO_PAN_MAX;
				}
			}
			else if( servo_sts_pan[NOW] == 0x01 )
			{
				servo_pwm_pan -= SERVO_PAN_GAIN;
				if( servo_pwm_pan < SERVO_PAN_MIN )
				{
					servo_pwm_pan = SERVO_PAN_MIN;
				}			
			}
		}
	}
	
	//------
	//------ TILT Control
	//------
	
	if( servo_sts_tilt[NOW] != 0x00 )
	{
		servo_tilt_cnt++;
		if( servo_tilt_cnt >= servo_tilt_cnt_max )
		{
			servo_tilt_cnt = 0;
			
			if( servo_sts_tilt[NOW] == 0x02 )
			{
				servo_pwm_tilt += SERVO_TILT_GAIN;
				if( servo_pwm_tilt > SERVO_TILT_MAX )
				{
					servo_pwm_tilt = SERVO_TILT_MAX;
				}
			}
			else if( servo_sts_tilt[NOW] == 0x01 )
			{
				servo_pwm_tilt -= SERVO_TILT_GAIN;
				if( servo_pwm_tilt < SERVO_TILT_MIN )
				{
					servo_pwm_tilt = SERVO_TILT_MIN;
				}			
			}		
		}
	}

	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,servo_pwm_pan);		
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,servo_pwm_tilt);		


	
	//------
	//------ SysTick Timer
	//------
	systick_cnt++;
	if( systick_cnt >= 1000 )
	{
		systick_cnt = 0;

		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);		
	}
	
	if( mode == 0x01 )
	{
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
	}
	else
	{
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
	}
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
