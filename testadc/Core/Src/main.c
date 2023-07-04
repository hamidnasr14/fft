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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "arm_const_structs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define SAMPLES                    512             /* 256 real party and 256 imaginary parts */
#define FFT_SIZE               SAMPLES / 2        /* FFT size is always the same size as we have samples, so 256 in our case */
#define FFT_INVERSE_FLAG        ((uint8_t)0)
#define FFT_Normal_OUTPUT_FLAG  ((uint8_t)1)


uint16_t adc_val[SAMPLES];
q15_t FFT_Input_f32[FFT_SIZE*2];
q15_t FFT_Output_f32[FFT_SIZE];
q15_t maxValue=0;
uint16_t tag=0;
//q15_t FFT_Input_f32[SAMPLES];
//q15_t FFT_Output_f32[SAMPLES*2];
//q15_t maxValue;

//float32_t FFT_Input_f32[FFT_SIZE*2];
//float32_t FFT_Output_f32[FFT_SIZE];
//float32_t FFT_OutputFilter_f32[FFT_SIZE*2];
//float32_t maxValue;
uint16_t global_pulse_flag = 0;
uint16_t pulseEnd=0;
uint32_t maxIndex=0;
uint16_t BufferFlag=0;
#define NUM_TAPS 115
#define BLOCK_SIZE 512
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS -1];
static float32_t firCeffs[NUM_TAPS]= {
    0.000000000000000000,
    -0.000001313523696031,
    -0.000006045600631796,
    -0.000005391705956365,
    0.000012876350688274,
    0.000042207631095607,
    0.000049160753288544,
    0.000000000000000000,
    -0.000093495662352919,
    -0.000156458139528984,
    -0.000098411892031813,
    0.000094663715790658,
    0.000303189468920621,
    0.000329439281059466,
    0.000059204395866005,
    -0.000387818287369625,
    -0.000672559646994957,
    -0.000467746171394902,
    0.000232182269721180,
    0.000985307337439024,
    0.001143134936861696,
    0.000362749111054153,
    -0.000985477046233643,
    -0.001915421171109029,
    -0.001498197428606539,
    0.000311706861481382,
    0.002378415892033811,
    0.003025276997866369,
    0.001318618646342356,
    -0.001938171907916035,
    -0.004428097444937380,
    -0.003883551726046047,
    -0.000000000000000015,
    0.004827376867569123,
    0.006844939664729055,
    0.003729019080618624,
    -0.003161902009773621,
    -0.009057440458735459,
    -0.008911370218357424,
    -0.001465763470095635,
    0.008872689019475683,
    0.014345690837972453,
    0.009380187935203779,
    -0.004413697608035452,
    -0.017902031757367123,
    -0.020021915997521410,
    -0.006181225792915156,
    0.016503429923908217,
    0.031891188832734084,
    0.025140771560349908,
    -0.005360616978954799,
    -0.042823512659950928,
    -0.058686112468638876,
    -0.028715514040432644,
    0.050545074398097495,
    0.155557392997279859,
    0.244954572543420429,
    0.279997587009433113,
    0.244954572543420485,
    0.155557392997279859,
    0.050545074398097495,
    -0.028715514040432644,
    -0.058686112468638855,
    -0.042823512659950935,
    -0.005360616978954799,
    0.025140771560349912,
    0.031891188832734091,
    0.016503429923908217,
    -0.006181225792915157,
    -0.020021915997521410,
    -0.017902031757367130,
    -0.004413697608035453,
    0.009380187935203779,
    0.014345690837972458,
    0.008872689019475688,
    -0.001465763470095636,
    -0.008911370218357428,
    -0.009057440458735459,
    -0.003161902009773623,
    0.003729019080618626,
    0.006844939664729060,
    0.004827376867569123,
    -0.000000000000000015,
    -0.003883551726046047,
    -0.004428097444937378,
    -0.001938171907916035,
    0.001318618646342357,
    0.003025276997866374,
    0.002378415892033812,
    0.000311706861481382,
    -0.001498197428606540,
    -0.001915421171109029,
    -0.000985477046233643,
    0.000362749111054154,
    0.001143134936861694,
    0.000985307337439024,
    0.000232182269721180,
    -0.000467746171394903,
    -0.000672559646994958,
    -0.000387818287369625,
    0.000059204395866005,
    0.000329439281059466,
    0.000303189468920622,
    0.000094663715790658,
    -0.000098411892031812,
    -0.000156458139528984,
    -0.000093495662352919,
    0.000000000000000000,
    0.000049160753288544,
    0.000042207631095608,
    0.000012876350688274,
    -0.000005391705956365,
    -0.000006045600631796,
    -0.000001313523696031,
    0.000000000000000000,};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
/* Maps buffer values 113..742, inclusive, to -8192..24575, with correct rounding. */
//void prepare_adc(uint32_t buffer[], uint32_t count)
//{
//		uint32_t xmin=0, xmax=4095, ymin=0, ymax=32767, r=0;
//		for (uint32_t  i = 0; i < count; i++){
//			if (buffer[i] <= xmin)
//					buffer[i] = ymin;
//			else
//			if (buffer[i] < xmax)
//					buffer[i] = ((int32_t)(buffer[i] - xmin)*32767 + r) / 629 + ymin;
//			else
//					buffer[i] = ymax;
//			}
//}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    if(GPIO_Pin == KEY1_Pin) 
    {
			while(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET){};
				
			HAL_TIM_Base_Stop_IT(&htim1);
			HAL_GPIO_TogglePin(RGB_LED_GPIO_Port, RGB_LED_Pin);
//			HAL_TIM_OnePulse_Start(&htim2, TIM_CHANNEL_3);
//			__HAL_TIM_ENABLE(&htim2);
			HAL_TIM_Base_Start_IT(&htim1);
			
    }

    if(GPIO_Pin == ZCD_PULSE_Pin)
    {
			global_pulse_flag = 0;
//			HAL_GPIO_TogglePin(RGB_LED_GPIO_Port, RGB_LED_Pin);
			HAL_TIM_Base_Start_IT(&htim4); //TX PULSE 
			HAL_TIM_Base_Stop(&htim3); //Stop adc trigger
			
    }			
		
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	//arm_cfft_radix4_instance_f32 FFT_struct;	
	// Fill FFT input buffer with the required data from ADCConvertedValue

	for(uint16_t index_input_buffer = 0; index_input_buffer < FFT_SIZE * 2; index_input_buffer+=2)
	{
		 FFT_Input_f32[(uint16_t)index_input_buffer] = (q15_t)adc_val[index_input_buffer] << 3;
		// Imaginary Part 
		FFT_Input_f32[(uint16_t)(index_input_buffer + 1)] = 0;	
	}

	BufferFlag=1;

	
}
uint16_t tag_processing(q15_t a[]){
	
	if (3800< a[59]){
		HAL_GPIO_WritePin(RGB_LED_GPIO_Port, RGB_LED_Pin, 1);
		tag=1;
		HAL_Delay(10);
	}
	else{
		HAL_GPIO_WritePin(RGB_LED_GPIO_Port, RGB_LED_Pin, 0);
		tag=0;
	}
	
return tag;
}


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if(htim -> Instance == TIM1)
    {
        if(htim-> Channel == HAL_TIM_ACTIVE_CHANNEL_2)
        {
            global_pulse_flag++;
            if(global_pulse_flag>=87){ //1ms after pulse started (87*1/58000=1.5ms)
							global_pulse_flag = 0;
							HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_2); 
							HAL_TIMEx_PWMN_Stop_IT(&htim1, TIM_CHANNEL_2);
							HAL_TIM_Base_Start(&htim3); //Start adc trigger
						}							
        }
    }
	
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{

	if(htim-> Instance == TIM4){
		HAL_TIM_Base_Stop_IT(&htim4);
		HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2); 
		HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_2);
	}
	
//	if(htim-> Instance == TIM2){
//		pulseEnd=1;
//	}
	
}

uint16_t find_maximum(q15_t a[], uint16_t n) {
  q15_t c, index = 0;

  for (c = 50; c < n; c++)
    if (a[c] > a[index])
      index = c;

  return index;
}

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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_ADCEx_Calibration_Start(&hadc1);
	
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start(&htim3);
	global_pulse_flag=0;
	uint32_t CH1_DC=1032/2;
	TIM1->CCR2 = CH1_DC;
	TIM4->ARR=12-1;
	
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val, SAMPLES);
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_GPIO_WritePin(RGB_LED_GPIO_Port, RGB_LED_Pin, 1);
	HAL_Delay(800);
	HAL_GPIO_WritePin(RGB_LED_GPIO_Port, RGB_LED_Pin, 0);
	
//	arm_fir_instance_f32 F;
//	arm_cfft_radix4_instance_f32 S;
//	arm_rfft_fast_instance_f32 S;
//	arm_rfft_instance_q15 S;
//	arm_cfft_instance_q15 S;
	arm_cfft_radix4_instance_q15 S;
//	arm_cfft_radix2_instance_q15 S;
	arm_status status;
	
//	arm_cfft_radix4_init_f32(&S, FFT_SIZE, 0, 1);
//	arm_rfft_fast_init_f32(&S, FFT_SIZE);
//	arm_rfft_init_q15(&S, FFT_SIZE, 0, 1);
//	arm_cfft_init_q15(&S, FFT_SIZE);
	arm_cfft_radix4_init_q15(&S, FFT_SIZE, 0, 1);
//	arm_cfft_radix2_init_q15(&S, FFT_SIZE, 0, 1);
	
  while (1)
  {
		
//		if(maxIndex == 30){
//			
//		HAL_GPIO_WritePin(RGB_LED_GPIO_Port, RGB_LED_Pin, 0);
			HAL_GPIO_TogglePin(RGB_LED_GPIO_Port, RGB_LED_Pin);
//			HAL_Delay(500);
		if (BufferFlag == 1){
		HAL_TIM_Base_Stop(&htim3);
		
//		arm_fir_init_f32(&F, NUM_TAPS, firCeffs, firStateF32, BLOCK_SIZE);
//		arm_fir_f32(&F, FFT_Input_f32, FFT_OutputFilter_f32, BLOCK_SIZE);
			
		/* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
		
		/* Process the data through the CFFT/CIFFT module */
//		arm_cfft_radix4_f32(&S, FFT_Input_f32);
//		arm_rfft_fast_f32(&S, FFT_Input_f32,FFT_Output_f32,0);
//		arm_rfft_q15(&S, FFT_Input_f32, FFT_Output_f32);
		arm_cfft_radix4_q15(&S, FFT_Input_f32);
		
//		arm_cfft_radix2_q15(&S, FFT_Input_f32);
		/* Process the data through the Complex Magniture Module for calculating the magnitude at each bin */
//		arm_cmplx_mag_f32(FFT_Input_f32, FFT_Output_f32, FFT_SIZE);
		arm_cmplx_mag_q15(FFT_Input_f32, FFT_Output_f32, FFT_SIZE);

		FFT_Output_f32[0] = 0;
			
		/* Calculates maxValue and returns corresponding value */
////		arm_max_f32(FFT_Output_f32, FFT_SIZE, &maxValue, &maxIndex);
		arm_max_q15(FFT_Output_f32, FFT_SIZE, &maxValue, &maxIndex);

//		maxIndex = find_maximum(FFT_Output_f32, 70);
//		maxValue = FFT_Output_f32[maxIndex];
//		tag_processing(FFT_Output_f32);

		BufferFlag = 0;
		HAL_TIM_Base_Start(&htim3); //Stop adc Trig
		}
		
		
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL7;
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
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 965-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 50;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 70-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 250;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 56;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RGB_LED_GPIO_Port, RGB_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : KEY1_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RGB_LED_Pin */
  GPIO_InitStruct.Pin = RGB_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RGB_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ZCD_PULSE_Pin */
  GPIO_InitStruct.Pin = ZCD_PULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ZCD_PULSE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
