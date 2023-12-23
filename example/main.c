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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> // removes warning: implicit declaration of a function
#include "st7789.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	float Tcyc;
	float D;
	float Vnorm;
	float Tacc;
	float Tnorm;
	float v1;
	float t1;
	float t2;
	float t3;
	float a1;
	float a2;
	float a3;
	float angle;
	float velocity;
	uint8_t running;
	uint8_t stopping;
	uint8_t start_rq;
	uint32_t start_rq_cycles;
	uint32_t encoder[10];
} MOT_Control_Params;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_TIMEOUT       100
#define ADC_VREF          3.3     // V
#define MOT_ACCEL         6.28    // rad/s^2
#define MOT_MAX_CYC_TIME  10.0    // s
#define MOT_MAX_DPT       1.0     // 0..1 (0..100%)
#define MOT_ENCODER_360   2.92    // V
#define MOT_CTRL_Kp       2.0     // proportional coefficient
#define PI                3.1415926

#define ADC_BITS      0x0FFF
#define ADC_0V_VALUE  0
#define ADC_1V_VALUE  1241
#define ADC_2V_VALUE  2482
#define ADC_3V_VALUE  3723

#define TIM5_MHZ  72
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2004c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2004c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2004c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2004c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

ETH_HandleTypeDef heth;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
uint32_t GLOBAL_VAR_32BIT = 0;
// ADCx conversion results
uint32_t ADC1_VALUE = 0;
uint32_t ADC2_VALUE = 0;
uint32_t ADC3_VALUE = 0;
// Control
MOT_Control_Params MOT;
// Time measurements
uint32_t LOOP_START_TIME = 0;
uint32_t LOOP_EXEC_TIME = 0;
uint32_t LOOP_CYCLE_TIME = 0;
uint32_t MAIN_START_TIME = 0;
uint32_t MAIN_EXEC_TIME = 0;
uint32_t MAIN_CYCLE_TIME = 0;

// Display diagnostic strings
char DPY_adc1[24];
char DPY_par0[24];
char DPY_par1[24];
char DPY_par2[24];
char DPY_par3[24];
char DPY_par4[24];
char DPY_par5[24];
char DPY_par6[24];
char DPY_par7[24];
char DPY_par8[24];
char DPY_par9[24];
uint32_t CycleCount = 0;
GPIO_PinState PS3 = GPIO_PIN_RESET;

uint16_t adcBuf[BUFFER_LEN];             // this is where we'll store waveform data from ADC
uint16_t dpyBuf[BUFFER_LEN];             // this is where we'll store waveform data for display
uint16_t dpyBuf_prev[BUFFER_LEN];             // this is where we'll store previous waveform data for display
volatile uint8_t finishedConversion = 0; // this lets us know when we're done capturing data
volatile uint32_t ADC_ConvCpltCallback_TimeStamp = 0;
volatile uint32_t ADC_ConvCycleTime = 0;
volatile uint32_t DPY_DisplayFBTime = 0;
volatile uint8_t dpyBufLocked = 0;

uint32_t adc_val;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM5_Init(void);
static void MX_ETH_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Display update
void DPY_ScreenUpdate()
{
	// Screen update

	ST7789_WriteString_Fast(0, 0, (char*) DPY_adc1, Font_11x18, YELLOW, BLUE);
	ST7789_WriteString_Fast(0, 15, (char*) DPY_par0, Font_11x18, YELLOW, BLUE);
	ST7789_WriteString_Fast(0, 30, (char*) DPY_par1, Font_11x18, YELLOW, BLUE);
	ST7789_WriteString_Fast(0, 45, (char*) DPY_par2, Font_11x18, YELLOW, BLUE);
	ST7789_WriteString_Fast(0, 60, (char*) DPY_par3, Font_11x18, YELLOW, BLUE);
	ST7789_WriteString_Fast(0, 75, (char*) DPY_par4, Font_11x18, YELLOW, BLUE);
	ST7789_WriteString_Fast(0, 90, (char*) DPY_par5, Font_11x18, YELLOW, BLUE);
	ST7789_WriteString_Fast(0, 105, (char*) DPY_par6, Font_11x18, YELLOW, BLUE);
	ST7789_WriteString_Fast(0, 120, (char*) DPY_par7, Font_11x18, YELLOW, BLUE);
	ST7789_WriteString_Fast(0, 135, (char*) DPY_par8, Font_11x18, YELLOW, BLUE);
	ST7789_WriteString_Fast(0, 150, (char*) DPY_par9, Font_11x18, YELLOW, BLUE);

}

void DrawScope(uint16_t *buff, uint16_t length, uint16_t color)
{
	uint16_t i, x0, y0, x1, y1;

	x0 = 0;
	y0 = ST7789_HEIGHT * buff[0] / ADC_BITS;

	for (i = 0; i < length; i++)
	{
		if (i >= ST7789_WIDTH)
			break;
		x1 = i;
		y1 = ST7789_HEIGHT * buff[i] / ADC_BITS;
		ST7789_DrawLine_BUF(x0, y0, x1, y1, color);
		x0 = x1;
		y0 = y1;

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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_DAC_Init();
  MX_TIM5_Init();
  MX_ETH_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay(1000);
	{
		uint8_t tx[] = "-----------------------------------------\r\n";
		HAL_UART_Transmit(&huart3, tx, sizeof(tx), 100);
	}

	ST7789_Init();
	HAL_GPIO_WritePin(DPY_BLK_GPIO_Port, DPY_BLK_Pin, GPIO_PIN_SET);
/*
	// Start timer TIM3 for framebuffer display
	if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
		Error_Handler();
*/
	// Start timer TIM5 for microseconds counting
	if (HAL_TIM_Base_Start_IT(&htim5) != HAL_OK)
		Error_Handler();

	// start timer TIM4 for pwm generation
	if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK)
		Error_Handler();

	memset(adcBuf, 0, sizeof(adcBuf));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	ST7789_Fill_Color_BUF(DARKBLUE);

	{
		uint8_t tx[] = "HAL_ADC_Start_DMA() started...\r\n";
		HAL_UART_Transmit(&huart3, tx, sizeof(tx), 100);
	}

	finishedConversion = 0;
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcBuf, BUFFER_LEN) != HAL_OK)
		Error_Handler();

	while (!finishedConversion)	{};

	{
		uint8_t tx[] = "HAL_ADC_Start_DMA() conversion done.\r\n";
		HAL_UART_Transmit(&huart3, tx, sizeof(tx), 100);
	}

	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		uint32_t timer_start = htim5.Instance->CNT;
		MAIN_CYCLE_TIME = htim5.Instance->CNT - MAIN_START_TIME;
		MAIN_START_TIME = timer_start;

		uint32_t timer_s1 = htim5.Instance->CNT;
		dpyBufLocked = 1;
		DrawScope(dpyBuf_prev, BUFFER_LEN, BLACK);
		DrawScope(dpyBuf, BUFFER_LEN, WHITE);
		memcpy(dpyBuf_prev, dpyBuf, sizeof(dpyBuf_prev));
		dpyBufLocked = 0;
		uint32_t timer_s2 = htim5.Instance->CNT;

		// Profiling
		MAIN_EXEC_TIME = htim5.Instance->CNT - MAIN_START_TIME;

		sprintf(DPY_adc1, "ADC Cycle = %d us", (int) ADC_ConvCycleTime);
		ST7789_WriteString_BUF(0, 0, DPY_adc1, Font_11x18, YELLOW, BLUE);

		sprintf(DPY_par0, "DPY FB Time = %d us", (int) DPY_DisplayFBTime);
		ST7789_WriteString_BUF(0, 15, DPY_par0, Font_11x18, YELLOW, BLUE);

		sprintf(DPY_par0, "Draw Time = %d us", (int) ((timer_s2 - timer_s1) / TIM5_MHZ));
		ST7789_WriteString_BUF(0, 30, DPY_par0, Font_11x18, YELLOW, BLUE);

		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

		uint32_t start = htim5.Instance->CNT;
		ST7789_Display_BUF();
		uint32_t finish = htim5.Instance->CNT;
		DPY_DisplayFBTime = (finish - start) / TIM5_MHZ;

		HAL_Delay(250);
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T4_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim3.Init.Prescaler = 10000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 7200;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 10000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7200;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DPY_RES_GPIO_Port, DPY_RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DPY_DC_Pin|DPY_BLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DPY_RES_Pin */
  GPIO_InitStruct.Pin = DPY_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DPY_RES_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DPY_DC_Pin DPY_BLK_Pin */
  GPIO_InitStruct.Pin = DPY_DC_Pin|DPY_BLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Callback from roll over timers
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3)
	{
		{
			uint32_t start = htim5.Instance->CNT;
			//ST7789_Display_BUF();
			uint32_t finish = htim5.Instance->CNT;
			DPY_DisplayFBTime = (finish - start) / TIM5_MHZ;
		}

		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	}

	if (htim == &htim4)
	{
		{
			uint8_t tx[] = "HAL_TIM_PeriodElapsedCallback() - TIM4\r\n";
			HAL_UART_Transmit(&huart3, tx, sizeof(tx), 100);
		}

		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
	if (htim == &htim5)
	{
		{
			uint8_t tx[] = "HAL_TIM_PeriodElapsedCallback() - TIM5\r\n";
			HAL_UART_Transmit(&huart3, tx, sizeof(tx), 100);
		}

		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
}
/**
 * @brief  Conversion complete callback in non blocking mode
 * @param  AdcHandle : AdcHandle handle
 * @note   This example shows a simple way to report end of conversion
 *         and get conversion result. You can add your own implementation.
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	uint32_t new_TimeStamp = htim5.Instance->CNT;
	uint32_t callbackCycleTime = new_TimeStamp - ADC_ConvCpltCallback_TimeStamp;
	ADC_ConvCpltCallback_TimeStamp = new_TimeStamp;
	finishedConversion = 1;
	ADC_ConvCycleTime = callbackCycleTime / TIM5_MHZ;
	if (!dpyBufLocked)
		memcpy(&dpyBuf, &adcBuf, sizeof(dpyBuf));

	if (hadc == &hadc1)
	{
	}

}

/**
 * @brief  Conversion DMA half-transfer callback in non blocking mode
 * @param  hadc: ADC handle
 * @retval None
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	{
		//uint8_t tx[] = "HAL_ADC_ConvHalfCpltCallback()\r\n";
		//HAL_UART_Transmit(&huart3, tx, sizeof(tx), 100);
	}
}

/**
 * @brief  Error ADC callback.
 * @note   In case of error due to overrun when using ADC with DMA transfer
 *         (HAL ADC handle parameter "ErrorCode" to state "HAL_ADC_ERROR_OVR"):
 *         - Reinitialize the DMA using function "HAL_ADC_Stop_DMA()".
 *         - If needed, restart a new ADC conversion using function
 *           "HAL_ADC_Start_DMA()"
 *           (this function is also clearing overrun flag)
 * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
 *         the configuration information for the specified ADC.
 * @retval None
 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
	{
		uint8_t tx[] = "HAL_ADC_ErrorCallback()\r\n";
		HAL_UART_Transmit(&huart3, tx, sizeof(tx), 100);
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	{
		GLOBAL_VAR_32BIT++;
		uint8_t tx[] = "HAL_SPI_TxCpltCallback()";
		HAL_UART_Transmit(&huart3, tx, sizeof(tx), 100);
		uint8_t txe[30];
		sprintf(txe, " count: %d\r\n",GLOBAL_VAR_32BIT);
		HAL_UART_Transmit(&huart3, txe, strlen(txe), 100);
	}
}

void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
	{
		uint8_t tx[] = "HAL_SPI_TxHalfCpltCallback()\r\n";
		HAL_UART_Transmit(&huart3, tx, sizeof(tx), 100);
	}
}

void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
	{
		uint8_t tx[] = "HAL_SPI_TxRxHalfCpltCallback()\r\n";
		HAL_UART_Transmit(&huart3, tx, sizeof(tx), 100);
	}
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	{
		const char *ers[] =
		{ "No error", "MODF error", "CRC error", "OVR error", "FRE error", "DMA transfer error", "Error on RXNE/TXE/BSY/FTLVL/FRLVL Flag",
				"Error during SPI Abort procedure" };

		uint32_t error_code = HAL_SPI_GetError(hspi);

		uint8_t eri = 0;

		char erm[100] = "SPI Error:";
		if (error_code & HAL_SPI_ERROR_NONE)
			sprintf(erm, "%s %s," ,erm, ers[0]);
		if (error_code & HAL_SPI_ERROR_MODF)
			sprintf(erm, "%s %s," ,erm, ers[1]);
		if (error_code & HAL_SPI_ERROR_CRC)
			sprintf(erm, "%s %s," ,erm, ers[2]);
		if (error_code & HAL_SPI_ERROR_OVR)
			sprintf(erm, "%s %s," ,erm, ers[3]);
		if (error_code & HAL_SPI_ERROR_FRE)
			sprintf(erm, "%s %s," ,erm, ers[4]);
		if (error_code & HAL_SPI_ERROR_DMA)
			sprintf(erm, "%s %s," ,erm, ers[5]);
		if (error_code & HAL_SPI_ERROR_FLAG)
			sprintf(erm, "%s %s," ,erm, ers[6]);
		if (error_code & HAL_SPI_ERROR_ABORT)
			sprintf(erm, "%s %s," ,erm, ers[7]);

		uint8_t tx[] = "HAL_SPI_ErrorCallback()\r\n";
		HAL_UART_Transmit(&huart3, tx, sizeof(tx), 100);

		uint8_t txe[100];
		sprintf(txe, "SPI Error 0x%X(%d): %s\r\nSR: 0x%X DMA ErrCode: 0x%X\r\n", error_code, error_code, erm, hspi->Instance->SR, hspi->hdmatx->ErrorCode);

		error_code = HAL_SPI_ERROR_DMA;
		HAL_UART_Transmit(&huart3, txe, strlen(txe), 100);
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
	uint8_t tx[] = "(!!) DEBUG: Error_Handler()\r\n";
	HAL_UART_Transmit(&huart3, tx, sizeof(tx), 100);
	ST7789_Fill_Color(RED);
	ST7789_WriteString(50, 50, "! ERROR !", Font_11x18, YELLOW, RED);
	GPIO_PinState PS_Error = GPIO_PIN_RESET;
	while (1)
	{
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, PS_Error);
		PS_Error = !PS_Error;
		HAL_Delay(250);
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
