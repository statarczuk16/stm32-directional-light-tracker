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
#include "st7789.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "string.h"
#include "task.h"
#define SENSOR_PIPE_SIZE 10
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CENTER_BOX_SIZE 60U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint32_t pot = 0;
volatile uint32_t pwm = 0;
uint16_t sensor_e;
uint16_t sensor_s;
uint16_t sensor_w;
uint16_t sensor_n;
static uint16_t sensor_n_highest = 0;
static uint16_t sensor_e_highest = 0;
static uint16_t sensor_s_highest = 0;
static uint16_t sensor_w_highest = 0;

static bool valid_reading = false;
static float aoa_degree_final = 0.0f;
uint32_t readValue[4];

static uint8_t SENSOR_PIPE_INDEX = 0;
static uint32_t sensor_pipe_n[SENSOR_PIPE_SIZE];
static uint32_t sensor_pipe_e[SENSOR_PIPE_SIZE];
static uint32_t sensor_pipe_s[SENSOR_PIPE_SIZE];
static uint32_t sensor_pipe_w[SENSOR_PIPE_SIZE];

char buffer[50];
static uint16_t north_reading_x;
static uint16_t north_reading_y;
static uint16_t east_reading_x;
static uint16_t east_reading_y;
static uint16_t west_reading_x;
static uint16_t west_reading_y;
static uint16_t south_reading_x;
static uint16_t south_reading_y;
static uint16_t heading_x;
static uint16_t heading_y;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Magnitudes are ordered clockwise: north, east, south, west.
 * Select the strongest sensor and its strongest perpendicular neighbor.
 * Equal magnitudes prefer the first primary and the clockwise neighbor.
 * Returns false, with a zero heading, when all magnitudes are zero.
 */
static bool calculate_heading(float north, float east, float south, float west,
                              float *heading_degrees)
{
    const float magnitudes[4] = {north, east, south, west};
    unsigned primary = 0U;

    for (unsigned i = 1U; i < 4U; i++)
    {
        if (magnitudes[i] > magnitudes[primary])
        {
            primary = i;
        }
    }

    *heading_degrees = 0.0f;
    if (magnitudes[primary] <= 0.0f)
    {
        return false;
    }

    const unsigned clockwise = (primary + 1U) % 4U;
    const unsigned counterclockwise = (primary + 3U) % 4U;
    const bool use_clockwise =
        magnitudes[clockwise] >= magnitudes[counterclockwise];
    const float secondary = use_clockwise ? magnitudes[clockwise] :
                                            magnitudes[counterclockwise];
    const float offset = atan2f(secondary, magnitudes[primary]) *
                         (180.0f / 3.14159265358979323846f);
    float heading = (float)primary * 90.0f +
                    (use_clockwise ? offset : -offset);

    if (heading < 0.0f)
    {
        heading += 360.0f;
    }
    else if (heading >= 360.0f)
    {
        heading -= 360.0f;
    }

    *heading_degrees = heading;
    return true;
}

static void task_light_sensors(void)
{
	sensor_e = (uint16_t) readValue[0];
	sensor_s = (uint16_t) readValue[1];
	sensor_w = (uint16_t) readValue[2];
	sensor_n = (uint16_t) readValue[3];

	sensor_pipe_n[SENSOR_PIPE_INDEX] = sensor_n;
	sensor_pipe_e[SENSOR_PIPE_INDEX] = sensor_e;
	sensor_pipe_s[SENSOR_PIPE_INDEX] = sensor_s;
	sensor_pipe_w[SENSOR_PIPE_INDEX] = sensor_w;
	SENSOR_PIPE_INDEX += 1;
	if(SENSOR_PIPE_INDEX == SENSOR_PIPE_SIZE)
	{
		SENSOR_PIPE_INDEX = 0;
	}
}

static void task_display(void)
{
	uint32_t n_sum = 0;
	uint32_t e_sum = 0;
	uint32_t s_sum = 0;
	uint32_t w_sum = 0;
	for(int i = 0; i < SENSOR_PIPE_SIZE; i++)
	{
		n_sum += sensor_pipe_n[i];
		e_sum += sensor_pipe_e[i];
		s_sum += sensor_pipe_s[i];
		w_sum += sensor_pipe_w[i];
	}
	uint16_t n_avg = n_sum / SENSOR_PIPE_SIZE;
	uint16_t e_avg = e_sum / SENSOR_PIPE_SIZE;
	uint16_t s_avg = s_sum / SENSOR_PIPE_SIZE;
	uint16_t w_avg = w_sum / SENSOR_PIPE_SIZE;

	/* Normalize each averaged reading to its own highest observed value. */
	if (n_avg > sensor_n_highest) sensor_n_highest = n_avg;
	if (e_avg > sensor_e_highest) sensor_e_highest = e_avg;
	if (s_avg > sensor_s_highest) sensor_s_highest = s_avg;
	if (w_avg > sensor_w_highest) sensor_w_highest = w_avg;

	const unsigned n_percent = sensor_n_highest == 0U ? 0U :
		((uint32_t)n_avg * 100U + sensor_n_highest / 2U) / sensor_n_highest;
	const unsigned e_percent = sensor_e_highest == 0U ? 0U :
		((uint32_t)e_avg * 100U + sensor_e_highest / 2U) / sensor_e_highest;
	const unsigned s_percent = sensor_s_highest == 0U ? 0U :
		((uint32_t)s_avg * 100U + sensor_s_highest / 2U) / sensor_s_highest;
	const unsigned w_percent = sensor_w_highest == 0U ? 0U :
		((uint32_t)w_avg * 100U + sensor_w_highest / 2U) / sensor_w_highest;

	valid_reading = calculate_heading((float)n_percent, (float)e_percent,
	                                  (float)s_percent, (float)w_percent,
	                                  &aoa_degree_final);



	//snprintf(buffer, sizeof(buffer), "e: %d,s %d, w: %d, n: %d", sensor_e, sensor_s, sensor_w, sensor_n);
	buffer[0] = '\0';
	snprintf(buffer, sizeof(buffer), "N:%3u%%", n_percent);
	ST7789_WriteString(north_reading_x, north_reading_y, buffer, Font_11x18, WHITE, BLACK);
	buffer[0] = '\0';
	snprintf(buffer, sizeof(buffer), "E:%3u%%", e_percent);
	ST7789_WriteString(east_reading_x, east_reading_y, buffer, Font_11x18, WHITE, BLACK);
	buffer[0] = '\0';
	snprintf(buffer, sizeof(buffer), "S:%3u%%", s_percent);
	ST7789_WriteString(south_reading_x, south_reading_y, buffer, Font_11x18, WHITE, BLACK);
	buffer[0] = '\0';
	snprintf(buffer, sizeof(buffer), "W:%3u%%", w_percent);
	ST7789_WriteString(west_reading_x, west_reading_y, buffer, Font_11x18, WHITE, BLACK);

	/* Erase the previous pointer without clearing the whole square. */
	static bool heading_line_drawn = false;
	static uint16_t previous_line_x;
	static uint16_t previous_line_y;
	const uint16_t center_x = ST7789_WIDTH / 2U;
	const uint16_t center_y = ST7789_HEIGHT / 2U - Font_7x10.height / 2U - 4U;

	if (heading_line_drawn)
	{
		ST7789_DrawLine(center_x, center_y,
		                previous_line_x, previous_line_y, WHITE);
		heading_line_drawn = false;
	}

	/* Screen Y increases downward: north is -Y and east is +X. */
	if (valid_reading)
	{
		const float radians = aoa_degree_final *
		                      (3.14159265358979323846f / 180.0f);
		const float radius = (float)(CENTER_BOX_SIZE / 2U - Font_7x10.height - 2U);
		previous_line_x = (uint16_t)((int32_t)center_x +
		                            lroundf(sinf(radians) * radius));
		previous_line_y = (uint16_t)((int32_t)center_y -
		                            lroundf(cosf(radians) * radius));
		ST7789_DrawLine(center_x, center_y,
		                previous_line_x, previous_line_y, BLUE);
		heading_line_drawn = true;

		/* Fixed-width whole degrees; avoid requiring printf float support. */
		const unsigned degrees = ((unsigned)(aoa_degree_final + 0.5f)) % 360U;
		snprintf(buffer, sizeof(buffer), "%3u deg", degrees);
	}
	else
	{
		snprintf(buffer, sizeof(buffer), "--- deg");
	}
	/* The label sits below the pointer inside the white square. */
	ST7789_WriteString(heading_x, heading_y, buffer, Font_7x10, BLACK, WHITE);
	//ST7789_WriteString(20, 30, buffer, Font_11x18, WHITE, BLACK);
}

static struct task tasks[] =
{
		  {task_light_sensors, 25, 0},
		  {task_display, 600, 0}
};
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  static const char sample_text[] = "N:100% ";
  const uint16_t sample_len = strlen(sample_text) * Font_11x18.width;
  north_reading_x = (ST7789_WIDTH - sample_len) / 2U;
  north_reading_y = Font_11x18.height;

  east_reading_x = ST7789_WIDTH - sample_len;
  east_reading_y = ST7789_HEIGHT / 2;

  west_reading_x = Font_11x18.width;
  west_reading_y = east_reading_y;

  south_reading_x = north_reading_x;
  south_reading_y = ST7789_HEIGHT - Font_11x18.height;

  /* The smaller font fits between the east and west labels. */
  heading_x = (ST7789_WIDTH - (sizeof("000 deg") - 1U) * Font_7x10.width) / 2U;
  heading_y = (ST7789_HEIGHT + CENTER_BOX_SIZE) / 2U - Font_7x10.height - 4U;

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) readValue, 4);

  ST7789_Init();
  ST7789_Fill_Color(BLACK);
  const uint16_t center_box_x = (ST7789_WIDTH - CENTER_BOX_SIZE) / 2U;
  const uint16_t center_box_y = (ST7789_HEIGHT - CENTER_BOX_SIZE) / 2U;
  ST7789_Fill(center_box_x, center_box_y,
              center_box_x + CENTER_BOX_SIZE - 1U,
              center_box_y + CENTER_BOX_SIZE - 1U, WHITE);
  HAL_Delay(1000);




  //
  //ST7789_WriteString(20, 30, "another", Font_11x18, WHITE, BLACK);

  //ST7789_Test();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint32_t now = HAL_GetTick();
	  for(size_t i = 0; i < sizeof(tasks) / sizeof(tasks[0]); i++)
	  {
		  if(now - tasks[i].last_ms >= tasks[i].interval_ms)
		  {
			  tasks[i].last_ms = now;
			  tasks[i].run();
		  }
	  }



	  //HAL_UART_Transmit(&huart2, (uint8_t*)buffer, (uint16_t)strlen(buffer), HAL_MAX_DELAY);


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|ST7789_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ST7789_RST_Pin|ST7789_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin ST7789_RST_Pin ST7789_DC_Pin ST7789_CS_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|ST7789_RST_Pin|ST7789_DC_Pin|ST7789_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

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
