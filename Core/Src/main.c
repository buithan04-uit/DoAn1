/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stdio.h"
#include "ili9341.h"
#include "math.h"
#include "string.h"
#include "front_end.h"
#include "touch.h"
#include "cJSON.h"
#include "stdlib.h"
#include "stdbool.h"
#include "DHT.h"
#include "MAX30100_PulseOximeter.h"
#include "AD8232.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define min(a,b) (((a)<(b))?(a):(b))

#define UART_RX_BUFFER_SIZE 1024
#define UART_BUFFER_SIZE 2048
uint8_t uart_rx_buffer[UART_BUFFER_SIZE];
#define ECG_X_MAX 320  // chi�?u rộng màn hình LCD
#define ECG_Y_MAX 240  // chi�?u cao màn hình LCD
#define ECG_BASELINE 120  // đư�?ng trung tâm (ngang) để vẽ ECG
#define ECG_SCALE 0.05f    // hệ số scale giá trị ADC xuống pixel (tùy chỉnh nếu thấy lệch)

uint16_t ecg_x = 0;
uint16_t old_y = ECG_BASELINE;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void UART_ReceiveString(UART_HandleTypeDef *huart, char *buffer, int buffer_size);
void Send_AT_Commands(UART_HandleTypeDef *huart);
void Send_AT_Command(UART_HandleTypeDef *huart, const char *command, uint32_t timeout );

//
void RunProgram();
void Max30100 ();
void Max30100_Init();
void AD8232_ReadData();
void ECG_DrawWave(uint16_t value);
void onBeatDetected(void) {
    // Xử lý khi phát hiện nhịp tim (bật LED, gửi UART, ...)

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int count = 0;
float temperature = 1;
float humidity = 1;
volatile bool readDHT = false;
volatile bool Recall = false;
volatile bool Reload = false;
int *temp_max = {0};
int *temp_min = {0};
double *UV = {0};
int *wind_speed = {0};
int *day_code = {0};
char day_name[7][4];
char Date[7][6];
char current_time[6];
char current_date[11];
int current_temp;
int current_humi;
int current_code;
int current_cloud;
int isDay;
int AD8232_value = 0;
int choice = 1;
int choiceTmp = 1;
PulseOximeter pox;
uint32_t tsLastReport = 0;
//
int current = 1;
bool updated = true;
int16_t tx , ty;
char MAX30100_data[1024];
uint8_t heartRate;
uint8_t spo2;

int i = 0;

// AT Commands
uint8_t ATCommand1[UART_BUFFER_SIZE] = "GET /v1/forecast?latitude=10.7769&longitude=106.7009&current=temperature_2m,relative_humidity_2m,is_day,weather_code,cloud_cover&daily=weather_code,temperature_2m_max,temperature_2m_min,uv_index_max,wind_speed_10m_max&timezone=Asia%2FBangkok HTTP/1.1\r\nHost: api.open-meteo.com\r\nConnection: close\r\n\r\n";

uint8_t ATCommand2[UART_BUFFER_SIZE] = "GET /v1/forecast?latitude=21.0285&longitude=105.8542&current=temperature_2m,relative_humidity_2m,is_day,weather_code,cloud_cover&daily=weather_code,temperature_2m_max,temperature_2m_min,uv_index_max,wind_speed_10m_max&timezone=Asia%2FBangkok HTTP/1.1\r\nHost: api.open-meteo.com\r\nConnection: close\r\n\r\n";

uint8_t ATCommand3[UART_BUFFER_SIZE] = "GET /v1/forecast?latitude=20.8449&longitude=106.6881&current=temperature_2m,relative_humidity_2m,is_day,weather_code,cloud_cover&daily=weather_code,temperature_2m_max,temperature_2m_min,uv_index_max,wind_speed_10m_max&timezone=Asia%2FBangkok HTTP/1.1\r\nHost: api.open-meteo.com\r\nConnection: close\r\n\r\n";

uint8_t ATCommand4[UART_BUFFER_SIZE] = "GET /v1/forecast?latitude=10.0452&longitude=105.7469&current=temperature_2m,relative_humidity_2m,is_day,weather_code,cloud_cover&daily=weather_code,temperature_2m_max,temperature_2m_min,uv_index_max,wind_speed_10m_max&timezone=Asia%2FBangkok HTTP/1.1\r\nHost: api.open-meteo.com\r\nConnection: close\r\n\r\n";

uint8_t ATCommand5[UART_BUFFER_SIZE] = "GET /v1/forecast?latitude=16.0471&longitude=108.2068&current=temperature_2m,relative_humidity_2m,is_day,weather_code,cloud_cover&daily=weather_code,temperature_2m_max,temperature_2m_min,uv_index_max,wind_speed_10m_max&timezone=Asia%2FBangkok HTTP/1.1\r\nHost: api.open-meteo.com\r\nConnection: close\r\n\r\n";

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
  MX_FSMC_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  //-----------------------------
  // LCD init
  LCD_BL_ON();
  HAL_TIM_Base_Start_IT(&htim2);
  TouchCalibrate();

  lcdInit();
  int i = 2;

  lcdSetOrientation(i%4);
  lcdFillRGB(COLOR_BLACK);
  lcdSetTextColor(COLOR_YELLOW,COLOR_BLACK);
  lcdSetTextFont(&Font16);
  // Max30100 init
  Max30100_Init();

  // Connect wifi
//  Send_AT_Commands(&huart1);
//  processWeather(uart_rx_buffer);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //RunProgram();
	  Max30100();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8499;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_BL_Pin|TOUCH_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TC_PEN_Pin */
  GPIO_InitStruct.Pin = TC_PEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TC_PEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_BL_Pin TOUCH_CS_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin|TOUCH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 1;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 5;
  Timing.BusTurnAroundDuration = 0;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */
// Hàm callback khi ngắt xảy ra
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
    	count++;
    	if (count % 5 == 0){
        	readDHT = true; // �?ặt c�? để báo hiệu cần đ�?c dữ liệu
		}
    	if (count == 300){
    		//call API
    		Reload = true;
    		count = 0;
    	}
    }
}
void Send_AT_Command(UART_HandleTypeDef *huart, const char *command, uint32_t timeout) {
    // Gửi lệnh qua UART

    HAL_UART_Transmit(huart, (uint8_t *)command, strlen(command), HAL_MAX_DELAY);

//    memset(uart_rx_buffer, 0, UART_BUFFER_SIZE); // Xóa buffer
//    HAL_UART_Receive(huart, uart_rx_buffer, UART_BUFFER_SIZE, timeout);

    lcdSetCursor(0,220);
    lcdPrintf("ESP Send: %s           ", command);
}



void Send_AT_Commands(UART_HandleTypeDef *huart) {
	sprintf(MAX30100_data, "HR=%d;SPO2=%d;AD8232=%d\n", heartRate,spo2,AD8232_value);
	Send_AT_Command(huart, MAX30100_data , 10000);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) { // Kiểm tra UART đúng
        // Xử lý dữ liệu nhận được ở đây
        HAL_UART_Receive_IT(huart, uart_rx_buffer, UART_BUFFER_SIZE);
    }
}
void Max30100_Init (){
	// Init
	PulseOximeter_Init(&pox);
	PulseOximeter_SetOnBeatDetectedCallback(&pox, onBeatDetected);
	if (!PulseOximeter_Begin(&pox, PULSEOXIMETER_DEBUGGINGMODE_NONE)) {
	  lcdSetCursor(0, 0);
	  lcdPrintf("Initializing!!");
	  while (1);
	} else {
	  lcdSetCursor(0, 0);
	  lcdPrintf("Init successfull");
	}
	// Check
	uint8_t spo2cfg = MAX30100_ReadRegister(MAX30100_REG_SPO2_CONFIGURATION);
	spo2cfg |= (1 << 6);
	MAX30100_WriteRegister(MAX30100_REG_SPO2_CONFIGURATION, spo2cfg);
	spo2cfg = MAX30100_ReadRegister(MAX30100_REG_SPO2_CONFIGURATION);
	sprintf(MAX30100_data, "SPO2CFG:0x%02X", spo2cfg);
	lcdSetCursor(0, 15);
	lcdPrintf(MAX30100_data);
	//
	MAX30100_WriteRegister(MAX30100_REG_INTERRUPT_STATUS, 0x00);
	uint8_t mode = MAX30100_ReadRegister(MAX30100_REG_MODE_CONFIGURATION);
	sprintf(MAX30100_data, "MODE:0x%02X", mode);
	lcdSetCursor(0, 30);
	lcdPrintf(MAX30100_data);
	//
	uint8_t status = MAX30100_ReadRegister(MAX30100_REG_INTERRUPT_STATUS);
	sprintf(MAX30100_data, "INT:0x%02X", status);
	lcdSetCursor(0, 45);
	lcdPrintf(MAX30100_data);
	//
	uint8_t part_id = MAX30100_GetPartId(&pox.hrm);
	sprintf(MAX30100_data, "PartID: 0x%02X", part_id);
	lcdSetCursor(0, 60);
	lcdPrintf(MAX30100_data);
	//
	HAL_Delay(3000);
	tsLastReport = HAL_GetTick();
	lcdFillRGB(COLOR_BLACK);
}
void Max30100 (){
    PulseOximeter_Update(&pox);
    heartRate = PulseOximeter_GetHeartRate(&pox);
    spo2 = PulseOximeter_GetSpO2(&pox);

    // Hiển thị LCD
    sprintf(MAX30100_data, "HR: %d bpm", heartRate);
    lcdSetCursor(0, 15);
    lcdPrintf(MAX30100_data);
    sprintf(MAX30100_data, "SpO2: %d", spo2);
    lcdSetCursor(0, 30);
    lcdPrintf(MAX30100_data);
    sprintf(MAX30100_data, "AD8232: %d", AD8232_value);
    lcdSetCursor(0, 45);
    lcdPrintf(MAX30100_data);

    // Send Data định kỳ
    if (HAL_GetTick() - tsLastReport > 5000) {
        Send_AT_Commands(&huart1);
        tsLastReport = HAL_GetTick();
    }

    HAL_Delay(10);
}
void AD8232_ReadData()
{
    // Đọc trạng thái dây điện cực (LO+ và LO-)
    GPIO_PinState lo_plus = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
    GPIO_PinState lo_minus = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);

    lcdSetTextColor(COLOR_YELLOW, COLOR_BLACK);
    lcdSetTextFont(&Font16);
    lcdSetCursor(5, 20);

    // Kiểm tra xem có bị rơi điện cực không
    if (lo_plus == GPIO_PIN_SET || lo_minus == GPIO_PIN_SET)
    {
    	lcdSetCursor(5, 60);
        lcdPrintf("ECG Error!!!");
    }
    else
    {
    	lcdSetCursor(5, 60);
        AD8232_value = AD8232_Read();
        lcdPrintf("AD8232 : %d       ", AD8232_value);
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
