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
#include "max30100_for_stm32_hal.h"
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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
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
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void processWeather(char *jsonString);

void UART_ReceiveString(UART_HandleTypeDef *huart, char *buffer, int buffer_size);
void Send_AT_Commands(UART_HandleTypeDef *huart);
void Resend_AT_Commands(UART_HandleTypeDef *huart);
void Send_AT_Command(UART_HandleTypeDef *huart, const char *command, uint32_t timeout );
void Send_AT_Command1(UART_HandleTypeDef *huart, const char *command, uint32_t timeout );
//
void RunProgram();
void Max30100 ();

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

int choice = 1;
int choiceTmp = 1;
//
int current = 1;
bool updated = true;
int16_t tx , ty;

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
  /* USER CODE BEGIN 2 */
  //-----------------------------

  LCD_BL_ON();
  HAL_TIM_Base_Start_IT(&htim2);
  TouchCalibrate();

  lcdInit();
  int i = 2;

  lcdSetOrientation(i%4);
  lcdFillRGB(COLOR_BLACK);


//  if (HAL_I2C_IsDeviceReady(&hi2c1, 0x57, 3, 100) == HAL_OK) {
//	  lcdSetTextColor(COLOR_CYAN, COLOR_BLACK);
//	  lcdSetTextFont(&Font16);
//	  lcdSetCursor(5, 5);
//      lcdPrintf("MAX30100 OK\n");
//      HAL_Delay(3000);
//  } else {
//	  lcdSetTextColor(COLOR_CYAN, COLOR_BLACK);
//	  lcdSetTextFont(&Font16);
//	  lcdSetCursor(5, 5);
//      lcdPrintf("MAX30100 not found\n");
//      HAL_Delay(3000);
//  }

  MAX30100_Init(&hi2c1, &huart1);
  MAX30100_SetSpO2SampleRate(MAX30100_SPO2SR_DEFAULT);
  MAX30100_SetLEDPulseWidth(MAX30100_LEDPW_DEFAULT);
  MAX30100_SetLEDCurrent(MAX30100_LEDCURRENT_DEFAULT, MAX30100_LEDCURRENT_DEFAULT);
  MAX30100_SetMode(MAX30100_SPO2_MODE);




//  Send_AT_Commands(&huart1);
//  processWeather(uart_rx_buffer);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //RunProgram();
	  Max30100();

	  HAL_Delay(2000);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
    {
        // Xử lý khi có ngắt từ PB0 tại đây
		  lcdSetTextColor(COLOR_CYAN, COLOR_BLACK);
		  lcdSetTextFont(&Font16);
		  lcdSetCursor(5, 5);
		  lcdPrintf("MAX30100 is ready.\r\n");
		  HAL_Delay(3000);
    }
}


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

    // Ch�? phản hồi từ ESP
    memset(uart_rx_buffer, 0, UART_BUFFER_SIZE); // Xóa buffer
    HAL_UART_Receive(huart, uart_rx_buffer, UART_BUFFER_SIZE, timeout);

    // Hiển thị phản hồi lên màn hình
    lcdSetCursor(5,220);
    Screen0();
    lcdSetTextColor(COLOR_BLACK, COLOR_THEME_SKYBLUE_BASE);
    lcdSetTextFont(&Font16);
    lcdPrintf("ESP: %s\n", uart_rx_buffer);
}
void Send_AT_Command1(UART_HandleTypeDef *huart, const char *command, uint32_t timeout) {
    // Gửi lệnh qua UART

    HAL_UART_Transmit(huart, (uint8_t *)command, strlen(command), HAL_MAX_DELAY);

    // Ch�? phản hồi từ ESP
    memset(uart_rx_buffer, 0, UART_BUFFER_SIZE); // Xóa buffer
    HAL_UART_Receive(huart, uart_rx_buffer, UART_BUFFER_SIZE, timeout);
}


void Send_AT_Commands(UART_HandleTypeDef *huart) {
    // Gửi từng lệnh AT và xử lý phản hồi

    Send_AT_Command(huart, "AT\r\n", 3000 );
    Send_AT_Command(huart, "AT+CWMODE=3\r\n", 3000 );
    Send_AT_Command(huart, "AT+CWJAP=\"Redmi Turbo 3\",\"88888888\"\r\n", 9000 );
    Send_AT_Command(huart, "AT+CIPSTART=\"TCP\",\"api.open-meteo.com\",80\r\n", 3000 );
    Send_AT_Command(huart, "AT+CIPSEND=299\r\n", 3000);

    // Gửi yêu cầu GET cuối cùng
    if (choice == 1){
    	Send_AT_Command(huart, ATCommand1, 5000);
    }
    else if(choice == 2){
    	Send_AT_Command(huart, ATCommand2, 5000);
    }
    else if(choice == 3){
		Send_AT_Command(huart, ATCommand3, 5000);
	}
    else if(choice == 4){
		Send_AT_Command(huart, ATCommand4, 5000);
	}
    else if(choice == 5){
		Send_AT_Command(huart, ATCommand5, 5000);
	}

}
void Resend_AT_Commands(UART_HandleTypeDef *huart){
	Send_AT_Command1(huart, "AT+CIPSTART=\"TCP\",\"api.open-meteo.com\",80\r\n", 3000 );
	Send_AT_Command1(huart, "AT+CIPSEND=299\r\n", 3000);

	// Gửi yêu cầu GET cuối cùng
	if (choice == 1){
		Send_AT_Command1(huart, ATCommand1, 5000);
	}
	else if(choice == 2){
		Send_AT_Command1(huart, ATCommand2, 5000);
	}
	else if(choice == 3){
		Send_AT_Command1(huart, ATCommand3, 5000);
	}
	else if(choice == 4){
		Send_AT_Command1(huart, ATCommand4, 5000);
	}
	else if(choice == 5){
		Send_AT_Command1(huart, ATCommand5, 5000);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) { // Kiểm tra UART đúng
        // Xử lý dữ liệu nhận được ở đây

        // Tiếp tục nhận dữ liệu
        HAL_UART_Receive_IT(huart, uart_rx_buffer, UART_BUFFER_SIZE);
    }
}
char* extractJsonObject(const char *jsonString) {
    const char *start = strstr(jsonString, "\"current\":{\"t");
    const char *end = strrchr(jsonString, '}');
    if (start && end && end > start) {
        size_t length = end - start + 1;
        char *result = (char *)malloc(length + 2); // Allocate extra space for the '{' character
        if (result) {
            result[0] = '{'; // Add '{' at the beginning
            strncpy(result + 1, start, length); // Copy the rest of the string
            result[length + 1] = '\0'; // Null-terminate the string
            return result;
        }
    }
    return NULL;
}


void processWeather( char *jsonString) {

	jsonString = extractJsonObject(jsonString);

    cJSON *json = cJSON_Parse(jsonString);

    if (json == NULL) {
        lcdSetCursor(10, 10);
        lcdSetTextColor(COLOR_RED, COLOR_BLACK);
        lcdPrintf("Error parsing JSON!");
        return;
    }

    // Get daily forecast information
    cJSON *daily = cJSON_GetObjectItem(json, "daily");
    cJSON *current = cJSON_GetObjectItem(json, "current");
    if (!daily) {
        lcdSetCursor(10, 10);
        lcdSetTextColor(COLOR_RED, COLOR_BLACK);
        lcdPrintf("Error: Missing 'daily' object!");
        cJSON_Delete(json);
        return;
    }

    cJSON *dates = cJSON_GetObjectItem(daily, "time");
    cJSON *temp_max_json = cJSON_GetObjectItem(daily, "temperature_2m_max");
    cJSON *temp_min_json = cJSON_GetObjectItem(daily, "temperature_2m_min");
    cJSON *uv_json = cJSON_GetObjectItem(daily, "uv_index_max");
    cJSON *weather_code = cJSON_GetObjectItem(daily, "weather_code");
    cJSON *wind_speed_json = cJSON_GetObjectItem(daily, "wind_speed_10m_max");

    char * Tmp =  cJSON_GetObjectItem(current , "time")->valuestring;
    int year, month, day, hour , minute;
	char  tmp[17];
	sscanf(Tmp, "%d-%d-%dT%d:%d", &year, &month, &day , &hour ,&minute);
	snprintf(tmp, sizeof(tmp), "%02d-%02d-%02d", day, month , year);
	strncpy(current_date, tmp, sizeof(current_date) - 1);
	current_date[sizeof(current_date) - 1] = '\0';

	snprintf(tmp, sizeof(tmp), "%02d:%02d", hour, minute);
	strncpy(current_time, tmp, sizeof(current_time) - 1);
	current_time[sizeof(current_time) - 1] = '\0';

    current_temp =  cJSON_GetObjectItem(current , "temperature_2m")->valueint;

	current_humi =  cJSON_GetObjectItem(current , "relative_humidity_2m")->valueint;

	current_code =  cJSON_GetObjectItem(current , "weather_code")->valueint;

	current_cloud =  cJSON_GetObjectItem(current , "cloud_cover")->valueint;

	isDay = cJSON_GetObjectItem(current , "is_day")->valueint;

    // Allocate memory for global variables
    int num_days = cJSON_GetArraySize(dates);
    temp_max = (int *)malloc(num_days * sizeof(int));
    temp_min = (int *)malloc(num_days * sizeof(int));
    UV = (double *)malloc(num_days * sizeof(double));
    wind_speed = (int *)malloc(num_days * sizeof(int));
    day_code = (int *)malloc(num_days * sizeof(int));
    if (!temp_max || !temp_min || !wind_speed || !day_code) {
        lcdSetCursor(10, 10);
        lcdSetTextColor(COLOR_RED, COLOR_BLACK);
        lcdPrintf("Error allocating memory!");
        cJSON_Delete(json);
        return;
    }

    // Convert dates and extract weather data
    for (int i = 0; i < num_days; i++) {
        char *date = cJSON_GetArrayItem(dates, i)->valuestring;
        temp_max[i] = (int)cJSON_GetArrayItem(temp_max_json, i)->valuedouble;
        temp_min[i] = (int)cJSON_GetArrayItem(temp_min_json, i)->valuedouble;
        UV[i] = cJSON_GetArrayItem(uv_json, i)->valuedouble;
        wind_speed[i] = (int)cJSON_GetArrayItem(wind_speed_json, i)->valuedouble;
        day_code[i] = cJSON_GetArrayItem(weather_code, i)->valueint;

        // Extract month and day from date string
        int year, month, days;
        char tmp[6];
        sscanf(date, "%d-%d-%d", &year, &month, &days);
        snprintf(tmp, sizeof(tmp), "%02d-%02d", days, month);
        strncat(Date[i], tmp, sizeof(Date[i]) - strlen(Date[i]) - 1);

        // Convert day index to day name
        char day[4];
        int M;
        M = FindDate(days, month, year);
        switch (M) {
            case 0: snprintf(day, sizeof(day), "Mon"); break;
            case 1: snprintf(day, sizeof(day), "Tue"); break;
            case 2: snprintf(day, sizeof(day), "Wed"); break;
            case 3: snprintf(day, sizeof(day), "Thu"); break;
            case 4: snprintf(day, sizeof(day), "Fri"); break;
            case 5: snprintf(day, sizeof(day), "sat"); break;
            case 6: snprintf(day, sizeof(day), "Sun"); break;
            default: break;
        }
        strncat(day_name[i], day, sizeof(day_name[i]) - strlen(day_name[i]) - 1);
    }

    // Clean up memory
    cJSON_Delete(json);
}

//

void RunProgram(){
  if (readDHT && current == 1)
  {
	  readDHT = false; // Xóa c�?
	  DHT_ReadData(&temperature, &humidity);
	  TextSensor(5, 245, temperature, humidity);
  }
  if(choice != choiceTmp && current == 1){
	  Recall = true;
	  choiceTmp = choice;
  }

  if (current == 1){
	  if (updated == true){
		  Screen1(temp_max[0] , temp_min[0] , current_temp ,  current_humi , current_code ,current_cloud ,current_time ,current_date ,choice);
		  HAL_Delay(1000);
		  updated = false;
	  }
	  if (TouchGetCalibratedPoint(&tx, &ty)){
		  if (tx >= 199 && tx <= 239 && ty >= 25 && ty <= 65){
			  current = 2;
			  updated = true;
			  HAL_Delay(200);
		  }
		  if (tx >= 158 && tx <= 198 && ty >= 25 && ty <= 65){
			  current = 3;
			  updated = true;
			  HAL_Delay(200);
		  }
	  }
  }
  else if (current == 2){
	  if (updated == true){
		  Screen2(temp_max, temp_min, day_code, day_name, wind_speed, Date,current_time ,current_date ,choice);
		  HAL_Delay(1000);
		  updated = false;
	  }
	  if (TouchGetCalibratedPoint(&tx, &ty)){
		  if (tx >= 180 && tx <= 230 && ty >= 10 && ty <= 100){
			  current = 1;
			  updated = true;
			  HAL_Delay(100);
		  }
	  }
  }
  else if (current == 3){
	  if (updated == true){
		  Screen3(choice);
		  HAL_Delay(1000);
		  updated = false;
		  choiceTmp=choice;
	  }
	  if (TouchGetCalibratedPoint(&tx, &ty)){
		  if (tx >= 100 && tx <= 140 && ty >= 275 && ty <= 315){
			  current = 1;
			  updated = true;
			  HAL_Delay(100);
		  }
		  if (tx >= 20 && tx <= 220 && ty >= 50 && ty <= 85){
			  choice = 1;
			  lcdFillRoundRect(20,50, 200, 35, 6, COLOR_GREEN);
			  lcdDrawRoundRect(20,50, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextColor(COLOR_WHITE, COLOR_GREEN);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 60);
			  lcdPrintf("TP.HCM");
			  //
			  lcdFillRoundRect(20,95, 200, 35, 6, COLOR_LIGHTGREY);
			  lcdDrawRoundRect(20,95, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextColor(COLOR_WHITE, COLOR_LIGHTGREY);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 105);
			  lcdPrintf("Ha Noi");
			  //
			  lcdSetTextColor(COLOR_WHITE, COLOR_LIGHTGREY);
			  lcdFillRoundRect(20,140, 200, 35, 6, COLOR_LIGHTGREY);
			  lcdDrawRoundRect(20,140, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 150);
			  lcdPrintf("Hai Phong");
			  //
			  lcdSetTextColor(COLOR_WHITE, COLOR_LIGHTGREY);
			  lcdFillRoundRect(20,185, 200, 35, 6, COLOR_LIGHTGREY);
			  lcdDrawRoundRect(20,185, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 195);
			  lcdPrintf("Can Tho");
			  //
			  lcdSetTextColor(COLOR_WHITE, COLOR_LIGHTGREY);
			  lcdFillRoundRect(20,230, 200, 35, 6, COLOR_LIGHTGREY);
			  lcdDrawRoundRect(20,230, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 240);
			  lcdPrintf("Da Nang");
			  HAL_Delay(200);
		  }
		  if (tx >= 20 && tx <= 220 && ty >= 95 && ty <= 130){
			  choice = 2;
			  lcdFillRoundRect(20,50, 200, 35, 6, COLOR_LIGHTGREY);
			  lcdDrawRoundRect(20,50, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextColor(COLOR_WHITE, COLOR_LIGHTGREY);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 60);
			  lcdPrintf("TP.HCM");
			  //
			  lcdFillRoundRect(20,95, 200, 35, 6, COLOR_GREEN);
			  lcdDrawRoundRect(20,95, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextColor(COLOR_WHITE, COLOR_GREEN);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 105);
			  lcdPrintf("Ha Noi");
			  //
			  lcdSetTextColor(COLOR_WHITE, COLOR_LIGHTGREY);
			  lcdFillRoundRect(20,140, 200, 35, 6, COLOR_LIGHTGREY);
			  lcdDrawRoundRect(20,140, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 150);
			  lcdPrintf("Hai Phong");
			  //
			  lcdSetTextColor(COLOR_WHITE, COLOR_LIGHTGREY);
			  lcdFillRoundRect(20,185, 200, 35, 6, COLOR_LIGHTGREY);
			  lcdDrawRoundRect(20,185, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 195);
			  lcdPrintf("Can Tho");
			  //
			  lcdSetTextColor(COLOR_WHITE, COLOR_LIGHTGREY);
			  lcdFillRoundRect(20,230, 200, 35, 6, COLOR_LIGHTGREY);
			  lcdDrawRoundRect(20,230, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 240);
			  lcdPrintf("Da Nang");
			  HAL_Delay(200);
		  }
		  if (tx >= 20 && tx <= 220 && ty >= 140 && ty <= 175){
			  choice = 3;
			  lcdFillRoundRect(20,50, 200, 35, 6, COLOR_LIGHTGREY);
			  lcdDrawRoundRect(20,50, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextColor(COLOR_WHITE, COLOR_LIGHTGREY);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 60);
			  lcdPrintf("TP.HCM");
			  //
			  lcdFillRoundRect(20,95, 200, 35, 6, COLOR_LIGHTGREY);
			  lcdDrawRoundRect(20,95, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextColor(COLOR_WHITE, COLOR_LIGHTGREY);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 105);
			  lcdPrintf("Ha Noi");
			  //
			  lcdSetTextColor(COLOR_WHITE, COLOR_GREEN);
			  lcdFillRoundRect(20,140, 200, 35, 6, COLOR_GREEN);
			  lcdDrawRoundRect(20,140, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 150);
			  lcdPrintf("Hai Phong");
			  //
			  lcdSetTextColor(COLOR_WHITE, COLOR_LIGHTGREY);
			  lcdFillRoundRect(20,185, 200, 35, 6, COLOR_LIGHTGREY);
			  lcdDrawRoundRect(20,185, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 195);
			  lcdPrintf("Can Tho");
			  //
			  lcdSetTextColor(COLOR_WHITE, COLOR_LIGHTGREY);
			  lcdFillRoundRect(20,230, 200, 35, 6, COLOR_LIGHTGREY);
			  lcdDrawRoundRect(20,230, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 240);
			  lcdPrintf("Da Nang");
			  HAL_Delay(200);
					  }
		  if (tx >= 20 && tx <= 220 && ty >= 185 && ty <= 220){
			  choice = 4;
			  lcdFillRoundRect(20,50, 200, 35, 6, COLOR_LIGHTGREY);
			  lcdDrawRoundRect(20,50, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextColor(COLOR_WHITE, COLOR_LIGHTGREY);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 60);
			  lcdPrintf("TP.HCM");
			  //
			  lcdFillRoundRect(20,95, 200, 35, 6, COLOR_LIGHTGREY);
			  lcdDrawRoundRect(20,95, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextColor(COLOR_WHITE, COLOR_LIGHTGREY);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 105);
			  lcdPrintf("Ha Noi");
			  //
			  lcdSetTextColor(COLOR_WHITE, COLOR_LIGHTGREY);
			  lcdFillRoundRect(20,140, 200, 35, 6, COLOR_LIGHTGREY);
			  lcdDrawRoundRect(20,140, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 150);
			  lcdPrintf("Hai Phong");
			  //
			  lcdSetTextColor(COLOR_WHITE, COLOR_GREEN);
			  lcdFillRoundRect(20,185, 200, 35, 6, COLOR_GREEN);
			  lcdDrawRoundRect(20,185, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 195);
			  lcdPrintf("Can Tho");
			  //
			  lcdSetTextColor(COLOR_WHITE, COLOR_LIGHTGREY);
			  lcdFillRoundRect(20,230, 200, 35, 6, COLOR_LIGHTGREY);
			  lcdDrawRoundRect(20,230, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 240);
			  lcdPrintf("Da Nang");
			  HAL_Delay(200);
		  }
		  if (tx >= 20 && tx <= 220 && ty >= 230 && ty <= 265){
			  choice = 5;
			  lcdFillRoundRect(20,50, 200, 35, 6, COLOR_LIGHTGREY);
			  lcdDrawRoundRect(20,50, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextColor(COLOR_WHITE, COLOR_LIGHTGREY);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 60);
			  lcdPrintf("TP.HCM");
			  //
			  lcdFillRoundRect(20,95, 200, 35, 6, COLOR_LIGHTGREY);
			  lcdDrawRoundRect(20,95, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextColor(COLOR_WHITE, COLOR_LIGHTGREY);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 105);
			  lcdPrintf("Ha Noi");
			  //
			  lcdSetTextColor(COLOR_WHITE, COLOR_LIGHTGREY);
			  lcdFillRoundRect(20,140, 200, 35, 6, COLOR_LIGHTGREY);
			  lcdDrawRoundRect(20,140, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 150);
			  lcdPrintf("Hai Phong");
			  //
			  lcdSetTextColor(COLOR_WHITE, COLOR_LIGHTGREY);
			  lcdFillRoundRect(20,185, 200, 35, 6, COLOR_LIGHTGREY);
			  lcdDrawRoundRect(20,185, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 195);
			  lcdPrintf("Can Tho");
			  //
			  lcdSetTextColor(COLOR_WHITE, COLOR_GREEN);
			  lcdFillRoundRect(20,230, 200, 35, 6, COLOR_GREEN);
			  lcdDrawRoundRect(20,230, 200, 36, 6, COLOR_BLACK);
			  lcdSetTextFont(&Font20);
			  lcdSetCursor(25, 240);
			  lcdPrintf("Da Nang");
			  HAL_Delay(200);
		  }
	  }
  }
  if(Recall){
	  Screen0();
	  Resend_AT_Commands(&huart1);
	  processWeather(uart_rx_buffer);
	  if (current == 1){
		  Screen1(temp_max[0] , temp_min[0] , current_temp ,  current_humi , current_code ,current_cloud ,current_time ,current_date , choice);
	  }
	  if (current == 2){
		  Screen2(temp_max, temp_min, day_code, day_name, wind_speed, Date,current_time ,current_date ,choice);
	  }
	  if (current == 3){
		  Screen3(choice);
	  }
	  Recall = false;
  }
  if(Reload){
  	  Resend_AT_Commands(&huart1);
  	  processWeather(uart_rx_buffer);
  	  if (current == 1){
  		  Screen1(temp_max[0] , temp_min[0] , current_temp ,  current_humi , current_code ,current_cloud ,current_time ,current_date , choice);
  	  }
  	  if (current == 2){
  		  Screen2(temp_max, temp_min, day_code, day_name, wind_speed, Date,current_time ,current_date ,choice);
  	  }
  	  if (current == 3){
  		  Screen3(choice);
  	  }
  	  Reload = false;
    }
}

void Max30100 (){
	  lcdSetTextColor(COLOR_CYAN, COLOR_BLACK);
	  lcdSetTextFont(&Font16);
	  lcdSetCursor(5, 5);
	  lcdPrintf("MAX30100 : %d\n" , i);
	  i++;
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
