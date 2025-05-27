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
#define ECG_X_MAX 320  // chiều rộng màn hình LCD
#define ECG_Y_MAX 240  // chiều cao màn hình LCD
#define ECG_BASELINE 120  // đường trung tâm (ngang) để vẽ ECG
#define ECG_SCALE 0.05f    // hệ số scale giá trị ADC xuống pixel (tùy chỉnh nếu thấy lệch)

uint16_t ecg_x = 0;
uint16_t old_y = ECG_BASELINE;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

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
static void MX_ADC1_Init(void);
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
void AD8232_ReadData();
void ECG_DrawWave(uint16_t value);
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
int AD8232_value = 0;		// READ AD8232

int choice = 1;
int choiceTmp = 1;
//
int current = 1;
bool updated = true;
int16_t tx , ty;

int i = 0;
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  //-----------------------------

  LCD_BL_ON();
  HAL_TIM_Base_Start_IT(&htim2);
  TouchCalibrate();

  lcdInit();
  int i = 2;

  lcdSetOrientation(i%4);
  lcdFillRGB(COLOR_BLUE);


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
//	  Max30100();
	  AD8232_ReadData();
	  HAL_Delay(1000);
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
    	Send_AT_Command(huart, ATCommand1, 1000);
    }
    else if(choice == 2){
    	Send_AT_Command(huart, ATCommand2, 1000);
    }
    else if(choice == 3){
		Send_AT_Command(huart, ATCommand3, 1000);
	}
    else if(choice == 4){
		Send_AT_Command(huart, ATCommand4, 1000);
	}
    else if(choice == 5){
		Send_AT_Command(huart, ATCommand5, 1000);
	}

}
void Resend_AT_Commands(UART_HandleTypeDef *huart){
//	Send_AT_Command1(huart, "AT+CIPSTART=\"TCP\",\"api.open-meteo.com\",80\r\n", 3000 );
//	Send_AT_Command1(huart, "AT+CIPSEND=299\r\n", 3000);
//
//	// Gửi yêu cầu GET cuối cùng
//	if (choice == 1){
//		Send_AT_Command1(huart, ATCommand1, 1000);
//	}
//	else if(choice == 2){
//		Send_AT_Command1(huart, ATCommand2, 1000);
//	}
//	else if(choice == 3){
//		Send_AT_Command1(huart, ATCommand3, 1000);
//	}
//	else if(choice == 4){
//		Send_AT_Command1(huart, ATCommand4, 1000);
//	}
//	else if(choice == 5){
//		Send_AT_Command1(huart, ATCommand5, 1000);
//	}
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

}

void Max30100 (){
//	  lcdSetTextColor(COLOR_CYAN, COLOR_BLACK);
//	  lcdSetTextFont(&Font16);
//	  lcdSetCursor(5, 5);
//	  lcdPrintf("MAX30100 : %d\n" , i);
//	  i++;
}

void AD8232_ReadData()
{
    // Đọc trạng thái dây điện cực (LO+ và LO-)
    GPIO_PinState lo_plus = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
    GPIO_PinState lo_minus = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);

    lcdSetTextColor(COLOR_YELLOW, COLOR_BLACK);   // Màu chữ: vàng, nền: đen
    lcdSetTextFont(&Font16);                      // Font 16
    lcdSetCursor(5, 20);                           // Vị trí tùy chọn

    // Kiểm tra xem có bị rơi điện cực không
    if (lo_plus == GPIO_PIN_SET || lo_minus == GPIO_PIN_SET)
    {
    	lcdSetCursor(5, 40);
        lcdPrintf("ECG Error: Roi day!");
    }
    else
    {
    	lcdSetCursor(5, 40);
        // Nếu dây điện cực ổn, đọc ADC
        AD8232_value = AD8232_Read();
        lcdPrintf("AD8232  : %d    ", AD8232_value); // In giá trị ECG
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
