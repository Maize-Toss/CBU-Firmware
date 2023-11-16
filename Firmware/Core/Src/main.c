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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bluetooth.h"
#include "reader.h"
#include <stdbool.h>
#include "MaizeJSON.h"
#include "st25r95.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RFID_READ_PERIOD_MS 3000
#define BAT_READ_PERIOD_MS 30000
#define VBAT_CONVERSION_FACTOR (float) 3 * 3.3 / 4096 // VBat/3 = rawADC_IN18 * VREF / 2^12

#define RX_BUFF_SIZE 256			// TODO: change to appropriate size
#define TX_BUFF_SIZE 256			// TODO: change to appropriate size

#define BLE_CS_PORT GPIOA
#define BLE_CS_PIN GPIO_PIN_0

#define ADDR_PORT GPIOA
#define ADDR_PIN GPIO_PIN_1

#define SDU_NAME "SDU"

#define SEL0_GPIO_Port GPIOC
#define SEL1_GPIO_Port GPIOC
#define SEL2_GPIO_Port GPIOB
#define SEL3_GPIO_Port GPIOB
#define SEL4_GPIO_Port GPIOC
#define SEL5_GPIO_Port GPIOC
#define SEL6_GPIO_Port GPIOB
#define SEL7_GPIO_Port GPIOB

#define SEL0_Pin GPIO_PIN_2
#define SEL1_Pin GPIO_PIN_1
#define SEL2_Pin GPIO_PIN_15
#define SEL3_Pin GPIO_PIN_14
#define SEL4_Pin GPIO_PIN_7
#define SEL5_Pin GPIO_PIN_6
#define SEL6_Pin GPIO_PIN_13
#define SEL7_Pin GPIO_PIN_12

#define RFID_CS_PORT GPIOA
#define RFID_CS_PIN GPIO_PIN_4
#define RFID_NIRQ_IN_PORT GPIOC
#define RFID_NIRQ_IN_PIN GPIO_PIN_4
#define RFID_NIRQ_OUT_PORT GPIOB
#define RFID_NIRQ_OUT_PIN GPIO_PIN_11


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for readRFIDTask */
osThreadId_t readRFIDTaskHandle;
const osThreadAttr_t readRFIDTask_attributes = {
  .name = "readRFIDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for BluetoothRXTask */
osThreadId_t BluetoothRXTaskHandle;
const osThreadAttr_t BluetoothRXTask_attributes = {
  .name = "BluetoothRXTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for readBatteryVolt */
osThreadId_t readBatteryVoltHandle;
const osThreadAttr_t readBatteryVolt_attributes = {
  .name = "readBatteryVolt",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for BluetoothRX */
osSemaphoreId_t BluetoothRXHandle;
const osSemaphoreAttr_t BluetoothRX_attributes = {
  .name = "BluetoothRX"
};
/* USER CODE BEGIN PV */
BroadcastPacket broadcastPacket = {0,0,0};


char CBU_ID[4] = "CBUx";

BLE_interface ble;

uint8_t rx_buffer[RX_BUFF_SIZE] = {0};
uint8_t tx_buffer[TX_BUFF_SIZE] = {0};

GameInfo gameInfo = {{0,0}, {0,0}, 0 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);
void StartRFIDTask(void *argument);
void StartBluetoothTask(void *argument);
void StartBatteryTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/**
 * channel index is 0 indexed, 0-11
 */
uint8_t select_rfid_channel(uint8_t channel_index) {
	channel_index++;

	// lol I originally wrote this to be 1 indexed, so I added
	// channel_index++ to make it 0 indexed, the rest of the
	// logic is 1 indexed though

	if (channel_index < 1 || channel_index > 12) {
		return 1;  // Invalid switch index
	}

	// Set U2.4
	uint8_t switch_refdes = 0;  // U2.X for 1st layer switch
	if (channel_index <= 4) {
		HAL_GPIO_WritePin(SEL6_GPIO_Port, SEL6_Pin, 0);
		HAL_GPIO_WritePin(SEL7_GPIO_Port, SEL7_Pin, 0);
		switch_refdes = 1;
	}
	else if (channel_index <= 8) {
		HAL_GPIO_WritePin(SEL6_GPIO_Port, SEL6_Pin, 1);
		HAL_GPIO_WritePin(SEL7_GPIO_Port, SEL7_Pin, 1);
		switch_refdes = 2;
	}
	else { // if channel_index <= 12
		HAL_GPIO_WritePin(SEL6_GPIO_Port, SEL6_Pin, 1);
		HAL_GPIO_WritePin(SEL7_GPIO_Port, SEL7_Pin, 0);
		switch_refdes = 3;
	}

	// The pattern on each 2nd layer switch is the same, use multiplexing
	uint8_t V0 = 0;
	uint8_t V1 = 0;
	if (channel_index % 4 == 0) {  // 4, 8, 12
		V0 = 1;
		V1 = 0;
	}
	else if (channel_index % 4 == 1) {  // 1, 5, 9
		V0 = 0;
		V1 = 0;
	}
	else if (channel_index % 4 == 2) {  // 2, 6, 10
		V0 = 0;
		V1 = 1;
	}
	else {  // if channel_index % 4 == 3  // 3, 7, 11
		V0 = 1;
		V1 = 1;
	}

	// The other switches' V0/V1 are don't care, but set them to 0 anyways
	switch(switch_refdes) {
	case 1:
		HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, V0);
		HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, V1);
		HAL_GPIO_WritePin(SEL2_GPIO_Port, SEL2_Pin, 0);
		HAL_GPIO_WritePin(SEL3_GPIO_Port, SEL3_Pin, 0);
		HAL_GPIO_WritePin(SEL4_GPIO_Port, SEL4_Pin, 0);
		HAL_GPIO_WritePin(SEL5_GPIO_Port, SEL5_Pin, 0);
		break;
	case 2:
		HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, 0);
		HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, 0);
		HAL_GPIO_WritePin(SEL2_GPIO_Port, SEL2_Pin, V0);
		HAL_GPIO_WritePin(SEL3_GPIO_Port, SEL3_Pin, V1);
		HAL_GPIO_WritePin(SEL4_GPIO_Port, SEL4_Pin, 0);
		HAL_GPIO_WritePin(SEL5_GPIO_Port, SEL5_Pin, 0);
		break;
	case 3:
		HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, 0);
		HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, 0);
		HAL_GPIO_WritePin(SEL2_GPIO_Port, SEL2_Pin, 0);
		HAL_GPIO_WritePin(SEL3_GPIO_Port, SEL3_Pin, 0);
		HAL_GPIO_WritePin(SEL4_GPIO_Port, SEL4_Pin, V0);
		HAL_GPIO_WritePin(SEL5_GPIO_Port, SEL5_Pin, V1);
		break;
	default:
		break;
	}

	return 0;
}


// UPDATE ALL THESE TO INCLUDE DELAYUNTIL or OSDELAY
volatile st25r95_handle reader_handler;

void reader_irq_pulse() {
  HAL_GPIO_WritePin(RFID_NIRQ_IN_PORT, RFID_NIRQ_IN_PIN, GPIO_PIN_RESET);
  // HAL_delay(1);
  vTaskDelay(xTaskGetTickCount() + pdMS_TO_TICKS(1)); // delay 1 ms
  HAL_GPIO_WritePin(RFID_NIRQ_IN_PORT, RFID_NIRQ_IN_PIN, GPIO_PIN_SET);
  // HAL_delay(8);
  vTaskDelay(xTaskGetTickCount() + pdMS_TO_TICKS(8)); // delay 8 ms
}

void reader_nss(uint8_t enable)
{
  HAL_GPIO_WritePin(RFID_CS_PORT, RFID_CS_PIN, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

int reader_tx(uint8_t *data, size_t len)
{
  int ret = HAL_SPI_Transmit(&hspi1, data, len, HAL_MAX_DELAY);
  return ret;
}

int reader_rx(uint8_t *data, size_t len)
{
  int ret = HAL_SPI_Receive(&hspi1, data, len, HAL_MAX_DELAY);
  return ret;
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
  if (pin == RFID_NIRQ_OUT_PIN) {
    reader_handler.irq_flag = 1;
  }
}

void st25_card_callback(uint8_t *uid)
{
  //HAL_Delay(uid[0]);
  vTaskDelay(xTaskGetTickCount() + pdMS_TO_TICKS(uid[0])); // delay uid[0]
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // RFID INITIALIZATION
  reader_handler.protocol = ST25_PROTOCOL_14443A;
  reader_handler.tx_speed = ST25_26K_106K; // datarate used by 14443A
  reader_handler.rx_speed = ST25_26K_106K; // same for up and downlinks
  reader_handler.timerw = 0x58;
  reader_handler.ARC = 0xD1;
  reader_handler.irq_flag = 0;

  /* Bind BSP Functions */
  reader_handler.nss = reader_nss;
  reader_handler.tx = reader_tx;
  reader_handler.rx = reader_rx;
  reader_handler.irq_pulse = reader_irq_pulse;
  reader_handler.callback = st25_card_callback;

  st25r95_init(&reader_handler);
  st25r95_calibrate(&reader_handler);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of BluetoothRX */
  BluetoothRXHandle = osSemaphoreNew(1, 1, &BluetoothRX_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of readRFIDTask */
  readRFIDTaskHandle = osThreadNew(StartRFIDTask, NULL, &readRFIDTask_attributes);

  /* creation of BluetoothRXTask */
  BluetoothRXTaskHandle = osThreadNew(StartBluetoothTask, NULL, &BluetoothRXTask_attributes);

  /* creation of readBatteryVolt */
  readBatteryVoltHandle = osThreadNew(StartBatteryTask, NULL, &readBatteryVolt_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */

  	  if (HAL_GPIO_ReadPin(ADDR_PORT, ADDR_PIN) == GPIO_PIN_SET) {
  		  CBU_ID[3] = '1';
  	  } else {
  		  CBU_ID[3] = '0';
  	  }

  	  ble.huart = &huart1;
  	  ble.cs_base = BLE_CS_PORT;
  	  ble.cs_pin = BLE_CS_PIN;
  	  ble.name = CBU_ID;
  	  ble.mutex = &BluetoothRXHandle;

//  	  initBLE(&ble);

//#ifndef DEBUG
//  	  while (!connect(SDU_NAME, &ble)); // try to reconnect in Release mode
//#else
//  	connect(SDU_NAME, &ble); // only try to connect once in Debug mode
//#endif


//  for(;;){
//	  for(volatile int i = 0; i  <40000; i++)
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
//	  for(volatile int i = 0; i  <40000; i++)
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
//  }
  	  for(;;){
	   for(uint8_t i = 0; i < 12;  i++){
			  select_rfid_channel(i);
			  for(volatile int x = 0; x  <1000000; x++);
		}
	  }
//	  select_rfid_channel(8);
//	  for(;;);
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV256;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.DFSDMConfig = ADC_DFSDM_MODE_ENABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart1.Init.BaudRate = 38400;
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
  huart2.Init.BaudRate = 38400;
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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC2 PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB12
                           PB13 PB14 PB15 PB3
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	readyToRead(&ble, rx_buffer);
	osSemaphoreRelease(BluetoothRXHandle);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartRFIDTask */
/**
* @brief Function implementing the RFIDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRFIDTask */
void StartRFIDTask(void *argument)
{
  /* USER CODE BEGIN StartRFIDTask */

	TickType_t xLastWakeTime;
	const TickType_t period = pdMS_TO_TICKS(RFID_READ_PERIOD_MS);

	xLastWakeTime = xTaskGetTickCount();

	uint8_t team0RawScore = 0;
	uint8_t team1RawScore = 0;

	uint8_t tx_buf[2];

  /* Infinite loop */
	for(;;)
		// osDelay(RFID_READ_PERIOD_MS); // temp for commenting out the rest of the task
	{
		RFID_readArray();

		calculateRawScore(&team0RawScore, false);
		calculateRawScore(&team1RawScore, true); // true to move BagStatus pointer to the Team 1 section

		broadcastPacket.team0DeltaScore = team0RawScore;
		broadcastPacket.team1DeltaScore = team1RawScore;


//		broadcast(tx_buf, (uint32_t) 2, &ble);

		vTaskDelayUntil( &xLastWakeTime, period );
	}
  /* USER CODE END StartRFIDTask */
}

/* USER CODE BEGIN Header_StartBluetoothTask */
/**
* @brief Function implementing the BluetoothTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBluetoothTask */
void StartBluetoothTask(void *argument)
{
  /* USER CODE BEGIN StartBluetoothTask */

	uint8_t rx_buff[RX_BUFF_SIZE];
	uint32_t size = 0;

 /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(BluetoothRXHandle, osWaitForever);
	  deserializeJSON((char*)rx_buff, &gameInfo);
	  if (gameInfo.end_of_round){ //TODO: verify we only want to send at end of round
		  // Alternatively, we could have an extra element in the JSON like a boolean so whenever it isnt set, we know only to care about the battery voltage
		  for(uint32_t i = 0; i < 300; i++){
			  serializeJSON(&broadcastPacket,tx_buffer);
		  }
	  }
  }
  /* USER CODE END StartBluetoothTask */
}

/* USER CODE BEGIN Header_StartBatteryTask */
/**
* @brief Function implementing the readBatteryVolt thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBatteryTask */
void StartBatteryTask(void *argument)
{
  /* USER CODE BEGIN StartBatteryTask */

	TickType_t xLastWakeTime;
	const TickType_t period = pdMS_TO_TICKS(BAT_READ_PERIOD_MS);

	xLastWakeTime = xTaskGetTickCount();

	uint32_t rawVBat = 0;
	float VBat = 0.0;
	uint32_t VBat_conv = 0;

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&rawVBat, 1);

	uint8_t tx_buf[4];

  /* Infinite loop */
  for(;;)
  {

	  VBat = rawVBat * VBAT_CONVERSION_FACTOR;
	  VBat_conv = *((uint32_t*)&VBat);
	  tx_buf[0] = VBat_conv & 0xFF000000;
	  tx_buf[1] = VBat_conv & 0x00FF0000;
	  tx_buf[2] = VBat_conv & 0x0000FF00;
	  tx_buf[3] = VBat_conv & 0x000000FF;

	  broadcastPacket.batteryVoltage = tx_buf;
//	  broadcast(tx_buf, (uint32_t) 4, &ble);

	  vTaskDelayUntil( &xLastWakeTime, period );

  }
  /* USER CODE END StartBatteryTask */
}

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
