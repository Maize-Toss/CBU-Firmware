/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "st25r95.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RFID_CS_PORT GPIOA
#define RFID_CS_PIN GPIO_PIN_4
#define RFID_NIRQ_IN_PORT GPIOC
#define RFID_NIRQ_IN_PIN GPIO_PIN_4
#define RFID_NIRQ_OUT_PORT GPIOB
#define RFID_NIRQ_OUT_PIN GPIO_PIN_11

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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void select_switch_port(uint8_t channel_index) {

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

volatile st25r95_handle reader_handler;

void reader_irq_pulse() {
  HAL_GPIO_WritePin(RFID_NIRQ_IN_PORT, RFID_NIRQ_IN_PIN, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(RFID_NIRQ_IN_PORT, RFID_NIRQ_IN_PIN, GPIO_PIN_SET);
  HAL_Delay(100);
}

void reader_nss(uint8_t enable) {
  HAL_GPIO_WritePin(RFID_CS_PORT, RFID_CS_PIN, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

int reader_tx(uint8_t *data, size_t len) {
  int ret = HAL_SPI_Transmit(&hspi1, data, len, HAL_MAX_DELAY);
  return ret;
}

int reader_rx(uint8_t *data, size_t len, uint32_t delay) {
  int ret = HAL_SPI_Receive(&hspi1, data, len, delay);
  return ret;
}

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
  if (pin == RFID_NIRQ_OUT_PIN) {
    reader_handler.irq_flag = 1;
  }
}

void st25_card_callback(uint8_t *uid) {
  HAL_Delay(uid[0]);
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* Setup all protocol */
  reader_handler.protocol = ST25_PROTOCOL_15693;
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

  select_switch_port(7);
  st25r95_init(&reader_handler);
  st25r95_calibrate(&reader_handler);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  st25r95_idle(&reader_handler);
  while (1) {
    st25r95_service(&reader_handler);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_2|NIRQ_IN_Pin|GPIO_PIN_6
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_RFID_CS_GPIO_Port, SPI1_RFID_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC1 PC2 PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_RFID_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_RFID_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI1_RFID_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NIRQ_IN_Pin */
  GPIO_InitStruct.Pin = NIRQ_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NIRQ_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NIRQ_OUT_Pin */
  GPIO_InitStruct.Pin = NIRQ_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(NIRQ_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15
                           PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
  while (1) {
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
