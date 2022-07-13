/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
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
#include "util_eprom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_WAIT 1000
#define BUTTON_WAIT 100
#define COMM_WAIT	60000  //60 second

#define TXBUFFER_SIZE 5
#define RXBUFFER_SIZE 100
#define STRBUFFER_SIZE 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
uint8_t uart1RxBuff[RXBUFFER_SIZE];
uint8_t txBuffer[TXBUFFER_SIZE];
uint8_t strBuffer[STRBUFFER_SIZE];
uint32_t flashTickTime = 0, btnTickTime = 0, commTickTime = 0;

uint8_t flashSize = 0, jetsonWakeup = 0, jetsonWakeupOld = 0;
uint8_t buttonStatus = 0, btnStatus = 0, btnStatusOld = 0;
uint8_t command = 0, commandStatus = 0, flashDefault = 1;
volatile uint8_t commandReceived = 0;
volatile HAL_StatusTypeDef uartStatus;

extern Lamp_t flash[LAMP_SIZE];
extern uint8_t lampDefaultValue[LAMP_SIZE];
extern const Lamp_t lamp[LAMP_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

void uartRxRestart();
void checkButtonStatus();
void sendStatus();
void performCommand();
void flashLamp();
void lampAction(uint8_t *lampValue);
void closeAllLamp();
uint8_t checksum(uint8_t *source, uint16_t size);
void uartPrint(UART_HandleTypeDef *huart, char *message);
void debug(char *message);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Uart Idle Interrupt Callback Function
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	//debug("Received\n");
	if (huart->Instance == USART1) {
		if (Size == 48) {
			commandReceived = 1;
		}
		commTickTime = HAL_GetTick();
		/* start the DMA again */
		uartStatus = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1RxBuff, RXBUFFER_SIZE);
		uartRxRestart();
	}
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */

	//Uart Idle Interrupt Start
	uartStatus = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1RxBuff, RXBUFFER_SIZE);
	uartRxRestart();

	readEpromLampDefault();
	printf("==>lampDefaultValue:\n");
	for (int i = 0; i < LAMP_SIZE; ++i) {
		printf("l%d = %d, ",i,lampDefaultValue[i]);
	}

	lampAction(lampDefaultValue);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		if ((HAL_GetTick() - btnTickTime) > BUTTON_WAIT) {
			btnTickTime = HAL_GetTick();

			checkButtonStatus();
			buttonStatus = ~btnStatusOld & btnStatus;
			btnStatusOld = btnStatus;
			jetsonWakeupOld = jetsonWakeup;

			//Rising edge --> Jetson closed or Communication timeout
			if ((jetsonWakeupOld == 0 && jetsonWakeup == 1) || (HAL_GetTick() - commTickTime) > COMM_WAIT)
			{
				HAL_Delay(50); //Debounce time
				jetsonWakeup = HAL_GPIO_ReadPin(JETSON_WKP_IN_GPIO_Port, JETSON_WKP_IN_Pin);
				if ((jetsonWakeupOld == 0 && jetsonWakeup == 1) || (HAL_GetTick() - commTickTime) > COMM_WAIT)
				{
					printf("Jetson closed or Communication timeout:\n");
					commTickTime = HAL_GetTick();
					lampAction(lampDefaultValue);
				}
			}
		}

		if ((HAL_GetTick() - flashTickTime) > FLASH_WAIT) {
			flashTickTime = HAL_GetTick();
			flashLamp();

			uartRxRestart();
		}

		if (commandReceived == 1) {
			commandReceived = 0;
			performCommand();
			sendStatus();
			command = 0;
			commandStatus = 0;
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

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
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SYSMCU_LED_GPIO_Port, SYSMCU_LED_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE,
			O5_3_Pin | O4_3_Pin | O8_5_Pin | O7_5_Pin | O6_5_Pin | O5_5_Pin
					| O2_1_Pin | O1_1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	O3_5_Pin | O2_5_Pin | O1_5_Pin | O6_1_Pin | O5_1_Pin | O4_1_Pin | O3_1_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD,
			O8_4_Pin | O7_4_Pin | O6_4_Pin | O5_4_Pin | O4_4_Pin | O3_4_Pin
					| O6_2_Pin | O5_2_Pin | O4_2_Pin | O3_2_Pin | O2_2_Pin
					| O1_2_Pin | O8_1_Pin | O7_1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
	O2_4_Pin | O1_4_Pin | O8_3_Pin | O7_3_Pin | O1_3_Pin | O8_2_Pin | O7_2_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, O6_3_Pin | FAKE_Pin | O2_3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(O3_3_GPIO_Port, O3_3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : JETSON_WKP_IN_Pin */
	GPIO_InitStruct.Pin = JETSON_WKP_IN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(JETSON_WKP_IN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SYSMCU_LED_Pin O6_3_Pin FAKE_Pin O2_3_Pin */
	GPIO_InitStruct.Pin = SYSMCU_LED_Pin | O6_3_Pin | FAKE_Pin | O2_3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : IN1_BTN_Pin IN2_BTN_Pin O4_5_Pin */
	GPIO_InitStruct.Pin = IN1_BTN_Pin | IN2_BTN_Pin | O4_5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : O5_3_Pin O4_3_Pin O8_5_Pin O7_5_Pin
	 O6_5_Pin O5_5_Pin O2_1_Pin O1_1_Pin */
	GPIO_InitStruct.Pin = O5_3_Pin | O4_3_Pin | O8_5_Pin | O7_5_Pin | O6_5_Pin
			| O5_5_Pin | O2_1_Pin | O1_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : O3_5_Pin O2_5_Pin O1_5_Pin O6_1_Pin
	 O5_1_Pin O4_1_Pin O3_1_Pin */
	GPIO_InitStruct.Pin = O3_5_Pin | O2_5_Pin | O1_5_Pin | O6_1_Pin | O5_1_Pin
			| O4_1_Pin | O3_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : O8_4_Pin O7_4_Pin O6_4_Pin O5_4_Pin
	 O4_4_Pin O3_4_Pin O6_2_Pin O5_2_Pin
	 O4_2_Pin O3_2_Pin O2_2_Pin O1_2_Pin
	 O8_1_Pin O7_1_Pin */
	GPIO_InitStruct.Pin = O8_4_Pin | O7_4_Pin | O6_4_Pin | O5_4_Pin | O4_4_Pin
			| O3_4_Pin | O6_2_Pin | O5_2_Pin | O4_2_Pin | O3_2_Pin | O2_2_Pin
			| O1_2_Pin | O8_1_Pin | O7_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : O2_4_Pin O1_4_Pin O8_3_Pin O7_3_Pin
	 O1_3_Pin O8_2_Pin O7_2_Pin */
	GPIO_InitStruct.Pin = O2_4_Pin | O1_4_Pin | O8_3_Pin | O7_3_Pin | O1_3_Pin
			| O8_2_Pin | O7_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : O3_3_Pin */
	GPIO_InitStruct.Pin = O3_3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(O3_3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void uartRxRestart() {
	if(uartStatus != HAL_OK) //Restart the Uart Interrupt
	{
		HAL_UART_AbortReceive_IT(&huart1);
		uartStatus = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1RxBuff, RXBUFFER_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}
}


void checkButtonStatus() {
	btnStatus = 0;
	bitWrite(btnStatus, 0, HAL_GPIO_ReadPin(IN1_BTN_GPIO_Port, IN1_BTN_Pin));
	bitWrite(btnStatus, 1, HAL_GPIO_ReadPin(IN2_BTN_GPIO_Port, IN2_BTN_Pin));
	jetsonWakeup = HAL_GPIO_ReadPin(JETSON_WKP_IN_GPIO_Port, JETSON_WKP_IN_Pin);
}

void sendStatus() {
	memset(txBuffer, 0, sizeof(txBuffer));  //Zero fill the values
	txBuffer[0] = 0xFB;	          //StartByte1 0xFB
	txBuffer[1] = 0x2C; 	      //StartByte2 0x2C
	txBuffer[2] = command;        //Command
	txBuffer[3] = commandStatus;  //Command status
	txBuffer[4] = buttonStatus;
	txBuffer[5] = checksum(&txBuffer[2], 3);   //Checksum calculate for Bytes2-4
	HAL_UART_Transmit(&huart1, txBuffer, 6, 50); //Transmit TX Buffer buffer_size=6 byte
}

void performCommand() {
	uint8_t *rxBuffer = uart1RxBuff;

	command = rxBuffer[2];
	commandStatus = 0;
	uint8_t checkByte = checksum(&rxBuffer[2], 45);

	if (rxBuffer[0] == 0xFB && rxBuffer[1] == 0x2C && checkByte == rxBuffer[47]) //Also add checkByte condition
	{
		commandStatus = 1;
		if (command == 11)       //11: Perform Lamp Operations
		{
			lampAction(&rxBuffer[3]);
		}
		if (command == 12)       //12: Save Default Lamp Operations
		{
			commandStatus = writeEpromLampDefault(&rxBuffer[3]);
		}
		if (command == 13)
		{
			//Ping command
		}
	}
}

void flashLamp() {
	printf("==>flashLamp:\n");
	for (int i = 0; i < flashSize; ++i) {
		HAL_GPIO_TogglePin(flash[i].port, flash[i].pin);
	}
}


void lampAction(uint8_t *lampValue) {
	uint16_t flashIndex = 0;
	printf("==>lampAction:\n");
	for (int i = 0; i < LAMP_SIZE; ++i) {
		printf(" l%d = %d, ",i,lampValue[i]);
		if(lampValue[i] == 0 || lampValue[i] == 1) {
			HAL_GPIO_WritePin(lamp[i].port, lamp[i].pin, lampValue[i]);
		} else if (lampValue[i] == 2) {
			HAL_GPIO_WritePin(lamp[i].port, lamp[i].pin, 0);  //Close lamp
			flash[flashIndex] = lamp[i];
			flashIndex++;
		}
	}
	flashSize = flashIndex;
	printf("\n flashSize =  %d",flashSize);
}

void closeAllLamp() {
	printf("==>closeAllLamp:\n");
	for (int i = 0; i < LAMP_SIZE; ++i) {
		printf(" l%d = %d, ",i, 0);
		HAL_GPIO_WritePin(lamp[i].port, lamp[i].pin, 0);
	}
}

uint8_t checksum(uint8_t *source, uint16_t size) {
	uint8_t result = 0;
	for (int i = 0; i < size; ++i) {
		result = result ^ source[i];
	}
	return result;
}

void uartPrint(UART_HandleTypeDef *huart, char *message) {
	HAL_UART_Transmit(huart, (uint8_t*) message, (uint16_t) strlen(message), 100);
}

void debug(char *message) {
	uartPrint(&huart3, message);
}

int __io_putchar(int ch) //Used for printf
{
  while (HAL_OK != HAL_UART_Transmit(&huart1, (uint8_t *) &ch, 1, 50))
  {
    ;
  }
  return ch;
}

//Using printf function on Debugger SWO pin
int _write(int32_t file, uint8_t *ptr, int32_t len) {
	for (int i = 0; i < len; i++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
