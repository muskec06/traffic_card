/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define JETSON_WKP_IN_Pin GPIO_PIN_0
#define JETSON_WKP_IN_GPIO_Port GPIOA
#define SYSMCU_LED_Pin GPIO_PIN_1
#define SYSMCU_LED_GPIO_Port GPIOA
#define IN1_BTN_Pin GPIO_PIN_0
#define IN1_BTN_GPIO_Port GPIOB
#define IN2_BTN_Pin GPIO_PIN_1
#define IN2_BTN_GPIO_Port GPIOB
#define O5_3_Pin GPIO_PIN_10
#define O5_3_GPIO_Port GPIOE
#define O4_3_Pin GPIO_PIN_11
#define O4_3_GPIO_Port GPIOE
#define O8_5_Pin GPIO_PIN_12
#define O8_5_GPIO_Port GPIOE
#define O7_5_Pin GPIO_PIN_13
#define O7_5_GPIO_Port GPIOE
#define O6_5_Pin GPIO_PIN_14
#define O6_5_GPIO_Port GPIOE
#define O5_5_Pin GPIO_PIN_15
#define O5_5_GPIO_Port GPIOE
#define O4_5_Pin GPIO_PIN_12
#define O4_5_GPIO_Port GPIOB
#define O3_5_Pin GPIO_PIN_13
#define O3_5_GPIO_Port GPIOB
#define O2_5_Pin GPIO_PIN_14
#define O2_5_GPIO_Port GPIOB
#define O1_5_Pin GPIO_PIN_15
#define O1_5_GPIO_Port GPIOB
#define O8_4_Pin GPIO_PIN_10
#define O8_4_GPIO_Port GPIOD
#define O7_4_Pin GPIO_PIN_11
#define O7_4_GPIO_Port GPIOD
#define O6_4_Pin GPIO_PIN_12
#define O6_4_GPIO_Port GPIOD
#define O5_4_Pin GPIO_PIN_13
#define O5_4_GPIO_Port GPIOD
#define O4_4_Pin GPIO_PIN_14
#define O4_4_GPIO_Port GPIOD
#define O3_4_Pin GPIO_PIN_15
#define O3_4_GPIO_Port GPIOD
#define O2_4_Pin GPIO_PIN_6
#define O2_4_GPIO_Port GPIOC
#define O1_4_Pin GPIO_PIN_7
#define O1_4_GPIO_Port GPIOC
#define O8_3_Pin GPIO_PIN_8
#define O8_3_GPIO_Port GPIOC
#define O7_3_Pin GPIO_PIN_9
#define O7_3_GPIO_Port GPIOC
#define O6_3_Pin GPIO_PIN_8
#define O6_3_GPIO_Port GPIOA
#define FAKE_Pin GPIO_PIN_13
#define FAKE_GPIO_Port GPIOA
#define O3_3_Pin GPIO_PIN_2
#define O3_3_GPIO_Port GPIOH
#define O2_3_Pin GPIO_PIN_15
#define O2_3_GPIO_Port GPIOA
#define O1_3_Pin GPIO_PIN_10
#define O1_3_GPIO_Port GPIOC
#define O8_2_Pin GPIO_PIN_11
#define O8_2_GPIO_Port GPIOC
#define O7_2_Pin GPIO_PIN_12
#define O7_2_GPIO_Port GPIOC
#define O6_2_Pin GPIO_PIN_0
#define O6_2_GPIO_Port GPIOD
#define O5_2_Pin GPIO_PIN_1
#define O5_2_GPIO_Port GPIOD
#define O4_2_Pin GPIO_PIN_2
#define O4_2_GPIO_Port GPIOD
#define O3_2_Pin GPIO_PIN_3
#define O3_2_GPIO_Port GPIOD
#define O2_2_Pin GPIO_PIN_4
#define O2_2_GPIO_Port GPIOD
#define O1_2_Pin GPIO_PIN_5
#define O1_2_GPIO_Port GPIOD
#define O8_1_Pin GPIO_PIN_6
#define O8_1_GPIO_Port GPIOD
#define O7_1_Pin GPIO_PIN_7
#define O7_1_GPIO_Port GPIOD
#define O6_1_Pin GPIO_PIN_4
#define O6_1_GPIO_Port GPIOB
#define O5_1_Pin GPIO_PIN_5
#define O5_1_GPIO_Port GPIOB
#define O4_1_Pin GPIO_PIN_8
#define O4_1_GPIO_Port GPIOB
#define O3_1_Pin GPIO_PIN_9
#define O3_1_GPIO_Port GPIOB
#define O2_1_Pin GPIO_PIN_0
#define O2_1_GPIO_Port GPIOE
#define O1_1_Pin GPIO_PIN_1
#define O1_1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
