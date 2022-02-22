/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

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
#define LED_F_Pin GPIO_PIN_0
#define LED_F_GPIO_Port GPIOF
#define LED_G_Pin GPIO_PIN_1
#define LED_G_GPIO_Port GPIOF
#define D4_Pin GPIO_PIN_1
#define D4_GPIO_Port GPIOC
#define LED_B_Pin GPIO_PIN_2
#define LED_B_GPIO_Port GPIOC
#define LED_A_Pin GPIO_PIN_3
#define LED_A_GPIO_Port GPIOC
#define LED_DP_Pin GPIO_PIN_5
#define LED_DP_GPIO_Port GPIOC
#define C1_Pin GPIO_PIN_14
#define C1_GPIO_Port GPIOB
#define C2_Pin GPIO_PIN_8
#define C2_GPIO_Port GPIOA
#define C3_Pin GPIO_PIN_9
#define C3_GPIO_Port GPIOA
#define C4_Pin GPIO_PIN_7
#define C4_GPIO_Port GPIOC
#define R4_Pin GPIO_PIN_10
#define R4_GPIO_Port GPIOA
#define D3_Pin GPIO_PIN_11
#define D3_GPIO_Port GPIOA
#define D2_Pin GPIO_PIN_12
#define D2_GPIO_Port GPIOA
#define LED_D_Pin GPIO_PIN_15
#define LED_D_GPIO_Port GPIOA
#define LED_E_Pin GPIO_PIN_4
#define LED_E_GPIO_Port GPIOD
#define D1_Pin GPIO_PIN_6
#define D1_GPIO_Port GPIOD
#define R3_Pin GPIO_PIN_3
#define R3_GPIO_Port GPIOB
#define R1_Pin GPIO_PIN_4
#define R1_GPIO_Port GPIOB
#define R2_Pin GPIO_PIN_5
#define R2_GPIO_Port GPIOB
#define LED_C_Pin GPIO_PIN_7
#define LED_C_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
