/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define y_n_Pin GPIO_PIN_0
#define y_n_GPIO_Port GPIOC
#define SD_pin_Pin GPIO_PIN_3
#define SD_pin_GPIO_Port GPIOA
#define HALL_C_Pin GPIO_PIN_5
#define HALL_C_GPIO_Port GPIOA
#define HALL_C_EXTI_IRQn EXTI9_5_IRQn
#define HALL_B_Pin GPIO_PIN_6
#define HALL_B_GPIO_Port GPIOA
#define HALL_B_EXTI_IRQn EXTI9_5_IRQn
#define HALL_A_Pin GPIO_PIN_7
#define HALL_A_GPIO_Port GPIOA
#define HALL_A_EXTI_IRQn EXTI9_5_IRQn
#define SD_Pin GPIO_PIN_14
#define SD_GPIO_Port GPIOB
#define SDB15_Pin GPIO_PIN_15
#define SDB15_GPIO_Port GPIOB
#define low_Pin GPIO_PIN_6
#define low_GPIO_Port GPIOC
#define lowC7_Pin GPIO_PIN_7
#define lowC7_GPIO_Port GPIOC
#define lowC8_Pin GPIO_PIN_8
#define lowC8_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
