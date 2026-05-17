/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

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
#define LTC6820_CS_Pin GPIO_PIN_4
#define LTC6820_CS_GPIO_Port GPIOA
#define AMS_OK_Pin GPIO_PIN_4
#define AMS_OK_GPIO_Port GPIOB
#define TSMS_Pin GPIO_PIN_9
#define TSMS_GPIO_Port GPIOF
#define RST_PIL_Pin GPIO_PIN_10
#define RST_PIL_GPIO_Port GPIOF
#define RELAY_AIR_P_Pin GPIO_PIN_5
#define RELAY_AIR_P_GPIO_Port GPIOB
#define RELAY_AIR_N_Pin GPIO_PIN_6
#define RELAY_AIR_N_GPIO_Port GPIOB
#define RELAY_PRECHARGE_Pin GPIO_PIN_7
#define RELAY_PRECHARGE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
