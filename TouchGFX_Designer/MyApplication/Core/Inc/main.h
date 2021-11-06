/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#define BUTTON_RIGHT_Pin GPIO_PIN_11
#define BUTTON_RIGHT_GPIO_Port GPIOC
#define BUTTON_UP_Pin GPIO_PIN_12
#define BUTTON_UP_GPIO_Port GPIOC
#define BUTTON_USER_Pin GPIO_PIN_13
#define BUTTON_USER_GPIO_Port GPIOC
#define DISPLAY_RESET_Pin GPIO_PIN_1
#define DISPLAY_RESET_GPIO_Port GPIOA
#define BUTTON_CENTER_Pin GPIO_PIN_8
#define BUTTON_CENTER_GPIO_Port GPIOC
#define BUTTON_LEFT_Pin GPIO_PIN_9
#define BUTTON_LEFT_GPIO_Port GPIOC
#define DISPLAY_DCX_Pin GPIO_PIN_3
#define DISPLAY_DCX_GPIO_Port GPIOB
#define DISPLAY_CSX_Pin GPIO_PIN_5
#define DISPLAY_CSX_GPIO_Port GPIOB
#define FLASH_CS_Pin GPIO_PIN_9
#define FLASH_CS_GPIO_Port GPIOB
#define BUTTON_DOWN_Pin GPIO_PIN_10
#define BUTTON_DOWN_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
