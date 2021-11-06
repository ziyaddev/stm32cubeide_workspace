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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32l4xx_hal.h"

#include "lcd_io.h"
#include "mem_io.h"
#include "key_io.h"
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
#define BUTTON_USER_Pin GPIO_PIN_13
#define BUTTON_USER_GPIO_Port GPIOC
#define SPIB_MISO_Pin GPIO_PIN_2
#define SPIB_MISO_GPIO_Port GPIOC
#define SPIB_MOSI_Pin GPIO_PIN_3
#define SPIB_MOSI_GPIO_Port GPIOC
#define DISP_TE_Pin GPIO_PIN_0
#define DISP_TE_GPIO_Port GPIOA
#define DISP_TE_EXTI_IRQn EXTI0_IRQn
#define DISP_NRESET_Pin GPIO_PIN_1
#define DISP_NRESET_GPIO_Port GPIOA
#define SPIA_SCK_Pin GPIO_PIN_5
#define SPIA_SCK_GPIO_Port GPIOA
#define SPIA_MISO_Pin GPIO_PIN_6
#define SPIA_MISO_GPIO_Port GPIOA
#define SPIA_MOSI_Pin GPIO_PIN_7
#define SPIA_MOSI_GPIO_Port GPIOA
#define SPIB_SCK_Pin GPIO_PIN_13
#define SPIB_SCK_GPIO_Port GPIOB
#define JOY_CENTER_Pin GPIO_PIN_8
#define JOY_CENTER_GPIO_Port GPIOC
#define JOY_LEFT_Pin GPIO_PIN_9
#define JOY_LEFT_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define JOY_DOWN_Pin GPIO_PIN_10
#define JOY_DOWN_GPIO_Port GPIOC
#define JOY_RIGHT_Pin GPIO_PIN_11
#define JOY_RIGHT_GPIO_Port GPIOC
#define JOY_UP_Pin GPIO_PIN_12
#define JOY_UP_GPIO_Port GPIOC
#define SPIA_DCX_Pin GPIO_PIN_3
#define SPIA_DCX_GPIO_Port GPIOB
#define SPIA_NCS_Pin GPIO_PIN_5
#define SPIA_NCS_GPIO_Port GPIOB
#define SPIB_NCS_Pin GPIO_PIN_9
#define SPIB_NCS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
