/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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

#include "MotionSP_Manager.h"

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

/** @defgroup PREDCTIVE_MAINTENANCE_MAIN_EXPORTED_FUNCTIONS_PROTOTYPES Predictive Maintenance Main Exported Functions Prototypes
  * @{
  */

/* Exported functions ------------------------------------------------------- */

extern unsigned char SaveVibrationParamToMemory(void);

/** @defgroup PREDCTIVE_MAINTENANCE_MAIN_EXPORTED_VARIABLES Predictive Maintenance Main Exported Variables
  * @{
  */

/* Exported Variables ------------------------------------------------------- */
extern volatile uint32_t HCI_ProcessEvent;
extern volatile uint8_t FifoEnabled;

extern TIM_HandleTypeDef TimCCHandle;

extern uint32_t uhCCR1_Val;
extern uint32_t uhCCR2_Val;
extern uint32_t uhCCR3_Val;

extern uint8_t EnvironmentalTimerEnabled;
extern uint8_t AudioLevelTimerEnabled;
extern uint8_t InertialTimerEnabled;

extern uint8_t AudioLevelEnable;

extern uint8_t NodeName[];

extern float RMS_Ch[];
extern float DBNOISE_Value_Old_Ch[];

extern volatile uint32_t PredictiveMaintenance;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define IIS3DWB_CS_Pin GPIO_PIN_0
#define IIS3DWB_CS_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/** @defgroup PREDCTIVE_MAINTENANCE_MAIN_EXPORTED_DEFINES Predictive Maintenance Main Exported Defines
  * @{
  */

/* Exported defines --------------------------------------------------------- */

    /* Update frequency for environmental sensor [Hz] */
#define ALGO_FREQ_ENV   2U
/* Update period for environmental sensor [ms] */
#define ALGO_PERIOD_ENV (1000U / ALGO_FREQ_ENV)
/* 10kHz/2  for environmental @2Hz */
#define DEFAULT_uhCCR1_Val      (10000U / ALGO_FREQ_ENV)

/* Update frequency for mic audio level [Hz] */
#define ALGO_FREQ_AUDIO_LEVEL   20U
/* Update period for mic audio level [ms] */
#define ALGO_PERIOD_AUDIO_LEVEL (1000U / ALGO_FREQ_AUDIO_LEVEL)
/* 10kHz/20  for mic audio level @20Hz */
#define DEFAULT_uhCCR2_Val      (10000U / ALGO_FREQ_AUDIO_LEVEL)

/* Update frequency for Acc/Gyro/Mag sensor [Hz] */
#define FREQ_ACC_GYRO_MAG               20U
/* Update period for Acc/Gyro/Mag [ms] */
#define ALGO_PERIOD_ACC_GYRO_MAG        (1000U / FREQ_ACC_GYRO_MAG)
/* 10kHz/20  for Acc/Gyro/Mag @20Hz */
#define DEFAULT_uhCCR3_Val              (10000U / FREQ_ACC_GYRO_MAG)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
