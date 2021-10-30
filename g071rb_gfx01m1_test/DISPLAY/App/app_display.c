/**
  ******************************************************************************
  * File Name          : app_display.c
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_display.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include "mem_io.h"
#include "lcd_io.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static uint8_t TransferAllowed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * Initialize DISPLAY application
 */
void MX_DISPLAY_Init(void)
{
  /* USER CODE BEGIN MX_DISPLAY_Init 0 */

  /* USER CODE END MX_DISPLAY_Init 0 */
  if(BSP_MEM_Init(0) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  if(BSP_LCD_Init(0, 0) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  if(BSP_LCD_DisplayOn(0) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN MX_DISPLAY_Init 1 */

  /* USER CODE END MX_DISPLAY_Init 1 */
}

/**
 * DISPLAY application entry function
 */
void MX_DISPLAY_Process(void)
{
  /* USER CODE BEGIN MX_DISPLAY_Process 0 */

  /* USER CODE END MX_DISPLAY_Process 0 */
  /* Wait for TE */
  while(TransferAllowed == 0);
  TransferAllowed = 0;
  /* Display something */
  /* USER CODE BEGIN MX_DISPLAY_Process 1 */

  /* USER CODE END MX_DISPLAY_Process 1 */
}

/**
 * DISPLAY application task
 */
void DISPLAY_Task(void *argument)
{
  /* USER CODE BEGIN DISPLAY_Task 0 */

  /* USER CODE END DISPLAY_Task 0 */
  /* Wait for TE */
  while(TransferAllowed == 0);
  TransferAllowed = 0;
  /* Display something */
  /* USER CODE BEGIN DISPLAY_Task 1 */

  /* USER CODE END DISPLAY_Task 1 */
}

/* USER CODE BEGIN 1 */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  if(GPIO_Pin == LCD_TE_GPIO_PIN)
  {
    TransferAllowed = 1;
  }
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Rising_Callback could be implemented in the user file
   */
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
