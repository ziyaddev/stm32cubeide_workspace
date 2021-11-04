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
#include "key_io.h"
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
  if(BSP_LCD_Init(0, LCD_ORIENTATION_PORTRAIT) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  if(BSP_KEY_Init(0, KEY_ORIENTATION_PORTRAIT) != BSP_ERROR_NONE)
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

  /* USER CODE BEGIN DISPLAY_Task 1 */

  /* USER CODE END DISPLAY_Task 1 */
}

void BSP_LCD_SignalTearingEffectEvent(uint32_t Instance, uint8_t state, uint16_t Line)
{
  if(Instance == 0)
  {
    /* USER CODE BEGIN BSP_LCD_SignalTearingEffectEvent */
    if(state)
    {
      /* Line '0' is the Vsync event */
      if(Line == 0)
      {
        /* TE event is received : allow display refresh */
      }
    }
    else
    {
      /* TE event is done : de-allow display refresh */
    }
    /* USER CODE END BSP_LCD_SignalTearingEffectEvent */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
