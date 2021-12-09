/**
  ******************************************************************************
  * @file    bfa001Vx.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 1.0.0
  * @date    12 November 2020
  * @brief   This file provides the IO driver for the BFA001Vx
  *          board.
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

/* Includes ------------------------------------------------------------------*/
#include "bfa001Vx.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STEVAL-BFA001Vx
  * @{
  */ 

/** @addtogroup STEVAL-BFA001Vx_Private_Defines STEVAL-BFA001Vx Private Defines
  * @{
  */ 

/**
  * @brief STEVAL BSP Driver version number
  */
#define __STEVAL_BFA001Vx_V1_BOARD_BSP_VERSION_MAJOR   (0x00) //!< [31:24] major version
#define __STEVAL_BFA001Vx_V1_BOARD_BSP_VERSION_MINOR   (0x00) //!< [23:16] minor version
#define __STEVAL_BFA001Vx_V1_BOARD_BSP_VERSION_REVISON (0x01) //!< [15:8]  revision version
#define __STEVAL_BFA001Vx_V1_BOARD_BSP_VERSION_RC      (0x00) //!< [7:0]   release candidate
#define __STEVAL_BFA001Vx_V1_BOARD_BSP_VERSION         ((__STEVAL_BFA001Vx_V1_BOARD_BSP_VERSION_MAJOR << 24)\
                                                    |(__STEVAL_BFA001Vx_V1_BOARD_BSP_VERSION_MINOR << 16)\
                                                    |(__STEVAL_BFA001Vx_V1_BOARD_BSP_VERSION_REVISON << 8 )\
                                                    |(__STEVAL_BFA001Vx_V1_BOARD_BSP_VERSION_RC))

/**
  * @}
  */ 

/** @addtogroup STEVAL-BFA001Vx_Private_Variables STEVAL-BFA001Vx Private variables
  * @{
  */

GPIO_TypeDef* GPIO_PORT[LEDn] = {USER_LED_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {USER_LED_PIN};

/**
  * @}
  */ 
    
/** @addtogroup STEVAL-BFA001Vx_DATA_COMM STEVAL-BFA001Vx Data Communication
 * @{
 */

/** @addtogroup STEVAL-BFA001Vx_DATA_COMM_Public_Functions STEVAL-BFA001Vx Data Communication Public Functions
  * @{
  */

/** @brief  Enable/Diag GPIO Port driving 
  * @param  status Pin Status
  */
void BSP_L6362A_ENDIAG(uint8_t status)
{
  if(status==0)
    HAL_GPIO_WritePin(L6362A_EN_DIAG_GPIO_PORT, L6362A_EN_DIAG_PIN, GPIO_PIN_RESET); 
  else if(status==1)
    HAL_GPIO_WritePin(L6362A_EN_DIAG_GPIO_PORT, L6362A_EN_DIAG_PIN, GPIO_PIN_SET);
}


/**
 * @}
 */

/**
 * @}
 */
 
 
/** @defgroup STEVAL-BFA001Vx_Private_Functions STEVAL-BFA001Vx Private Functions
  * @{
  */ 

/**
  * @brief  This method returns the STM32F4xx NUCLEO BSP Driver revision
  * @retval version: 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t BSP_GetVersion(void)
{
  return __STEVAL_BFA001Vx_V1_BOARD_BSP_VERSION;
}

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured. 
  *   This parameter can be one of following parameters:
  *     @arg USER_LED
  */
void BSP_LED_Init(BoardLed_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable the GPIO_LED Clock */
  LEDx_GPIO_CLK_ENABLE(Led);
  
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET); 

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIO_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
  HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);
}

/**
  * @brief  DeInit LEDs.
  * @param  Led: LED to be de-init. 
  *   This parameter can be one of the following values:
  *     @arg  USER_LED
  * @note Led DeInit does not disable the GPIO clock nor disable the Mfx 
  */
void BSP_LED_DeInit(BoardLed_TypeDef Led)
{
  GPIO_InitTypeDef  gpio_init_structure;

  /* Turn off LED */
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
  /* DeInit the GPIO_LED pin */
  gpio_init_structure.Pin = GPIO_PIN[Led];
  HAL_GPIO_DeInit(GPIO_PORT[Led], gpio_init_structure.Pin);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *     @arg USER_LED
  */
void BSP_LED_On(BoardLed_TypeDef Led)
{
  HAL_GPIO_WritePin(USER_LED_GPIO_PORT, USER_LED_PIN, GPIO_PIN_SET); 
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg USER_LED
  */
void BSP_LED_Off(BoardLed_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET); 
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *     @arg USER_LED  
  */
void BSP_LED_Toggle(BoardLed_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}

/**
 * @}
 */

/** @addtogroup STEVAL-BFA001Vx_IO STEVAL-BFA001Vx IO
  * @{
  */ 


/**
  * @brief  Configures the I/Os for the HTS221 relative humidity and temperature sensor.
  * @retval COMPONENT_OK in case of success
  * @retval COMPONENT_ERROR in case of failure
*/
void L6362A_IO_Init(uint8_t IO_Link_TX)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  L6362A_EN_DIAG_GPIO_CLK_DISABLE();
  L6362A_OL_GPIO_CLK_DISABLE();
  
  /* Configure GPIO pin : L6362A*/
  if (IO_Link_TX == 1)
  {
    L6362A_EN_DIAG_GPIO_CLK_ENABLE();
    L6362A_OL_GPIO_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = L6362A_EN_DIAG_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(L6362A_EN_DIAG_GPIO_PORT, &GPIO_InitStruct); 
    
    GPIO_InitStruct.Pin = L6362A_OL_PIN;
    HAL_GPIO_Init(L6362A_OL_GPIO_PORT, &GPIO_InitStruct);  
    
    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(L6362A_EXTI_IRQn, L6362A_EXTI_IRQ_PP, L6362A_EXTI_IRQ_SP);
    HAL_NVIC_EnableIRQ(L6362A_EXTI_IRQn);
    while(HAL_GPIO_ReadPin(L6362A_EN_DIAG_GPIO_PORT,L6362A_EN_DIAG_PIN)!=1);
  }
  else if (IO_Link_TX == 0)
  {  
    HAL_NVIC_DisableIRQ(L6362A_EXTI_IRQn);
    
    L6362A_EN_DIAG_GPIO_CLK_ENABLE();
    L6362A_OL_GPIO_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = L6362A_EN_DIAG_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(L6362A_EN_DIAG_GPIO_PORT, &GPIO_InitStruct);
    while(HAL_GPIO_ReadPin(L6362A_EN_DIAG_GPIO_PORT,L6362A_EN_DIAG_PIN)!=0);
  }  
}

void L6362A_IO_DeInit( void )
{
  HAL_GPIO_DeInit(L6362A_EN_DIAG_GPIO_PORT, L6362A_EN_DIAG_PIN);
  HAL_GPIO_DeInit(L6362A_OL_GPIO_PORT, L6362A_OL_PIN);
}


/**
  * @}
  */ 

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
