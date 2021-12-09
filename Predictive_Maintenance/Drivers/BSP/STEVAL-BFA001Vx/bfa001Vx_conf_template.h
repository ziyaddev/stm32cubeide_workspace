/**
  ******************************************************************************
  * @file    bfa001Vx_conf_template.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 1.0.0
  * @date    12 November 2020
  * @brief   bfa001Vx configuration template file.
  *          This file should be copied to the application folder and renamed
  *          to bfa001Vx_conf.h.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BFA001Vx_CONF_H
#define __BFA001Vx_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "bfa001Vx.h"
#include "bfa001Vx_bus.h"
#include "bfa001Vx_errno.h"

/** @addtogroup Projects
  * @{
  */

/** @addtogroup DEMONSTRATIONS Demonstrations
  * @{
  */

/** @addtogroup PREDICTIVE_MAINTENANCE_IOL Predictive Maintenance IOL
  * @{
  */

/** @addtogroup BFA001Vx_CONF BFA001Vx Configuration Define
  * @{
  */
 
/* ########################## HSE/HSI Values adaptation ##################### */
/**
  * @brief Adjust the value of External High Speed oscillator (HSE) used in your application.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSE is used as system clock source, directly or through the PLL).  
  */
#if !defined  (HSE_VALUE) 
#define HSE_VALUE    (24000000U) /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#define USE_MOTION_SENSOR_IIS3DWB_0     1U /* 1 to use it. It is mounted on BFA001V2 */ 
#define USE_MOTION_SENSOR_ISM330DLC_0   0U /* 1 to use it. It is mounted on BFA001V1 */
#define USE_ENV_SENSOR_HTS221_0         1U /* 1 to use it */
#define USE_ENV_SENSOR_LPS22HB_0        1U /* 1 to use it */


#define IIS3DWB_SPI_MAX_CLOCK                   10000000 /* in MHz */
#define IIS3DWB_SPI_IRQ_PP                      2
#define IIS3DWB_SPI_IRQ_SP                      0
#define IIS3DWB_SPI_RX_DMA_Stream_IRQ_PP        2
#define IIS3DWB_SPI_RX_DMA_Stream_IRQ_SP        0
#define IIS3DWB_SPI_TX_DMA_Stream_IRQ_PP        2
#define IIS3DWB_SPI_TX_DMA_Stream_IRQ_SP        0

#define ISM330DLC_SPI_MAX_CLOCK                 10000000 /* in MHz */
#define ISM330DLC_SPI_IRQ_PP                    2
#define ISM330DLC_SPI_IRQ_SP                    0
#define ISM330DLC_SPI_RX_DMA_Stream_IRQ_PP      2
#define ISM330DLC_SPI_RX_DMA_Stream_IRQ_SP      0  
#define ISM330DLC_SPI_TX_DMA_Stream_IRQ_PP      2
#define ISM330DLC_SPI_TX_DMA_Stream_IRQ_SP      0    
  
/****** MANDATORY for the BFA001V2 STACK functionality ****/
#define IIS3DWB_INT1_EXTI_IRQ_PP          2
#define IIS3DWB_INT1_EXTI_IRQ_SP          0
#define IIS3DWB_INT2_EXTI_IRQ_PP          2
#define IIS3DWB_INT2_EXTI_IRQ_SP          0  
  
/****** MANDATORY for the BFA001V1 STACK functionality ****/
#define ISM330DLC_INT1_EXTI_IRQ_PP        2
#define ISM330DLC_INT1_EXTI_IRQ_SP        0
#define ISM330DLC_INT2_EXTI_IRQ_PP        2
#define ISM330DLC_INT2_EXTI_IRQ_SP        0  
  

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __BFA001Vx_CONF_H*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
