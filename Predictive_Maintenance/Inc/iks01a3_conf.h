/**
  ******************************************************************************
  * @file    iks01a3_conf.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version V2.4.0
  * @date    07-June-2021
  * @brief   IKS01A3 configuration template file.
  *          This file should be copied to the application folder and renamed
  *          to iks01a3_conf.h.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0055, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0055
  *
  ******************************************************************************
  */

/* Replace the header file names with the ones of the target platform */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo_bus.h"
#include "stm32f4xx_nucleo_errno.h"

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IKS01A3_CONF_H__
#define __IKS01A3_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif
  
/*
Setect I2C or SPI connection mode for DIL24 socket:
  0 --> I2C
  1 --> SPI
These hardware patch to be performed on the IKS01A3 to enable SPI support on DIL24:
  - Open solder bridges  SB5, SB12, SB19 and SB23
  - Close solder bridges SB6, SB10, SB18 and SB22
*/  
#define USE_SPI_FOR_DIL24 0U 
  
#define M_INT2_O_GPIO_PORT           GPIOA
#define M_INT2_O_GPIO_CLK_ENABLE()   __GPIOA_CLK_ENABLE()
#define M_INT2_O_GPIO_CLK_DISABLE()  __GPIOA_CLK_DISABLE()
#define M_INT2_O_PIN                 GPIO_PIN_10
#define M_INT2_O_EXTI_IRQn           EXTI15_10_IRQn

#if (USE_SPI_FOR_DIL24 == 1)
#define DIL24_SPI_CS_PORT       GPIOB
#define DIL24_SPI_CS_MODE       GPIO_MODE_OUTPUT_PP
#define DIL24_SPI_CS_PULL       GPIO_PULLUP //GPIO_NOPULL //
#define DIL24_SPI_CS_SPEED      GPIO_SPEED_HIGH
#define DIL24_SPI_CS_PIN        GPIO_PIN_6
#define DIL24_SPI_CS_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define DIL24_SPI_CS_CLK_DISABLE()     __HAL_RCC_GPIOB_CLK_DISABLE()
#endif /* USE_SPI_FOR_DIL24 */
  
/* USER CODE BEGIN 1 */
#define USE_IKS01A3_ENV_SENSOR_HTS221_0                1U
#define USE_IKS01A3_ENV_SENSOR_LPS22HH_0               1U
#define USE_IKS01A3_ENV_SENSOR_STTS751_0               0U

#define USE_IKS01A3_MOTION_SENSOR_LSM6DSO_0            0U
#define USE_IKS01A3_MOTION_SENSOR_LIS2DW12_0           0U
#define USE_IKS01A3_MOTION_SENSOR_LIS2MDL_0            1U
#define USE_IKS01A3_MOTION_SENSOR_ISM330DLC_0          1U
  
#if ((USE_IKS01A3_MOTION_SENSOR_LSM6DSO_0 + USE_IKS01A3_MOTION_SENSOR_ISM330DLC_0) > 1)
  #error Select only one between accelerometers lsm6dsl and ism330dlc 
#endif
/* USER CODE END 1 */

#define IKS01A3_I2C_Init        BSP_I2C1_Init
#define IKS01A3_I2C_DeInit      BSP_I2C1_DeInit
#define IKS01A3_I2C_ReadReg     BSP_I2C1_ReadReg
#define IKS01A3_I2C_WriteReg    BSP_I2C1_WriteReg
  
#if (USE_SPI_FOR_DIL24 == 1)
#define IKS01A3_ISM330DLC_0_SPI_Init     BSP_SPI1_Init
#define IKS01A3_ISM330DLC_0_SPI_DeInit   BSP_SPI1_DeInit
#define IKS01A3_ISM330DLC_0_SPI_Send     BSP_SPI1_Send
#define IKS01A3_ISM330DLC_0_SPI_Recv     BSP_SPI1_Recv
#endif /* USE_SPI_FOR_DIL24 */
  
#define IKS01A3_GetTick         BSP_GetTick

#ifdef __cplusplus
}
#endif

#endif /* __IKS01A3_CONF_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

