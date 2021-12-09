/**
  ******************************************************************************
  * @file    PREDMNT1_config.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version V2.4.0
  * @date    07-June-2021
  * @brief   FP-IND-PREDMNT1 configuration
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PREDMNT1_CONFIG_H
#define __PREDMNT1_CONFIG_H

/*************** Debug Defines ******************/
/* For enabling the printf on UART */
#define PREDMNT1_ENABLE_PRINTF

/* For enabling connection and notification subscriptions debug */
#define PREDMNT1_DEBUG_CONNECTION

/* For enabling trasmission for notified services */
#define PREDMNT1_DEBUG_NOTIFY_TRAMISSION

/*************** Don't Change the following defines *************/

/* Package Version only numbers 0->9 */
#define PREDMNT1_VERSION_MAJOR '2'
#define PREDMNT1_VERSION_MINOR '4'
#define PREDMNT1_VERSION_PATCH '0'

/* Define the ALLMEMS1 Name MUST be 7 char long */
#define NAME_BLUEMS 'P','M','1','V',PREDMNT1_VERSION_MAJOR,PREDMNT1_VERSION_MINOR,PREDMNT1_VERSION_PATCH

/* Package Name */
#define PREDMNT1_PACKAGENAME "FP-IND-PREDMNT1"
#define CONFIG_NAME "Application - Predictive Maintenance"

/* Environmental Sensor Istance */
#define TEMPERATURE_INSTANCE_1  IKS01A3_HTS221_0
#define HUMIDITY_INSTANCE       IKS01A3_HTS221_0
#define TEMPERATURE_INSTANCE_2  IKS01A3_LPS22HH_0
#define PRESSURE_INSTANCE       IKS01A3_LPS22HH_0

/* Motion Sensor Istance */
#include "iks01a3_motion_sensors_Patch.h"

#ifdef IKS01A3_ISM330DLC_0
  #define ACCELERO_INSTANCE     IKS01A3_ISM330DLC_0
  #define GYRO_INSTANCE         IKS01A3_ISM330DLC_0
#endif /* IKS01A3_ISM330DLC_0 */

#ifdef IKS01A3_LSM6DSO_0
  #define ACCELERO_INSTANCE     IKS01A3_LSM6DSO_0
  #define GYRO_INSTANCE         IKS01A3_LSM6DSO_0
#endif /* IKS01A3_LSM6DSO_0 */

#define MAGNETO_INSTANCE        IKS01A3_LIS2MDL_0

/* Environmental Sensor API */
#define ENV_SENSOR_Init         IKS01A3_ENV_SENSOR_Init
#define ENV_SENSOR_Enable       IKS01A3_ENV_SENSOR_Enable
#define ENV_SENSOR_GetValue     IKS01A3_ENV_SENSOR_GetValue

/* Motion Sensor API */
#define MOTION_SENSOR_Init                    IKS01A3_MOTION_SENSOR_Init
#define MOTION_SENSOR_Enable                  IKS01A3_MOTION_SENSOR_Enable

#define MOTION_SENSOR_AxesRaw_t               IKS01A3_MOTION_SENSOR_AxesRaw_t
#define MOTION_SENSOR_Axes_t                  IKS01A3_MOTION_SENSOR_Axes_t

#define MOTION_SENSOR_GetAxes                 IKS01A3_MOTION_SENSOR_GetAxes

#define MOTION_SENSOR_GetSensitivity          IKS01A3_MOTION_SENSOR_GetSensitivity
#define MOTION_SENSOR_SetFullScale            IKS01A3_MOTION_SENSOR_SetFullScale

#define MOTION_SENSOR_Write_Register          IKS01A3_MOTION_SENSOR_Write_Register

#define MOTION_SENSOR_SetOutputDataRate               IKS01A3_MOTION_SENSOR_SetOutputDataRate
#define MOTION_SENSOR_Enable_HP_Filter                IKS01A3_MOTION_SENSOR_Enable_HP_Filter
#define MOTION_SENSOR_Set_INT2_DRDY                   IKS01A3_MOTION_SENSOR_Set_INT2_DRDY
#define MOTION_SENSOR_DRDY_Set_Mode                   IKS01A3_MOTION_SENSOR_DRDY_Set_Mode

#define MOTION_SENSOR_FIFO_Set_Mode                   IKS01A3_MOTION_SENSOR_FIFO_Set_Mode
#define MOTION_SENSOR_FIFO_Set_INT2_FIFO_Full         IKS01A3_MOTION_SENSOR_FIFO_Set_INT2_FIFO_Full
#define MOTION_SENSOR_FIFO_Set_INT2_FIFO_TH           IKS01A3_MOTION_SENSOR_FIFO_Set_INT2_FIFO_TH
#define MOTION_SENSOR_FIFO_Read                       IKS01A3_MOTION_SENSOR_FIFO_Read
#define MOTION_SENSOR_FIFO_Get_Data_Word              IKS01A3_MOTION_SENSOR_FIFO_Get_Data_Word
#define MOTION_SENSOR_FIFO_Set_Decimation             IKS01A3_MOTION_SENSOR_FIFO_Set_Decimation
#define MOTION_SENSOR_FIFO_Set_ODR_Value              IKS01A3_MOTION_SENSOR_FIFO_Set_ODR_Value
#define MOTION_SENSOR_FIFO_Set_Watermark_Level        IKS01A3_MOTION_SENSOR_FIFO_Set_Watermark_Level
#define MOTION_SENSOR_FIFO_Set_Stop_On_Fth            IKS01A3_MOTION_SENSOR_FIFO_Set_Stop_On_Fth

#ifdef IKS01A3_ISM330DLC_0
  #define ACCELERO_FIFO_XL_NO_DEC     ISM330DLC_FIFO_XL_NO_DEC
  #define ACCELERO_FIFO_MODE          ISM330DLC_FIFO_MODE
  #define ACCELERO_DRDY_PULSED        ISM330DLC_DRDY_PULSED
  #define ACCELERO_DRDY_LATCHED       ISM330DLC_DRDY_LATCHED
#endif /* IKS01A3_ISM330DLC_0 */

//#ifdef IKS01A3_LSM6DSO_0
//  #define ACCELERO_FIFO_XL_NO_DEC       LSM6DSO_FIFO_XL_NO_DEC
//  #define ACCELERO_BYPASS_MODE          LSM6DSL_BYPASS_MODE
//  #define ACCELERO_FIFO_MODE            LSM6DSL_FIFO_MODE
//  #define ACCELERO_DRDY_PULSED          LSM6DSO_DRDY_PULSED
//  #define ACCELERO_DRDY_LATCHED         LSM6DSL_DRDY_LATCHED
//#endif /* IKS01A3_LSM6DSO_0

/*****************
* Sensor Setting *
******************/
#ifdef IKS01A3_ISM330DLC_0
  #define HPF_ODR_DIV_4           ISM330DLC_XL_HP_ODR_DIV_4   //!< ISM330DLC HPF Configuration  
  #define HPF_ODR_DIV_100         ISM330DLC_XL_HP_ODR_DIV_100 //!< ISM330DLC HPF Configuration 
  #define HPF_ODR_DIV_9           ISM330DLC_XL_HP_ODR_DIV_9   //!< ISM330DLC HPF Configuration  
  #define HPF_ODR_DIV_400         ISM330DLC_XL_HP_ODR_DIV_400 //!< ISM330DLC HPF Configuration
  #define HPF_NONE                ISM330DLC_XL_HP_NA          //!< HP Filter Disabling
#endif /* IKS01A3_ISM330DLC_0 */

//#ifdef IKS01A2_LSM6DSL_0
//  #define HPF_ODR_DIV_4           LSM6DSL_XL_HP_ODR_DIV_4   //!< ISM330DLC HPF Configuration  
//  #define HPF_ODR_DIV_100         LSM6DSL_XL_HP_ODR_DIV_100 //!< ISM330DLC HPF Configuration 
//  #define HPF_ODR_DIV_9           LSM6DSL_XL_HP_ODR_DIV_9   //!< ISM330DLC HPF Configuration  
//  #define HPF_ODR_DIV_400         LSM6DSO_XL_HP_ODR_DIV_400 //!< ISM330DLC HPF Configuration
//  #define HPF_NONE                LSM6DSL_XL_HP_NA          //!< HP Filter Disabling
//#endif /* IKS01A2_LSM6DSL_0

/*************************
* Serial control section *
**************************/
#ifdef PREDMNT1_ENABLE_PRINTF
  #define PREDMNT1_PRINTF(...) printf(__VA_ARGS__)
#else /* PREDMNT1_ENABLE_PRINTF */
  #define PREDMNT1_PRINTF(...)
#endif /* PREDMNT1_ENABLE_PRINTF */

#define PREDMNT1_SCANF(...) scanf(__VA_ARGS__)

/* STM32 Unique ID */
#define STM32_UUID ((uint32_t *)0x1FFF7A10)

/* STM32 MCU_ID */
#define STM32_MCU_ID ((uint32_t *)0xE0042000)
/* Control Section */

#endif /* __PREDMNT1_CONFIG_H */

/******************* (C) COPYRIGHT 2021 STMicroelectronics *****END OF FILE****/
