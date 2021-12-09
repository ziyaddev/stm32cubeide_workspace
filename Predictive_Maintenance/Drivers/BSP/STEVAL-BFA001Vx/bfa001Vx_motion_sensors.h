/**
  ******************************************************************************
  * @file    bfa001Vx_motion_sensors.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 1.0.0
  * @date    12 November 2020
  * @brief   This file provides a set of functions needed
  *          to manage the motion sensors
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
#ifndef __BFA001Vx_MOTION_SENSORS_H
#define __BFA001Vx_MOTION_SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bfa001Vx_conf.h"
#include "motion_sensor.h"

#ifndef USE_MOTION_SENSOR_IIS3DWB_0
#define USE_MOTION_SENSOR_IIS3DWB_0     1U
#endif
  
#ifndef USE_MOTION_SENSOR_ISM330DLC_0
#define USE_MOTION_SENSOR_ISM330DLC_0   1U
#endif
  
#if (USE_MOTION_SENSOR_IIS3DWB_0 == 1U)
#include "iis3dwb.h"
#endif  
  
#if (USE_MOTION_SENSOR_IIS3DWB_0 == 1U)
#define BFA001Vx_IIS3DWB_0 (0)
#endif
  
#if (USE_MOTION_SENSOR_ISM330DLC_0 == 1U)
#include "ism330dlc.h"
#endif  
  
#if (USE_MOTION_SENSOR_ISM330DLC_0 == 1U)
#define BFA001Vx_ISM330DLC_0 (USE_MOTION_SENSOR_IIS3DWB_0)
#endif  

/** @addtogroup BSP
 * @{
 */

/** @addtogroup STEVAL-BFA001Vx
 * @{
 */

/** @addtogroup STEVAL-BFA001Vx_MOTION_SENSOR STEVAL-BFA001Vx MOTION SENSOR
 * @{
 */

/** @defgroup STEVAL-BFA001Vx_MOTION_SENSOR_Exported_Types STEVAL-BFA001Vx MOTION SENSOR Exported Types
 * @{
 */

typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} BSP_MOTION_SENSOR_Axes_t;

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} BSP_MOTION_SENSOR_AxesRaw_t;

/* Motion Sensor instance Info */
typedef struct
{
  uint8_t  Acc;
  uint8_t  Gyro;
  uint8_t  Magneto;
  uint8_t  LowPower;
  uint32_t GyroMaxFS;
  uint32_t AccMaxFS;
  uint32_t MagMaxFS;
  float    GyroMaxOdr;
  float    AccMaxOdr;
  float    MagMaxOdr;
} MOTION_SENSOR_Capabilities_t;

typedef struct
{
  uint32_t Functions;
} MOTION_SENSOR_Ctx_t;

/**
 * @}
 */

/** @defgroup STEVAL-BFA001Vx_MOTION_SENSOR_Exported_Constants STEVAL-BFA001Vx MOTION SENSOR Exported Constants
 * @{
 */

#ifndef MOTION_GYRO
#define MOTION_GYRO             1U
#endif
#ifndef MOTION_ACCELERO
#define MOTION_ACCELERO         2U
#endif

#define MOTION_FUNCTIONS_NBR    2U
#define MOTION_INSTANCES_NBR    (USE_MOTION_SENSOR_IIS3DWB_0+USE_MOTION_SENSOR_ISM330DLC_0)

#if (MOTION_INSTANCES_NBR == 0)
#error "No motion sensor instance has been selected"
#endif

/**
 * @}
 */

/** @addtogroup STEVAL-BFA001Vx_MOTION_SENSOR_Exported_Functions STEVAL-BFA001Vx MOTION SENSOR Exported Functions
 * @{
 */

int32_t BSP_MOTION_SENSOR_Init(uint32_t Instance, uint32_t Functions);
int32_t BSP_MOTION_SENSOR_DeInit(uint32_t Instance);
int32_t BSP_MOTION_SENSOR_GetCapabilities(uint32_t Instance, MOTION_SENSOR_Capabilities_t *Capabilities);
int32_t BSP_MOTION_SENSOR_ReadID(uint32_t Instance, uint8_t *Id);
int32_t BSP_MOTION_SENSOR_Enable(uint32_t Instance, uint32_t Function);
int32_t BSP_MOTION_SENSOR_Disable(uint32_t Instance, uint32_t Function);
int32_t BSP_MOTION_SENSOR_GetAxes(uint32_t Instance, uint32_t Function, BSP_MOTION_SENSOR_Axes_t *Axes);
int32_t BSP_MOTION_SENSOR_GetAxesRaw(uint32_t Instance, uint32_t Function, BSP_MOTION_SENSOR_AxesRaw_t *Axes);
int32_t BSP_MOTION_SENSOR_GetSensitivity(uint32_t Instance, uint32_t Function, float *Sensitivity);
int32_t BSP_MOTION_SENSOR_GetOutputDataRate(uint32_t Instance, uint32_t Function, float *Odr);
int32_t BSP_MOTION_SENSOR_SetOutputDataRate(uint32_t Instance, uint32_t Function, float Odr);
int32_t BSP_MOTION_SENSOR_GetFullScale(uint32_t Instance, uint32_t Function, int32_t *Fullscale);
int32_t BSP_MOTION_SENSOR_SetFullScale(uint32_t Instance, uint32_t Function, int32_t Fullscale);

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

#endif /* __BFA001Vx_MOTION_SENSORS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
