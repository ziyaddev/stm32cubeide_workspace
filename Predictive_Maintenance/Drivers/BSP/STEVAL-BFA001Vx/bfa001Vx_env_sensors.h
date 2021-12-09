/**
  ******************************************************************************
  * @file    bfa001Vx_env_sensors.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 1.0.0
  * @date    12 November 2020
  * @brief   This file provides a set of functions needed to manage the
  *          environmental sensors
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
#ifndef __BFA001Vx_ENV_SENSORS_H
#define __BFA001Vx_ENV_SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bfa001Vx_conf.h"
#include "env_sensor.h"

#ifndef USE_ENV_SENSOR_HTS221_0
#define USE_ENV_SENSOR_HTS221_0          1U
#endif

#ifndef USE_ENV_SENSOR_LPS22HB_0
#define USE_ENV_SENSOR_LPS22HB_0         1U
#endif

#if (USE_ENV_SENSOR_LPS22HB_0 == 1U)
#include "lps22hb.h"
#endif

#if (USE_ENV_SENSOR_HTS221_0 == 1U)
#include "hts221.h"
#endif

/** @addtogroup BSP
 * @{
 */

/** @addtogroup STEVAL-BFA001Vx
 * @{
 */

/** @addtogroup STEVAL-BFA001Vx_ENV_SENSORS STEVAL-BFA001Vx ENV SENSORS
 * @{
 */

/** @defgroup STEVAL-BFA001Vx_ENV_SENSORS_Exported_Types STEVAL-BFA001Vx ENV SENSORS Exported Types
 * @{
 */

/* Environmental Sensor instance Info */
typedef struct
{
  uint8_t Temperature;
  uint8_t Pressure;
  uint8_t Humidity;
  uint8_t LowPower;
  float   HumMaxOdr;
  float   TempMaxOdr;
  float   PressMaxOdr;
} BFA001Vx_ENV_SENSOR_Capabilities_t;

typedef struct
{
  uint32_t Functions;
} BFA001Vx_ENV_SENSOR_Ctx_t;

/**
 * @}
 */

/** @defgroup STEVAL-BFA001Vx_ENV_SENSOR_Exported_Constants STEVAL-BFA001Vx ENV SENSOR Exported Constants
 * @{
 */

#if (USE_ENV_SENSOR_HTS221_0 == 1)
#define BFA001Vx_HTS221_0 0
#endif

#if (USE_ENV_SENSOR_LPS22HB_0 == 1)
#define BFA001Vx_LPS22HB_0 (USE_ENV_SENSOR_HTS221_0)
#endif

#ifndef ENV_TEMPERATURE
#define ENV_TEMPERATURE      1U
#endif
#ifndef ENV_PRESSURE
#define ENV_PRESSURE         2U
#endif
#ifndef ENV_HUMIDITY
#define ENV_HUMIDITY         4U
#endif

#define BFA001Vx_ENV_FUNCTIONS_NBR    3U
#define BFA001Vx_ENV_INSTANCES_NBR    (USE_ENV_SENSOR_HTS221_0 + USE_ENV_SENSOR_LPS22HB_0)

#if (BFA001Vx_ENV_INSTANCES_NBR == 0)
#error "No environmental sensor instance has been selected"
#endif

/**
 * @}
 */

/** @addtogroup STEVAL-BFA001Vx_ENV_SENSORS_Exported_Functions STEVAL-BFA001Vx ENV SENSOR Exported Functions
 * @{
 */

int32_t BFA001Vx_ENV_SENSOR_Init(uint32_t Instance, uint32_t Functions);
int32_t BFA001Vx_ENV_SENSOR_DeInit(uint32_t Instance);
int32_t BFA001Vx_ENV_SENSOR_GetCapabilities(uint32_t Instance, BFA001Vx_ENV_SENSOR_Capabilities_t *Capabilities);
int32_t BFA001Vx_ENV_SENSOR_ReadID(uint32_t Instance, uint8_t *Id);
int32_t BFA001Vx_ENV_SENSOR_Enable(uint32_t Instance, uint32_t Function);
int32_t BFA001Vx_ENV_SENSOR_Disable(uint32_t Instance, uint32_t Function);
int32_t BFA001Vx_ENV_SENSOR_GetOutputDataRate(uint32_t Instance, uint32_t Function, float *Odr);
int32_t BFA001Vx_ENV_SENSOR_SetOutputDataRate(uint32_t Instance, uint32_t Function, float Odr);
int32_t BFA001Vx_ENV_SENSOR_GetValue(uint32_t Instance, uint32_t Function, float *Value);

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

#endif /* BFA001Vx_ENV_SENSORS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
