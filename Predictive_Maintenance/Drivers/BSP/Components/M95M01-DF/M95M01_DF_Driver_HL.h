/**
  ******************************************************************************
  * @file    M95M01_DF_Driver_HL.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version V1.0.1
  * @date    08-Feb-2019
  * @brief   This file contains definitions for the M95M01_DF_Driver_HL.c firmware driver
  ******************************************************************************
 * @attention
 *
 * COPYRIGHT(c) 2019 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __M95M01_DF_DRIVER_HL_H
#define __M95M01_DF_DRIVER_HL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "eeprom.h"

/* Include epprom component drivers. */
#include "M95M01_DF_Driver.h"

/** @addtogroup BSP BSP
  * @{
  */

/** @addtogroup Component Component
  * @{
  */

/** @addtogroup M95M01_DF M95M01-DF
  * @{
  */

/** @addtogroup M95M01_DF_DRIVER_HL M95M01_DF_DRIVER_HL
  * @{
  */

/** @addtogroup M95M01_DF_DRIVER_HL_Public_Constants M95M01_DF_DRIVER_HL Public constants
 * @{
 */

#define M95M01_DF_MODULES_MAX_NUM  1     //!< M95M01_DF max number of instances

/**
  * @}
  */

/** @addtogroup M95M01_DF_DRIVER_HL_Public_Types M95M01_DF_DRIVER_HL Public Types
 * @{
 */

/**
 * @brief M95M01_DF specific data internal structure definition
  */
typedef struct
{
  uint8_t isMemInitialized;
  uint8_t isMemEnabled;
} M95M01_DF_Data_t;

/**
  * @brief M95M01_DF eeprom specific data internal structure definition
  */
typedef struct
{
  M95M01_DF_Data_t *Data;       /* Data to manage in software enable/disable of the epproms */
} M95M01_DF_M_Data_t;

/**
  * @}
  */

/** @addtogroup M95M01_DF_DRIVER_HL_Public_Variables M95M01_DF_DRIVER_HL Public variables
  * @{
  */

extern EEPROM_Drv_t M95M01_DF_M_Drv;
extern M95M01_DF_Data_t M95M01_DF_Data[M95M01_DF_MODULES_MAX_NUM];

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

#endif /* __M95M01_DF_DRIVER_HL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
