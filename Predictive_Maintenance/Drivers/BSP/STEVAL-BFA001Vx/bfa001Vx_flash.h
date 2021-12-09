/**
 ******************************************************************************
 * @file    bfa001Vx_flash.h
 * @author  System Research & Applications Team - Catania Lab.
 * @version 1.0.0
 * @date    12 November 2020
 * @brief   This file contains definitions for the bfa001Vx_flash.c
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
#ifndef __BFA001Vx_FLASH_H
#define __BFA001Vx_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "bfa001Vx_flash_mapping.h"

/** @addtogroup BSP
 * @{
 */

/** @addtogroup STEVAL-BFA001Vx
 * @{
 */

/** @addtogroup STEVAL-BFA001Vx_FLASH STEVAL-BFA001Vx Flash
 * @{
 */

/** @addtogroup STEVAL-BFA001Vx_FLASH_Exported_Variables STEVAL-BFA001Vx Flash Exported Variables
  * @{
  */

extern void *FLASH_handle[];

/**
 * @}
 */

 /** @addtogroup STEVAL-BFA001Vx_FLASH_Public_Function_Prototypes STEVAL-BFA001Vx Flash Public function prototypes
 * @{
 */

HAL_StatusTypeDef BSP_FLASH_EraseArea (uint32_t StartAddress, uint32_t EndAddress);
HAL_StatusTypeDef BSP_FLASH_WriteArea (uint32_t StartAddress, uint32_t EndAddress, uint32_t Data_32);
HAL_StatusTypeDef BSP_FLASH_CheckArea (uint32_t StartAddress, uint32_t EndAddress, uint32_t Data_32);
HAL_StatusTypeDef BSP_FLASH_Write (uint32_t StartAddress, uint32_t *pData_32, uint32_t Size);

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

#endif /* __BFA001Vx_FLASH_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
