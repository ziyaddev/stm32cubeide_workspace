/**
 ******************************************************************************
 * @file    bfa001Vx_flash_mapping.h
 * @author  System Research & Applications Team - Catania Lab.
 * @version 1.0.0
 * @date    12 November 2020
 * @brief   This file contains definitions for the MCU flash memory mounted on
 *          BFA001Vx board used
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
#ifndef __BFA001Vx_FLASH_MAPPING_H
#define __BFA001Vx_FLASH_MAPPING_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

/** @addtogroup BSP
 * @{
 */

/** @addtogroup STEVAL-BFA001Vx
 * @{
 */

/** @addtogroup STEVAL-BFA001Vx_FLASH STEVAL-BFA001Vx Flash
 * @{
 */

/** @addtogroup STEVAL-BFA001Vx_FLASH_MAPPING STEVAL-BFA001Vx Flash Mapping
  * @{
  */
  
/** @addtogroup STEVAL-BFA001Vx_FLASH_MAPPING_Exported_Constants STEVAL-BFA001Vx Flash Mapping Exported Constants
  * @{
  */
    
/* Base address of the Flash sectors (Bank 1) */
#define ADDR_FLASH_SECTOR_0   ((uint32_t)0x08000000) /* Base address of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1   ((uint32_t)0x08004000) /* Base address of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2   ((uint32_t)0x08008000) /* Base address of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3   ((uint32_t)0x0800C000) /* Base address of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4   ((uint32_t)0x08010000) /* Base address of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5   ((uint32_t)0x08020000) /* Base address of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6   ((uint32_t)0x08040000) /* Base address of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7   ((uint32_t)0x08060000) /* Base address of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8   ((uint32_t)0x08080000) /* Base address of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9   ((uint32_t)0x080A0000) /* Base address of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10  ((uint32_t)0x080C0000) /* Base address of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11  ((uint32_t)0x080E0000) /* Base address of Sector 11, 128 Kbytes */
/* Base address of the Flash sectors (Bank 2) */
#define ADDR_FLASH_SECTOR_12  ((uint32_t)0x08100000) /* Base address of Sector 12, 16 Kbytes */
#define ADDR_FLASH_SECTOR_13  ((uint32_t)0x08104000) /* Base address of Sector 13, 16 Kbytes */
#define ADDR_FLASH_SECTOR_14  ((uint32_t)0x08108000) /* Base address of Sector 14, 16 Kbytes */
#define ADDR_FLASH_SECTOR_15  ((uint32_t)0x0810C000) /* Base address of Sector 15, 16 Kbytes */
#define ADDR_FLASH_SECTOR_16  ((uint32_t)0x08110000) /* Base address of Sector 16, 64 Kbytes */
#define ADDR_FLASH_SECTOR_17  ((uint32_t)0x08120000) /* Base address of Sector 17, 128 Kbytes */
#define ADDR_FLASH_SECTOR_18  ((uint32_t)0x08140000) /* Base address of Sector 18, 128 Kbytes */
#define ADDR_FLASH_SECTOR_19  ((uint32_t)0x08160000) /* Base address of Sector 19, 128 Kbytes */
#define ADDR_FLASH_SECTOR_20  ((uint32_t)0x08180000) /* Base address of Sector 20, 128 Kbytes */
#define ADDR_FLASH_SECTOR_21  ((uint32_t)0x081A0000) /* Base address of Sector 21, 128 Kbytes */
#define ADDR_FLASH_SECTOR_22  ((uint32_t)0x081C0000) /* Base address of Sector 22, 128 Kbytes */
#define ADDR_FLASH_SECTOR_23  ((uint32_t)0x081E0000) /* Base address of Sector 23, 128 Kbytes */
    
/**
  * @}
  */

/** @addtogroup STEVAL-BFA001Vx_FLASH_MAPPING_Exported_Defines STEVAL-BFA001Vx Flash Mapping Exported Defines
  * @{
  */

#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_23 /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     (ADDR_FLASH_SECTOR_23+ GetFlashSectorSize(ADDR_FLASH_SECTOR_23)-1) /* End @ of user Flash area : sector start address + sector size -1 */

/* DFU Memory Area
#define DFU_START_ADDRESS                       0x08000000
#define DFU_END_ADDRESS                         0x0800BFFF
*/


/* APPLICATION Memory Area
#define APPLICATION_START_ADDRESS               0x08020000
#define APPLICATION_END_ADDRESS                 0x080FFFFF
*/

/* APPLICATION COPY Memory Area
#define APPLICATION_COPY_START_ADDRESS          0x08100000
#define APPLICATION_COPY_END_ADDRESS            0x081FFFFF
*/

/* DATA Memory Area*/

             
#define DATA_MEMORY_ACC_ADDRESS                 0x0800C000
#define DATA_MEMORY_COMPUTATION_ADDRESS         0x0800C008
#define DATA_MEMORY_END_ADDRESS                 0x0800CFFF

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

#endif /* __BFA001Vx_FLASH_MAPPING_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
