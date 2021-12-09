/**
 ******************************************************************************
 * @file    bfa001Vx_flash.c
 * @author  System Research & Applications Team - Catania Lab.
 * @version 1.0.0
 * @date    12 November 2020
 * @brief   This file provides a set of functions needed to manage
 *          the flash memory
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

#include "bfa001Vx_flash.h"

/** @addtogroup BSP
 * @{
 */

/** @addtogroup STEVAL-BFA001Vx
 * @{
 */

/** @addtogroup STEVAL-BFA001Vx_FLASH STEVAL-BFA001Vx Flash
 * @{
 */
    
/** @addtogroup STEVAL-BFA001Vx_FLASH_Private_FunctionPrototypes STEVAL-BFA001Vx Flash Private function prototypes
 * @{
 */

static uint8_t GetFlashSector(uint32_t Address, uint32_t *pSector);
//static uint32_t GetFlashSectorSize(uint32_t Sector);

/**
 * @}
 */

/** @addtogroup STEVAL-BFA001Vx_FLASH_Public_Functions STEVAL-BFA001Vx Flash Public functions
 * @{
 */

/**
  * @brief Gets the flash sector of a given address
  * @param Address The flash address
  * @param pSector Pointer to the flash sector value
  * @retval 0 in case of success
  * @retval 1 in case of failure
  */  
static uint8_t GetFlashSector(uint32_t Address, uint32_t *pSector) {
  
  uint8_t retValue = 1;
  
  *pSector = 0;
  
  if ( (Address >= ADDR_FLASH_SECTOR_0) && (Address <= (ADDR_FLASH_SECTOR_23 + (128*1024) - 1)) ) {
    
    if(Address < ADDR_FLASH_SECTOR_1)
      *pSector = FLASH_SECTOR_0;
    else if(Address < ADDR_FLASH_SECTOR_2)
      *pSector = FLASH_SECTOR_1;
    else if(Address < ADDR_FLASH_SECTOR_3)
      *pSector = FLASH_SECTOR_2;
    else if(Address < ADDR_FLASH_SECTOR_4)
      *pSector = FLASH_SECTOR_3;
    else if(Address < ADDR_FLASH_SECTOR_5)
      *pSector = FLASH_SECTOR_4;
    else if(Address < ADDR_FLASH_SECTOR_6)
      *pSector = FLASH_SECTOR_5;
    else if(Address < ADDR_FLASH_SECTOR_7)
      *pSector = FLASH_SECTOR_6;
    else if(Address < ADDR_FLASH_SECTOR_8)
      *pSector = FLASH_SECTOR_7;
    else if(Address < ADDR_FLASH_SECTOR_9)
      *pSector = FLASH_SECTOR_8;
    else if(Address < ADDR_FLASH_SECTOR_10)
      *pSector = FLASH_SECTOR_9;
    else if(Address < ADDR_FLASH_SECTOR_11)
      *pSector = FLASH_SECTOR_10;
    else if(Address < ADDR_FLASH_SECTOR_12)
      *pSector = FLASH_SECTOR_11;
    else if(Address < ADDR_FLASH_SECTOR_13)
      *pSector = FLASH_SECTOR_12;
    else if(Address < ADDR_FLASH_SECTOR_14)
      *pSector = FLASH_SECTOR_13;
    else if(Address < ADDR_FLASH_SECTOR_15)
      *pSector = FLASH_SECTOR_14;
    else if(Address < ADDR_FLASH_SECTOR_16)
      *pSector = FLASH_SECTOR_15;
    else if(Address < ADDR_FLASH_SECTOR_17)
      *pSector = FLASH_SECTOR_16;
    else if(Address < ADDR_FLASH_SECTOR_18)
      *pSector = FLASH_SECTOR_17;
    else if(Address < ADDR_FLASH_SECTOR_19)
      *pSector = FLASH_SECTOR_18;
    else if(Address < ADDR_FLASH_SECTOR_20)
      *pSector = FLASH_SECTOR_19;
    else if(Address < ADDR_FLASH_SECTOR_21)
      *pSector = FLASH_SECTOR_20;
    else if(Address < ADDR_FLASH_SECTOR_22)
      *pSector = FLASH_SECTOR_21;
    else if(Address < ADDR_FLASH_SECTOR_23)
      *pSector = FLASH_SECTOR_22;
    else
      *pSector = FLASH_SECTOR_23;
    
    retValue = 0;
  }
  return retValue;
}


/**
  * @brief Erase flash memory area
  * @param StartAddress The flash memory address inside the fisrt sector to erase
  * @param EndAddress The flash memory address inside the last sector to erase
  * @retval HAL_Status
  */  
HAL_StatusTypeDef BSP_FLASH_EraseArea (uint32_t StartAddress, uint32_t EndAddress)
{
  
  HAL_StatusTypeDef status = HAL_ERROR;
  uint32_t FirstSector = 0, LastSector = 0, NbOfSectors = 0, SECTORError = 0;
  FLASH_EraseInitTypeDef EraseInitStruct; //!< Variable used for Erase procedure 
  
  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* Erase the user Flash area */
  
  /* Get the 1st sector to erase */
  if (GetFlashSector(StartAddress, &FirstSector) == 0) {
    
    /* Get the last sector to erase */
    if (GetFlashSector(EndAddress, &LastSector) == 0) {
      
      /* Get the number of sector to erase from 1st sector*/
      NbOfSectors = LastSector - FirstSector + 1;
      
      /* Fill EraseInit structure*/
      EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
      EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
      EraseInitStruct.Sector        = FirstSector;
      EraseInitStruct.NbSectors     = NbOfSectors;
      
      /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
      you have to make sure that these data are rewritten before they are accessed during code
      execution. If this cannot be done safely, it is recommended to flush the caches by setting the
      DCRST and ICRST bits in the FLASH_CR register. */
      if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) == HAL_OK) {
        
        status = HAL_OK;
      }
      else {
        
        /*
        Error occurred while sector erase.
        User can add here some code to deal with this error.
        SECTORError will contain the faulty sector and then to know the code error on this sector,
        user can call function 'HAL_FLASH_GetError()'
        */
//        /* Infinite loop */
//        while (1) {
          
          __NOP();
//        }
      }
    }
  }
  
  /* Lock the Flash to disable the flash control register access (recommended
  to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
  
  return status; 
}

/**
  * @brief Brief
  * @param StartAddress The start flash memory address
  * @param EndAddress The end flash memory address
  * @param Data_32 Data to be written
  * @retval HAL_Status
  */
HAL_StatusTypeDef BSP_FLASH_WriteArea (uint32_t StartAddress, uint32_t EndAddress, uint32_t Data_32) {

  HAL_StatusTypeDef status = HAL_ERROR;
  uint32_t Address;
  
  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* Program the user Flash area word by word */
  
  Address = StartAddress;
  status = HAL_OK;
  
  while ( (Address < EndAddress) && (status == HAL_OK) ) {
    
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, Data_32);
    if (status == HAL_OK) {
      
      Address = Address + sizeof(uint32_t);
    }
    else {
      
//      /* Error occurred while writing data in Flash memory.
//      User can add here some code to deal with this error */
//      while (1) {
        
        __NOP();
//      }
    }
  }
  
  /* Lock the Flash to disable the flash control register access (recommended
  to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
  
  return status;
}

/**
  * @brief Brief
  * @param StartAddress The start flash memory address
  * @param EndAddress The end flash memory address
  * @param Data_32 Data to be compared
  * @retval HAL_Status
  */
HAL_StatusTypeDef BSP_FLASH_CheckArea (uint32_t StartAddress, uint32_t EndAddress, uint32_t Data_32) {

  HAL_StatusTypeDef status = HAL_ERROR;
  uint32_t Address;
   __IO uint32_t data32 = 0 , MemoryProgramStatus = 0;

  /* Check if the programmed data is OK
  MemoryProgramStatus = 0: data programmed correctly
  MemoryProgramStatus != 0: number of words not programmed correctly ******/
  Address = StartAddress;
  MemoryProgramStatus = 0x0;
  
  while (Address < EndAddress)
  {
    data32 = *(__IO uint32_t *)Address;
    
    if (data32 != Data_32)
    {
      MemoryProgramStatus++;
    }
    Address = Address + sizeof(uint32_t);
  }
  
  /*Check if there is an issue to program data*/
  if (MemoryProgramStatus == 0)
  {
    /* No error detected */
    status = HAL_OK;
  }
  else
  {
    /* Error detected */
    __NOP();
  }

  return status;
}

/**
  * @brief Write flash memory area
  * @param StartAddress The start flash memory address
  * @param pData_32 Pointer to an array of uint32_t to be written
  * @param Size The array size
  * @retval HAL_Status
  */
HAL_StatusTypeDef BSP_FLASH_Write (uint32_t StartAddress, uint32_t *pData_32, uint32_t Size) {
  
  HAL_StatusTypeDef status = HAL_ERROR;
  uint32_t Address;
  uint32_t s;
  
  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* Program the user Flash area word by word */
  
  Address = StartAddress;
  s = 0;
  status = HAL_OK;
  
  while ( (s < Size) && (status == HAL_OK) ) {
    
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, *(pData_32+s));
    if (status == HAL_OK) {
      
      Address = Address + sizeof(uint32_t);
      s++;
    }
    else {
      
//      /* Error occurred while writing data in Flash memory.
//      User can add here some code to deal with this error */
//      while (1) {
        
        __NOP();
//      }
    }
  }
  
  /* Lock the Flash to disable the flash control register access (recommended
  to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
  
  return status; 
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

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
