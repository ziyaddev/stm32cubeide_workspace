/**
  ******************************************************************************
  * @file    OTA.h 
  * @author  System Research & Applications Team - Catania Lab.
  * @version V2.4.0
  * @date    07-June-2021
  * @brief   Over-the-Air Update API
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
#ifndef _OTA_H_
#define _OTA_H_

#ifdef __cplusplus
 extern "C" {
#endif
   
/** @addtogroup Projects
  * @{
  */

/** @addtogroup DEMONSTRATIONS Demonstrations
  * @{
  */

/** @addtogroup PREDCTIVE_MAINTENANCE Predictive Maintenance
  * @{
  */

/** @addtogroup PREDCTIVE_MAINTENANCE_OTA Predictive Maintenance OTA
  * @{
  */

/** @defgroup PREDCTIVE_MAINTENANCE_OTA_EXPORTED_DEFINES Predictive Maintenance OTA Exported Defines
  * @{
  */
   
/* Exported defines ---------------------------------------------------------*/
#ifdef USE_STM32F4XX_NUCLEO
  /* 240Kbytes Max Program Size */
  #define OTA_MAX_PROG_SIZE (0x40000-0x4000-8)
#endif /* USE_STM32F4XX_NUCLEO */
   
#ifdef USE_STM32L4XX_NUCLEO
  /* 496Kbytes Max Program Size */
  #define OTA_MAX_PROG_SIZE (0x80000-0x4000-8)
#endif /* USE_STM32L4XX_NUCLEO */
   
/**
  * @}
  */

/** @defgroup PREDCTIVE_MAINTENANCE_OTA_EXPORTED_FUNCTIONS_PROTOTYPES Predictive Maintenance OTA Exported functions Prototypes
  * @{
  */
   
/* Exported functions ---------------------------------------------------------*/

/* API for preparing the Flash for receiving the Update. It defines also the Size of the Update and the CRC value aspected */
extern void StartUpdateFWBlueMS(uint32_t SizeOfUpdate,uint32_t uwCRCValue);

/* API for storing chuck of data to Flash.
 * When it has recived the total number of byte defined by StartUpdateFWBlueMS,
 * it computes the CRC value and if it matches the aspected CRC value,
 * it writes the Magic Number in Flash for BootLoader */
extern int8_t UpdateFWBlueMS(uint32_t *SizeOfUpdateBlueFW,uint8_t * att_data, int32_t data_length,uint8_t WriteMagicNum);

/* API for checking the BootLoader compliance */
extern int8_t CheckBootLoaderCompliance(void);

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

#endif /* _OTA_H_ */

/******************* (C) COPYRIGHT 2021 STMicroelectronics *****END OF FILE****/
