/**
  ******************************************************************************
  * @file    TemplateFlashManagement.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version V0.1.0
  * @date    02-November-2016
  * @brief   Templete File for User specialization of Flash Management
  *          for MetaDataManager
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0094, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0094
  *
  ******************************************************************************
  */

#ifndef _TEMPLATE_FLASH_MANAGEMENT_H_
#define _TEMPLATE_FLASH_MANAGEMENT_H_

#ifdef __cplusplus
 extern "C" {
#endif 


/* Includes ------------------------------------------------------------------*/
#include "MetaDataManager.h"

/* Exported function prototypes -----------------------------------------------*/
extern uint32_t UserFunctionForErasingFlash(void);
extern uint32_t UserFunctionForSavingFlash(void *InitMetaDataVector,void *EndMetaDataVector);

#ifdef __cplusplus
}
#endif

#endif /* _TEMPLATE_FLASH_MANAGEMENT_H_ */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
