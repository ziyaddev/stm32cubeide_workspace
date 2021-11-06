/**
  ******************************************************************************
  * File Name          : Target/lcd_io.h
  * Description        : This file provides code for the exported APIs
  *                      of the LCD instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD_IO_H__
#define __LCD_IO_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "lcd.h"
#include "lcd_conf.h"
#include "../ili9341/ili9341.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* BSP Common Error codes */
#ifndef BSP_ERROR_NONE
#define BSP_ERROR_NONE                      0
#define BSP_ERROR_NO_INIT                   -1
#define BSP_ERROR_WRONG_PARAM               -2
#define BSP_ERROR_BUSY                      -3
#define BSP_ERROR_PERIPH_FAILURE            -4
#define BSP_ERROR_COMPONENT_FAILURE         -5
#define BSP_ERROR_UNKNOWN_FAILURE           -6
#define BSP_ERROR_UNKNOWN_COMPONENT         -7
#define BSP_ERROR_BUS_FAILURE               -8
#define BSP_ERROR_CLOCK_FAILURE             -9
#define BSP_ERROR_MSP_FAILURE               -10
#define BSP_ERROR_FEATURE_NOT_SUPPORTED     -11
#endif /* BSP_ERROR_NONE */

/* LCD Cache lines */
#define BUFFER_CACHE_LINES                  24

/* LCD Orientation */
#define LCD_ORIENTATION_PORTRAIT            ILI9341_ORIENTATION_PORTRAIT
#define LCD_ORIENTATION_LANDSCAPE           ILI9341_ORIENTATION_LANDSCAPE
#define LCD_ORIENTATION_PORTRAIT_ROT180     ILI9341_ORIENTATION_PORTRAIT_ROT180
#define LCD_ORIENTATION_LANDSCAPE_ROT180    ILI9341_ORIENTATION_LANDSCAPE_ROT180

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
int32_t BSP_LCD_Init(uint32_t Instance, uint32_t Orientation);
int32_t BSP_LCD_DeInit(uint32_t Instance);
int32_t BSP_LCD_SetOrientation(uint32_t Instance, uint32_t Orientation);
int32_t BSP_LCD_GetOrientation(uint32_t Instance, uint32_t *Orientation);
int32_t BSP_LCD_GetXSize(uint32_t Instance, uint32_t *XSize);
int32_t BSP_LCD_GetYSize(uint32_t Instance, uint32_t *YSize);
int32_t BSP_LCD_DisplayOn(uint32_t Instance);
int32_t BSP_LCD_DisplayOff(uint32_t Instance);
int32_t BSP_LCD_WriteData(uint32_t Instance, uint8_t *pData, uint32_t Length);
int32_t BSP_LCD_WriteDataDMA(uint32_t Instance, uint8_t *pData, uint32_t Length);
int32_t BSP_LCD_SetDisplayWindow(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t Width, uint32_t Height);
int32_t BSP_LCD_FillRGBRect(uint32_t Instance, uint8_t UseDMA, uint8_t *pData, uint32_t Xpos, uint32_t Ypos, uint32_t Width, uint32_t Height);
uint8_t BSP_LCD_GetTransfertStatus(uint32_t Instance);
void    BSP_LCD_WaitForTransferToBeDone(uint32_t Instance);
void    BSP_LCD_SignalTransfertDone(uint32_t Instance, uint8_t Event);
void    BSP_LCD_SignalTearingEffectEvent(uint32_t Instance, uint8_t state, uint16_t Line);

#ifdef __cplusplus
}
#endif
#endif /* __LCD_IO_H__ */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
