/**
  ******************************************************************************
  * File Name          : Target/lcd_io.c
  * Description        : This file provides code for the configuration
  *                      of the LCD IO instances.
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

/* Includes ------------------------------------------------------------------*/
#include "lcd_io.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L4XX_NUCLEO
  * @{
  */

/** @defgroup STM32L4XX_NUCLEO_LCD STM32L4XX_NUCLEO LCD
  * @brief      This file includes the LCD driver of
  *             STM32L4XX_NUCLEO boards.
  * @{
  */

/** @defgroup STM32L4XX_NUCLEO_LCD_Private_Types Private Types
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32L4XX_NUCLEO_LCD_Private_Functions_Prototypes Private Functions Prototypes
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32L4XX_NUCLEO_LCD_Private_Constants Private Constants
  * @{
  */
/* Number of LCD instances */
#define   LCD_INSTANCES_NBR     1U

/**
  * @}
  */

/** @defgroup STM32L4XX_NUCLEO_LCD_Private_Macros Private Macros
  * @{
  */
#define BSP_LCD_CHECK_PARAMS(Instance)

/**
  * @}
  */

/** @defgroup STM32L4XX_NUCLEO_LCD_Exported_Variables Exported Variables
  * @{
  */
EXTI_HandleTypeDef      hexti_lcd_te;

/**
  * @}
  */

/** @defgroup STM32L4XX_NUCLEO_LCD_Private_Variables Private Variables
  * @{
  */
static void             *LcdCompObj = NULL;
static LCD_Drv_t        *LcdDrv = NULL;
static ILI9341_IO_t     IOCtx = { 0 };
static ILI9341_Object_t ObjCtx = { 0 };
static uint8_t __IO     isWritingData = 0;
static uint16_t         line_counter = 0;

/**
  * @}
  */

/** @defgroup STM32L4XX_NUCLEO_LCD_Private_FunctionPrototypes Private Functions
  */
static int32_t LCD_Probe(uint32_t Instance, uint32_t Orientation);
static int32_t LCD_IO_GetTick(void);
static int32_t LCD_IO_Delay(uint32_t Delay);
static int32_t LCD_IO_Init(void);
static int32_t LCD_IO_DeInit(void);
static int32_t LCD_IO_WriteReg(uint8_t *Reg, uint32_t Length);
static int32_t LCD_IO_ReadReg(uint8_t *Reg, uint32_t Length);
static int32_t LCD_IO_SendData(uint8_t *pData, uint32_t Length);
static int32_t LCD_IO_SendDataDMA(uint8_t *pData, uint32_t Length);
static int32_t LCD_IO_RecvData(uint8_t *pData, uint32_t Length);
static int32_t LCD_IO_RecvDataDMA(uint8_t *pData, uint32_t Length);

/**
  * @}
  */

/** @addtogroup STM32L4XX_NUCLEO_LCD_Exported_Functions
  * @{
  */
/**
  * @brief  Initializes the LCD.
  * @param  Instance    LCD Instance
  * @param  Orientation LCD_ORIENTATION_PORTRAIT or LCD_ORIENTATION_LANDSCAPE
  *                     or LCD_ORIENTATION_PORTRAIT_ROT180
  *                     or LCD_ORIENTATION_LANDSCAPE_ROT180
  * @retval BSP status
  */
int32_t BSP_LCD_Init(uint32_t Instance, uint32_t Orientation)
{
  int32_t ret = BSP_ERROR_NONE;

  if ((Instance >= LCD_INSTANCES_NBR ) || (Orientation > LCD_ORIENTATION_LANDSCAPE_ROT180))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Reset Write state */
    isWritingData = 0;

    /* Reset line counter */
    line_counter = 0;

    /* Probe the LCD Component driver */
    ret = LCD_Probe(Instance, Orientation);
  }

  return ret;
}

/**
  * @brief  De-Initializes the LCD resources.
  * @param  Instance LCD Instance
  * @retval BSP status
  */
int32_t BSP_LCD_DeInit(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  BSP_LCD_CHECK_PARAMS(Instance);

  if(LcdDrv->DeInit != NULL)
  {
    if(LcdDrv->DeInit(LcdCompObj) < 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
  }
  else
  {
    ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  return ret;
}

/**
  * @brief  Sets the LCD Orientation.
  * @param  Instance LCD Instance
  * @param  pOrientation New LCD Orientation
  * @retval BSP status
  */
int32_t BSP_LCD_SetOrientation(uint32_t Instance, uint32_t Orientation)
{
  int32_t ret = BSP_ERROR_NONE;

  BSP_LCD_CHECK_PARAMS(Instance);

  if(LcdDrv->SetOrientation != NULL)
  {
    if(LcdDrv->SetOrientation(LcdCompObj, Orientation) < 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
  }
  else
  {
    ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  return ret;
}

/**
  * @brief  Gets the LCD Orientation.
  * @param  Instance LCD Instance
  * @param  pOrientation pointer to Used LCD Orientation
  * @retval BSP status
  */
int32_t BSP_LCD_GetOrientation(uint32_t Instance, uint32_t *pOrientation)
{
  int32_t ret = BSP_ERROR_NONE;

  BSP_LCD_CHECK_PARAMS(Instance);

  if(LcdDrv->GetOrientation != NULL)
  {
    if(LcdDrv->GetOrientation(LcdCompObj, pOrientation) < 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
  }
  else
  {
    ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  return ret;
}

/**
  * @brief  Gets the LCD X size.
  * @param  Instance LCD Instance
  * @param  pXSize pointer to Used LCD X size
  * @retval BSP status
  */
int32_t BSP_LCD_GetXSize(uint32_t Instance, uint32_t *pXSize)
{
  int32_t ret = BSP_ERROR_NONE;

  BSP_LCD_CHECK_PARAMS(Instance);

  if(LcdDrv->GetXSize != NULL)
  {
    if(LcdDrv->GetXSize(LcdCompObj, pXSize) < 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
  }
  else
  {
    ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  return ret;
}

/**
  * @brief  Gets the LCD Y size.
  * @param  Instance LCD Instance
  * @param  pYSize pointer to Used LCD Y size
  * @retval BSP status
  */
int32_t BSP_LCD_GetYSize(uint32_t Instance, uint32_t *pYSize)
{
  int32_t ret = BSP_ERROR_NONE;

  BSP_LCD_CHECK_PARAMS(Instance);

  if(LcdDrv->GetYSize != NULL)
  {
    if(LcdDrv->GetYSize(LcdCompObj, pYSize) < 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
  }
  else
  {
    ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  return ret;
}

/**
  * @brief  Switch On the display.
  * @param  Instance    LCD Instance
  * @retval BSP status
  */
int32_t BSP_LCD_DisplayOn(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  BSP_LCD_CHECK_PARAMS(Instance);

  if(LcdDrv->DisplayOn != NULL)
  {
    if(LcdDrv->DisplayOn(LcdCompObj) < 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
  }
  else
  {
    ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  return ret;
}

/**
  * @brief  Switch Off the display.
  * @param  Instance    LCD Instance
  * @retval BSP status
  */
int32_t BSP_LCD_DisplayOff(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  BSP_LCD_CHECK_PARAMS(Instance);

  if(LcdDrv->DisplayOff != NULL)
  {
    if(LcdDrv->DisplayOff(LcdCompObj) < 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
  }
  else
  {
    ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  return ret;
}

/**
  * @brief  Send data to select the LCD GRAM.
  * @param  pData  pointer to data to write to LCD GRAM.
  * @param  Length length of data to write to LCD GRAM
  * @retval Error status
  */
int32_t BSP_LCD_WriteData(uint32_t Instance, uint8_t *pData, uint32_t Length)
{
  int32_t ret = BSP_ERROR_NONE;

  BSP_LCD_CHECK_PARAMS(Instance);

  if(isWritingData)
  {
    ret = BSP_ERROR_BUSY;
  }
  else if(IOCtx.SendData)
  {
    isWritingData = (1U << 1);  /* SPI Write OP is 0x2 */

    /* Set the SPI in 16-bit mode to match endianess */
    hLCDSPI.Init.DataSize = SPI_DATASIZE_16BIT;
    HAL_SPI_Init(&hLCDSPI);
    if(IOCtx.SendData(pData, (Length / 2)) < 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }

    /* Go back to 8-bit mode */
    hLCDSPI.Init.DataSize = SPI_DATASIZE_8BIT;
    HAL_SPI_Init(&hLCDSPI);

    isWritingData = 0;
  }
  else
  {
    ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  return ret;
}

/**
  * @brief  Send data to select the LCD GRAM.
  * @param  pData  pointer to data to write to LCD GRAM.
  * @param  Length length of data to write to LCD GRAM
  * @retval Error status
  */
int32_t BSP_LCD_WriteDataDMA(uint32_t Instance, uint8_t *pData, uint32_t Length)
{
  int32_t ret = BSP_ERROR_NONE;

  BSP_LCD_CHECK_PARAMS(Instance);

  if(isWritingData)
  {
    ret = BSP_ERROR_BUSY;
  }
  else if(IOCtx.SendDataDMA)
  {
    isWritingData = (1U << 1);  /* SPI Write OP is 0x2 */

    /* Set the SPI in 16-bit mode to match endianess */
    hLCDSPI.Init.DataSize = SPI_DATASIZE_16BIT;
    HAL_SPI_Init(&hLCDSPI);
    if(IOCtx.SendDataDMA(pData, (Length / 2)) < 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
  }
  else
  {
    ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  return ret;
}

/**
  * @brief  Sets a display window
  * @param  Xpos X position.
  * @param  Ypos   specifies the Y bottom left position.
  * @param  Height height of the rectangle to fill.
  * @param  Width  display window width.
  * @retval Component status
  */
int32_t BSP_LCD_SetDisplayWindow(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t Width, uint32_t Height)
{
  int32_t ret = BSP_ERROR_NONE;

  BSP_LCD_CHECK_PARAMS(Instance);

  if(LcdDrv->SetDisplayWindow != NULL)
  {
    /* Fill the Rectangle lines on LCD */
    if (LcdDrv->SetDisplayWindow(LcdCompObj, Xpos, Ypos, Width, Height) < 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
  }
  else
  {
    ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  return ret;
}

/**
  * @brief  Send RGB data to select the LCD GRAM.
  * @param  UseDMA Transfert data using DMA ?.
  * @param  pData  pointer to data to write to LCD GRAM.
  * @param  Xpos X position.
  * @param  Ypos   specifies the Y bottom left position.
  * @param  Width  display window width.
  * @param  Height height of the rectangle to fill.
  * @retval Component status
  */
int32_t BSP_LCD_FillRGBRect(uint32_t Instance, uint8_t UseDMA, uint8_t *pData, uint32_t Xpos, uint32_t Ypos, uint32_t Width, uint32_t Height)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Set display window */
  if(BSP_LCD_SetDisplayWindow(Instance, Xpos, Ypos, Width, Height) == BSP_ERROR_NONE)
  {
    /* Send Pixels Data */
    if(UseDMA)
    {
      ret = BSP_LCD_WriteDataDMA(Instance, pData, (2*Width*Height));
    }
    else
    {
      ret = BSP_LCD_WriteData(Instance, pData, (2*Width*Height));
    }
  }
  else
  {
    ret = BSP_ERROR_BUS_FAILURE;
  }

  return ret;
}

/**
  * @brief  Get the status of the SPI Transfer.
  * @param  Instance LCD Instance
  * @retval Zero if no transfert, transfert Operation code otherwise
  */
uint8_t BSP_LCD_GetTransfertStatus(uint32_t Instance)
{
  if (Instance < LCD_INSTANCES_NBR)
  {
    return isWritingData;
  }

  return 0;
}

/**
  * @brief  Wait for until complete SPI Transfer.
  * @param  Instance LCD Instance
  * @retval None
  */
void BSP_LCD_WaitForTransferToBeDone(uint32_t Instance)
{
  if (Instance < LCD_INSTANCES_NBR)
  {
    while (isWritingData);
  }
}

/**
  * @brief  Signal transfert event.
  * @param  Instance LCD Instance
  * @param  Event Transfert ID : '1' read done, '2' 'write done'
  * @retval None
  */
__weak void BSP_LCD_SignalTransfertDone(uint32_t Instance, uint8_t Event)
{
  /* Prevent unused argument(s) compilation warning */;
  UNUSED(Event);

  if (Instance < LCD_INSTANCES_NBR)
  {
    /* This is the user's Callback to be implemented at the application level */
  }
}

/**
  * @brief  Signal transfert event.
  * @param  Instance LCD Instance
  * @param  Line counter
  * @retval None
  */
__weak void BSP_LCD_SignalTearingEffectEvent(uint32_t Instance, uint8_t state, uint16_t Line)
{
  /* Prevent unused argument(s) compilation warning */;
  UNUSED(Line);

  if (Instance < LCD_INSTANCES_NBR)
  {
    /* This is the user's Callback to be implemented at the application level */
    if(state)
    {
      /* TE event is done : de-allow display refresh */
    }
    else
    {
      /* TE event is received : allow display refresh */
    }
  }
}

/**
  * @}
  */

/** @defgroup STM32L4XX_NUCLEO_LCD_Private_Functions Private Functions
  * @{
  */
/**
  * @brief  Register Bus IOs for instance 0 if ILI9341 ID is OK
  * @param  Instance    LCD Instance
  * @param  Orientation LCD Orientation
  * @retval BSP status
  */
static int32_t LCD_Probe(uint32_t Instance, uint32_t Orientation)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Configure the lcd driver : map to LCD_IO function*/
  IOCtx.Init             = LCD_IO_Init;
  IOCtx.DeInit           = LCD_IO_DeInit;
  IOCtx.ReadReg          = LCD_IO_ReadReg;
  IOCtx.WriteReg         = LCD_IO_WriteReg;
  IOCtx.SendData         = LCD_IO_SendData;
  IOCtx.SendDataDMA      = LCD_IO_SendDataDMA;
  IOCtx.RecvData         = LCD_IO_RecvData;
  IOCtx.RecvDataDMA      = LCD_IO_RecvDataDMA;
  IOCtx.GetTick          = LCD_IO_GetTick;
  IOCtx.Delay            = LCD_IO_Delay;

  if(ILI9341_RegisterBusIO(&ObjCtx, &IOCtx) != ILI9341_OK)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else
  {
    LcdCompObj = &ObjCtx;

    /* LCD Initialization */
    LcdDrv = (LCD_Drv_t *)&ILI9341_LCD_Driver;

    ObjCtx.IsInitialized = 0;
    if(LcdDrv->Init(LcdCompObj, ILI9341_FORMAT_DEFAULT, Orientation) != ILI9341_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  return ret;
}

/**
  * @brief  Provide a tick value in millisecond.
  * @retval Tick value.
  */
static int32_t LCD_IO_GetTick(void)
{
  uint32_t ret;
  ret = HAL_GetTick();
  return (int32_t)ret;
}

/**
  * @brief  LCD IO delay
  * @param  Delay  Delay in ms
  * @retval Error status
  */
static int32_t LCD_IO_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
  return BSP_ERROR_NONE;
}

/**
  * @}
  */

/** @defgroup SPI Driver for LCD controller
  * @{
  */
/**
  * @brief HCI Transport Layer Low Level Interrupt Service Routine
  *
  * @param  None
  * @retval None
  */
static void LCD_TECallback(void)
{
  uint8_t state = ((LCD_TE_GPIO_PORT->IDR & LCD_TE_GPIO_PIN) == 0x00u ? 0 : 1);

  HAL_EXTI_ClearPending(&hexti_lcd_te, EXTI_TRIGGER_RISING_FALLING);

  if(state == 1)
  {
    if(line_counter > ObjCtx.YSize)
    {
      line_counter = 0;
    }
    /* Call BSP_LCD_SignalTearingEffectEvent() */
    BSP_LCD_SignalTearingEffectEvent(0, state, line_counter);
  }
  else
  {
    /* Call BSP_LCD_SignalTearingEffectEvent() */
    BSP_LCD_SignalTearingEffectEvent(0, state, line_counter);
    line_counter++;
  }
}

#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
static void SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if(hspi == &hLCDSPI)
  {
    /* Deselect : Chip Select high */
    LCD_CS_HIGH();

    /* Go back to 8-bit mode */
    hspi->Init.DataSize = SPI_DATASIZE_8BIT;
    HAL_SPI_Init(hspi);

    /* Signal Transfert Done Event */
    BSP_LCD_SignalTransfertDone(0, isWritingData);

    /* Reset Transfert state */
    isWritingData = 0;
  }
}
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */

/**
  * @brief  Initializes LCD low level.
  * @retval BSP status
  */
static int32_t LCD_IO_Init(void)
{
  int32_t ret = BSP_ERROR_NONE;

  LCD_RST_LOW();
  HAL_Delay(100);
  LCD_RST_HIGH();
  HAL_Delay(100);

  /* turn LCD on = drive pin low (active low) */
  /* Set or Reset the control line */
  LCD_CS_HIGH();
  LCD_DC_HIGH();

  ret = LCD_SPI_Init();

  if(ret == BSP_ERROR_NONE)
  {
    /* Register TE event IRQ handler */
    HAL_EXTI_GetHandle(&hexti_lcd_te, LCD_TE_GPIO_LINE);
    HAL_EXTI_RegisterCallback(&hexti_lcd_te, HAL_EXTI_COMMON_CB_ID, LCD_TECallback);

#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
    /* Register SPI Tx Complete Callback */
    HAL_SPI_RegisterCallback(&hLCDSPI, HAL_SPI_TX_COMPLETE_CB_ID, SPI_TxCpltCallback);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
  }

  return ret;
}

/**
  * @brief  DeInitializes LCD low level
  * @retval BSP status
  */
static int32_t LCD_IO_DeInit(void)
{
  int32_t ret = BSP_ERROR_NONE;

  ret = LCD_SPI_DeInit();

  return ret;
}

/**
  * @brief  Writes register on LCD register.
  * @param  Reg    Register to be written
  * @param  Length length of data be read from the LCD GRAM
  * @retval BSP status
  */
static int32_t LCD_IO_WriteReg(uint8_t *Reg, uint32_t Length)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Select : Chip Select low */
  LCD_CS_LOW();

  LCD_DC_HIGH();

  ret = LCD_SPI_Send(Reg, Length);

  LCD_DC_LOW();

  /* Deselect : Chip Select high */
  LCD_CS_HIGH();

  return ret;
}

/**
  * @brief  Read register on LCD register.
  * @param  Reg    Register to be read
  * @param  Length length of data be read from the LCD GRAM
  * @retval BSP status
  */
static int32_t LCD_IO_ReadReg(uint8_t *Reg, uint32_t Length)
{
  return BSP_ERROR_FEATURE_NOT_SUPPORTED;
}

/**
  * @brief  Send data to select the LCD GRAM.
  * @param  pData  pointer to data to write to LCD GRAM.
  * @param  Length length of data to write to LCD GRAM
  * @retval Error status
  */
static int32_t LCD_IO_SendData(uint8_t *pData, uint32_t Length)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Select : Chip Select low */
  LCD_CS_LOW();

  /* Send Data */
  ret = LCD_SPI_Send(pData, Length);

  /* Deselect : Chip Select high */
  LCD_CS_HIGH();

  return ret;
}

/**
  * @brief  Send data to select the LCD GRAM using DMA.
  * @param  pData  pointer to data to write to LCD GRAM.
  * @param  Length length of data to write to LCD GRAM
  * @retval Error status
  */
static int32_t LCD_IO_SendDataDMA(uint8_t *pData, uint32_t Length)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Select : Chip Select low */
  LCD_CS_LOW();

  /* Send Data */
  ret = LCD_SPI_Send_DMA(pData, Length);

  return ret;
}

/**
  * @brief  Receive data from selected LCD GRAM.
  * @param  pData  pointer to data to read to from selected LCD GRAM.
  * @param  Length length of data to read from selected LCD GRAM
  * @retval Error status
  */
static int32_t LCD_IO_RecvData(uint8_t *pData, uint32_t Length)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Select : Chip Select low */
  LCD_CS_LOW();

  LCD_DC_HIGH();

  /* Write Register to read */
  ret = LCD_SPI_Send(pData, 1);

  LCD_DC_LOW();

  /* Receive the Data */
  ret = LCD_SPI_Recv(pData, Length);

  /* Deselect : Chip Select high */
  LCD_CS_HIGH();

  return ret;
}

/**
  * @brief  Receive data using DMA from selected LCD GRAM.
  * @param  pData  pointer to data to read to from selected LCD GRAM.
  * @param  Length length of data to read from selected LCD GRAM
  * @retval Error status
  */
static int32_t LCD_IO_RecvDataDMA(uint8_t *pData, uint32_t Length)
{
  return BSP_ERROR_FEATURE_NOT_SUPPORTED;
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
