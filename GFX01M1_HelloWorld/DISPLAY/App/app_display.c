/**
  ******************************************************************************
  * File Name          : app_display.c
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
#include "app_display.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include "key_io.h"
#include "mem_io.h"
#include "lcd_io.h"
/* USER CODE BEGIN Includes */
#include "string.h"
#include "Image1.h"
#include "Image2.h"
#include "Image3.h"
#include "Image4.h"
#include "Image5.h"
#include "Image6.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct image_s
{
  uint32_t  Width;
  uint32_t  Height;
  uint8_t   bpp;
  uint8_t*  Data;
} image_t;

typedef struct orientation_s
{
  uint32_t  lcd;
  uint32_t  key;
} orientation_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STEP_LINES                  1
#define USE_MEM_DMA                 1
#define USE_LCD_DMA                 1
#define BUTTON_USER_PRESSED_STATE   GPIO_PIN_RESET
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MIN(w,h)            (w < h ? w : h)
#if (USE_MEM_DMA == 1)
#define MEM_READ_DATA       BSP_MEM_ReadDataDMA
#else
#define MEM_READ_DATA       BSP_MEM_ReadData
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t __IO TransferAllowed = 0;
static uint8_t CacheBuffer[(320*2*BUFFER_CACHE_LINES)];
static uint16_t posy0 = 0;
static uint16_t posx = 0;
static uint16_t posy = 0;
static uint8_t key = 1;
static uint8_t image_id = 0;
static uint32_t LCD_Width = 0;
static uint32_t LCD_Height = 0;
static uint32_t LCD_Orientation = 0;
static uint8_t orientation_id = 0;

static image_t Images[] = { { 240, 240, 2, (uint8_t *)Image1 }
                          , { 240, 320, 2, (uint8_t *)Image2 }
                          , { 320, 240, 2, (uint8_t *)Image3 }
                          , { 240, 240, 2, (uint8_t *)Image4 }
                          , { 240, 320, 2, (uint8_t *)Image5 }
                          , { 240, 240, 2, (uint8_t *)Image6 }
                          , {0, 0, 0, 0} };

static const orientation_t orientations[] = { { LCD_ORIENTATION_PORTRAIT, KEY_ORIENTATION_PORTRAIT }
                                            , { LCD_ORIENTATION_LANDSCAPE, KEY_ORIENTATION_LANDSCAPE }
                                            , { LCD_ORIENTATION_PORTRAIT_ROT180, KEY_ORIENTATION_PORTRAIT_ROT180 }
                                            , { LCD_ORIENTATION_LANDSCAPE_ROT180, KEY_ORIENTATION_LANDSCAPE_ROT180 }} ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void BSP_LCD_Clear(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t Width, uint32_t Height);
static void Display_Image(image_t *image, uint16_t posx, uint16_t posy);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void BSP_LCD_Clear(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t Width, uint32_t Height)
{
  uint32_t size;
  uint32_t CacheLinesCnt, CacheLinesSz;
  uint32_t offset = 0, line_cnt = Ypos;

  size = (2*Width*Height);
  CacheLinesCnt = (Height > BUFFER_CACHE_LINES ? BUFFER_CACHE_LINES : Height);
  CacheLinesSz = (2*Width*CacheLinesCnt);
  memset(CacheBuffer, 0, CacheLinesSz);

  while(1)
  {
    if(BSP_LCD_FillRGBRect(Instance, USE_LCD_DMA, CacheBuffer, Xpos, line_cnt, Width, CacheLinesCnt) != BSP_ERROR_NONE)
    {
      Error_Handler();
    }
    BSP_LCD_WaitForTransferToBeDone(0);
    line_cnt += CacheLinesCnt;
    offset += CacheLinesSz;
    /* Check remaining data size */
    if(offset == size)
    {
      /* last block transfer was done, so exit */
      break;
    }
    else if((offset + CacheLinesSz) > size)
    {
      /* Transfer last block and exit */
      CacheLinesCnt = ((size - offset)/ (2*Width));
      if(BSP_LCD_FillRGBRect(Instance, USE_LCD_DMA, CacheBuffer, Xpos, line_cnt, Width, CacheLinesCnt) != BSP_ERROR_NONE)
      {
        Error_Handler();
      }
      BSP_LCD_WaitForTransferToBeDone(0);
      break;
    }
  }
}

/**
  * @brief  Refresh the display.
  * @param  None.
  * @retval None.
  */
static void Display_Image(image_t *image, uint16_t posx, uint16_t posy)
{
    uint8_t *pData = (uint8_t *)image->Data;
    uint32_t Height = image->Height;

    if(image->Height > LCD_Height)
    {
      pData += (image->bpp * image->Width * ((image->Height - LCD_Height)/2));
      Height = LCD_Height;
    }

    // Send the frambuffer n times
    if(((uint32_t )pData & 0xFF000000) == FLASH_BASE)
    {
      if(BSP_LCD_FillRGBRect(0, USE_LCD_DMA, pData, posx, posy, image->Width, Height) != BSP_ERROR_NONE)
      {
        Error_Handler();
      }
      BSP_LCD_WaitForTransferToBeDone(0);
    }
    else
    {
      uint32_t size = (image->bpp*image->Width*Height);
      uint32_t CacheLinesSz = (2*MIN(LCD_Width,LCD_Height)*BUFFER_CACHE_LINES);
      uint32_t CacheLinesCnt = (CacheLinesSz/(image->bpp*image->Width));
      uint32_t line_cnt = 0;
      uint32_t offset = 0;

      /* One block read */
      if(size < CacheLinesSz)
      {
        if(MEM_READ_DATA(0, CacheBuffer, (uint32_t )pData, size) != BSP_ERROR_NONE)
        {
          Error_Handler();
        }
        BSP_MEM_WaitForTransferToBeDone(0);
        if(BSP_LCD_FillRGBRect(0, USE_LCD_DMA, CacheBuffer, posx, posy, image->Width, Height) != BSP_ERROR_NONE)
        {
          Error_Handler();
        }
        BSP_LCD_WaitForTransferToBeDone(0);
      }
      else
      {
        while(1)
        {
          /* Multi-block read/write */
          if(MEM_READ_DATA(0, CacheBuffer, (uint32_t )(pData+offset), CacheLinesSz) != BSP_ERROR_NONE)
          {
            Error_Handler();
          }
          BSP_MEM_WaitForTransferToBeDone(0);
          if(BSP_LCD_FillRGBRect(0, USE_LCD_DMA, CacheBuffer, posx, posy+line_cnt, image->Width, CacheLinesCnt) != BSP_ERROR_NONE)
          {
            Error_Handler();
          }
          BSP_LCD_WaitForTransferToBeDone(0);
          line_cnt += CacheLinesCnt;
          offset += CacheLinesSz;
          /* Check remaining data size */
          if(offset == size)
          {
            /* last block transfer was done, so exit */
            break;
          }
          else if((offset + CacheLinesSz) > size)
          {
            /* Transfer last block and exit */
            if(MEM_READ_DATA(0, CacheBuffer, (uint32_t )(pData+offset), (size - offset)) != BSP_ERROR_NONE)
            {
              Error_Handler();
            }
            BSP_MEM_WaitForTransferToBeDone(0);
            CacheLinesCnt = ((size - offset)/ (image->bpp*image->Width));
            if(BSP_LCD_FillRGBRect(0, USE_LCD_DMA, CacheBuffer, posx, posy+line_cnt, image->Width, CacheLinesCnt) != BSP_ERROR_NONE)
            {
              Error_Handler();
            }
            BSP_LCD_WaitForTransferToBeDone(0);
            break;
          }
        }
      }
    }
}
/* USER CODE END 0 */

/**
 * Initialize DISPLAY application
 */
void MX_DISPLAY_Init(void)
{
  /* USER CODE BEGIN MX_DISPLAY_Init 0 */

  /* USER CODE END MX_DISPLAY_Init 0 */
  if(BSP_MEM_Init(0) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  if(BSP_LCD_Init(0, LCD_ORIENTATION_PORTRAIT) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  if(BSP_KEY_Init(0, KEY_ORIENTATION_PORTRAIT) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN MX_DISPLAY_Init 1 */
  if((BSP_LCD_GetXSize(0, &LCD_Width) != BSP_ERROR_NONE) \
  || (BSP_LCD_GetYSize(0, &LCD_Height) != BSP_ERROR_NONE) \
  || (BSP_LCD_GetOrientation(0, &LCD_Orientation) != BSP_ERROR_NONE) )
  {
    Error_Handler();
  }
  if(BSP_LCD_DisplayOn(0) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  /* USER CODE END MX_DISPLAY_Init 1 */
}

/**
 * DISPLAY application entry function
 */
void MX_DISPLAY_Process(void)
{
  /* USER CODE BEGIN MX_DISPLAY_Process 0 */
  int i;
  static __IO uint8_t can_move = 0;
  /* Wait for TE */
  if (TransferAllowed)
  {
    /* Display something */
    if(key)
    {
      /* Reset key value */
      key = 0;

      /* Check if we can allow scrolling up/down the picture */
      if ((can_move == 0) && (Images[image_id].Height < LCD_Height) \
      && ((LCD_Orientation == LCD_ORIENTATION_PORTRAIT) || (LCD_Orientation == KEY_ORIENTATION_PORTRAIT_ROT180)))
      {
        /* Allow moving the picture on screen */
        can_move = 1;
      }
      if(posy == 0) /* reload new image */
      {
        BSP_LCD_Clear(0, 0, 0, LCD_Width, LCD_Height);
        posx = ((LCD_Width - Images[image_id].Width)/2);
        if(Images[image_id].Height < LCD_Height)
        {
          posy = ((LCD_Height - Images[image_id].Height)/2);
        }
        else
        {
          posy = 0;
        }
        posy0 = posy;
      }
      else if(posy == posy0) /* center current image */
      {
        posy = ((LCD_Height - Images[image_id].Height)/2);
        BSP_LCD_Clear(0, 0, 0, LCD_Width, posy);
        BSP_LCD_Clear(0, 0, posy+Images[image_id].Height, LCD_Width, posy);
      }
      Display_Image(&Images[image_id], posx, posy);
    }

    if(BSP_KEY_GetState(0, &key) == BSP_ERROR_NONE)
    {
      switch(key)
      {
        case BSP_KEY_CENTER:
          if(posy != posy0)
          {
            posy = posy0;
            HAL_Delay(200);
          }
          else
          {
            /* Ignore this key */
            key = 0;
          }
          break;
        case BSP_KEY_UP:
          if (can_move && (posy > STEP_LINES))
          {
            BSP_LCD_Clear(0, 0, (posy + Images[image_id].Height), LCD_Width, STEP_LINES);
            posy -= STEP_LINES;
          }
          else
          {
            /* Ignore this key */
            key = 0;
          }
          break;
        case BSP_KEY_DOWN:
          if (can_move && (posy <(LCD_Height-Images[image_id].Height-STEP_LINES)))
          {
            BSP_LCD_Clear(0, 0, posy, LCD_Width, STEP_LINES);
            posy += STEP_LINES;
          }
          else
          {
            /* Ignore this key */
            key = 0;
          }
          break;
        case BSP_KEY_LEFT:
          i = image_id-1;
          while ((i >= 0) && (Images[i].Width > LCD_Width)) { i--; }
          if (image_id > 0)
          {
            can_move = 0;
            image_id = i;
            posy = 0;
            HAL_Delay(200);
          }
          else
          {
            /* Ignore this key */
            key = 0;
          }
          break;
        case BSP_KEY_RIGHT:
          i = image_id+1;
          while ((Images[i].Height > 0) && (Images[i].Width > LCD_Width)) { i++; }
          if (Images[image_id+1].Height > 0)
          {
            can_move = 0;
            image_id = i;
            posy = 0;
            HAL_Delay(200);
          }
          else
          {
            /* Ignore this key */
            key = 0;
          }
          break;
        default:
          break;
      }
    }
    else if (HAL_GPIO_ReadPin(BUTTON_USER_GPIO_Port, BUTTON_USER_Pin) == BUTTON_USER_PRESSED_STATE)
    {
      /* Read User Button */
      orientation_id++;
      if(orientation_id == 4)
      {
        orientation_id = 0;
      }
      BSP_LCD_Clear(0, 0, 0, LCD_Width, LCD_Height);
      if(BSP_LCD_SetOrientation(0, orientations[orientation_id].lcd) != BSP_ERROR_NONE)
      {
        Error_Handler();
      }
      if(BSP_KEY_SetOrientation(0, orientations[orientation_id].key) != BSP_ERROR_NONE)
      {
        Error_Handler();
      }
      if((BSP_LCD_GetXSize(0, &LCD_Width) != BSP_ERROR_NONE) \
      || (BSP_LCD_GetYSize(0, &LCD_Height) != BSP_ERROR_NONE) \
      || (BSP_LCD_GetOrientation(0, &LCD_Orientation) != BSP_ERROR_NONE) )
      {
        Error_Handler();
      }
      i = image_id;
      while ((i >= 0) && (Images[i].Width > LCD_Width)) { i--; }
      while ((Images[i].Height > 0) && (Images[i].Width > LCD_Width)) { i++; }
      image_id = i;
      key = 255;
      posy = 0;
      can_move = 0;
      HAL_Delay(200);
    }
  }
  /* USER CODE END MX_DISPLAY_Process 0 */

  /* USER CODE BEGIN MX_DISPLAY_Process 1 */

  /* USER CODE END MX_DISPLAY_Process 1 */
}

/**
 * DISPLAY application task
 */
void DISPLAY_Task(void *argument)
{
  /* USER CODE BEGIN DISPLAY_Task 0 */

  /* USER CODE END DISPLAY_Task 0 */

  /* USER CODE BEGIN DISPLAY_Task 1 */

  /* USER CODE END DISPLAY_Task 1 */
}

void BSP_LCD_SignalTearingEffectEvent(uint32_t Instance, uint8_t state, uint16_t Line)
{
  if(Instance == 0)
  {
    /* USER CODE BEGIN BSP_LCD_SignalTearingEffectEvent */
    if(state)
    {
      /* Line '0' is the Vsync event */
      if(Line == 0)
      {
        /* TE event is received : allow display refresh */
        TransferAllowed = 1;
      }
    }
    else
    {
      /* TE event is done : de-allow display refresh */
      TransferAllowed = 0;
    }
    /* USER CODE END BSP_LCD_SignalTearingEffectEvent */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
