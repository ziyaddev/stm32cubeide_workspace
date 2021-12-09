/**
  ******************************************************************************
  * @file    bfa001Vx_audio.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 1.0.0
  * @date    12 November 2020
  * @brief   This file contains the AUDIO functions for the bfa001Vx driver.
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
#include "bfa001Vx_audio.h"
#include "bfa001Vx_bus.h"
#include "audio.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STEVAL-BFA001Vx
  * @{
  */ 
  
/** @defgroup STEVAL-BFA001Vx_AUDIO STEVAL-BFA001Vx AUDIO
  * @{
  */ 
/** @defgroup STEVAL-BFA001Vx_AUDIO_Private_Defines STEVAL-BFA001Vx AUDIO Private Defines
  * @{
  */
/**
  * @}
  */

/** @defgroup STEVAL-BFA001Vx_AUDIO_Private_Macros STEVAL-BFA001Vx AUDIO Private Macros
  * @{
  */
/*### RECORD ###*/

#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))

#define DFSDM_OVER_SAMPLING(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (128U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (256U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (64U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (128U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (64U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (64U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (64U)  \
	  : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (32U) : (32U) 
	  
#define DFSDM_CLOCK_DIVIDER(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (17U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (17U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (24U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (4U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (16U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (16U) : (16U)

#define DFSDM_FILTER_ORDER(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (DFSDM_FILTER_SINC4_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (DFSDM_FILTER_SINC5_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (DFSDM_FILTER_SINC5_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (DFSDM_FILTER_SINC4_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (DFSDM_FILTER_SINC5_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (DFSDM_FILTER_SINC5_ORDER) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (DFSDM_FILTER_SINC5_ORDER) \
	  : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (DFSDM_FILTER_SINC5_ORDER) : (DFSDM_FILTER_SINC5_ORDER)

#define DFSDM_MIC_BIT_SHIFT(__FREQUENCY__) \
        ((__FREQUENCY__) == (AUDIO_FREQUENCY_8K))  ? (8U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_11K)) ? (5U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_16K)) ? (10U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_22K)) ? (8U)  \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_32K)) ? (10U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_44K)) ? (10U) \
      : ((__FREQUENCY__) == (AUDIO_FREQUENCY_48K)) ? (10U) \
	  : ((__FREQUENCY__) == (AUDIO_FREQUENCY_96K)) ? (5U) : (5U)

static int16_t VolumeGain[] = 
{
  -12,-12,-6,-3,0,2,3,5,6,7,8,9,9,10,11,11,12,12,13,13,14,14,15,15,15,
  16,16,17,17,17,17,18,18,18,19,19,19,19,19,20,20,20,20,21,21,21,21,21,
  22,22,22,22,22,22,23,23,23,23,23,23,23,24,24,24,24,24,24,24,25,25,25,
  25,25,25,25,25,25,26,26,26,26,26,26,26,26,26,27,27,27,27,27,27,27,27,
  27,27,28,28,28,28,28,28,28,28,28,28,28,28,29,29,29,29,29,29,29,29,29,
  29,29,29,29,30,30,30,30,30,30,30,31  
};
            
uint8_t Channel_Demux[128] = {
  0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
  0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
  0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
  0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
  0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
  0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
  0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
  0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
  0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
  0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
  0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f,
  0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f,
  0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
  0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
  0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f,
  0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f
};

/**
  * @}
  */ 

/** @defgroup STEVAL-BFA001Vx_AUDIO_Exported_Variables STEVAL-BFA001Vx AUDIO Exported Variables
  * @{
  */
/* Recording context */
AUDIO_IN_Ctx_t                         AudioInCtx[AUDIO_IN_INSTANCES_NBR] = {0};

/**
  * @}
  */
  
/** @defgroup STEVAL-BFA001Vx_AUDIO_Private_Variables STEVAL-BFA001Vx AUDIO Private Variables
  * @{
  */
#include "arm_math.h"
static int16_t aCoeffs[] = { -1406, 1634, -1943, 2386, -3080, 4325, -7223, 21690, 21690, -7223, 4325, -3080, 2386, -1943, 1634, -1406, };
#define DECIMATOR_NUM_TAPS 16
#define DECIMATOR_BLOCK_SIZE 16 
#define DECIMATOR_FACTOR 2
#define DECIMATOR_STATE_LENGTH (DECIMATOR_BLOCK_SIZE + (DECIMATOR_NUM_TAPS) -1)
static arm_fir_decimate_instance_q15 ARM_Decimator_State[4];
static int16_t aState_ARM[4][DECIMATOR_STATE_LENGTH];

//static AUDIO_Drv_t                     *AudioDrv = NULL;
//static void                            *CompObj = NULL;
I2S_HandleTypeDef                 hAudioInI2s;
SPI_HandleTypeDef          hAudioInSPI;

static TIM_HandleTypeDef          TimDividerHandle;
static TIM_SlaveConfigTypeDef     sSlaveConfig;
static TIM_IC_InitTypeDef         sICConfig;
static TIM_OC_InitTypeDef         sOCConfig;

static uint16_t I2S_InternalBuffer[PDM_INTERNAL_BUFFER_SIZE_I2S];
static uint16_t SPI_InternalBuffer[PDM_INTERNAL_BUFFER_SIZE_SPI];

/* PDM filters params */
static PDM_Filter_Handler_t  PDM_FilterHandler[4];
static PDM_Filter_Config_t   PDM_FilterConfig[4];

/* Recording Buffer Trigger */
static __IO uint32_t              RecBuffTrigger          = 0;
static __IO uint32_t              RecBuffHalf             = 0;
static __IO int32_t               MicRecBuff[4][DEFAULT_AUDIO_IN_BUFFER_SIZE];
static __IO uint32_t              MicBuffIndex[4];


/**
  * @}
  */ 

/** @defgroup STEVAL-BFA001Vx_AUDIO_Private_Function_Prototypes STEVAL-BFA001Vx AUDIO Private Function Prototypes
  * @{
  */
static uint8_t AUDIO_IN_Timer_Init(void);
static uint8_t AUDIO_IN_Timer_Start(void);
static void I2S_MspInit(I2S_HandleTypeDef *hi2s);
static void SPI_MspInit(SPI_HandleTypeDef *hspi);

/**
  * @}
  */ 

/** @defgroup STEVAL-BFA001Vx_AUDIO_IN_Private_Functions STEVAL-BFA001Vx_AUDIO_IN Private Functions
  * @{
  */ 
  
  
/**
* @brief  Initialize wave recording.
* @param  Instance  AUDIO IN Instance. It can be:
*       - 0 when I2S is used 
*       - 1 if DFSDM is used
*       - 2 if PDM is used
* @param  AudioInit Init structure
* @retval BSP status
*/
__weak int32_t BSP_AUDIO_IN_Init(uint32_t Instance, BSP_AUDIO_Init_t* AudioInit)
{
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    return BSP_ERROR_WRONG_PARAM;  
  }
  else
  {
    /* Store the audio record context */
    AudioInCtx[Instance].Device          = AudioInit->Device;
    AudioInCtx[Instance].ChannelsNbr     = AudioInit->ChannelsNbr;  
    AudioInCtx[Instance].SampleRate      = AudioInit->SampleRate; 
    AudioInCtx[Instance].BitsPerSample   = AudioInit->BitsPerSample;
    AudioInCtx[Instance].Volume          = AudioInit->Volume;
    AudioInCtx[Instance].State           = AUDIO_IN_STATE_RESET;
    
    if(Instance == 0U)
    {
      uint32_t PDM_Clock_Freq;      
      MX_I2S_Config i2s_config;
      
      switch (AudioInit->SampleRate)
      {
      case AUDIO_FREQUENCY_8K:
        PDM_Clock_Freq = 1280;
        break;
        
      case AUDIO_FREQUENCY_16K:
        PDM_Clock_Freq = 1280;
        break;
        
      case AUDIO_FREQUENCY_32K:
        PDM_Clock_Freq = 2048;
        break;
        
      case AUDIO_FREQUENCY_48K:
        PDM_Clock_Freq = 3072;
        break;
        
      default:
        return BSP_ERROR_WRONG_PARAM;
      }
      
      AudioInCtx[Instance].DecimationFactor = (PDM_Clock_Freq * 1000)/AudioInit->SampleRate;
      AudioInCtx[Instance].Size = (PDM_Clock_Freq/8) * 2 * N_MS_PER_INTERRUPT;
            
      if(AudioInCtx[0].ChannelsNbr == 1)
      {
        i2s_config.DataFormat   = I2S_DATAFORMAT_16B;
      }
      else
      {
        i2s_config.DataFormat   = I2S_DATAFORMAT_32B;
      }
      
      i2s_config.AudioFreq = ((PDM_Clock_Freq * 1000) / 32);
      i2s_config.CPOL           = I2S_CPOL_HIGH;
      i2s_config.MCLKOutput     = I2S_MCLKOUTPUT_DISABLE;
      i2s_config.Mode           = I2S_MODE_MASTER_RX;
      i2s_config.Standard       = I2S_STANDARD_MSB;
      i2s_config.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
      i2s_config.ClockSource    = I2S_CLOCK_PLL;
      
      if (AudioInCtx[0].ChannelsNbr>1)
      {
        PDM_Clock_Freq *=2;
        if (AUDIO_IN_Timer_Init() != HAL_OK)
        {
          return HAL_ERROR;
        }
      }
      
      /* PLL clock is set depending by the AudioFreq (44.1khz vs 48khz groups) */ 
      if(MX_I2S_ClockConfig(&hAudioInI2s, PDM_Clock_Freq) != HAL_OK)
      {
        return BSP_ERROR_CLOCK_FAILURE;
      }
      
      /* I2S Peripheral configuration */
      hAudioInI2s.Instance          = AUDIO_IN_I2S_INSTANCE;
      __HAL_I2S_DISABLE(&hAudioInI2s);
      I2S_MspInit(&hAudioInI2s);
        
      if (MX_I2S_Init(&hAudioInI2s, &i2s_config)!= HAL_OK)
      {
        return BSP_ERROR_PERIPH_FAILURE;
      }
      if (HAL_I2S_Init(&hAudioInI2s) != HAL_OK)
      {
        return HAL_ERROR;
      }
      
      if (AudioInCtx[0].ChannelsNbr>2)
      {
        /* Set the SPI parameters */
        hAudioInSPI.Instance               = AUDIO_IN_SPI_INSTANCE;
        
        __HAL_SPI_DISABLE(&hAudioInSPI);
        SPI_MspInit(&hAudioInSPI);
        
        MX_SPI_Config spi_config;
        spi_config.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
        spi_config.Direction         = SPI_DIRECTION_2LINES_RXONLY;
        spi_config.CLKPhase          = SPI_PHASE_2EDGE;
        spi_config.CLKPolarity       = SPI_POLARITY_HIGH;
        spi_config.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
        spi_config.CRCPolynomial     = 7;
        spi_config.DataSize          = SPI_DATASIZE_16BIT;
        spi_config.FirstBit          = SPI_FIRSTBIT_MSB;
        spi_config.NSS               = SPI_NSS_SOFT;
        spi_config.TIMode            = SPI_TIMODE_DISABLED;
        spi_config.Mode              = SPI_MODE_SLAVE;
        
        if (MX_SPI_Init(&hAudioInSPI, &spi_config)!= HAL_OK)
        {
          return BSP_ERROR_PERIPH_FAILURE;
        }
        if (HAL_SPI_Init(&hAudioInSPI) != HAL_OK)
        {
          return HAL_ERROR;
        }
  
      }
      
      if (BSP_AUDIO_IN_PDMToPCM_Init(Instance, AudioInCtx[0].SampleRate, AudioInCtx[0].ChannelsNbr, AudioInCtx[0].ChannelsNbr)!= HAL_OK)
      {
        return BSP_ERROR_NO_INIT;
      }
    }
    else if(Instance == 1U)
    {
      return BSP_ERROR_WRONG_PARAM;
    }
    else /* Instance = 2 */
    {      
    
    }
    
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_STOP; 
    /* Return BSP status */
    return BSP_ERROR_NONE; 
  }
}

/**
* @brief  Deinit the audio IN peripherals.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/

__weak int32_t BSP_AUDIO_IN_DeInit(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if(Instance == 0U)
    {

    }
    
    else /* (Instance == 1U) */
    {
      return BSP_ERROR_WRONG_PARAM;
    }
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_RESET;   
  }
  /* Return BSP status */
  return ret;
}

/**
* @brief  Clock Config.
* @param  hi2s: I2S handle if required
* @param  PDM_rate: Audio frequency used to play the audio stream.
* @note   This API is called by BSP_AUDIO_IN_Init() 
*         Being __weak it can be overwritten by the application     
* @retval HAL_OK if no problem during execution, HAL_ERROR otherwise
*/
__weak HAL_StatusTypeDef MX_I2S_ClockConfig(I2S_HandleTypeDef *hi2s, uint32_t PDM_rate)
{ 
  
  HAL_StatusTypeDef ret = HAL_OK;
  /*I2S PLL Configuration*/
  RCC_PeriphCLKInitTypeDef rccclkinit;
  HAL_RCCEx_GetPeriphCLKConfig(&rccclkinit); 
  
#if defined(STM32F446xx)
  rccclkinit.PLLI2S.PLLI2SQ = 2;
  rccclkinit.PLLI2SDivQ = 1;
#endif
  if (PDM_rate % 1280 == 0)
  {
#if defined(STM32F411xE) || defined (STM32F446xx)    
    rccclkinit.PLLI2S.PLLI2SM = 10;
    rccclkinit.PLLI2S.PLLI2SN = 96;
#else
    rccclkinit.PLLI2S.PLLI2SN = 192;
#endif
    rccclkinit.PLLI2S.PLLI2SR = 5;
  }
  else
  {
#if defined(STM32F411xE) || defined (STM32F446xx)
    
    rccclkinit.PLLI2S.PLLI2SM = 8;
#endif
    rccclkinit.PLLI2S.PLLI2SN = 258;
    rccclkinit.PLLI2S.PLLI2SR = 3;
  }   
  
#if defined(STM32F446xx)
  rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB2;
#else
  rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
#endif
  
  if(HAL_RCCEx_PeriphCLKConfig(&rccclkinit) != HAL_OK)
  {
    ret = HAL_ERROR;
  }
  ret = HAL_OK;
  
  return ret;
}


__weak HAL_StatusTypeDef MX_SPI_Init(SPI_HandleTypeDef* hspi, MX_SPI_Config *MXConfig)
{  
  static DMA_HandleTypeDef hdma_rx;
  HAL_StatusTypeDef ret = HAL_OK;
       
  hspi->Init.BaudRatePrescaler = MXConfig->BaudRatePrescaler; 
  hspi->Init.Direction         = MXConfig->Direction;         
  hspi->Init.CLKPhase          = MXConfig->CLKPhase;          
  hspi->Init.CLKPolarity       = MXConfig->CLKPolarity;       
  hspi->Init.CRCCalculation    = MXConfig->CRCCalculation;    
  hspi->Init.CRCPolynomial     = MXConfig->CRCPolynomial;     
  hspi->Init.DataSize          = MXConfig->DataSize;          
  hspi->Init.FirstBit          = MXConfig->FirstBit;         
  hspi->Init.NSS               = MXConfig->NSS;               
  hspi->Init.TIMode            = MXConfig->TIMode;            
  hspi->Init.Mode              = MXConfig->Mode; 
  
    /* Configure the DMA handler for Transmission process */
  hdma_rx.Instance                 = AUDIO_IN_SPI_RX_DMA_STREAM;
  hdma_rx.Init.Channel             = AUDIO_IN_SPI_RX_DMA_CHANNEL;
  hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  hdma_rx.Init.Mode                = DMA_CIRCULAR;
  hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
  hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma_rx.Init.MemBurst            = DMA_MBURST_INC4;
  hdma_rx.Init.PeriphBurst         = DMA_PBURST_INC4;

  /* Configure the DMA Stream */ 
  HAL_DMA_Init(&hdma_rx);
    
  /* Associate the initialized DMA handle to the the SPI handle */
  __HAL_LINKDMA(hspi, hdmarx, hdma_rx);      

  return ret;
}


__weak HAL_StatusTypeDef MX_I2S_Init(I2S_HandleTypeDef* hi2s, MX_I2S_Config *MXConfig)
{
  static DMA_HandleTypeDef hdma_i2sRx;
  HAL_StatusTypeDef ret = HAL_OK;
  
  hi2s->Init.DataFormat = MXConfig->DataFormat;
  hi2s->Init.AudioFreq = MXConfig->AudioFreq;
  hi2s->Init.ClockSource = MXConfig->ClockSource;
  hi2s->Init.CPOL = MXConfig->CPOL;
  hi2s->Init.MCLKOutput = MXConfig->MCLKOutput;
  hi2s->Init.Mode = MXConfig->Mode;
  hi2s->Init.Standard = MXConfig->Standard;
  hi2s->Init.FullDuplexMode = MXConfig->FullDuplexMode;  
  
  /* Enable the DMA clock */
  AUDIO_IN_I2S_DMAx_CLK_ENABLE();
  
  if(hi2s->Instance == AUDIO_IN_I2S_INSTANCE)
  {
    /* Configure the hdma_i2sRx handle parameters */
    hdma_i2sRx.Init.Channel             = AUDIO_IN_I2S_DMAx_CHANNEL;
    hdma_i2sRx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_i2sRx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_i2sRx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_i2sRx.Init.PeriphDataAlignment = AUDIO_IN_I2S_DMAx_PERIPH_DATA_SIZE;
    hdma_i2sRx.Init.MemDataAlignment    = AUDIO_IN_I2S_DMAx_MEM_DATA_SIZE;
    hdma_i2sRx.Init.Mode                = DMA_CIRCULAR;
    hdma_i2sRx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_i2sRx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_i2sRx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_i2sRx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_i2sRx.Init.PeriphBurst         = DMA_MBURST_SINGLE;
    
    hdma_i2sRx.Instance = AUDIO_IN_I2S_DMAx_STREAM;
    
    /* Associate the DMA handle */
    __HAL_LINKDMA(hi2s, hdmarx, hdma_i2sRx);
    
    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hdma_i2sRx);
    
    /* Configure the DMA Stream */
    HAL_DMA_Init(&hdma_i2sRx);
  }
  
  /* I2S DMA IRQ Channel configuration */
  HAL_NVIC_SetPriority(AUDIO_IN_I2S_DMAx_IRQ, BSP_AUDIO_IN_IT_PRIORITY, BSP_AUDIO_IN_IT_PRIORITY);   //GRO
  HAL_NVIC_EnableIRQ(AUDIO_IN_I2S_DMAx_IRQ); 
  
  return ret;
}

/**
* @brief  Initialize the PDM library.
* @param Instance    AUDIO IN Instance
* @param  AudioFreq  Audio sampling frequency
* @param  ChnlNbrIn  Number of input audio channels in the PDM buffer
* @param  ChnlNbrOut Number of desired output audio channels in the  resulting PCM buffer
* @retval BSP status
*/
__weak int32_t BSP_AUDIO_IN_PDMToPCM_Init(uint32_t Instance, uint32_t AudioFreq, uint32_t ChnlNbrIn, uint32_t ChnlNbrOut)
{
  
  if(Instance != 0U)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else 
  {
    uint32_t index = 0;
    
    /* Enable CRC peripheral to unlock the PDM library */
    __HAL_RCC_CRC_CLK_ENABLE();
    
    for(index = 0; index < ChnlNbrIn; index++)
    {
      volatile uint32_t error = 0;
      /* Init PDM filters */
      PDM_FilterHandler[index].bit_order  = PDM_FILTER_BIT_ORDER_LSB;
      PDM_FilterHandler[index].endianness = PDM_FILTER_ENDIANNESS_LE;
      PDM_FilterHandler[index].high_pass_tap = 2122358088;
      PDM_FilterHandler[index].out_ptr_channels = ChnlNbrOut;
      PDM_FilterHandler[index].in_ptr_channels  = ChnlNbrIn;
      
      /* PDM lib config phase */
      PDM_FilterConfig[index].output_samples_number = (AudioFreq/1000) * N_MS_PER_INTERRUPT;
      PDM_FilterConfig[index].mic_gain = 24;
      
      switch (AudioInCtx[0].DecimationFactor)
      {
      case 16:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_16;
        break;
      case 24:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_24;
        break;
      case 32:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_32;
        break;
      case 48:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_48;
        break;
      case 64:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_64;
        break;
      case 80:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_80;
        break;
      case 128:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_128;
        break;
      case 160:
        PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_80;
        PDM_FilterConfig[index].output_samples_number *= 2;        
        PDM_FilterHandler[index].out_ptr_channels = 1;
        arm_fir_decimate_init_q15  (&ARM_Decimator_State[index], DECIMATOR_NUM_TAPS, DECIMATOR_FACTOR,
                                    aCoeffs, aState_ARM[index], DECIMATOR_BLOCK_SIZE);
        break;
      default:
        return HAL_ERROR;
      }
      
      error = PDM_Filter_Init((PDM_Filter_Handler_t *)(&PDM_FilterHandler[index]));
      if (error!=0)
      {
        while(1);
      }
      
      error = PDM_Filter_setConfig((PDM_Filter_Handler_t *)&PDM_FilterHandler[index], &PDM_FilterConfig[index]);
      if (error!=0)
      {
        while(1);
      }
    }
    return BSP_ERROR_NONE;
  }  
}

/**
* @brief  Converts audio format from PDM to PCM.
* @param  Instance  AUDIO IN Instance  
* @param  PDMBuf    Pointer to PDM buffer data
* @param  PCMBuf    Pointer to PCM buffer data
* @retval BSP status
*/
__weak int32_t BSP_AUDIO_IN_PDMToPCM(uint32_t Instance, uint16_t *PDMBuf, uint16_t *PCMBuf)
{  
  if(Instance != 0U)
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  else 
  {
    uint32_t index = 0;
    
    for(index = 0; index < AudioInCtx[Instance].ChannelsNbr; index++)
    {
      if (AudioInCtx[Instance].SampleRate == 8000)
      {
        uint16_t Decimate_Out[8];
        uint32_t ii = 0;
        uint16_t PDM_Filter_Out[16];
        
        PDM_Filter(&((uint8_t*)(PDMBuf))[index], (uint16_t*)&(PDM_Filter_Out), &PDM_FilterHandler[index]);
        arm_fir_decimate_q15 (&ARM_Decimator_State[index], (q15_t *)&(PDM_Filter_Out), (q15_t*)&(Decimate_Out), DECIMATOR_BLOCK_SIZE);
        for (ii=0; ii<8; ii++)
        {
          PCMBuf[ii * AudioInCtx[Instance].ChannelsNbr + index] = Decimate_Out[ii];
        }
      }
      else
      {
        PDM_Filter(&((uint8_t*)(PDMBuf))[index], (uint16_t*)&(PCMBuf[index]), &PDM_FilterHandler[index]);
      }
    }
    
    return BSP_ERROR_NONE;
  }  
}

/**
* @brief  Start audio recording.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  pBuf     Main buffer pointer for the recorded data storing  
* @param  NbrOfBytes Not used
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_Record(uint32_t Instance, uint8_t* pBuf, uint32_t NbrOfBytes)
{
  int32_t ret;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR - 1)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else 
  {
    AudioInCtx[Instance].pBuff = (uint16_t*)pBuf;
    
    if(Instance == 0U)
    {
      
      if(AudioInCtx[Instance].ChannelsNbr > 2)
      {
        if(HAL_SPI_Receive_DMA(&hAudioInSPI, (uint8_t *)SPI_InternalBuffer, AudioInCtx[Instance].Size) != HAL_OK)
        {
          ret = BSP_ERROR_PERIPH_FAILURE;
        }
      }
      
      if(AudioInCtx[Instance].ChannelsNbr != 1)
      {
        if(AUDIO_IN_Timer_Start() != HAL_OK)
        {
          ret = BSP_ERROR_PERIPH_FAILURE;
        }
      }
      
      if(HAL_I2S_Receive_DMA(&hAudioInI2s, I2S_InternalBuffer, AudioInCtx[Instance].Size/2) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      } 
      
      /* Update BSP AUDIO IN state */     
      AudioInCtx[Instance].State = AUDIO_IN_STATE_RECORDING;
      ret = BSP_ERROR_NONE;
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }
  /* Return BSP status */
  return ret;
}

/**
* @brief  Stop audio recording.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_Stop(uint32_t Instance)
{
  int32_t ret;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;  
  }
  else
  {
    if(Instance == 0U)
    {
      if(AudioInCtx[Instance].ChannelsNbr > 2)
      {
        if(HAL_SPI_DMAStop(&hAudioInSPI)!= HAL_OK)
        {
          ret = BSP_ERROR_PERIPH_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      
      if(HAL_I2S_DMAStop(&hAudioInI2s) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else /*(Instance == 1U) */
    { 
      return BSP_ERROR_WRONG_PARAM;
    }
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_STOP;
  } 
  /* Return BSP status */
  return ret;  
}

/**
* @brief  Pause the audio file stream.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_Pause(uint32_t Instance)
{
  int32_t ret;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;  
  }
  else
  {  
    if(Instance == 0U)
    { 
      if(HAL_I2S_DMAPause(&hAudioInI2s)!= HAL_OK)
      {
        ret = BSP_ERROR_WRONG_PARAM;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else /* (Instance == 1U) */
    {
      return BSP_ERROR_WRONG_PARAM;
    }
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_PAUSE;    
  }
  /* Return BSP status */
  return ret;
}

/**
* @brief  Resume the audio file stream.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_Resume(uint32_t Instance)
{
  int32_t ret;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR) 
  {
    ret = BSP_ERROR_WRONG_PARAM;  
  }
  else 
  {
    if(Instance == 0U)
    {   
      if(HAL_I2S_DMAResume(&hAudioInI2s)!= HAL_OK)
      {
        ret = BSP_ERROR_WRONG_PARAM;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else /* (Instance == 1U) */
    {
      return BSP_ERROR_WRONG_PARAM;
    }
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_RECORDING;
  }
  /* Return BSP status */
  return ret;
}

/**
* @brief  Starts audio recording.
* @param  Instance  AUDIO IN Instance. It can be 1(DFSDM used)
* @param  pBuf      Main buffer pointer for the recorded data storing
* @param  NbrOfBytes      Size of the recorded buffer
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_RecordChannels(uint32_t Instance, uint8_t **pBuf, uint32_t NbrOfBytes)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance != 1U)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  
  /* Return BSP status */
  return ret;
}

/**
* @brief  Stop audio recording.
* @param  Instance  AUDIO IN Instance. It can be 1(DFSDM used)
* @param  Device    Digital input device to be stopped
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_StopChannels(uint32_t Instance, uint32_t Device)
{
  int32_t ret;
  
  /* Stop selected devices */
  ret = BSP_AUDIO_IN_PauseChannels(Instance, Device);
  
  if(ret == BSP_ERROR_NONE)
  {    
    /* Update BSP AUDIO IN state */     
    AudioInCtx[Instance].State = AUDIO_IN_STATE_STOP;
  }
  
  /* Return BSP status */
  return ret; 
}

/**
* @brief  Pause the audio file stream.
* @param  Instance  AUDIO IN Instance. It can be 1(DFSDM used)
* @param  Device    Digital mic to be paused
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_PauseChannels(uint32_t Instance, uint32_t Device)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if((Instance != 1U) || ((Device < AUDIO_IN_DIGITAL_MIC1) && (Device > AUDIO_IN_DIGITAL_MIC_LAST)))
  {
    ret = BSP_ERROR_WRONG_PARAM;  
  }
  else
  {
    return BSP_ERROR_WRONG_PARAM;
  }      
  
  /* Return BSP status */
  return ret;
}

/**
* @brief  Resume the audio file stream
* @param  Instance  AUDIO IN Instance. It can be 1(DFSDM used)
* @param  Device    Digital mic to be resumed
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_ResumeChannels(uint32_t Instance, uint32_t Device)
{
  int32_t ret = BSP_ERROR_NONE;
  if((Instance != 1U) || ((Device < AUDIO_IN_DIGITAL_MIC1) && (Device > AUDIO_IN_DIGITAL_MIC_LAST)))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  
  /* Return BSP status */
  return ret;
}

/**
* @brief  Start audio recording.
* @param  Instance  AUDIO IN SAI PDM Instance. It can be only 2
* @param  pBuf     Main buffer pointer for the recorded data storing  
* @param  NbrOfBytes     Size of the record buffer
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_RecordPDM(uint32_t Instance, uint8_t* pBuf, uint32_t NbrOfBytes)
{
  int32_t ret;
  
  if(Instance != 2U)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else 
  {
    return BSP_ERROR_WRONG_PARAM;
  }
  
  /* Return BSP status */
  return ret;
}


/**
* @brief  Set Audio In device
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Device    The audio input device to be used
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_SetDevice(uint32_t Instance, uint32_t Device)
{
  int32_t ret = BSP_ERROR_NONE;
  BSP_AUDIO_Init_t audio_init;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioInCtx[Instance].State == AUDIO_IN_STATE_STOP)
  {  
    if(Instance == 1U)
    {
      return BSP_ERROR_WRONG_PARAM;
    }
    audio_init.Device = Device;
    audio_init.ChannelsNbr   = AudioInCtx[Instance].ChannelsNbr;  
    audio_init.SampleRate    = AudioInCtx[Instance].SampleRate;   
    audio_init.BitsPerSample = AudioInCtx[Instance].BitsPerSample;
    audio_init.Volume        = AudioInCtx[Instance].Volume;
    
    if(BSP_AUDIO_IN_Init(Instance, &audio_init) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_NO_INIT;
    }
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */  
  return ret;
}

/**
* @brief  Get Audio In device
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Device    The audio input device used
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_GetDevice(uint32_t Instance, uint32_t *Device)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
    /* Return audio Input Device */
    *Device = AudioInCtx[Instance].Device;
  }
  return ret;
}

/**
* @brief  Set Audio In frequency
* @param  Instance     Audio IN instance
* @param  SampleRate  Input frequency to be set
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_SetSampleRate(uint32_t Instance, uint32_t  SampleRate)
{
  int32_t ret = BSP_ERROR_NONE;
  BSP_AUDIO_Init_t audio_init;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioInCtx[Instance].State == AUDIO_IN_STATE_STOP)
  {
    if(Instance == 1U)
    {
      return BSP_ERROR_WRONG_PARAM;
    }
    audio_init.Device        = AudioInCtx[Instance].Device;
    audio_init.ChannelsNbr   = AudioInCtx[Instance].ChannelsNbr;  
    audio_init.SampleRate    = SampleRate;   
    audio_init.BitsPerSample = AudioInCtx[Instance].BitsPerSample;
    audio_init.Volume        = AudioInCtx[Instance].Volume; 
    if(BSP_AUDIO_IN_Init(Instance, &audio_init) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_NO_INIT;
    }   
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */
  return ret;  
}

/**
* @brief  Get Audio In frequency
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  SampleRate  Audio Input frequency to be returned
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_GetSampleRate(uint32_t Instance, uint32_t *SampleRate)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Return audio in frequency */
    *SampleRate = AudioInCtx[Instance].SampleRate;
  }
  
  /* Return BSP status */  
  return ret;
}

/**
* @brief  Set Audio In Resolution
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  BitsPerSample  Input resolution to be set
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_SetBitsPerSample(uint32_t Instance, uint32_t BitsPerSample)
{
  int32_t ret = BSP_ERROR_NONE;
  BSP_AUDIO_Init_t audio_init;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if(AudioInCtx[Instance].State == AUDIO_IN_STATE_STOP)
  {
    if(Instance == 1U)
    {
      return BSP_ERROR_WRONG_PARAM;
    }
    audio_init.Device        = AudioInCtx[Instance].Device;
    audio_init.ChannelsNbr   = AudioInCtx[Instance].ChannelsNbr;  
    audio_init.SampleRate    = AudioInCtx[Instance].SampleRate;   
    audio_init.BitsPerSample = BitsPerSample;
    audio_init.Volume        = AudioInCtx[Instance].Volume; 
    if(BSP_AUDIO_IN_Init(Instance, &audio_init) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_NO_INIT;
    }
  }
  else
  {
    ret = BSP_ERROR_BUSY;
  }
  
  /* Return BSP status */  
  return ret;
}

/**
* @brief  Get Audio In Resolution
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  BitsPerSample  Input resolution to be returned
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_GetBitsPerSample(uint32_t Instance, uint32_t *BitsPerSample)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
    /* Return audio in resolution */
    *BitsPerSample = AudioInCtx[Instance].BitsPerSample;
  }
  return ret;
}

/**
* @brief  Set Audio In Channel number
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  ChannelNbr  Channel number to be used
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_SetChannelsNbr(uint32_t Instance, uint32_t ChannelNbr)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if((Instance >= AUDIO_IN_INSTANCES_NBR) || (ChannelNbr > 2U))
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Update AudioIn Context */
    AudioInCtx[Instance].ChannelsNbr = ChannelNbr;
  }
  /* Return BSP status */
  return ret;
}

/**
* @brief  Get Audio In Channel number
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  ChannelNbr  Channel number to be used
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_GetChannelsNbr(uint32_t Instance, uint32_t *ChannelNbr)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Channel number to be returned */
    *ChannelNbr = AudioInCtx[Instance].ChannelsNbr;
  }
  return ret;
}

/**
* @brief  Set the current audio in volume level.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Volume    Volume level to be returnd
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_SetVolume(uint32_t Instance, uint32_t Volume)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (Instance == 0U)
  {
    uint32_t index = 0;
    for (index = 0; index < AudioInCtx[Instance].ChannelsNbr; index++)
    {
      if (PDM_FilterConfig[index].mic_gain != VolumeGain[Volume])
      {
        PDM_FilterConfig[index].mic_gain = VolumeGain[Volume];
        PDM_Filter_setConfig((PDM_Filter_Handler_t *)&PDM_FilterHandler[index], &PDM_FilterConfig[index]);
      }
    }
  }
  else
  {
    /* Update AudioIn Context */
    AudioInCtx[Instance].Volume = Volume;
  }
  /* Return BSP status */
  return ret;  
}

/**
* @brief  Get the current audio in volume level.
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  Volume    Volume level to be returnd
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_GetVolume(uint32_t Instance, uint32_t *Volume)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }  
  else
  {
    /* Input Volume to be returned */
    *Volume = AudioInCtx[Instance].Volume;
  }
  /* Return BSP status */
  return ret;  
}

/**
* @brief  Get Audio In device
* @param  Instance  AUDIO IN Instance. It can be 0 when I2S / SPI is used or 1 if DFSDM is used
* @param  State     Audio Out state
* @retval BSP status
*/
int32_t BSP_AUDIO_IN_GetState(uint32_t Instance, uint32_t *State)
{
  int32_t ret = BSP_ERROR_NONE;
  
  if(Instance >= AUDIO_IN_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {  
    /* Input State to be returned */
    *State = AudioInCtx[Instance].State;
  }
  return ret;
}

/**
* @brief Rx Transfer completed callbacks. It performs demuxing of the bit-interleaved PDM streams into 
byte-interleaved data suitable for PDM to PCM conversion. 1 ms of data for each microphone is 
written into the buffer that the user indicates when calling the BSP_AUDIO_IN_Start(...) function.
* @param hi2s: I2S handle
* @retval None
*/
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  
  uint32_t index = 0;
  
  switch(AudioInCtx[0].ChannelsNbr){
  case 1:
    {
      uint16_t * DataTempI2S = &I2S_InternalBuffer[AudioInCtx[0].Size/4] ;
      for(index = 0; index < AudioInCtx[0].Size/4; index++)
      {
        AudioInCtx[0].pBuff[index] = HTONS(DataTempI2S[index]);
      }
      break;
    }
    
  case 2:
    {      
      uint16_t * DataTempI2S = &(I2S_InternalBuffer[AudioInCtx[0].Size/2]);
      uint8_t a,b=0;
      for(index=0; index<AudioInCtx[0].Size/2; index++) {
        a = ((uint8_t *)(DataTempI2S))[(index*2)];
        b = ((uint8_t *)(DataTempI2S))[(index*2)+1];
        ((uint8_t *)(AudioInCtx[0].pBuff))[(index*2)] = Channel_Demux[a & CHANNEL_DEMUX_MASK] | Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4;;
        ((uint8_t *)(AudioInCtx[0].pBuff))[(index*2)+1] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] |Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4;
      }
      break;
    }    
  case 4:
    {      
      uint16_t * DataTempI2S = &(I2S_InternalBuffer[AudioInCtx[0].Size/2]);
      uint16_t * DataTempSPI = &(SPI_InternalBuffer[AudioInCtx[0].Size/2]);
      uint8_t a,b=0;
      for(index=0; index<AudioInCtx[0].Size/2; index++) {
        
        a = ((uint8_t *)(DataTempI2S))[(index*2)];
        b = ((uint8_t *)(DataTempI2S))[(index*2)+1];
        ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4)] = Channel_Demux[a & CHANNEL_DEMUX_MASK] |
          Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4;;
          ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4)+1] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] |
            Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4;
            
            a = ((uint8_t *)(DataTempSPI))[(index*2)];
            b = ((uint8_t *)(DataTempSPI))[(index*2)+1];
            ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4)+2] = Channel_Demux[a & CHANNEL_DEMUX_MASK] |
              Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4;;
              ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4)+3] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] |
                Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4;
      }
      break;
    }
  default:
    {
      
      break;
    }
    
  }
  
  BSP_AUDIO_IN_TransferComplete_CallBack(0);
}

/**
* @brief Rx Transfer completed callbacks. It performs demuxing of the bit-interleaved PDM streams into 
byte-interleaved data suitable for PDM to PCM conversion. 1 ms of data for each microphone is 
written into the buffer that the user indicates when calling the BSP_AUDIO_IN_Start(...) function.
* @param hi2s: I2S handle
* @retval None
*/
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  uint32_t index = 0;
  switch(AudioInCtx[0].ChannelsNbr){
  case 1:
    {
      uint16_t * DataTempI2S = I2S_InternalBuffer;
      for(index = 0; index < AudioInCtx[0].Size/4; index++)
      {
        AudioInCtx[0].pBuff[index] = HTONS(DataTempI2S[index]);
      }
      break;
    }    
  case 2:
    {      
      uint16_t * DataTempI2S = I2S_InternalBuffer;
      uint8_t a,b=0;
      for(index=0; index<AudioInCtx[0].Size/2; index++) {
        a = ((uint8_t *)(DataTempI2S))[(index*2)];
        b = ((uint8_t *)(DataTempI2S))[(index*2)+1];
        ((uint8_t *)(AudioInCtx[0].pBuff))[(index*2)] = Channel_Demux[a & CHANNEL_DEMUX_MASK] |
          Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4;;
          ((uint8_t *)(AudioInCtx[0].pBuff))[(index*2)+1] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] |
            Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4;
      }      
      break;
    }    
  case 4:
    {      
      uint16_t * DataTempI2S = I2S_InternalBuffer;
      uint16_t * DataTempSPI = SPI_InternalBuffer;
      uint8_t a,b=0;
      for(index=0; index<AudioInCtx[0].Size/2; index++) 
      {        
        a = ((uint8_t *)(DataTempI2S))[(index*2)];
        b = ((uint8_t *)(DataTempI2S))[(index*2)+1];
        ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4)] = Channel_Demux[a & CHANNEL_DEMUX_MASK] |
          Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4;;
          ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4)+1] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] |
            Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4;
            
            a = ((uint8_t *)(DataTempSPI))[(index*2)];
            b = ((uint8_t *)(DataTempSPI))[(index*2)+1];
            ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4)+2] = Channel_Demux[a & CHANNEL_DEMUX_MASK] |
              Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4;;
              ((uint8_t *)(AudioInCtx[0].pBuff))[(index*4)+3] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] |
                Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4;
      }
      break;   
    }
  default:
    {      
      break;
    }
    
  }
  
  BSP_AUDIO_IN_HalfTransfer_CallBack(0);
}

/**
* @brief  User callback when record buffer is filled.
* @param  Instance  AUDIO IN Instance.
* @retval None
*/
__weak void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
  
  /* This function should be implemented by the user application.
  It is called into this driver when the current buffer is filled
  to prepare the next buffer pointer and its size. */
}

/**
* @brief  Manages the DMA Half Transfer complete event.
* @param  Instance  AUDIO IN Instance.
* @retval None
*/
__weak void BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
  
  /* This function should be implemented by the user application.
  It is called into this driver when the current buffer is filled
  to prepare the next buffer pointer and its size. */
}

/**
* @brief  Audio IN Error callback function.
* @param  Instance  AUDIO IN Instance.
* @retval None
*/
__weak void BSP_AUDIO_IN_Error_CallBack(uint32_t Instance)
{ 
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
  
  /* This function is called when an Interrupt due to transfer error on or peripheral
  error occurs. */
}

/*******************************************************************************
Static Functions
*******************************************************************************/
#if (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1U)
/**
* @brief  Regular conversion complete callback. 
* @note   In interrupt mode, user has to read conversion value in this function
using HAL_DFSDM_FilterGetRegularValue.
* @param  hdfsdm_filter   DFSDM filter handle.
* @retval None
*/
static void DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  uint32_t i, j = 0;
  uint32_t index;
  
  if(AudioInCtx[1].IsMultiBuff == 1U)
  {
    /* Call the record update function to get the second half */
    BSP_AUDIO_IN_TransferComplete_CallBack(1);
  }
  else
  {   
    if(hdfsdm_filter == &hAudioInDfsdmFilter[POS_VAL(AUDIO_IN_DIGITAL_MIC1)])
    {
      
      for(j=0; j < AudioInCtx[1].ChannelsNbr; j ++)
      {
        for (i = 0; i < (AudioInCtx[1].SampleRate / 1000); i++)
        {
          AudioInCtx[1].HP_Filters[j].Z = ((MicRecBuff[j][i + (AudioInCtx[1].SampleRate / 1000) ] >> 8) * AudioInCtx[1].Volume) >> 7;
          AudioInCtx[1].HP_Filters[j].oldOut = (0xFC * (AudioInCtx[1].HP_Filters[j].oldOut + AudioInCtx[1].HP_Filters[j].Z - AudioInCtx[1].HP_Filters[j].oldIn)) / 256;
          AudioInCtx[1].HP_Filters[j].oldIn = AudioInCtx[1].HP_Filters[j].Z;
          AudioInCtx[1].pBuff[i * AudioInCtx[1].ChannelsNbr + j] = SaturaLH(AudioInCtx[1].HP_Filters[j].oldOut, -32760, 32760);
          RecBuffTrigger +=1U;
        }		
      }	
    }
    
    /* Call Half Transfer Complete callback */
    if(RecBuffTrigger == (AudioInCtx[1].Size/2U))
    {
      if(RecBuffHalf == 0U)
      {
        RecBuffHalf = 1;  
        BSP_AUDIO_IN_HalfTransfer_CallBack(1);
      }
    }
    /* Call Transfer Complete callback */
    if(RecBuffTrigger == AudioInCtx[1].Size)
    {
      /* Reset Application Buffer Trigger */
      RecBuffTrigger = 0;
      RecBuffHalf = 0; 
      /* Call the record update function to get the next buffer to fill and its size (size is ignored) */
      BSP_AUDIO_IN_TransferComplete_CallBack(1);
    }
  }
}

/**
* @brief  Half regular conversion complete callback. 
* @param  hdfsdm_filter   DFSDM filter handle.
* @retval None
*/
static void DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  uint32_t i, j = 0;
  uint32_t index;
  
  if(AudioInCtx[1].IsMultiBuff == 1U)
  {
    /* Call the record update function to get the first half */
    BSP_AUDIO_IN_HalfTransfer_CallBack(1);
  }
  else
    if(hdfsdm_filter == &hAudioInDfsdmFilter[POS_VAL(AUDIO_IN_DIGITAL_MIC1)])
    {
      
      for(j=0; j < AudioInCtx[1].ChannelsNbr; j ++)
      {
        for (i = 0; i < (AudioInCtx[1].SampleRate / 1000); i++)
        {
          AudioInCtx[1].HP_Filters[j].Z = ((MicRecBuff[j][i] >> 8) * AudioInCtx[1].Volume) >> 7;
          AudioInCtx[1].HP_Filters[j].oldOut = (0xFC * (AudioInCtx[1].HP_Filters[j].oldOut + AudioInCtx[1].HP_Filters[j].Z - AudioInCtx[1].HP_Filters[j].oldIn)) / 256;
          AudioInCtx[1].HP_Filters[j].oldIn = AudioInCtx[1].HP_Filters[j].Z;
          AudioInCtx[1].pBuff[i * AudioInCtx[1].ChannelsNbr + j] = SaturaLH(AudioInCtx[1].HP_Filters[j].oldOut, -32760, 32760);
          RecBuffTrigger +=1U;
        }		
      }	
    }
  
  /* Call Half Transfer Complete callback */
  if(RecBuffTrigger == (AudioInCtx[1].Size/2U))
  {
    if(RecBuffHalf == 0U)
    {
      RecBuffHalf = 1;  
      BSP_AUDIO_IN_HalfTransfer_CallBack(1);
    }
  }
  /* Call Transfer Complete callback */
  if(RecBuffTrigger == AudioInCtx[1].Size)
  {
    /* Reset Application Buffer Trigger */
    RecBuffTrigger = 0;
    RecBuffHalf = 0; 
    /* Call the record update function to get the next buffer to fill and its size (size is ignored) */
    BSP_AUDIO_IN_TransferComplete_CallBack(1);
  }
}
}
#endif /* (USE_HAL_DFSDM_REGISTER_CALLBACKS == 1U) */

static void I2S_MspInit(I2S_HandleTypeDef *hi2s)
{	
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable the I2S2 peripheral clock */
  AUDIO_IN_I2S_CLK_ENABLE();
  
  /* Enable I2S GPIO clocks */
  AUDIO_IN_I2S_SCK_GPIO_CLK_ENABLE();
  AUDIO_IN_I2S_MOSI_GPIO_CLK_ENABLE();
  
  /* I2S2 pins configuration: SCK and MOSI pins ------------------------------*/
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  
  
  GPIO_InitStruct.Pin       = AUDIO_IN_I2S_SCK_PIN;
  GPIO_InitStruct.Alternate = AUDIO_IN_I2S_SCK_AF;
  HAL_GPIO_Init(AUDIO_IN_I2S_SCK_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin       = AUDIO_IN_I2S_MOSI_PIN ;
  GPIO_InitStruct.Alternate = AUDIO_IN_I2S_MOSI_AF;
  HAL_GPIO_Init(AUDIO_IN_I2S_MOSI_GPIO_PORT, &GPIO_InitStruct);
 
} 

static void SPI_MspInit(SPI_HandleTypeDef *hspi)
{  
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable GPIO TX/RX clock */
  AUDIO_IN_SPI_SCK_GPIO_CLK_ENABLE();
  AUDIO_IN_SPI_MISO_GPIO_CLK_ENABLE();
  AUDIO_IN_SPI_MOSI_GPIO_CLK_ENABLE();
  /* Enable SPI3 clock */
  AUDIO_IN_SPI_CLK_ENABLE();
  /* Enable DMA1 clock */
  AUDIO_IN_SPI_DMAx_CLK_ENABLE();
  
  /*##-2- Configure peripheral GPIO ##########################################*/
  /* SPI SCK GPIO pin configuration  */
  GPIO_InitStruct.Pin       = AUDIO_IN_SPI_SCK_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = AUDIO_IN_SPI_SCK_AF;
  
  HAL_GPIO_Init(AUDIO_IN_SPI_SCK_GPIO_PORT, &GPIO_InitStruct);
  
  /* SPI MOSI GPIO pin configuration  */
  GPIO_InitStruct.Pin = AUDIO_IN_SPI_MOSI_PIN;
  GPIO_InitStruct.Alternate = AUDIO_IN_SPI_MOSI_AF;
  HAL_GPIO_Init(AUDIO_IN_SPI_MOSI_GPIO_PORT, &GPIO_InitStruct);
  
  
}


/**
* @brief Audio Timer Init
* @retval None
*/
static uint8_t AUDIO_IN_Timer_Init(void)
{
  
  GPIO_InitTypeDef   GPIO_InitStruct;
  
  /* Enable AUDIO_TIMER clock*/
  AUDIO_IN_TIMER_CLK_ENABLE();
  AUDIO_IN_TIMER_CHOUT_GPIO_PORT_CLK_ENABLE();
  AUDIO_IN_TIMER_CHIN_GPIO_PORT_CLK_ENABLE();
  
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  
  GPIO_InitStruct.Alternate = AUDIO_IN_TIMER_CHIN_AF;
  GPIO_InitStruct.Pin = AUDIO_IN_TIMER_CHIN_PIN;
  HAL_GPIO_Init(AUDIO_IN_TIMER_CHIN_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Alternate = AUDIO_IN_TIMER_CHOUT_AF;
  GPIO_InitStruct.Pin = AUDIO_IN_TIMER_CHOUT_PIN;
  HAL_GPIO_Init(AUDIO_IN_TIMER_CHOUT_GPIO_PORT, &GPIO_InitStruct);
  
  TimDividerHandle.Instance = AUDIO_IN_TIMER;
  
  /* Configure the Input: channel_1 */
  sICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
  sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
  sICConfig.ICFilter = 0;
  if(HAL_TIM_IC_ConfigChannel(&TimDividerHandle, &sICConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  /* Configure TIM1 in Gated Slave mode for the external trigger (Filtered Timer
  Input 1) */
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.SlaveMode    = TIM_SLAVEMODE_EXTERNAL1;
  if( HAL_TIM_SlaveConfigSynchronization(&TimDividerHandle, &sSlaveConfig) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  /* Initialize TIM3 peripheral in PWM mode*/
  TimDividerHandle.Init.Period            = 1;
  TimDividerHandle.Init.Prescaler         = 0;
  TimDividerHandle.Init.ClockDivision     = 0;
  TimDividerHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimDividerHandle.Init.RepetitionCounter = 0;
  if(HAL_TIM_PWM_Init(&TimDividerHandle) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  /* Configure the PWM_channel_1  */
  sOCConfig.OCMode     = TIM_OCMODE_PWM1;
  sOCConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sOCConfig.Pulse = 1;
  if(HAL_TIM_PWM_ConfigChannel(&TimDividerHandle, &sOCConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

/**
* @brief Audio Timer Start
* @retval None
*/
static uint8_t AUDIO_IN_Timer_Start(){
  
  if(HAL_TIM_IC_Start(&TimDividerHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* Start the Output Compare */
  if(HAL_TIM_OC_Start(&TimDividerHandle, TIM_CHANNEL_2) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  return HAL_OK;
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
