/**
  ******************************************************************************
  * @file    cca02m2_conf.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version V2.4.0
  * @date    07-June-2021
  * @brief   This file contains definitions for the MEMSMIC1 applications
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
#ifndef CCA02M2_CONF_H__
#define CCA02M2_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif
  
/* Includes ------------------------------------------------------------------*/  
/* Replace the header file names with the ones of the target platform */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo_bus.h"
#include "stm32f4xx_nucleo_errno.h"

/* The N_MS value defines the number of millisecond to be processed at each AudioProcess call,
that must be consistent with the N_MS_PER_INTERRUPT defined in the audio driver
(cca02m2_audio.h).
The default value of the N_MS_PER_INTERRUPT directive in the driver is set to 1, 
for backward compatibility: leaving this values as it is allows to avoid any 
modification in the application layer developed with the older versions of the driver */

#define AUDIO_IN_CHANNELS 2
#define AUDIO_IN_SAMPLING_FREQUENCY     16000
  
#define AUDIO_IN_BUFFER_SIZE            DEFAULT_AUDIO_IN_BUFFER_SIZE  
#define AUDIO_VOLUME_INPUT              64U
#define CCA02M2_AUDIO_INSTANCE          0U /*Select 0U or 1U in base of the peripheral used (0U->I2S/SAI 1U->DFSDM)*/
#define CCA02M2_AUDIO_IN_IT_PRIORITY    6U

#if (AUDIO_IN_SAMPLING_FREQUENCY == 8000)
#define MAX_DECIMATION_FACTOR 160
#else
#define MAX_DECIMATION_FACTOR 128
#endif
  
#define N_MS                    (N_MS_PER_INTERRUPT)
#define PCM_AUDIO_IN_SAMPLES    AUDIO_IN_SAMPLING_FREQUENCY/1000

#define PDM_FREQ_16K 	1280
  
  /*#define USE_SPI3*/
  /*If you wanto to use SPI3 instead of SPI2 for M3 and M4, uncomment this define and 
  close SB20 and SB21*/

#ifdef __cplusplus
}
#endif

#endif /* CCA02M2_CONF_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

