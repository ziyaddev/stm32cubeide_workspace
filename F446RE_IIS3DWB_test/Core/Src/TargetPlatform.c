/**
  ******************************************************************************
  * @file    TargetPlatform.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version V2.4.0
  * @date    07-June-2021
  * @brief   Initialization of the Target Platform
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
#include <stdio.h>
#include "TargetFeatures.h"
#include "PREDMNT1_config.h"
//#include "sensor_service.h"

/** @addtogroup Projects
  * @{
  */

/** @addtogroup DEMONSTRATIONS Demonstrations
  * @{
  */

/** @addtogroup PREDCTIVE_MAINTENANCE Predictive Maintenance
  * @{
  */

/** @addtogroup PREDCTIVE_MAINTENANCE_TARGET_PLATFORM Predictive Maintenance Target Platform
  * @{
  */
    
/** @defgroup PREDCTIVE_MAINTENANCE_TARGET_PLATFORM_EXPORTED_VARIABLES Predictive Maintenance Target Platform Exported Variables
  * @{
  */

/* Exported variables ---------------------------------------------------------*/
TargetFeatures_t TargetBoardFeatures;

uint16_t PDM_Buffer[((((AUDIO_IN_CHANNELS * AUDIO_IN_SAMPLING_FREQUENCY) / 1000) * MAX_DECIMATION_FACTOR) / 16)* N_MS ];

/**
  * @}
  */

/** @defgroup PREDCTIVE_MAINTENANCE_TARGET_PLATFORM_PRIVATE_VARIABLES Predictive Maintenance Target Platform Private Variables
  * @{
  */


/**
  * @}
  */

/** @defgroup PREDCTIVE_MAINTENANCE_TARGET_PLATFORM_PRIVATE_FUNCTIONS_PROTOTYPES Predictive Maintenance Target Platform Private Functions prototypes
  * @{
  */

/* Local function prototypes --------------------------------------------------*/
#ifdef IKS01A2_LSM6DSL_0
  static void LSM6DSL_GPIO_Init(void);
#endif /* IKS01A2_LSM6DSL_0 */
  
#if (defined(IKS01A2_ISM330DLC_0) || defined(IKS01A3_ISM330DLC_0))
  static void ISM330DLC_GPIO_Init(void);
#endif /* (defined(IKS01A2_ISM330DLC_0) || defined(IKS01A3_ISM330DLC_0)) */
  
static void Init_MEMS_Sensors(void);
static void Init_MEMS_Mics(uint32_t AudioFreq, uint32_t AudioVolume);

/**
  * @}
  */

/** @defgroup PREDCTIVE_MAINTENANCE_TARGET_PLATFORM_PRIVATE_FUNCTIONS Predictive Maintenance Target Platform Private Functions
  * @{
  */

#ifdef IKS01A2_LSM6DSL_0
/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void LSM6DSL_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0x03, 0x00);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}
#endif /* IKS01A2_LSM6DSL_0 */

#ifdef IKS01A2_ISM330DLC_0
/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void ISM330DLC_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  M_INT2_O_GPIO_CLK_ENABLE();

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = M_INT2_O_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(M_INT2_O_GPIO_PORT, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(M_INT2_O_EXTI_IRQn, 0x01, 0x00);
  HAL_NVIC_EnableIRQ(M_INT2_O_EXTI_IRQn);
}
#endif /* IKS01A2_ISM330DLC_0 */

#ifdef IKS01A3_ISM330DLC_0
/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void ISM330DLC_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
#if (USE_SPI_FOR_DIL24 == 1)
  DIL24_SPI_CS_CLK_ENABLE();
  
  /* Configure CS_PIN pin */
  GPIO_InitStruct.Pin   = DIL24_SPI_CS_PIN;
  GPIO_InitStruct.Mode  = DIL24_SPI_CS_MODE;
  GPIO_InitStruct.Pull  = DIL24_SPI_CS_PULL;
  GPIO_InitStruct.Speed = DIL24_SPI_CS_SPEED;
  HAL_GPIO_Init(DIL24_SPI_CS_PORT, &GPIO_InitStruct);
  
  /* SD chip select high */
  HAL_GPIO_WritePin(DIL24_SPI_CS_PORT, DIL24_SPI_CS_PIN, GPIO_PIN_SET);
#endif /* USE_SPI_FOR_DIL24 */

  /* Configure ISM330DLC INT2 pin ************************************* */
  /* GPIO Ports Clock Enable */
  M_INT2_O_GPIO_CLK_ENABLE();

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = M_INT2_O_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(M_INT2_O_GPIO_PORT, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(M_INT2_O_EXTI_IRQn, 0x03, 0x00);
  HAL_NVIC_EnableIRQ(M_INT2_O_EXTI_IRQn);
}
#endif /* IKS01A3_ISM330DLC_0 */

/** @brief Initialize all the MEMS1 sensors
 * @param None
 * @retval None
 */
static void Init_MEMS_Sensors(void)
{
  PREDMNT1_PRINTF("\nCode compiled for X-NUCLEO-IKS01A3 board\n\r");
  
#if (defined(IKS01A2_ISM330DLC_0) || defined(IKS01A3_ISM330DLC_0))
  #if (USE_SPI_FOR_DIL24 == 1)
    PREDMNT1_PRINTF("\tISM330DLC DIL24 Present (SPI connection)\n\r");
  #else /* USE_SPI_FOR_DIL24 */
    PREDMNT1_PRINTF("\tISM330DLC DIL24 Present (I2C connection)\n\r");
  #endif /* USE_SPI_FOR_DIL24 */
#endif /* (defined(IKS01A2_ISM330DLC_0) || defined(IKS01A3_ISM330DLC_0)) */
  
   /* Accelero & Gyro initialization */
  if(MOTION_SENSOR_Init(ACCELERO_INSTANCE, MOTION_ACCELERO | MOTION_GYRO)==BSP_ERROR_NONE)
  {
    TargetBoardFeatures.AccSensorIsInit= 1;
    TargetBoardFeatures.GyroSensorIsInit= 1;
    
    PREDMNT1_PRINTF("\t\tOK Accelero Sensor\n\r\t\tOK Gyroscope Sensor\n\r");
    //PREDMNT1_PRINTF("\t\tOK Gyroscope Sensor\n\r");
  }
  else
  {
    PREDMNT1_PRINTF("\t\tError Accelero Sensor\n\r\t\tError Gyroscope Sensor\n\r");
    //PREDMNT1_PRINTF("\t\tError Gyroscope Sensor\n\r");
  }
    
  /* Magneto initialization */
  if(MOTION_SENSOR_Init(MAGNETO_INSTANCE, MOTION_MAGNETO)==BSP_ERROR_NONE)
  {
    TargetBoardFeatures.MagSensorIsInit= 1;

    PREDMNT1_PRINTF("\tOK Magneto Sensor\n\r");
  }
  else
  {
    PREDMNT1_PRINTF("\tError Magneto Sensor\n\r");
  }
    
  if(ENV_SENSOR_Init(HUMIDITY_INSTANCE,ENV_TEMPERATURE| ENV_HUMIDITY)==BSP_ERROR_NONE)
  {
    TargetBoardFeatures.TempSensorsIsInit[0]= 1;
    TargetBoardFeatures.HumSensorIsInit= 1;
    TargetBoardFeatures.NumTempSensors++;
    PREDMNT1_PRINTF("\tOK Temperature and Humidity (Sensor1)\n\r");
  }
  else
  {
    PREDMNT1_PRINTF("\tError Temperature and Humidity (Sensor1)\n\r");
  }

  if(ENV_SENSOR_Init(PRESSURE_INSTANCE,ENV_TEMPERATURE| ENV_PRESSURE)==BSP_ERROR_NONE)
  {
    TargetBoardFeatures.TempSensorsIsInit[1]= 1;
    TargetBoardFeatures.PressSensorIsInit= 1;
    TargetBoardFeatures.NumTempSensors++;
    PREDMNT1_PRINTF("\tOK Temperature and Pressure (Sensor2)\n\r");
  }
  else
  {
    PREDMNT1_PRINTF("\tError Temperature and Pressure (Sensor2)\n\r");
  }

  /*  Enable all the sensors */
  if(TargetBoardFeatures.AccSensorIsInit)
  {
    if(MOTION_SENSOR_Enable(ACCELERO_INSTANCE, MOTION_ACCELERO)==BSP_ERROR_NONE)
      PREDMNT1_PRINTF("\tEnabled Accelero Sensor\n\r");
  }
  
  if(TargetBoardFeatures.GyroSensorIsInit)
  {
    if(MOTION_SENSOR_Enable(GYRO_INSTANCE, MOTION_GYRO)==BSP_ERROR_NONE)
      PREDMNT1_PRINTF("\tEnabled Gyroscope Sensor\n\r");
  }
  
  if(TargetBoardFeatures.MagSensorIsInit)
  {
    if(MOTION_SENSOR_Enable(MAGNETO_INSTANCE, MOTION_MAGNETO)==BSP_ERROR_NONE)
      PREDMNT1_PRINTF("\tEnabled Magneto Sensor\n\r");
  }
   
  if(TargetBoardFeatures.TempSensorsIsInit[0])
  {
    if(ENV_SENSOR_Enable(TEMPERATURE_INSTANCE_1, ENV_TEMPERATURE)==BSP_ERROR_NONE)
      PREDMNT1_PRINTF("\tEnabled Temperature\t(Sensor1)\n\r");
  }
  
  if(TargetBoardFeatures.HumSensorIsInit)
  {
    if(ENV_SENSOR_Enable(HUMIDITY_INSTANCE, ENV_HUMIDITY)==BSP_ERROR_NONE)
      PREDMNT1_PRINTF("\tEnabled Humidity\t(Sensor1)\n\r");
  }
     
  if(TargetBoardFeatures.TempSensorsIsInit[1])
  {
    if(ENV_SENSOR_Enable(TEMPERATURE_INSTANCE_2, ENV_TEMPERATURE)==BSP_ERROR_NONE)
      PREDMNT1_PRINTF("\tEnabled Temperature\t(Sensor2)\n\r");
  }
  
  if(TargetBoardFeatures.PressSensorIsInit)
  {
    if(ENV_SENSOR_Enable(PRESSURE_INSTANCE, ENV_PRESSURE)==BSP_ERROR_NONE)
      PREDMNT1_PRINTF("\tEnabled Pressure\t(Sensor2)\n\r");
  }
}

/** @brief Initialize all the MEMS's Microphones
 * @param None
 * @retval None
 */
static void Init_MEMS_Mics(uint32_t AudioFreq, uint32_t AudioVolume)
{
  CCA02M2_AUDIO_Init_t MicParams;
  
  /* Initialize microphone acquisition */  
  MicParams.BitsPerSample = 16;
  MicParams.ChannelsNbr = AUDIO_IN_CHANNELS;
  MicParams.Device = AUDIO_IN_DIGITAL_MIC;
  MicParams.SampleRate = AudioFreq;
  MicParams.Volume = AudioVolume;
  
  if( CCA02M2_AUDIO_IN_Init(CCA02M2_AUDIO_INSTANCE, &MicParams) != BSP_ERROR_NONE )
  {
    PREDMNT1_PRINTF("\nError Audio Init\r\n");
    
    while(1) {
      ;
    }
  }
  else
  {
    PREDMNT1_PRINTF("\nOK Audio Init\t(Audio Freq.= %ld)\r\n", AudioFreq);
  }
  
  /* Set the volume level */
  if( CCA02M2_AUDIO_IN_SetVolume(CCA02M2_AUDIO_INSTANCE, AudioVolume) != BSP_ERROR_NONE )
  {
    PREDMNT1_PRINTF("Error Audio Volume\r\n\n");
    
    while(1) {
      ;
    }
  }
  else
  {
    PREDMNT1_PRINTF("OK Audio Volume\t(Volume= %ld)\r\n", AudioVolume);
  }
}

/**
  * @}
  */

/** @defgroup PREDCTIVE_MAINTENANCE_TARGET_PLATFORM_EXPORTED_FUNCTIONS Predictive Maintenance Target Platform Exported Functions
  * @{
  */

/**
  * @brief  Initialize all the Target platform's Features
  * @param  None
  * @retval None
  */
void InitTargetPlatform(void)
{
  #ifdef PREDMNT1_ENABLE_PRINTF
  /* UART Initialization */
  if(BSP_COM_Init(COM1) != BSP_ERROR_NONE) {
    Error_Handler();
  } else {
    PREDMNT1_PRINTF("\033[2J\033[1;1f");
    PREDMNT1_PRINTF("UART Initialized\r\n");
  }
  #endif /* PREDMNT1_ENABLE_PRINTF */
  
  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
  
  /* Initialize LED */
  BSP_LED_Init(LED2);

  PREDMNT1_PRINTF("\r\nSTMicroelectronics %s:\r\n"
          "\t%s\r\n"
          "\tVersion %c.%c.%c\r\n"
          "\tSTM32F446xx-Nucleo board"
          "\r\n\n",
          PREDMNT1_PACKAGENAME,
          CONFIG_NAME,
          PREDMNT1_VERSION_MAJOR,PREDMNT1_VERSION_MINOR,PREDMNT1_VERSION_PATCH);

  /* Reset all the Target's Features */
  memset(&TargetBoardFeatures, 0, sizeof(TargetFeatures_t));
  /* Discovery and Intialize all the MEMS Target's Features */
  
#ifdef IKS01A2_LSM6DSL_0
  LSM6DSL_GPIO_Init();
#endif /* IKS01A2_LSM6DSL_0 */
  
#if (defined(IKS01A2_ISM330DLC_0) || defined(IKS01A3_ISM330DLC_0))
  ISM330DLC_GPIO_Init();
#endif /* (defined(IKS01A2_ISM330DLC_0) || defined(IKS01A3_ISM330DLC_0)) */
  
  Init_MEMS_Sensors();
  
  /* Initialize Mic */
  Init_MEMS_Mics(AUDIO_IN_SAMPLING_FREQUENCY, AUDIO_VOLUME_INPUT);
  
  PREDMNT1_PRINTF("\n\r");
}

/** @brief Initialize all the MEMS's Microphones
 * @param None
 * @retval None
 */
void InitMics(uint32_t AudioFreq, uint32_t AudioVolume)
{
  Init_MEMS_Mics(AudioFreq, AudioVolume);
   
  CCA02M2_AUDIO_IN_Record(CCA02M2_AUDIO_INSTANCE, (uint8_t *) PDM_Buffer, 0);
}

/** @brief DeInitialize all the MEMS's Microphones
 * @param None
 * @retval None
 */
void DeInitMics(void)
{
  if( CCA02M2_AUDIO_IN_Stop(CCA02M2_AUDIO_INSTANCE) != BSP_ERROR_NONE )
  {
    PREDMNT1_PRINTF("Error Audio Stop\r\n");
    
    while(1) {
      ;
    }
  }
  else
    PREDMNT1_PRINTF("OK Audio Stop\r\n");
  
  
  if( CCA02M2_AUDIO_IN_DeInit(CCA02M2_AUDIO_INSTANCE) != BSP_ERROR_NONE )
  {
    PREDMNT1_PRINTF("Error Audio DeInit\r\n");
    
    while(1) {
      ;
    }
  }
  else
    PREDMNT1_PRINTF("OK Audio DeInit\r\n");
}


/**
  * @brief  This function switches on the LED
  * @param  None
  * @retval None
  */
void LedOnTargetPlatform(void)
{
  BSP_LED_On(LED2);
  TargetBoardFeatures.LedStatus=1;
}

/**
  * @brief  This function switches off the LED
  * @param  None
  * @retval None
  */
void LedOffTargetPlatform(void)
{
  BSP_LED_Off(LED2);
  TargetBoardFeatures.LedStatus=0;
}

/** @brief  This function toggles the LED
  * @param  None
  * @retval None
  */
void LedToggleTargetPlatform(void)
{
  BSP_LED_Toggle(LED2);
}

/**
 * @brief User function for Erasing the Flash data for MDM
 * @param None
 * @retval uint32_t Success/NotSuccess [1/0]
 */
uint32_t UserFunctionForErasingFlash(void) {
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  uint32_t Success=1;

  EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = FLASH_SECTOR_7;
  EraseInitStruct.NbSectors = 1;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK){
    /* Error occurred while sector erase. 
      User can add here some code to deal with this error. 
      SectorError will contain the faulty sector and then to know the code error on this sector,
      user can call function 'HAL_FLASH_GetError()'
      FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
    Success=0;
    Error_Handler();
  }

  /* Lock the Flash to disable the flash control register access (recommended
  to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  return Success;
}

/**
 * @brief User function for Saving the MDM  on the Flash
 * @param void *InitMetaDataVector Pointer to the MDM beginning
 * @param void *EndMetaDataVector Pointer to the MDM end
 * @retval uint32_t Success/NotSuccess [1/0]
 */
uint32_t UserFunctionForSavingFlash(void *InitMetaDataVector,void *EndMetaDataVector)
{
  uint32_t Success=1;

  /* Store in Flash Memory */
  uint32_t Address = MDM_FLASH_ADD;
  uint32_t *WriteIndex;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  for(WriteIndex =((uint32_t *) InitMetaDataVector); WriteIndex<((uint32_t *) EndMetaDataVector); WriteIndex++) {
    if (HAL_FLASH_Program(TYPEPROGRAM_WORD, Address,*WriteIndex) == HAL_OK){
      Address = Address + 4;
    } else {
      /* Error occurred while writing data in Flash memory.
         User can add here some code to deal with this error
         FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
      Error_Handler();
      Success=0;
    }
  }

  /* Lock the Flash to disable the flash control register access (recommended
   to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
 
  return Success;
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

/**
  * @}
  */

/******************* (C) COPYRIGHT 2021 STMicroelectronics *****END OF FILE****/
