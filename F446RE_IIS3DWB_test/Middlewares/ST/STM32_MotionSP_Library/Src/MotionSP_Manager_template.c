/**
  ******************************************************************************
  * @file    MotionSP_Manager_template.c
  * @author  System Research & Applications Team - Catania Lab
  * @version 2.3.2
  * @date    21-Oct-2020
  * @brief 	 MotionSP_Manager_template.c File
  * 		 This file should be copied to the application folder and renamed
  * 		 to MotionSP_Manager.c.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0055, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0055
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "TargetPlatform.h"
#include "MotionSP_Manager.h"
#include "idp005_motion_sensors.h"
#include "idp005_motion_sensors_ex.h"
#include <stdlib.h>


/** @addtogroup Projects
  * @{
  */

/** @addtogroup DEMONSTRATIONS Demonstrations
  * @{
  */

/** @addtogroup PREDICTIVE_MAINTENANCE_SRV Predictive Maintenance SRV
  * @{
  */

/** @addtogroup STM32_MOTIONSP_MANAGER MotionSP Manager
  * @{
  */

/** @defgroup STM32_MOTIONSP_MANAGER_PRIVATE_VARIABLES MotionSP Manager Private Variables
  * @{
  */

uint8_t RestartFlag = 1;
sAccPmtr_t AcceleroParams;
uint32_t StartTick = 0;
uint8_t MotionSP_Running = 0;
static uint16_t AccDrdyNr = 0;
static uint16_t accCircBuffIndexForFftDelta = 0;
static uint8_t accCircBuffIndexForFftOvf = 0;

uint8_t TD_Th_DFLT = 1;
uint8_t FD_Th_DFLT = 1;

/* X-Y-Z STATUS ALARM for TimeDomain */
sTimeDomainAlarm_t sTdAlarm;
/* X-Y-Z ALARM Threshold for Speed RMS TimeDomain */
sTimeDomainThresh_t sTdRmsThresholds;
/* X-Y-Z ALARM Threshold for Acc Peak TimeDomain */
sTimeDomainThresh_t sTdPkThresholds;
/* X-Y-Z parameter measured for TimeDomain */
sAcceleroParam_t sTimeDomainVal;
/* X-Y-Z Amplitude subranges Values that exceed thresholds */
sSubrange_t THR_Check;
/* X-Y-Z Frequency domain Subranges Status */
sFreqDomainAlarm_t THR_Fft_Alarms;

float *pFDAlarmThresh;
float *pFDWarnThresh;
Alarm_Type_t TotalTDAlarm;
Alarm_Type_t TotalFDAlarm;

float TDRmsThresh[3*2];
float TDPkThresh[3*2];
float FDWarnThresh_Sub[SUBRANGE_MAX*3];
float FDAlarmThresh_Sub[SUBRANGE_MAX*3];

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
static int32_t Accelerometer_Init(void);
static void MotionSP_SetDefaultVibrationParam(void);
static int32_t MotionSP_VibrationAnalysisInit(void);
static int32_t MotionSP_VibrationAnalysisStart(void);
static void MotionSP_VibrationAnalysisManager(void);
static int32_t MotionSP_VibrationAnalysisVariableInit(void);
static int32_t MotionSP_AcceleroConfig(void);
static int32_t MotionSP_AccMeasInit(void);
static int32_t AccOdrMeas(sAcceleroODR_t *pAcceleroODR);
static int32_t MotionSP_AcceleroFifoConfig(void);
static void MotionSP_TimeDomainAnalysis(void);
static void MotionSP_FrequencyDomainAnalysis(void);
static void FillAccCircBuffFromFifo(sCircBuff_t *pAccCircBuff, uint8_t *pFifoBuff, uint16_t FifoWtm, float AccSens, uint8_t Rst);

/** @addtogroup STM32_MOTIONSP_MANAGER_PRIVATE_FUNCTIONS MotionSP Manager Private Functions
  * @{
  */

/** @brief Initialize accelerometer
  * @retval BSP status
  */
static int32_t Accelerometer_Init(void)
{
  int32_t BSP_Error = BSP_ERROR_NONE;
  
  /* Accelero Init */
  if((BSP_Error = BSP_MOTION_SENSOR_Init(ACCELERO_INSTANCE, MOTION_ACCELERO)) == BSP_ERROR_NONE)
    IDP005_PRINTF("\tInitialized Acceleration Sensor\r\n");
  else
  {
    IDP005_PRINTF("\tError on Acceleration Sensor\r\n");
    return BSP_Error;
  }
    
  /*  Accelero Enable */
  if((BSP_Error = BSP_MOTION_SENSOR_Enable(ACCELERO_INSTANCE, MOTION_ACCELERO)) == BSP_ERROR_NONE)
    IDP005_PRINTF("\tEnabled Acceleration Sensor\r\n");
  else
  {
    IDP005_PRINTF("\tError on Acceleration Sensor\r\n");
    return BSP_Error;
  }
  
  return BSP_Error;
}

/**
  * @brief  Init the Accelerometer Settings and MotionSP Vibration parameters
  * @return None
  */   
static void MotionSP_SetDefaultVibrationParam(void)
{
  memset((void *)&AcceleroParams, 0, sizeof(sAccPmtr_t));
  
  /* Set default parameters for accelerometer */
  AcceleroParams.AccOdrCode     = ACCELERO_ODR_DEFAULT;
  AcceleroParams.AccFifoBdrCode = ACCELERO_FIFO_BDR_DEFAULT;
  AcceleroParams.fs             = ACCELERO_FS_DEFAULT;
  AcceleroParams.HwFilter       = ACCELERO_HW_FILTER_DEFAULT;
  
  switch(AcceleroParams.HwFilter)
  {
  case IIS3DWB_LP_5kHz:
    strcpy(MotionSP_ParameterName.HwFilterName, "LP 5 kHz");
    break;
  case IIS3DWB_LP_ODR_DIV_4:
    strcpy(MotionSP_ParameterName.HwFilterName, "LP ODR/4");
    break;
  case IIS3DWB_LP_ODR_DIV_10:
    strcpy(MotionSP_ParameterName.HwFilterName, "LP ODR/10");
    break;
  case IIS3DWB_LP_ODR_DIV_20:
    strcpy(MotionSP_ParameterName.HwFilterName, "LP ODR/20");
    break;
  case IIS3DWB_LP_ODR_DIV_45:
    strcpy(MotionSP_ParameterName.HwFilterName, "LP ODR/45");
    break;
  case IIS3DWB_LP_ODR_DIV_100:
    strcpy(MotionSP_ParameterName.HwFilterName, "LP ODR/100");
    break;
  case IIS3DWB_LP_ODR_DIV_200:
    strcpy(MotionSP_ParameterName.HwFilterName, "LP ODR/200");
    break;
  case IIS3DWB_LP_ODR_DIV_400:
    strcpy(MotionSP_ParameterName.HwFilterName, "LP ODR/400");
    break;
  case IIS3DWB_LP_ODR_DIV_800:
    strcpy(MotionSP_ParameterName.HwFilterName, "LP ODR/800");
    break;
  case IIS3DWB_SLOPE_ODR_DIV_4:
    strcpy(MotionSP_ParameterName.HwFilterName, "SLOPE ODR/4");
    break;
  case IIS3DWB_HP_ODR_DIV_10:
    strcpy(MotionSP_ParameterName.HwFilterName, "HP ODR/10");
    break;
  case IIS3DWB_HP_ODR_DIV_20:
    strcpy(MotionSP_ParameterName.HwFilterName, "HP ODR/20");
    break;
  case IIS3DWB_HP_ODR_DIV_45:
    strcpy(MotionSP_ParameterName.HwFilterName, "HP ODR/45");
    break;
  case IIS3DWB_HP_ODR_DIV_100:
    strcpy(MotionSP_ParameterName.HwFilterName, "HP ODR/100");
    break;
  case IIS3DWB_HP_ODR_DIV_200:
    strcpy(MotionSP_ParameterName.HwFilterName, "HP ODR/200");
    break;
  case IIS3DWB_HP_ODR_DIV_400:
    strcpy(MotionSP_ParameterName.HwFilterName, "HP ODR/400");
    break;
  case IIS3DWB_HP_ODR_DIV_800:
    strcpy(MotionSP_ParameterName.HwFilterName, "HP ODR/800");
    break;
    
  default:
    strcpy(MotionSP_ParameterName.HwFilterName, "");
    break;
  }
  
  switch(AcceleroParams.AccOdrCode)
  {

  case IIS3DWB_XL_ODR_26k7Hz:
    AcceleroParams.AccOdr = 26667;
    break;
    
  default:
    AcceleroParams.AccOdr = 0;
    break;
  }
  
  switch(AcceleroParams.AccFifoBdrCode)
  {
  case IIS3DWB_XL_BATCHED_AT_26k7Hz:
    AcceleroParams.AccFifoBdr = 26667;
    break;
    
  default:
    Error_Handler();
    break;
  }
  
  /* Set default parameters for MotionSP library */
  MotionSP_Parameters.FftSize       = FFT_SIZE_DEFAULT;
  MotionSP_Parameters.tau           = TAU_DEFAULT;
  MotionSP_Parameters.window        = WINDOW_DEFAULT;
  MotionSP_Parameters.td_type       = TD_DEFAULT;
  MotionSP_Parameters.tacq          = TACQ_DEFAULT;
  MotionSP_Parameters.FftOvl        = FFT_OVL_DEFAULT;
  MotionSP_Parameters.subrange_num  = SUBRANGE_DEFAULT;
  
  switch(MotionSP_Parameters.window)
  {
  case RECTANGULAR:
    strcpy(MotionSP_ParameterName.window_name, "RECTANGULAR");
    break;
  case HANNING:
    strcpy(MotionSP_ParameterName.window_name, "HANNING");
    break;
  case HAMMING:
    strcpy(MotionSP_ParameterName.window_name, "HAMMING");
    break;
  case FLAT_TOP:
    strcpy(MotionSP_ParameterName.window_name, "FLAT TOP");
    break;
  default:
    strcpy(MotionSP_ParameterName.window_name, "");
    break;
  }

  switch(MotionSP_Parameters.td_type)
  {
  case TD_SPEED:
    strcpy(MotionSP_ParameterName.td_type_name, "SPEED");
    break;
  case TD_ACCELERO:
    strcpy(MotionSP_ParameterName.td_type_name, "ACCELERO");
    break;
  case TD_BOTH_TAU:
    strcpy(MotionSP_ParameterName.td_type_name, "SPEED and ACCELERO");
    break;
  default:
    strcpy(MotionSP_ParameterName.td_type_name, "");
    break;
  }
}

/** @brief Init the MotionSP Vibration parameters
  * @retval BSP status
  */
static int32_t MotionSP_VibrationAnalysisInit(void)
{
  int32_t BSP_Error = BSP_ERROR_NONE;
  
  /* Set vibration analysis param */
  MotionSP_VibrationAnalysisSetParam(&AcceleroParams);
  
  /* Initializes the vibration parameters with default values */
  if ((BSP_Error = MotionSP_VibrationAnalysisVariableInit()) != BSP_ERROR_NONE)
    return BSP_Error;
  
  /* Initializes accelerometer with vibration parameters values */
  if ((BSP_Error = MotionSP_AcceleroConfig()) != BSP_ERROR_NONE)
    return BSP_Error;
  
  return BSP_Error;
}

/** @brief Condition Monitoring Start
  * @retval BSP status
  */
static int32_t MotionSP_VibrationAnalysisStart(void)
{
  int32_t BSP_Error = BSP_ERROR_NONE;
  
  /* Configure the FIFO settings in according with others parammeters changed */
  if ((BSP_Error = MotionSP_AcceleroFifoConfig()) != BSP_ERROR_NONE)
    return BSP_Error;
  
  return BSP_Error;
}

/**
  * @brief  Vibration Analysis
  * @retval None
  */
static void MotionSP_VibrationAnalysisManager(void)
{
  if(MotionSP_Data.FifoEmpty)
  {
    /* Reset the TimeDomainData Drdy */
    MotionSP_Data.FifoEmpty = 0;
    
    /* Time Domain Analysis */
    MotionSP_TimeDomainAnalysis();
    
    /* Status check during Time domain Analysis */
    MotionSP_TimeDomainAlarm(&sTdAlarm,
                             &sTimeDomainVal,
                             &sTdRmsThresholds,
                             &sTdPkThresholds,
                             &TimeDomainData);    

    /* Frequency Domain Analysis */
    MotionSP_FrequencyDomainAnalysis();
    
    if (RestartFlag)
      RestartFlag = 0;
  }
}

/**
  * @brief  Init the MotionSP Vibration parameters
  * @retval BSP status
  */ 
static int32_t MotionSP_VibrationAnalysisVariableInit(void)
{
  /* Check for right values */
  switch (MotionSP_Parameters.FftSize)
  {
  case FFT_SIZE_256:
  case FFT_SIZE_512:
  case FFT_SIZE_1024:
  case FFT_SIZE_2048:
    break;
  default:
    return BSP_ERROR_WRONG_PARAM;
  }
   
  if ( ((MotionSP_Parameters.FftOvl) >= FFT_OVL_MIN) && ((MotionSP_Parameters.FftOvl) <= FFT_OVL_MAX) )
  {
    // MotionSP TD Threshold Updating
    MotionSP_TD_Threshold_Updating(TD_Th_DFLT);
    TD_Th_DFLT = 0;
    
    // MotionSP FD Threshold Updating
    MotionSP_FD_Threshold_Updating(MotionSP_Parameters.subrange_num, FD_Th_DFLT);
    FD_Th_DFLT = 0;
    
    // MotionSP threshold init
    MotionSP_AlarmThreshold_Init();
    
    // Reset the real accelero ODR value
    memset((void *)&AcceleroODR, 0x00, sizeof(sAcceleroODR_t));
    
    // Reset the FFT magnitude data
    AccMagResults.MagSize = 0;
    AccMagResults.MagSizeTBU = 0;
    AccMagResults.BinFreqStep = 0;
    memset((void *)&AccMagResults.Max, 0, 3 * sizeof(sMaxOnArray_t));
    memset((void *)&AccMagResults.FFT_Items, 0, 3 * sizeof(uint16_t));
    
    // Set magnitude size
    AccMagResults.MagSize = MotionSP_Parameters.FftSize/2;
    
    // Memory allocation for FFT magnitude arrays
    if (AccMagResults.Array.X == NULL)
      AccMagResults.Array.X = (float *) malloc(sizeof(float) * AccMagResults.MagSize);
    else
      AccMagResults.Array.X = realloc(AccMagResults.Array.X, sizeof(float) * AccMagResults.MagSize);
    if (AccMagResults.Array.X == NULL)
      return BSP_ERROR_MALLOC_FAILURE;
    
    if (AccMagResults.Array.Y == NULL)
      AccMagResults.Array.Y = (float *) malloc(sizeof(float) * AccMagResults.MagSize);
    else
      AccMagResults.Array.Y = realloc(AccMagResults.Array.Y, sizeof(float) * AccMagResults.MagSize);
    if (AccMagResults.Array.Y == NULL)
      return BSP_ERROR_MALLOC_FAILURE;
    
    if (AccMagResults.Array.Z == NULL)
      AccMagResults.Array.Z = (float *) malloc(sizeof(float) * AccMagResults.MagSize);
    else
      AccMagResults.Array.Z = realloc(AccMagResults.Array.Z, sizeof(float) * AccMagResults.MagSize);
    if (AccMagResults.Array.Z == NULL)
      return BSP_ERROR_MALLOC_FAILURE;
    
    // Reset the TimeDomain parameter values
    memset((void *)&TimeDomainData, 0x00, sizeof(sTimeDomainData_t));
    
    // Create filter array related to windowing method
    MotionSP_SetWindFiltArray(Filter_Params, MotionSP_Parameters.FftSize, (Filt_Type_t)MotionSP_Parameters.window);

    // Initialize FFT seed
    arm_rfft_fast_init_f32(&fftS, MotionSP_Parameters.FftSize);

    /* It is the minimum value to do the first FFT */
    accCircBuffIndexForFft = MotionSP_Parameters.FftSize - 1;
    
    /* Reset variable used inside MotionSP_fftExecution() */
    accCircBuffIndexForFftDelta = (uint16_t)((float)(MotionSP_Parameters.FftSize)*((100.0f-(float)MotionSP_Parameters.FftOvl)/100.0f));
    accCircBuffIndexForFftOvf = 0;
    
    RestartFlag = 1;
    MotionSP_Running = 1;
  }
  else
    return BSP_ERROR_WRONG_PARAM;
  
  return BSP_ERROR_NONE;
}

/**
  * @brief  Set accelerometer parameters for MotionSP Vibration
  * @retval BSP status
  */   
static int32_t MotionSP_AcceleroConfig(void)
{
  int32_t BSP_Error = BSP_ERROR_NONE;
#if LIMIT_FFT_RESULT == 1
  float LpfCutFreq = 0.0;
  uint16_t MagSizeForLpf;
#endif
  
  /* Set filter */
  if ((BSP_Error = BSP_MOTION_SENSOR_Enable_HP_Filter(ACCELERO_INSTANCE, AcceleroParams.HwFilter)) != BSP_ERROR_NONE)
    return BSP_Error;

  /* Set FS value */
  if ((BSP_Error =  BSP_MOTION_SENSOR_SetFullScale(ACCELERO_INSTANCE, MOTION_ACCELERO, AcceleroParams.fs)) != BSP_ERROR_NONE)
     return BSP_Error;

  /* Get Sensitivity */
  BSP_MOTION_SENSOR_GetSensitivity(ACCELERO_INSTANCE, MOTION_ACCELERO, &AcceleroParams.AccSens );
  
  /* Set ODR value */
  if ((BSP_Error = BSP_MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO, AcceleroParams.AccOdr)) != BSP_ERROR_NONE)
    return BSP_Error;

  /* Turn-on time delay */
  HAL_Delay(100);
  
  /* Measure and calculate ODR */
  if ((BSP_Error = MotionSP_AccMeasInit()) != BSP_ERROR_NONE)
    return BSP_Error;

  /* Calculate the FFT bin frequency increment */
  AccMagResults.BinFreqStep = (AcceleroODR.Frequency / 2) / AccMagResults.MagSize;
  
  /* Set the mag size to be used */
  AccMagResults.MagSizeTBU = AccMagResults.MagSize;
  
#if LIMIT_FFT_RESULT == 1
  /* Calculate the number of FFT magnitute elements within Low Pass Filter range */
  switch (AcceleroParams.HwFilter)
  {
  case IIS3DWB_LP_5kHz:
    LpfCutFreq = 5000.0;
    break;
  case IIS3DWB_LP_ODR_DIV_4:
    LpfCutFreq = AcceleroParams.AccOdr/4;
    break;
  case IIS3DWB_LP_ODR_DIV_10:
    LpfCutFreq = AcceleroParams.AccOdr/10;
    break;
  case IIS3DWB_LP_ODR_DIV_20:
    LpfCutFreq = AcceleroParams.AccOdr/20;
    break;
  case IIS3DWB_LP_ODR_DIV_45:
    LpfCutFreq = AcceleroParams.AccOdr/45;
    break;
  case IIS3DWB_LP_ODR_DIV_100:
    LpfCutFreq = AcceleroParams.AccOdr/100;
    break;
  case IIS3DWB_LP_ODR_DIV_200:
    LpfCutFreq = AcceleroParams.AccOdr/200;
    break;
  case IIS3DWB_LP_ODR_DIV_400:
    LpfCutFreq = AcceleroParams.AccOdr/400;
    break;
  case IIS3DWB_LP_ODR_DIV_800:
    LpfCutFreq = AcceleroParams.AccOdr/800;
    break;
  default:
    LpfCutFreq = 5000.0;
    break;
  }
  MagSizeForLpf = (uint16_t)(LpfCutFreq / AccMagResults.BinFreqStep);
  
  /* Set the mag size to be used */
  AccMagResults.MagSizeTBU = MagSizeForLpf;
#endif
  
  return BSP_Error;
}

/**
  * @brief 	Measurement initialization for the accelerometer
  * @retval BSP status
  */    
static int32_t MotionSP_AccMeasInit(void)
{
  int32_t BSP_Error = BSP_ERROR_NONE;
  
  /* Evaluate the real accelerometer ODR **********************************/
  uint8_t iteration = 0;
  do
  {
    BSP_Error = AccOdrMeas(&AcceleroODR);
    iteration++;
  } while( (BSP_Error != BSP_ERROR_NONE) && (iteration < 3) );
  /************************************************************************/
  
  return BSP_Error;
}

/**
  * @brief 	Measurement of the accelerometer output data rate
  * @param 	pAcceleroODR Pointer to be fill with the new value
  * @retval BSP status
  */
static int32_t AccOdrMeas(sAcceleroODR_t *pAcceleroODR)
{
#define ODRMEASURINGTIME  1000  // in ms
#define TOLERANCE         5     // from 0 to 100
  
  int32_t BSP_Error = BSP_ERROR_NONE;
  uint32_t tkStart;
  uint32_t OdrRange[2] = {
    (AcceleroParams.AccOdr * (100-TOLERANCE))/100,
    (AcceleroParams.AccOdr * (100+TOLERANCE))/100
  };
  
  // The data-ready pulses are 75 µs long
  if ((BSP_Error = BSP_MOTION_SENSOR_DRDY_Set_Mode(ACCELERO_INSTANCE, IIS3DWB_DRDY_PULSED)) != BSP_ERROR_NONE)
    return BSP_Error;
  
  /* IIS3DWB INT1_DRDY_XL interrupt enable */
  if ((BSP_Error = BSP_MOTION_SENSOR_Set_INT1_DRDY(ACCELERO_INSTANCE, ENABLE)) != BSP_ERROR_NONE)
    return BSP_Error;

  AccDrdyNr = 0;
  tkStart = BSP_GetTick();
  while ( (BSP_GetTick() - tkStart) < ODRMEASURINGTIME);
  
  /* IIS3DWB INT1_DRDY_XL interrupt disable */
  if ((BSP_Error = BSP_MOTION_SENSOR_Set_INT1_DRDY(ACCELERO_INSTANCE, DISABLE)) != BSP_ERROR_NONE)
    return BSP_Error;
  
  /* Calculate measured ODR and Exponential parameters*/
  pAcceleroODR->Frequency = (AccDrdyNr * 1000) / ODRMEASURINGTIME;
  
  if ( (pAcceleroODR->Frequency > OdrRange[0]) && (pAcceleroODR->Frequency < OdrRange[1]) )
  {
    /* ODR is valid */
    pAcceleroODR->Period = 1.0f/(float)pAcceleroODR->Frequency;
    pAcceleroODR->Tau= exp(-(float)(1000.0f*pAcceleroODR->Period)/(float)MotionSP_Parameters.tau);
  }
  else
    BSP_Error = BSP_ERROR_COMPONENT_FAILURE;
  
  return BSP_Error;
}

/**
  * @brief  Configure the FIFO in according with the algorithm setting
  * @retval BSP status
  */
static int32_t MotionSP_AcceleroFifoConfig(void)
{
  int32_t BSP_Error = BSP_ERROR_NONE;
  float k = 0.80f;
  
  // Reset the STM32 Motion Signal Processing Data
  MotionSP_Data.FifoEmpty = 0;
  MotionSP_Data.AccCircBuff.Size = 0;
  MotionSP_Data.AccCircBuff.IdPos = 0;
  MotionSP_Data.AccCircBuff.Ovf = 0;
  
  // Set the initial position of the accelero circular buffer
  MotionSP_Data.AccCircBuff.IdPos = (uint16_t)(-1);
  
/*------------- WATERMARK Setting ---------*/
  if (MotionSP_Parameters.FftSize == 1024)
  {
    if (MotionSP_Parameters.FftOvl < 25)
      k = 0.50f;
    else if (MotionSP_Parameters.FftOvl < 35)
      k = 0.57f;
    else if (MotionSP_Parameters.FftOvl < 45)
      k = 0.65f;
    else if (MotionSP_Parameters.FftOvl < 50)
      k = 0.77f;
  }
  else if (MotionSP_Parameters.FftSize == 2048)
  {
    if (MotionSP_Parameters.FftOvl < 25)
      k = 0.25f;
    else if (MotionSP_Parameters.FftOvl < 35)
      k = 0.28f;
    else if (MotionSP_Parameters.FftOvl < 45)
      k = 0.32f;
    else if (MotionSP_Parameters.FftOvl < 55)
      k = 0.38f;
    else if (MotionSP_Parameters.FftOvl < 65)
      k = 0.47f;
    else if (MotionSP_Parameters.FftOvl < 70)
      k = 0.61f;
    else if (MotionSP_Parameters.FftOvl < 75)
      k = 0.71f;
  }
  
  AcceleroParams.FifoWtm = (uint16_t)(((float)MotionSP_Parameters.FftSize*(1.0f-((float)MotionSP_Parameters.FftOvl/100.0f)))*k);
  
  if ( (uint16_t)((AcceleroParams.FifoWtm*7) > (1024*3)) )
    return BSP_ERROR_WRONG_PARAM;
  
  /* Set FIFO watermark */
  if ((BSP_Error = BSP_MOTION_SENSOR_FIFO_Set_Watermark_Level(ACCELERO_INSTANCE, AcceleroParams.FifoWtm)) != BSP_ERROR_NONE)
    return BSP_Error;

  /* MCU memory allocation for FIFO buffer ************************************/
  if (MotionSP_Data.FifoBfr == NULL)
    MotionSP_Data.FifoBfr = (uint8_t *) malloc(sizeof(uint8_t) * (AcceleroParams.FifoWtm * 7));
  else
    MotionSP_Data.FifoBfr = realloc(MotionSP_Data.FifoBfr, sizeof(uint8_t) * (AcceleroParams.FifoWtm * 7));
  if (MotionSP_Data.FifoBfr == NULL)
    return BSP_ERROR_MALLOC_FAILURE;
  
  /* Set to zero the allocated memory for FIFO buffer */
  memset((void *)MotionSP_Data.FifoBfr, 0, sizeof(uint8_t) * (AcceleroParams.FifoWtm * 7));
  /****************************************************************************/

  /**** Create circular buffer ************************************************/
  MotionSP_Data.AccCircBuff.Size = (uint16_t)((MotionSP_Parameters.FftSize * CIRC_BUFFER_RATIO_NUM)/CIRC_BUFFER_RATIO_DEN);
  
  /* MCU memory allocation for X-axis array for circular buffer ***************/
  if (MotionSP_Data.AccCircBuff.Array.X == NULL)
    MotionSP_Data.AccCircBuff.Array.X = (float *) malloc(sizeof(float) * MotionSP_Data.AccCircBuff.Size);
  else
   MotionSP_Data.AccCircBuff.Array.X = realloc(MotionSP_Data.AccCircBuff.Array.X, sizeof(float) * MotionSP_Data.AccCircBuff.Size);
  if(MotionSP_Data.AccCircBuff.Array.X == NULL)
    return BSP_ERROR_MALLOC_FAILURE;

  /* Set to zero the allocated memory for X-axis array for circular buffer */
  memset((void *)MotionSP_Data.AccCircBuff.Array.X, 0, sizeof(float) * MotionSP_Data.AccCircBuff.Size);
  /****************************************************************************/
  
  /* MCU memory allocation for Y-axis array for circular buffer ***************/
  if (MotionSP_Data.AccCircBuff.Array.Y == NULL)
    MotionSP_Data.AccCircBuff.Array.Y = (float *) malloc(sizeof(float) * MotionSP_Data.AccCircBuff.Size);
  else
   MotionSP_Data.AccCircBuff.Array.Y = realloc(MotionSP_Data.AccCircBuff.Array.Y, sizeof(float) * MotionSP_Data.AccCircBuff.Size);
  if(MotionSP_Data.AccCircBuff.Array.Y == NULL)
    return BSP_ERROR_MALLOC_FAILURE;

  /* Set to zero the allocated memory for Y-axis array for circular buffer */
  memset((void *)MotionSP_Data.AccCircBuff.Array.Y, 0, sizeof(float) * MotionSP_Data.AccCircBuff.Size);
  /****************************************************************************/
  
  /* MCU memory allocation for Z-axis array for circular buffer ***************/
  if (MotionSP_Data.AccCircBuff.Array.Z == NULL)
    MotionSP_Data.AccCircBuff.Array.Z = (float *) malloc(sizeof(float) * MotionSP_Data.AccCircBuff.Size);
  else
   MotionSP_Data.AccCircBuff.Array.Z = realloc(MotionSP_Data.AccCircBuff.Array.Z, sizeof(float) * MotionSP_Data.AccCircBuff.Size);
  if(MotionSP_Data.AccCircBuff.Array.Z == NULL)
    return BSP_ERROR_MALLOC_FAILURE;

  /* Set to zero the allocated memory for Z-axis array for circular buffer */
  memset((void *)MotionSP_Data.AccCircBuff.Array.Z, 0, sizeof(float) * MotionSP_Data.AccCircBuff.Size);
  /****************************************************************************/
  /****************************************************************************/
  
  /* Set FIFO depth to be limited to watermark threshold level  */
  if ((BSP_Error = BSP_MOTION_SENSOR_FIFO_Set_Stop_On_Fth(ACCELERO_INSTANCE, ENABLE)) != BSP_ERROR_NONE)
    return BSP_Error;

  /* Set FIFO mode */
  if ((BSP_Error = BSP_MOTION_SENSOR_FIFO_Set_Mode(ACCELERO_INSTANCE, ACCELERO_FIFO_STREAM_MODE)) != BSP_ERROR_NONE)
    return BSP_Error;

  /* IIS3DWB INT2_FIFO_FULL interrupt enable */
  if ((BSP_Error = BSP_MOTION_SENSOR_FIFO_Set_INT2_FIFO_Full(ACCELERO_INSTANCE, ENABLE)) != BSP_ERROR_NONE)
    return BSP_Error;

  /* Start acquisition time */
  StartTick = BSP_GetTick();
  
  /* Set FIFO BDR value */
  if ((BSP_Error = BSP_MOTION_SENSOR_FIFO_Set_BDR(ACCELERO_INSTANCE, MOTION_ACCELERO, AcceleroParams.AccFifoBdrCode)) != BSP_ERROR_NONE)
    return BSP_Error;
  
  return BSP_Error;
}

/**
  * @brief  Time Domain Analysis
  * @retval None
  */
static void MotionSP_TimeDomainAnalysis(void)
{
  MotionSP_TimeDomainEvalFromCircBuff(&TimeDomainData,
                                      &MotionSP_Data.AccCircBuff,
                                      AcceleroParams.FifoWtm,
                                      (Td_Type_t)MotionSP_Parameters.td_type,
                                      AcceleroODR,
                                      RestartFlag);
}

/**
  * @brief  Frequency Domain Analysis
  * @retval None
  */
static void MotionSP_FrequencyDomainAnalysis(void)
{
  uint16_t accCircBuffIndexTmp = 0;
  uint16_t accCircBuffIndexForFftTmp = 0;
  uint8_t FFT_avg = 0;
  
  accCircBuffIndexTmp = MotionSP_Data.AccCircBuff.IdPos + (MotionSP_Data.AccCircBuff.Ovf * MotionSP_Data.AccCircBuff.Size);
  accCircBuffIndexForFftTmp = accCircBuffIndexForFft + (accCircBuffIndexForFftOvf * MotionSP_Data.AccCircBuff.Size);
  
  if (accCircBuffIndexTmp >= accCircBuffIndexForFftTmp)
  {
    if ((BSP_GetTick() - StartTick) > MotionSP_Parameters.tacq)
    {
      /* Power-down the accelerometer */
      BSP_MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO, 0.0);
 
      /* Switch from FIFO to BYPASS MODE */
      BSP_MOTION_SENSOR_FIFO_Set_Mode(ACCELERO_INSTANCE, ACCELERO_FIFO_BYPASS_MODE);
      
      MotionSP_Running = 0;
      FFT_avg = 1;
    }
    
    MotionSP_fftExecution(FFT_avg);
    
    accCircBuffIndexForFftOvf = 0;
    accCircBuffIndexForFft += accCircBuffIndexForFftDelta;
    if (accCircBuffIndexForFft >= MotionSP_Data.AccCircBuff.Size)
    {
      accCircBuffIndexForFft -= MotionSP_Data.AccCircBuff.Size;
      
      if (!MotionSP_Data.AccCircBuff.Ovf)
        accCircBuffIndexForFftOvf = 1;
    }
    
    MotionSP_Data.AccCircBuff.Ovf = 0;
  }
}

static void FillAccCircBuffFromFifo(sCircBuff_t *pAccCircBuff, uint8_t *pFifoBuff, uint16_t FifoWtm, float AccSens, uint8_t Rst)
{
  uint8_t *pFifoData = NULL;                  //!< FIFO pointer
  uint16_t i = 0;                             //!< FIFO index
  SensorVal_f_t mgAcc = {0.0, 0.0, 0.0};      //!< actual acceleration in mg
#if USE_SW_HP_FILTER == 1
  SensorVal_f_t mgAccNoDC = {0.0, 0.0, 0.0};  //!< actual acceleration in mg without DC component
#endif
  for(i=0; i<FifoWtm; i++)
  {
    pFifoData = pFifoBuff+(i*7);

    mgAcc.AXIS_X = (*(int16_t *)(pFifoData+1)) * AccSens;
    mgAcc.AXIS_Y = (*(int16_t *)(pFifoData+3)) * AccSens;
    mgAcc.AXIS_Z = (*(int16_t *)(pFifoData+5)) * AccSens;
    
#if USE_SW_HP_FILTER == 1
    // High Pass Filter to delete Accelerometer Offset
    MotionSP_accDelOffset(&mgAccNoDC, &mgAcc, DC_SMOOTH, Rst);
#endif
    
    /* Prepare the position to be filled */
    pAccCircBuff->IdPos += 1;
    
    /* Check for overflow */
    if (pAccCircBuff->IdPos == pAccCircBuff->Size)
    {
      pAccCircBuff->IdPos = 0;
      pAccCircBuff->Ovf = 1;
    }
    
#if USE_SW_HP_FILTER == 1
    pAccCircBuff->Array.X[pAccCircBuff->IdPos] = mgAccNoDC.AXIS_X * G_CONV;
    pAccCircBuff->Array.Y[pAccCircBuff->IdPos] = mgAccNoDC.AXIS_Y * G_CONV;
    pAccCircBuff->Array.Z[pAccCircBuff->IdPos] = mgAccNoDC.AXIS_Z * G_CONV;
#else
    pAccCircBuff->Array.X[pAccCircBuff->IdPos] = mgAcc.AXIS_X * G_CONV;
    pAccCircBuff->Array.Y[pAccCircBuff->IdPos] = mgAcc.AXIS_Y * G_CONV;
    pAccCircBuff->Array.Z[pAccCircBuff->IdPos] = mgAcc.AXIS_Z * G_CONV;
#endif
    
    if (Rst)
      Rst = 0;
  }
}

/**
  * @}
  */

/** @addtogroup STM32_MOTIONSP_MANAGER_EXPORTED_FUNCTIONS MotionSP_Manager Exported Functions
  * @{
  */

/**
  * @brief  Vibration analysis init
  * @retval Error codes
  */
int32_t MotionSP_Init(void)
{
  int32_t  BSP_Error = BSP_ERROR_NONE;
  
  /* Initialize accelerometer */
  if((BSP_Error = Accelerometer_Init()) != BSP_ERROR_NONE)
    return BSP_Error;
  
  /* Set the vibration parameters with default values */
  MotionSP_SetDefaultVibrationParam();
  IDP005_PRINTF("\tVibration parameters have been set as default values\r\n");
  
  return BSP_Error;
}

/**
  * @brief  MotionSP Alarm Threshold Init
  * @retval Error codes
  */
void MotionSP_AlarmThreshold_Init(void)
{
  MotionSP_TimeDomainAlarmInit(&sTdAlarm,
                               &sTimeDomainVal,
                               &sTdRmsThresholds,
                               &sTdPkThresholds);
  
  if (MotionSP_FreqDomainAlarmInit(&pFDWarnThresh,
                                   &pFDAlarmThresh,
                                   &THR_Fft_Alarms,
                                   MotionSP_Parameters.subrange_num) != BSP_ERROR_NONE)
    Error_Handler();
}

/**
  * @brief  MotionSP TD Threshold Updating
  * @param  dflt Force updating with default values
  * @retval Error codes
  */
void MotionSP_TD_Threshold_Updating(uint8_t dflt)
{
  if (dflt)
  {
    memcpy((void *)TDRmsThresh, (void *)&TDSpeedRMSThresh, sizeof(sTimeDomainThresh_t));
    memcpy((void *)TDPkThresh, (void *)&TDAccPeakThresh, sizeof(sTimeDomainThresh_t));
  }
  else
  {
    __NOP();
  }
}

/**
  * @brief  MotionSP FD Threshold Updating
  * @param  subrange_num Subrange number to be used
  * @param  dflt Force updating with default values
  * @retval Error codes
  */
void MotionSP_FD_Threshold_Updating(uint8_t subrange_num, uint8_t dflt)
{
  float *pFDWarnThresh_Sub;
  float *pFDAlarmThresh_Sub;
  
  uint16_t thr_array_size = 0;
  
  thr_array_size = 3 * subrange_num * sizeof(float);
  
  if (dflt)
  {
    switch (subrange_num)
    {
    case 8:
      pFDWarnThresh_Sub = (float *)FDWarnThresh_Sub8;
      pFDAlarmThresh_Sub = (float *)FDAlarmThresh_Sub8;
      break;
      
    case 16:
      pFDWarnThresh_Sub = (float *)FDWarnThresh_Sub16;
      pFDAlarmThresh_Sub = (float *)FDAlarmThresh_Sub16;
      break;
      
    case 32:
      pFDWarnThresh_Sub = (float *)FDWarnThresh_Sub32;
      pFDAlarmThresh_Sub = (float *)FDAlarmThresh_Sub32;
      break;
      
    case 64:
      pFDWarnThresh_Sub = (float *)FDWarnThresh_Sub64;
      pFDAlarmThresh_Sub = (float *)FDAlarmThresh_Sub64;
      break;
    }
    
    memcpy((void *)FDWarnThresh_Sub, (void *)pFDWarnThresh_Sub, thr_array_size);
    memcpy((void *)FDAlarmThresh_Sub, (void *)pFDAlarmThresh_Sub, thr_array_size);
  }
  else
  {
    __NOP();
  }
}

/**
  * @brief  Vibration analysis manager
  * @retval BSP status
  */
int32_t MotionSP_MainManager(void)
{
  int32_t  BSP_Error = BSP_ERROR_NONE;
  
  if (!MotionSP_Running)
  {
    /* Condition Monitoring Initialization */
    if((BSP_Error = MotionSP_VibrationAnalysisInit()) != BSP_ERROR_NONE)
      return BSP_Error;
    
    /* Send real Accelerometer ODR */
    MotionSP_AccelerometerSendRealOdr();
    
    IDP005_PRINTF("\033[%dm", FG_GREEN);
    IDP005_PRINTF("\r\n\tWait while acquisition ongoing for %.2f s ...\r\n", (float)(MotionSP_Parameters.tacq/1000.0));

#if (SEND_TIMEDOMAIN == 1)
    /* Send Time Domain Header */
    Send_TimeDomainHeader();
#endif /* SEND_TIMEDOMAIN */
    
    /* Vibration Analysis Start */
    if((BSP_Error = MotionSP_VibrationAnalysisStart()) != BSP_ERROR_NONE)
      return BSP_Error;
  }
  else
  {
    /* Frequency Domain Analysis */
    MotionSP_VibrationAnalysisManager();
    
#if (SEND_TIMEDOMAIN == 1)
    /* Send Time Domain Analysis */
    Send_TimeDomainResults(&TimeDomainData);
#endif /* SEND_TIMEDOMAIN */
    
    if (!MotionSP_Running)
    {
      /* Check thresholds for Accelerometer FFT Result values in SubRanges */
      MotionSP_FreqDomainAlarm (&SRAmplitude,
                                pFDWarnThresh,
                                pFDAlarmThresh,
                                MotionSP_Parameters.subrange_num,
                                &THR_Check, 
                                &THR_Fft_Alarms);
      
      /* Send Frequency Domain Analysis */
      MotionSP_VibrationAnalysisSendResult();
      
      /* Get Env DATA and sent to the terminal */
      SendEnvironmentalResult();
    }
  }
  return BSP_Error;
}

/**
  * @brief 	Routine to be executed on IRQ about accelerometer data ready
  * @return None
  */    
void MotionSP_DataReady_IRQ_Rtn(void)
{
  AccDrdyNr++;
}

/**
  * @brief 	Routine to be executed on IRQ about accelerometer fifo full
  * @return None
  */    
void MotionSP_FifoFull_IRQ_Rtn(void)
{
  // Read all FIFO data
  BSP_MOTION_SENSOR_FIFO_Read(ACCELERO_INSTANCE, MotionSP_Data.FifoBfr, AcceleroParams.FifoWtm);
  
  // Create circular buffer from accelerometer FIFO
  FillAccCircBuffFromFifo(&MotionSP_Data.AccCircBuff, MotionSP_Data.FifoBfr, AcceleroParams.FifoWtm, AcceleroParams.AccSens, RestartFlag);

  // Fifo has been read
  MotionSP_Data.FifoEmpty = 1;
}

/*------------------ Predictive APIs -------------------------*/

/**
  *  @brief Initialization of WARNING & ALARM thresholds values on Axes,
  *         using the default values included in MotionSP_Thresholds.h file
  *  @param pTdAlarm: Pointer to TimeDomain Alarms Result
  *  @param pTimeDomainVal: Pointer to TimeDomain Value Result
  *  @param pTdRmsThreshold: Pointer to TimeDomain RMS Threshlods to initialize
  *  @param pTdPkThreshold:  Pointer to TimeDomain PK Threshlods to initialize
  *  @return Return description
  */
void MotionSP_TimeDomainAlarmInit (sTimeDomainAlarm_t *pTdAlarm,
                                   sAcceleroParam_t *pTimeDomainVal,
                                   sTimeDomainThresh_t *pTdRmsThreshold,
                                   sTimeDomainThresh_t *pTdPkThreshold) 
{
  /* Reset status value for TimeDomain Alarms Result */
  memset(pTdAlarm, NONE, sizeof(sTimeDomainAlarm_t));
  
  /* Reset status value for TimeDomain Value Result */
  memset(pTimeDomainVal, 0, sizeof(sAcceleroParam_t));
  
  memcpy((void *)pTdRmsThreshold, (void *)TDRmsThresh, sizeof(sTimeDomainThresh_t));
  memcpy((void *)pTdPkThreshold, (void *)TDPkThresh, sizeof(sTimeDomainThresh_t));
}

/**
  *  @brief  Time Domain Alarm Analysis based just on Speed RMS FAST Moving Average
  *  @param  pTdAlarm: Pointer to TimeDomain Alarms Result
  *  @param  pTimeDomainVal: Pointer to TimeDomain Value Result
  *  @param  pTdRmsThreshold:  Pointer to TimeDomain RMS Threshlods Configured
  *  @param  pTdPkThreshold:  Pointer to TimeDomain PK Threshlods Configured
  *  @param  pTimeDomain:   Pointer to TimeDomain Parameters to monitor
  *  @return None
  */
void MotionSP_TimeDomainAlarm (sTimeDomainAlarm_t *pTdAlarm,
                               sAcceleroParam_t *pTimeDomainVal,
                               sTimeDomainThresh_t *pTdRmsThreshold,
                               sTimeDomainThresh_t *pTdPkThreshold,
                               sTimeDomainData_t *pTimeDomain) 
{
  /* Reset status value for Time Domain alarms */
  memset(pTdAlarm, GOOD, 3*2);
  
  pTimeDomainVal->SpeedRms.AXIS_X = pTimeDomain->SpeedRms.AXIS_X*1000;
  pTimeDomainVal->SpeedRms.AXIS_Y = pTimeDomain->SpeedRms.AXIS_Y*1000;
  pTimeDomainVal->SpeedRms.AXIS_Z = pTimeDomain->SpeedRms.AXIS_Z*1000;
  
  /* Speed RMS comparison with thresholds */      
  if ((pTimeDomain->SpeedRms.AXIS_X*1000) > pTdRmsThreshold->THR_WARN_AXIS_X)
  {
    pTdAlarm->RMS_STATUS_AXIS_X = WARNING;
    pTimeDomainVal->SpeedRms.AXIS_X = pTimeDomain->SpeedRms.AXIS_X*1000;
  }
  if ((pTimeDomain->SpeedRms.AXIS_Y*1000) > pTdRmsThreshold->THR_WARN_AXIS_Y)
  {
    pTdAlarm->RMS_STATUS_AXIS_Y = WARNING;
    pTimeDomainVal->SpeedRms.AXIS_Y = pTimeDomain->SpeedRms.AXIS_Y*1000;
  }
  if ((pTimeDomain->SpeedRms.AXIS_Z*1000) > pTdRmsThreshold->THR_WARN_AXIS_Z)
  {
    pTdAlarm->RMS_STATUS_AXIS_Z = WARNING;
    pTimeDomainVal->SpeedRms.AXIS_Z = pTimeDomain->SpeedRms.AXIS_Z*1000;
  }
  if ((pTimeDomain->SpeedRms.AXIS_X*1000) > pTdRmsThreshold->THR_ALARM_AXIS_X)
  {
    pTdAlarm->RMS_STATUS_AXIS_X = ALARM;
    pTimeDomainVal->SpeedRms.AXIS_X = pTimeDomain->SpeedRms.AXIS_X*1000;
  }
  if ((pTimeDomain->SpeedRms.AXIS_Y*1000) > pTdRmsThreshold->THR_ALARM_AXIS_Y)
  {
    pTdAlarm->RMS_STATUS_AXIS_Y = ALARM;
    pTimeDomainVal->SpeedRms.AXIS_Y = pTimeDomain->SpeedRms.AXIS_Y*1000;
  }
  if ((pTimeDomain->SpeedRms.AXIS_Z*1000) > pTdRmsThreshold->THR_ALARM_AXIS_Z)
  {
    pTdAlarm->RMS_STATUS_AXIS_Z = ALARM;
    pTimeDomainVal->SpeedRms.AXIS_Z = pTimeDomain->SpeedRms.AXIS_Z*1000;
  }
  
  pTimeDomainVal->AccPeak.AXIS_X = pTimeDomain->AccPeak.AXIS_X;
  pTimeDomainVal->AccPeak.AXIS_Y = pTimeDomain->AccPeak.AXIS_Y;
  pTimeDomainVal->AccPeak.AXIS_Z = pTimeDomain->AccPeak.AXIS_Z;
  
  /* Accelerometer Peak comparison with thresholds */      
  if ((pTimeDomain->AccPeak.AXIS_X) > pTdPkThreshold->THR_WARN_AXIS_X)
  {
    pTdAlarm->PK_STATUS_AXIS_X = WARNING;
    pTimeDomainVal->AccPeak.AXIS_X = pTimeDomain->AccPeak.AXIS_X;
  }
  if ((pTimeDomain->AccPeak.AXIS_Y) > pTdPkThreshold->THR_WARN_AXIS_Y)
  {
    pTdAlarm->PK_STATUS_AXIS_Y = WARNING;
    pTimeDomainVal->AccPeak.AXIS_Y = pTimeDomain->AccPeak.AXIS_Y;
  }
  if ((pTimeDomain->AccPeak.AXIS_Z) > pTdPkThreshold->THR_WARN_AXIS_Z)
  {
    pTdAlarm->PK_STATUS_AXIS_Z = WARNING;
    pTimeDomainVal->AccPeak.AXIS_Z = pTimeDomain->AccPeak.AXIS_Z;
  }
  if ((pTimeDomain->AccPeak.AXIS_X) > pTdPkThreshold->THR_ALARM_AXIS_X)
  {
    pTdAlarm->PK_STATUS_AXIS_X = ALARM;
    pTimeDomainVal->AccPeak.AXIS_X = pTimeDomain->AccPeak.AXIS_X;
  }
  if ((pTimeDomain->AccPeak.AXIS_Y) > pTdPkThreshold->THR_ALARM_AXIS_Y)
  {
    pTdAlarm->PK_STATUS_AXIS_Y = ALARM;
    pTimeDomainVal->AccPeak.AXIS_Y = pTimeDomain->AccPeak.AXIS_Y;
  }
  if ((pTimeDomain->AccPeak.AXIS_Z) > pTdPkThreshold->THR_ALARM_AXIS_Z)
  {
    pTdAlarm->PK_STATUS_AXIS_Z = ALARM;
    pTimeDomainVal->AccPeak.AXIS_Z = pTimeDomain->AccPeak.AXIS_Z;
  }
}

/**
  * @brief Initialization of Alarm Status on Axes, Alarm Values Reported
  *        and Thresholds to detect WARNING and ALARM conditions
  * @param pWarnThresh: Pointer to TimeDomain Alarms thresholds to use
  * @param pAlarmThresh: Pointer to TimeDomain Alarms thresholds to use
  * @param pTHR_Fft_Alarms: Pointer to Freq Domain Value Arrays Result
  * @param subrange_num:  Subranges numbers
  * @retval BSP status
  *  
  */
int32_t MotionSP_FreqDomainAlarmInit (float **pWarnThresh,
                                   float **pAlarmThresh,
                                   sFreqDomainAlarm_t *pTHR_Fft_Alarms,
                                   uint8_t subrange_num) 
{
  uint16_t thr_array_size = 0;
  
  thr_array_size = 3 * subrange_num * sizeof(float);
  
  /* Reset status value for FFT alarms */
  memset(pTHR_Fft_Alarms, NONE, sizeof(sFreqDomainAlarm_t));
  
  // Memory allocation for Warning Threshold array 
  if (*pWarnThresh == NULL)
    *pWarnThresh = (float *)malloc(thr_array_size);
  else
   *pWarnThresh = realloc(*pWarnThresh, thr_array_size);
  if (*pWarnThresh == NULL)
    return BSP_ERROR_MALLOC_FAILURE;

  // Memory allocation for Alarm Threshold array
  if (*pAlarmThresh == NULL)
    *pAlarmThresh = (float *)malloc(thr_array_size);
  else
   *pAlarmThresh = realloc(*pAlarmThresh, thr_array_size);
  if (*pAlarmThresh == NULL)
    return BSP_ERROR_MALLOC_FAILURE;

  memcpy((void *)*pWarnThresh, (void *)FDWarnThresh_Sub, thr_array_size);
  memcpy((void *)*pAlarmThresh, (void *)FDAlarmThresh_Sub, thr_array_size);
  
  return BSP_ERROR_NONE;
}

/**
  *  @brief  Compare the Frequency domain subrange comparison with external Threshold Arrays
  *  @param  pSRAmplitude: Pointer to Amplitude subranges Array resulting after Freq Analysis
  *  @param  pFDWarnThresh: Pointer to Amplitude Warning Threshold subranges Array
  *  @param  pFDAlarmThresh: Pointer to Amplitude Alarm Threshold subranges Array
  *  @param  subrange_num: Subranges number
  *  @param  pTHR_Check: Pointer to Amplitude subranges Values that exceed thresholds
  *  @param  pTHR_Fft_Alarms: Pointer to Amplitude subranges Threshold Status
  *  @return None
  */
void MotionSP_FreqDomainAlarm (sSubrange_t *pSRAmplitude,
                               float *pFDWarnThresh,
                               float *pFDAlarmThresh,
                               uint8_t subrange_num, 
                               sSubrange_t *pTHR_Check, 
                               sFreqDomainAlarm_t *pTHR_Fft_Alarms)
{
  float warn_thresholds;
  float alarm_thresholds;
  memset((void *)pTHR_Check, 0x00, 3*SUBRANGE_MAX * sizeof(float) );

  for(int i=0; i<subrange_num; i++)
  {
   for(int j=0; j<3; j++) 
   {
    warn_thresholds = *(pFDWarnThresh+(i*3)+j);
    alarm_thresholds = *(pFDAlarmThresh+(i*3)+j);
    switch (j)
    {
    case 0x00:  /* Axis X */
      pTHR_Check->AXIS_X[i] = pSRAmplitude->AXIS_X[i];
      pTHR_Fft_Alarms->STATUS_AXIS_X[i] = GOOD;       
      if(pSRAmplitude->AXIS_X[i] > warn_thresholds)
        pTHR_Fft_Alarms->STATUS_AXIS_X[i] = WARNING;
      if(pSRAmplitude->AXIS_X[i] > alarm_thresholds)
        pTHR_Fft_Alarms->STATUS_AXIS_X[i] = ALARM;
      break;
      
    case 0x01:  /* Axis Y */
      pTHR_Check->AXIS_Y[i] = pSRAmplitude->AXIS_Y[i];
      pTHR_Fft_Alarms->STATUS_AXIS_Y[i] = GOOD;
      if(pSRAmplitude->AXIS_Y[i] > warn_thresholds)
        pTHR_Fft_Alarms->STATUS_AXIS_Y[i] = WARNING;
      if(pSRAmplitude->AXIS_Y[i] > alarm_thresholds)
        pTHR_Fft_Alarms->STATUS_AXIS_Y[i] = ALARM;
      break;
      
    case 0x02:  /* Axis Z */
      pTHR_Check->AXIS_Z[i] = pSRAmplitude->AXIS_Z[i];
      pTHR_Fft_Alarms->STATUS_AXIS_Z[i] = GOOD;
      if(pSRAmplitude->AXIS_Z[i] > warn_thresholds)
        pTHR_Fft_Alarms->STATUS_AXIS_Z[i] = WARNING;
      if(pSRAmplitude->AXIS_Z[i] > alarm_thresholds)
        pTHR_Fft_Alarms->STATUS_AXIS_Z[i] = ALARM;
      break;
      
    default:
      __NOP();
      break;    
    }
   } 
  }
}

/**
  *  @brief Compare the Frequency domain subrange comparison with external Threshold Arrays
  *  @param pTdAlarm: Pointer to status array threshold resulting after Time Domain Analysis
  *  @param pTHR_Fft_Alarms: Pointer to status array threshold resulting after Freq Domain Analysis
  *  @param subrange_num: Subranges number
  *  @param pTotalTDAlarm: Pointer to total time domain alarms that exceed thresholds
  *  @param pTotalFDAlarm: Pointer to total frequency domain alarms that exceed thresholds
  *  @return None
  */
void MotionSP_TotalStatusAlarm(sTimeDomainAlarm_t *pTdAlarm,
                               sFreqDomainAlarm_t *pTHR_Fft_Alarms,
                               uint8_t subrange_num,
                               Alarm_Type_t *pTotalTDAlarm,
                               Alarm_Type_t *pTotalFDAlarm)
{
 Alarm_Type_t TempAlarm = GOOD;
 Alarm_Type_t TempFDAlarm = GOOD;
 
 TempAlarm = MAX4(TempAlarm,
                  pTdAlarm->PK_STATUS_AXIS_X,
                  pTdAlarm->PK_STATUS_AXIS_Y,
                  pTdAlarm->PK_STATUS_AXIS_Z);
 
 
 TempAlarm = MAX4(TempAlarm,
                  pTdAlarm->RMS_STATUS_AXIS_X,
                  pTdAlarm->RMS_STATUS_AXIS_Y,
                  pTdAlarm->RMS_STATUS_AXIS_Z);
 
 for(int i=0; i<subrange_num; i++)
 {
   TempFDAlarm = MAX4(TempFDAlarm,
                      pTHR_Fft_Alarms->STATUS_AXIS_X[i],
                      pTHR_Fft_Alarms->STATUS_AXIS_Y[i],
                      pTHR_Fft_Alarms->STATUS_AXIS_Z[i]);
 }
 
 *pTotalTDAlarm = TempAlarm;
 *pTotalFDAlarm = TempFDAlarm; 
  
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

/************************ (C) COPYRIGHT 2020 STMicroelectronics *****END OF FILE****/
