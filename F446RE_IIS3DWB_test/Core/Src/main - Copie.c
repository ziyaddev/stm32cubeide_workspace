/**
  ******************************************************************************
  * @file    main.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version V2.4.0
  * @date    07-June-2021
  * @brief   Main program body
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

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>

#include "main.h"
#include "MetaDataManager.h"

/** @addtogroup Projects
  * @{
  */

/** @addtogroup DEMONSTRATIONS Demonstrations
  * @{
  */

/** @addtogroup PREDCTIVE_MAINTENANCE Predictive Maintenance
  * @{
  */

/** @addtogroup PREDCTIVE_MAINTENANCE_MAIN Predictive Maintenance main
  * @{
  */
   
/** @defgroup PREDCTIVE_MAINTENANCE_MAIN_PRIVATE_DEFINE Predictive Maintenance Main Private Define
  * @{
  */

/* Private define ------------------------------------------------------------*/
#define CHECK_VIBRATION_PARAM ((uint16_t)0x1234)

/**
  * @}
  */

/** @defgroup PREDCTIVE_MAINTENANCE_MAIN_EXPORTED_VARIABLES Predictive Maintenance Main Exported Variables
  * @{
  */

/* Exported Variables -------------------------------------------------------------*/
volatile uint32_t HCI_ProcessEvent =      0;
volatile uint8_t FifoEnabled = 0;

volatile uint32_t PredictiveMaintenance = 0;

float RMS_Ch[AUDIO_IN_CHANNELS];
float DBNOISE_Value_Old_Ch[AUDIO_IN_CHANNELS];

uint32_t ConnectionBleStatus  =0;

TIM_HandleTypeDef    TimCCHandle;

uint8_t bdaddr[6];

uint8_t EnvironmentalTimerEnabled= 0;
uint8_t AudioLevelTimerEnabled= 0;
uint8_t InertialTimerEnabled= 0;

uint8_t AudioLevelEnable= 0;

uint32_t uhCCR1_Val = DEFAULT_uhCCR1_Val;
uint32_t uhCCR2_Val = DEFAULT_uhCCR2_Val;
uint32_t uhCCR3_Val = DEFAULT_uhCCR3_Val;

uint8_t  NodeName[8];

/**
  * @}
  */

/** @defgroup PREDCTIVE_MAINTENANCE_MAIN_PRIVATE_VARIABLES Predictive Maintenance Main Private Variables
  * @{
  */

/* Private variables ---------------------------------------------------------*/
uint16_t VibrationParam[11];

/* Table with All the known Meta Data */
MDM_knownGMD_t known_MetaData[]={
  {GMD_NODE_NAME,      (sizeof(NodeName))},
  {GMD_VIBRATION_PARAM,(sizeof(VibrationParam))},
  {GMD_END    ,0}/* THIS MUST BE THE LAST ONE */
};

static volatile uint32_t ButtonPressed=           0;
static volatile uint32_t SendEnv=                 0;
static volatile uint32_t SendAudioLevel=          0;
static volatile uint32_t SendAccGyroMag=          0;

static uint16_t PCM_Buffer[((AUDIO_IN_CHANNELS*AUDIO_IN_SAMPLING_FREQUENCY)/1000)  * N_MS ];
static uint32_t NumSample= ((AUDIO_IN_CHANNELS*AUDIO_IN_SAMPLING_FREQUENCY)/1000)  * N_MS;

/**
  * @}
  */

/** @defgroup PREDCTIVE_MAINTENANCE_MAIN_PRIVATE_FUNCTIONS_PROTOTYPES Predictive Maintenance Main Private Functions Prototypes
  * @{
  */

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void InitTimers(void);
static void InitPredictiveMaintenance(void);

static unsigned char ReCallNodeNameFromMemory(void);
static unsigned char ReCallVibrationParamFromMemory(void);

static void SendEnvironmentalData(void);
static void SendMotionData(void);
static void SendAudioLevelData(void);

static void ButtonCallback(void);
static void AudioProcess(void);

static void Environmental_StartStopTimer(void);
static void AudioLevel_StartStopTimer(void);
static void Inertial_StartStopTimer(void);

static void FFTAmplitude_EnableDisableFeature(void);
static void FFTAlarmSpeedRMSStatus_EnableDisableFeature(void);
static void FFTAlarmAccPeakStatus_EnableDisableFeature(void);
static void FFTAlarmSubrangeStatus_EnableDisableFeature(void);

//void APP_UserEvtRx(void *pData);

/**
  * @}
  */

/** @defgroup PREDCTIVE_MAINTENANCE_MAIN_PRIVATE_FUNCTIONS Predictive Maintenance Main Private Functions
  * @{
  */

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  HAL_Init();

  /* Configure the System clock */
  SystemClock_Config();
      
  InitTargetPlatform();
  
  /* Check the MetaDataManager */
 InitMetaDataManager((void *)&known_MetaData,MDM_DATA_TYPE_GMD,NULL); 
  
  PREDMNT1_PRINTF("\n\t(HAL %ld.%ld.%ld_%ld)\r\n"
        "\tCompiled %s %s"

#if defined (__IAR_SYSTEMS_ICC__)
        " (IAR)\r\n"
#elif defined (__CC_ARM)
        " (KEIL)\r\n"
#elif defined (__GNUC__)
        " (STM32CubeIDE)\r\n"
#endif
         "\tSend Every %4dmS Temperature/Humidity/Pressure\r\n"
         "\tSend Every %4dmS Acc/Gyro/Magneto\r\n"
         "\tSend Every %4dmS dB noise\r\n\n",
           HAL_GetHalVersion() >>24,
          (HAL_GetHalVersion() >>16)&0xFF,
          (HAL_GetHalVersion() >> 8)&0xFF,
           HAL_GetHalVersion()      &0xFF,
         __DATE__,__TIME__,
         ALGO_PERIOD_ENV,
         ALGO_PERIOD_ACC_GYRO_MAG,
         ALGO_PERIOD_AUDIO_LEVEL);

#ifdef PREDMNT1_DEBUG_CONNECTION
  PREDMNT1_PRINTF("Debug Connection         Enabled\r\n");
#endif /* PREDMNT1_DEBUG_CONNECTION */

#ifdef PREDMNT1_DEBUG_NOTIFY_TRAMISSION
  PREDMNT1_PRINTF("Debug Notify Trasmission Enabled\r\n\n");
#endif /* PREDMNT1_DEBUG_NOTIFY_TRAMISSION */

  /* Set Node Name */
  ReCallNodeNameFromMemory();
  
  MX_GPIO_Init();
  
  /* Initialize the BlueNRG stack and services */
  BluetoothInit();
  
  /* Check the BootLoader Compliance */
  PREDMNT1_PRINTF("\r\n");
  if(CheckBootLoaderCompliance()) {
    PREDMNT1_PRINTF("BootLoader Compliant with FOTA procedure\r\n\n");
  } else {
    PREDMNT1_PRINTF("ERROR: BootLoader NOT Compliant with FOTA procedure\r\n\n");
  }

  /* initialize timers */
  InitTimers();
  
  /* Predictive Maintenance Initialization */
  InitPredictiveMaintenance();
  
  /* Infinite loop */
  while (1)
  {
    /* When there is not a client connected */
    if(!connected)
    {
      /* Led Blinking when there is not a client connected */
      if(!TargetBoardFeatures.LedStatus) {
        if(!(HAL_GetTick()&0x3FF)) {
          LedOnTargetPlatform();
        }
      } else {
        if(!(HAL_GetTick()&0x3F)) {
          LedOffTargetPlatform();
        }
      }
    }

    if(set_connectable){     
      if(NecessityToSaveMetaDataManager) {
        uint32_t Success = EraseMetaDataManager();
        if(Success) {
          SaveMetaDataManager();
        }
      }

      /* Now update the BLE advertize data and make the Board connectable */
      setConnectable();
      set_connectable = FALSE;
    }
    
    /* Enviromental Features */
    if(BLE_Env_NotifyEvent != BLE_NOTIFY_NOTHING)
    {
      Environmental_StartStopTimer();
      BLE_Env_NotifyEvent = BLE_NOTIFY_NOTHING;
    }
    
    /* Audio Level Features */
    if(BLE_AudioLevel_NotifyEvent != BLE_NOTIFY_NOTHING)
    {
      AudioLevel_StartStopTimer(); 
      BLE_AudioLevel_NotifyEvent = BLE_NOTIFY_NOTHING;
    }
    
    /* Inertial Features */
    if(BLE_Inertial_NotifyEvent != BLE_NOTIFY_NOTHING)
    {
      Inertial_StartStopTimer();   
      BLE_Inertial_NotifyEvent = BLE_NOTIFY_NOTHING;
    }
    
    /* FFT Amplitude Features */
    if(BLE_FFT_Amplitude_NotifyEvent != BLE_NOTIFY_NOTHING)
    {
      FFTAmplitude_EnableDisableFeature();
      BLE_FFT_Amplitude_NotifyEvent = BLE_NOTIFY_NOTHING;
    }
    
    /* FFT FFT Alarm Speed Status Features */
    if(BLE_FFTAlarmSpeedStatus_NotifyEvent != BLE_NOTIFY_NOTHING)
    {
      FFTAlarmSpeedRMSStatus_EnableDisableFeature();      
      BLE_FFTAlarmSpeedStatus_NotifyEvent= BLE_NOTIFY_NOTHING;
    }
    
    /* FFT Alarm Acc Peak Status Features */
    if(BLE_FFTAlarmAccPeakStatus_NotifyEvent != BLE_NOTIFY_NOTHING)
    {
      FFTAlarmAccPeakStatus_EnableDisableFeature();
      BLE_FFTAlarmAccPeakStatus_NotifyEvent= BLE_NOTIFY_NOTHING;
    }
    
    /* FFT Alarm Subrange Status Features */
    if(BLE_FFTAlarmSubrangeStatus_NotifyEvent != BLE_NOTIFY_NOTHING)
    {
      FFTAlarmSubrangeStatus_EnableDisableFeature(); 
      BLE_FFTAlarmSubrangeStatus_NotifyEvent= BLE_NOTIFY_NOTHING;     
    }

    /* Handle user button */
    if(ButtonPressed) {
      ButtonCallback();
      ButtonPressed=0;       
    }
    
    if(PredictiveMaintenance){
      /* Manage the vibration analysis */
      if (MotionSP_MainManager() != BSP_ERROR_NONE)
        Error_Handler();
    }

    /* handle BLE event */
    if(HCI_ProcessEvent) {
      HCI_ProcessEvent=0;
      hci_user_evt_proc();
    }

    /* Environmental Data */
    if(SendEnv) {
      SendEnv=0;
      SendEnvironmentalData();
    }
    
    /* Mic Data */
    if (SendAudioLevel) {
      SendAudioLevel = 0;
      SendAudioLevelData();
    }

    /* Motion Data */
    if(SendAccGyroMag) {
      SendAccGyroMag=0;
      SendMotionData();
    }
    
    /* Wait for Event */
    __WFI();
  }
}

/**
  * @brief  Callback for user button
  * @param  None
  * @retval None
  */
static void ButtonCallback(void)
{
  PREDMNT1_PRINTF("\r\nUser Button Pressed\r\n\r\n");
}


/**
  * @brief  Send Motion Data Acc/Mag/Gyro to BLE
  * @param  None
  * @retval None
  */
static void SendMotionData(void)
{
  MOTION_SENSOR_Axes_t ACC_Value;
  MOTION_SENSOR_Axes_t GYR_Value;
  MOTION_SENSOR_Axes_t MAG_Value;
  
  BLE_MANAGER_INERTIAL_Axes_t ACC_SensorValue;
  BLE_MANAGER_INERTIAL_Axes_t GYR_SensorValue;
  BLE_MANAGER_INERTIAL_Axes_t MAG_SensorValue;

  /* Reset the Acc values */
  ACC_Value.x = ACC_Value.y = ACC_Value.z =0;
  
  /* Reset the Gyro values */
  GYR_Value.x = GYR_Value.y = GYR_Value.z =0;
  
  /* Reset the Magneto values */
  MAG_Value.x = MAG_Value.y = MAG_Value.z =0;

  /* Read the Acc values */
  if(TargetBoardFeatures.AccSensorIsInit)
  {
    MOTION_SENSOR_GetAxes(ACCELERO_INSTANCE, MOTION_ACCELERO, &ACC_Value);
  }
  
  /* Read the Gyro values */
  if(TargetBoardFeatures.GyroSensorIsInit)
  {
    MOTION_SENSOR_GetAxes(GYRO_INSTANCE,MOTION_GYRO, &GYR_Value);
  }

  /* Read the Magneto values */
  if(TargetBoardFeatures.MagSensorIsInit)
  {
    MOTION_SENSOR_GetAxes(MAGNETO_INSTANCE, MOTION_MAGNETO, &MAG_Value);
  }
  
  ACC_SensorValue.x= ACC_Value.x;
  ACC_SensorValue.y= ACC_Value.y;
  ACC_SensorValue.z= ACC_Value.z;
  
  GYR_SensorValue.x= GYR_Value.x;
  GYR_SensorValue.y= GYR_Value.y;
  GYR_SensorValue.z= GYR_Value.z;

  MAG_SensorValue.x= MAG_Value.x;
  MAG_SensorValue.y= MAG_Value.y;
  MAG_SensorValue.z= MAG_Value.z;
  
  /* Send the Data with BLE */
  BLE_AccGyroMagUpdate(&ACC_SensorValue,&GYR_SensorValue,&MAG_SensorValue);
}

/**
* @brief  User function that is called when 1 ms of PDM data is available.
* @param  none
* @retval None
*/
static void AudioProcess(void)
{
  int32_t i;
  int32_t NumberMic;
  
  CCA02M2_AUDIO_IN_PDMToPCM(CCA02M2_AUDIO_INSTANCE,(uint16_t * )PDM_Buffer,PCM_Buffer);

  if(AudioLevelEnable)
  {
    for(i = 0; i < (NumSample/AUDIO_IN_CHANNELS); i++){
      for(NumberMic=0;NumberMic<AUDIO_IN_CHANNELS;NumberMic++) {
        RMS_Ch[NumberMic] += (float)((int16_t)PCM_Buffer[i*AUDIO_IN_CHANNELS+NumberMic] * ((int16_t)PCM_Buffer[i*AUDIO_IN_CHANNELS+NumberMic]));
      }
    }
  }
}

/**
  * @brief  Send Audio Level Data (Ch1) to BLE
  * @param  None
  * @retval None
  */
static void SendAudioLevelData(void)
{
  int32_t NumberMic;
  uint16_t DBNOISE_Value_Ch[AUDIO_IN_CHANNELS];
  
  for(NumberMic=0;NumberMic<(AUDIO_IN_CHANNELS);NumberMic++) {
    DBNOISE_Value_Ch[NumberMic] = 0;

    RMS_Ch[NumberMic] /= ((float)(NumSample/AUDIO_IN_CHANNELS)*ALGO_PERIOD_AUDIO_LEVEL);

    DBNOISE_Value_Ch[NumberMic] = (uint16_t)((120.0f - 20 * log10f(32768 * (1 + 0.25f * (AUDIO_VOLUME_INPUT /*AudioInVolume*/ - 4))) + 10.0f * log10f(RMS_Ch[NumberMic])) * 0.3f + DBNOISE_Value_Old_Ch[NumberMic] * 0.7f);
    DBNOISE_Value_Old_Ch[NumberMic] = DBNOISE_Value_Ch[NumberMic];
    RMS_Ch[NumberMic] = 0.0f;
  }
  
  BLE_AudioLevelUpdate(DBNOISE_Value_Ch, AUDIO_IN_CHANNELS);
}

/**
  * @brief  Read Environmental Data (Temperature/Pressure/Humidity) from sensor
  * @param  int32_t *PressToSend
  * @param  uint16_t *HumToSend
  * @param  int16_t *Temp1ToSend
  * @param  int16_t *Temp2ToSend
  * @retval None
  */
void ReadEnvironmentalData(int32_t *PressToSend,uint16_t *HumToSend,int16_t *Temp1ToSend,int16_t *Temp2ToSend)
{
  float SensorValue;
  int32_t decPart, intPart;
  
  *PressToSend=0;
  *HumToSend=0;
  *Temp2ToSend=0,*Temp1ToSend=0;

  /* Read Humidity */
  if(TargetBoardFeatures.HumSensorIsInit) {
    ENV_SENSOR_GetValue(HUMIDITY_INSTANCE,ENV_HUMIDITY,&SensorValue);
    MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
    *HumToSend = intPart*10+decPart;
  }

  /* Read Temperature for sensor 1 */
  if(TargetBoardFeatures.TempSensorsIsInit[0]){
    ENV_SENSOR_GetValue(TEMPERATURE_INSTANCE_1,ENV_TEMPERATURE,&SensorValue);
    MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
    *Temp1ToSend = intPart*10+decPart;
  }
  
  /* Read Pressure */
  if(TargetBoardFeatures.PressSensorIsInit){
    ENV_SENSOR_GetValue(PRESSURE_INSTANCE,ENV_PRESSURE,&SensorValue);
    MCR_BLUEMS_F2I_2D(SensorValue, intPart, decPart);
    *PressToSend=intPart*100+decPart;
  }

  /* Read Temperature for sensor 2 */
  if(TargetBoardFeatures.TempSensorsIsInit[1]) {
    ENV_SENSOR_GetValue(TEMPERATURE_INSTANCE_2,ENV_TEMPERATURE,&SensorValue);
    MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
    *Temp2ToSend = intPart*10+decPart;
  }
}

/**
  * @brief  Send Environmetal Data (Temperature/Pressure/Humidity) to BLE
  * @param  None
  * @retval None
  */
static void SendEnvironmentalData(void)
{
  /* Pressure,Humidity, and Temperatures*/
  //if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV))
  {
    int32_t PressToSend;
    uint16_t HumToSend;
    int16_t Temp2ToSend,Temp1ToSend;
    
    /* Read all the Environmental Sensors */
    ReadEnvironmentalData(&PressToSend,&HumToSend, &Temp1ToSend,&Temp2ToSend);

#ifdef PREDMNT1_DEBUG_NOTIFY_TRAMISSION
    if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
      BytesToWrite = sprintf((char *)BufferToWrite,"Sending: Press=%ld Hum=%d Temp1=%d Temp2=%d \r\n", PressToSend, HumToSend, Temp1ToSend, Temp2ToSend);
      Term_Update(BufferToWrite,BytesToWrite);
    } else {
      PREDMNT1_PRINTF("Sending: Press=%ld Hum=%d Temp1=%d Temp2=%d \r\n", PressToSend, HumToSend, Temp1ToSend, Temp2ToSend);
    }
#endif /* PREDMNT1_DEBUG_NOTIFY_TRAMISSION */
    
    BLE_EnvironmentalUpdate(PressToSend,HumToSend,Temp2ToSend,Temp1ToSend);
  }
}

/**
* @brief  Function for initializing timers for sending the information to BLE:
 *  - 1 for sending MotionFX/AR/CP and Acc/Gyro/Mag
 *  - 1 for sending the Environmental info
 * @param  None
 * @retval None
 */
static void InitTimers(void)
{
  uint32_t uwPrescalerValue;
  
  /* Timer Output Compare Configuration Structure declaration */
  TIM_OC_InitTypeDef sConfig;
  
  /* Compute the prescaler value to have TIM1 counter clock equal to 10 KHz */
  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1); 
  
  /* Set TIM1 instance ( Motion ) */
  TimCCHandle.Instance = TIM1;  
  TimCCHandle.Init.Period        = 65535;
  TimCCHandle.Init.Prescaler     = uwPrescalerValue;
  TimCCHandle.Init.ClockDivision = 0;
  TimCCHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_OC_Init(&TimCCHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
 /* Configure the Output Compare channels */
 /* Common configuration for all channels */
  sConfig.OCMode     = TIM_OCMODE_TOGGLE;
  sConfig.OCPolarity = TIM_OCPOLARITY_LOW;
  
  /* Output Compare Toggle Mode configuration: Channel2 for environmental sensor */
  sConfig.Pulse = DEFAULT_uhCCR1_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  
  /* Output Compare Toggle Mode configuration: Channel2 for mic audio level */
  sConfig.Pulse = DEFAULT_uhCCR2_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  
  /* Output Compare Toggle Mode configuration: Channel3 for Acc/Gyro/Mag sensor */
  sConfig.Pulse = DEFAULT_uhCCR3_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0x03, 0x00);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

///** @brief 
// * @param None
// * @retval None
// */
//void APP_UserEvtRx(void *pData)
//{
//  uint32_t i;
//
//  hci_spi_pckt *hci_pckt = (hci_spi_pckt *)pData;
//
//  if(hci_pckt->type == HCI_EVENT_PKT) {
//    hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
//
//    if(event_pckt->evt == EVT_LE_META_EVENT) {
//      evt_le_meta_event *evt = (void *)event_pckt->data;
//
//      for (i = 0; i < (sizeof(hci_le_meta_events_table)/sizeof(hci_le_meta_events_table_type)); i++) {
//        if (evt->subevent == hci_le_meta_events_table[i].evt_code) {
//          hci_le_meta_events_table[i].process((void *)evt->data);
//        }
//      }
//    } else if(event_pckt->evt == EVT_VENDOR) {
//      evt_blue_aci *blue_evt = (void*)event_pckt->data;        
//
//      for (i = 0; i < (sizeof(hci_vendor_specific_events_table)/sizeof(hci_vendor_specific_events_table_type)); i++) {
//        if (blue_evt->ecode == hci_vendor_specific_events_table[i].evt_code) {
//          hci_vendor_specific_events_table[i].process((void *)blue_evt->data);
//        }
//      }
//    } else {
//      for (i = 0; i < (sizeof(hci_events_table)/sizeof(hci_events_table_type)); i++) {
//        if (event_pckt->evt == hci_events_table[i].evt_code) {
//          hci_events_table[i].process((void *)event_pckt->data);
//        }
//      }
//    }
//  }
//}

/**
 * @brief  Get hardware and firmware version
 *
 * @param  Hardware version
 * @param  Firmware version
 * @retval Status
 */
uint8_t getBlueNRG2_Version(uint8_t *hwVersion, uint16_t *fwVersion)
{
  uint8_t status;
  uint8_t hci_version, lmp_pal_version;
  uint16_t hci_revision, manufacturer_name, lmp_pal_subversion;
  uint8_t DTM_version_major, DTM_version_minor, DTM_version_patch, DTM_variant, BTLE_Stack_version_major, BTLE_Stack_version_minor, BTLE_Stack_version_patch, BTLE_Stack_development;
  uint16_t DTM_Build_Number, BTLE_Stack_variant, BTLE_Stack_Build_Number;


  status = hci_read_local_version_information(&hci_version, &hci_revision, &lmp_pal_version, 
				                              &manufacturer_name, &lmp_pal_subversion);

  if (status == BLE_STATUS_SUCCESS) {
    *hwVersion = hci_revision >> 8;
  }
  else {
    PREDMNT1_PRINTF("Error= %x \r\n", status);
  }
  
  
  status = aci_hal_get_firmware_details(&DTM_version_major,
                                        &DTM_version_minor,
                                        &DTM_version_patch,
                                        &DTM_variant,
                                        &DTM_Build_Number,
                                        &BTLE_Stack_version_major,
                                        &BTLE_Stack_version_minor,
                                        &BTLE_Stack_version_patch,
                                        &BTLE_Stack_development,
                                        &BTLE_Stack_variant,
                                        &BTLE_Stack_Build_Number);
  
  if (status == BLE_STATUS_SUCCESS) {
    *fwVersion = BTLE_Stack_version_major  << 8;  // Major Version Number
    *fwVersion |= BTLE_Stack_version_minor << 4;  // Minor Version Number
    *fwVersion |= BTLE_Stack_version_patch;       // Patch Version Number
  }
  else {
    PREDMNT1_PRINTF("Error= %x \r\n", status);
  }
  
    
  return status;
}

/** @brief Predictive Maintenance Initialization
  * @param None
  * @retval None
  */
static void InitPredictiveMaintenance(void)
{
  /* Set the vibration parameters with default values */
  MotionSP_SetDefaultVibrationParam();
  
  /* Read Vibration Parameters From Memory */
  ReCallVibrationParamFromMemory();
  
  PREDMNT1_PRINTF("\r\nAccelerometer parameters:\r\n");
  PREDMNT1_PRINTF("AccOdr= %d\tAccFifoBdr= %d\tfs= %d\r\n", AcceleroParams.AccOdr,
                                                            AcceleroParams.AccFifoBdr,
                                                            AcceleroParams.fs);


  PREDMNT1_PRINTF("\r\nMotionSP parameters:\r\n");
  PREDMNT1_PRINTF("size= %d\twind= %d\ttacq= %d\tovl= %d\tsubrange_num= %d\r\n\n", MotionSP_Parameters.FftSize,
                                                                                   MotionSP_Parameters.window,
                                                                                   MotionSP_Parameters.tacq,
                                                                                   MotionSP_Parameters.FftOvl,
                                                                                   MotionSP_Parameters.subrange_num); 
  
  PREDMNT1_PRINTF("************************************************************************\r\n\r\n");
  
  /* Initializes accelerometer with vibration parameters values */
  if(MotionSP_AcceleroConfig()) {
    PREDMNT1_PRINTF("\tFailed Set Accelerometer Parameters\r\n\n");
  } else {
    PREDMNT1_PRINTF("\tOK Set Accelerometer Parameters\r\n\n");
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 128000000
  *            HCLK(Hz)                       = 128000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 256
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
  clocked below the maximum system frequency, to update the voltage scaling value 
  regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 256; //336; //192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;//4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /* Activate the OverDrive to reach the 180 MHz Frequency */  
  //HAL_PWREx_EnableOverDrive();
  
  /*Select Main PLL output as USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CK48CLKSOURCE_PLLQ;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
  
  HAL_InitTick(0x00);
}

/**
 * @brief  Check if there are a valid Node Name Values in Memory and read them
 * @param  None
 * @retval unsigned char Success/Not Success
 */
static unsigned char ReCallNodeNameFromMemory(void)
{
  /* ReLoad the Node Name Values from RAM */
  unsigned char Success=0;
  
  //Set the BLE Board Name 
  sprintf(BlueNRG_StackValue.BoardName,"%s%c%c%c","PM1V",
          PREDMNT1_VERSION_MAJOR,
          PREDMNT1_VERSION_MINOR,
          PREDMNT1_VERSION_PATCH);

  /* Recall the node name Credential saved */
  MDM_ReCallGMD(GMD_NODE_NAME,(void *)&NodeName);
  
  if(NodeName[0] != 0x12)
  {
    NodeName[0]= 0x12;
    
    for(int i=0; i<7; i++)
      NodeName[i+1]= BlueNRG_StackValue.BoardName[i];
    
    MDM_SaveGMD(GMD_NODE_NAME,(void *)&NodeName);
    NecessityToSaveMetaDataManager=1;
  }
  else
  {
    for(int i=0; i<7; i++)
      BlueNRG_StackValue.BoardName[i]= NodeName[i+1];
  }

  return Success;
}

/**
 * @brief  Check if there are a valid Vibration Parameters Values in Memory and read them
 * @param pAccelerometer_Parameters Pointer to Accelerometer parameter structure
 * @param pMotionSP_Parameters Pointer to Board parameter structure
 * @retval unsigned char Success/Not Success
 */
static unsigned char ReCallVibrationParamFromMemory(void)
{
  /* ReLoad the Vibration Parameters Values from RAM */
  unsigned char Success=0;
  
  PREDMNT1_PRINTF("Recall the vibration parameter values from FLASH\r\n");

  /* Recall the Vibration Parameters Values saved */
  MDM_ReCallGMD(GMD_VIBRATION_PARAM,(void *)VibrationParam);
  
  if(VibrationParam[0] == CHECK_VIBRATION_PARAM)
  {
    AcceleroParams.AccOdr=              VibrationParam[1];
    AcceleroParams.AccFifoBdr=          VibrationParam[2];
    AcceleroParams.fs=                  VibrationParam[3];
    MotionSP_Parameters.FftSize=        VibrationParam[4];
    MotionSP_Parameters.tau=            VibrationParam[5];
    MotionSP_Parameters.window=         VibrationParam[6];
    MotionSP_Parameters.td_type=        VibrationParam[7];
    MotionSP_Parameters.tacq=           VibrationParam[8];
    MotionSP_Parameters.FftOvl=         VibrationParam[9];
    MotionSP_Parameters.subrange_num=   VibrationParam[10];
    
    PREDMNT1_PRINTF("Vibration parameter values read from FLASH\r\n");
    
#if (USE_SPI_FOR_DIL24 == 0)
    if(AcceleroParams.AccFifoBdr > 1660)
    {
      PREDMNT1_PRINTF("AccFifoBdr value out of limit\r\n");
      AcceleroParams.AccFifoBdr= 1660;
      AcceleroParams.AccOdr=  1660;
      MotionSP_AcceleroConfig();
      SaveVibrationParamToMemory();
    }
    else
#endif /* USE_SPI_FOR_DIL24 */
    {
      NecessityToSaveMetaDataManager=0;
    }
  }
  else
  {
    PREDMNT1_PRINTF("Vibration parameters values not present in FLASH\r\n");
    SaveVibrationParamToMemory();
  }

  return Success;
}

/**********************************/
/* Characteristics Notify Service */
/**********************************/

/**
 * @brief  This function is called when there is a change on the gatt attribute for Environmental
 *         for Start/Stop Timer
 * @param  None
 * @retval None
 */
static void Environmental_StartStopTimer(void)
{
  if( (BLE_Env_NotifyEvent == BLE_NOTIFY_SUB) &&
      (!EnvironmentalTimerEnabled) ){
    /* Start the TIM Base generation in interrupt mode (for environmental sensor) */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }

    /* Set the new Capture compare value */
    {
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
      /* Set the Capture Compare Register value (for environmental) */
      __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + uhCCR1_Val));
    }
    
    EnvironmentalTimerEnabled= 1;
  }
  
  if( (BLE_Env_NotifyEvent == BLE_NOTIFY_UNSUB) &&
      (EnvironmentalTimerEnabled) ){
    /* Stop the TIM Base generation in interrupt mode (for environmental sensor) */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }
    
    EnvironmentalTimerEnabled= 0;
  }
}

/**
 * @brief  This function is called when there is a change on the gatt attribute for Audio Level
 *         for Start/Stop Timer
 * @param  None
 * @retval None
 */
static void AudioLevel_StartStopTimer(void)
{
  if( (BLE_AudioLevel_NotifyEvent == BLE_NOTIFY_SUB) &&
      (!AudioLevelTimerEnabled) ) {
    int32_t Count;
      
    InitMics(AUDIO_IN_SAMPLING_FREQUENCY, AUDIO_VOLUME_INPUT);
    AudioLevelEnable= 1;
    
    for(Count=0;Count<AUDIO_IN_CHANNELS;Count++) {
      RMS_Ch[Count]=0;
      DBNOISE_Value_Old_Ch[Count] =0;
    }
    
    /* Start the TIM Base generation in interrupt mode (for mic audio level) */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }

    /* Set the new Capture compare value */
    {
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
      /* Set the Capture Compare Register value (for mic audio level) */
      __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_2, (uhCapture + uhCCR2_Val));
    }
    
    AudioLevelTimerEnabled= 1;
  }
  
  if( (BLE_AudioLevel_NotifyEvent == BLE_NOTIFY_UNSUB) &&
      (AudioLevelTimerEnabled) ) {
    DeInitMics();
    AudioLevelEnable= 0;
    
    /* Stop the TIM Base generation in interrupt mode (for mic audio level) */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }  
    
    AudioLevelTimerEnabled= 0;
  }
}

/**
 * @brief  This function is called when there is a change on the gatt attribute for inertial
 *         for Start/Stop Timer
 * @param  None
 * @retval None
 */
static void Inertial_StartStopTimer(void)
{ 
  if( (BLE_Inertial_NotifyEvent == BLE_NOTIFY_SUB) &&
      (!InertialTimerEnabled) ){
    /* Start the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_3) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }

    /* Set the new Capture compare value */
    {
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
      /* Set the Capture Compare Register value (for Acc/Gyro/Mag sensor) */
      __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_3, (uhCapture + uhCCR3_Val));
    }
    
    InertialTimerEnabled= 1;
  }
  
  if( (BLE_Inertial_NotifyEvent == BLE_NOTIFY_UNSUB) &&
      (InertialTimerEnabled) ){
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_3) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }      
    
    InertialTimerEnabled= 0;
  }
}

/**
 * @brief  This function is called when there is a change on the gatt attribute for FFT Amplitude
 *         for Enable/Disable Feature
 * @param  None
 * @retval None
 */
static void FFTAmplitude_EnableDisableFeature(void)
{
  if(BLE_FFT_Amplitude_NotifyEvent == BLE_NOTIFY_SUB) {
    PredictiveMaintenance= 1;
    FFT_Amplitude= 1;
  }
  
  if(BLE_FFT_Amplitude_NotifyEvent == BLE_NOTIFY_UNSUB) {
    disable_FIFO();
    EnableDisable_ACC_HP_Filter(HPF_NONE);
    PredictiveMaintenance= 0;
    FFT_Amplitude= 0;
    MotionSP_Running = 0;
  }
}

/**
 * @brief  This function is called when there is a change on the gatt attribute for FFT Alarm Speed RMS
 *         for Enable/Disable Feature
 * @param  None
 * @retval None
 */
static void FFTAlarmSpeedRMSStatus_EnableDisableFeature(void)
{
  if(BLE_FFTAlarmSpeedStatus_NotifyEvent == BLE_NOTIFY_SUB) {
    if(!PredictiveMaintenance)
    {
      PredictiveMaintenance= 1;
      FFT_Alarm= 1;
    }
  }
  
  if(BLE_FFTAlarmSpeedStatus_NotifyEvent == BLE_NOTIFY_UNSUB) {
    if(PredictiveMaintenance)
    {
      disable_FIFO();
      EnableDisable_ACC_HP_Filter(HPF_NONE);
      PredictiveMaintenance= 0;
      FFT_Alarm= 0;
      MotionSP_Running = 0;
    }
  }
}

/**
 * @brief  This function is called when there is a change on the gatt attribute for FFT Alarm Acc Peak Status
 *         for Enable/Disable Feature 
 * @param  None
 * @retval None
 */
static void FFTAlarmAccPeakStatus_EnableDisableFeature(void)
{
  if(BLE_FFTAlarmAccPeakStatus_NotifyEvent == BLE_NOTIFY_SUB) {
    if(!PredictiveMaintenance)
    {
      PredictiveMaintenance= 1;
      FFT_Alarm= 1;
    }
  }
  
  if(BLE_FFTAlarmAccPeakStatus_NotifyEvent == BLE_NOTIFY_UNSUB ){
    if(PredictiveMaintenance)
    {
      disable_FIFO();
      EnableDisable_ACC_HP_Filter(HPF_NONE);
      PredictiveMaintenance= 0;
      FFT_Alarm= 0;
      MotionSP_Running = 0;
    }
  }
}

/**
 * @brief  This function is called when there is a change on the gatt attribute for FFT Alarm Subrange Status
 *         for Enable/Disable Feature 
 * @param  None
 * @retval None
 */
static void FFTAlarmSubrangeStatus_EnableDisableFeature(void)
{
  if(BLE_FFTAlarmSubrangeStatus_NotifyEvent == BLE_NOTIFY_SUB) {
    if(!PredictiveMaintenance)
    {
      PredictiveMaintenance= 1;
      FFT_Alarm= 1;
    }
  }
  
  if(BLE_FFTAlarmSubrangeStatus_NotifyEvent == BLE_NOTIFY_UNSUB) {
    if(PredictiveMaintenance)
    {
      disable_FIFO();
      EnableDisable_ACC_HP_Filter(HPF_NONE);
      PredictiveMaintenance= 0;
      FFT_Alarm= 0;
      MotionSP_Running = 0;
    }
  }
}

/**
  * @}
  */

/** @defgroup PREDCTIVE_MAINTENANCE_MAIN_CALLBACK_FUNCTIONS Predictive Maintenance Main CallBack Functions
  * @{
  */

/**
  * @brief  Output Compare callback in non blocking mode 
  * @param  htim : TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t uhCapture=0;
  
  /* TIM1_CH1 toggling with frequency = 2 Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
     uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    /* Set the Capture Compare Register value (for environmental sensor) */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + uhCCR1_Val));
    SendEnv=1;
  }
 
  /* TIM1_CH2 toggling with frequency = 20 Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
     uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    /* Set the Capture Compare Register value (for mic audio level) */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_2, (uhCapture + uhCCR2_Val));
    SendAudioLevel=1;
  }

  /* TIM1_CH3 toggling with frequency = 20 Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
     uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
    /* Set the Capture Compare Register value (for Acc/Gyro/Mag sensor) */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_3, (uhCapture + uhCCR3_Val));
    SendAccGyroMag=1;
  }
}

/**
* @brief  Half Transfer user callback, called by BSP functions.
* @param  None
* @retval None
*/
void CCA02M2_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)
{
  AudioProcess();
}

/**
* @brief  Transfer Complete user callback, called by BSP functions.
* @param  None
* @retval None
*/
void CCA02M2_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
{
  AudioProcess();
}

/**
 * @brief  EXTI line detection callback.
 * @param  uint16_t GPIO_Pin Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{  
  switch(GPIO_Pin){
#ifdef IKS01A2_LSM6DSL_0
  case GPIO_PIN_5:
#endif /* IKS01A2_LSM6DSL_0 */
#if (defined(IKS01A2_ISM330DLC_0) || defined(IKS01A3_ISM330DLC_0))
  case M_INT2_O_PIN:
#endif /* (defined(IKS01A2_ISM330DLC_0) || defined(IKS01A3_ISM330DLC_0)) */
    if(FifoEnabled)
      MotionSP_FifoFull_IRQ_Rtn();
    else
      MotionSP_DataReady_IRQ_Rtn();
    break;
  }
}

/**
  * @}
  */

/** @defgroup PREDCTIVE_MAINTENANCE_MAIN_EXPORTED_FUNCTIONS Predictive Maintenance Main Exported Functions
  * @{
  */

/**
  * @brief This function provides accurate delay (in milliseconds) based 
  *        on variable incremented.
  * @note This is a user implementation using WFI state
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < Delay){
    __WFI();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1){
  }
}

/**
 * @brief  Save vibration parameters values to memory
 * @param pAccelerometer_Parameters Pointer to Accelerometer parameter structure
 * @param pMotionSP_Parameters Pointer to Board parameter structure
 * @retval unsigned char Success/Not Success
 */
unsigned char SaveVibrationParamToMemory(void)
{
  /* ReLoad the Vibration Parameters Values from RAM */
  unsigned char Success=0;

  VibrationParam[0]= CHECK_VIBRATION_PARAM;
  VibrationParam[1]=  (uint16_t)AcceleroParams.AccOdr;
  VibrationParam[2]=  (uint16_t)AcceleroParams.AccFifoBdr;
  VibrationParam[3]=  (uint16_t)AcceleroParams.fs;
  VibrationParam[4]=  (uint16_t)MotionSP_Parameters.FftSize;
  VibrationParam[5]=  (uint16_t)MotionSP_Parameters.tau;
  VibrationParam[6]=  (uint16_t)MotionSP_Parameters.window;
  VibrationParam[7]=  (uint16_t)MotionSP_Parameters.td_type;
  VibrationParam[8]=  (uint16_t)MotionSP_Parameters.tacq;
  VibrationParam[9]=  (uint16_t)MotionSP_Parameters.FftOvl;
  VibrationParam[10]= (uint16_t)MotionSP_Parameters.subrange_num;
  
  PREDMNT1_PRINTF("Vibration parameters values will be saved in FLASH\r\n");
  MDM_SaveGMD(GMD_VIBRATION_PARAM,(void *)VibrationParam);
  NecessityToSaveMetaDataManager=1;

  return Success;
}

/**
  * @}
  */

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: PREDMNT1_PRINTF("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1){
  }
}

#endif

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
