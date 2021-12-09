/**
  ******************************************************************************
  * @file    BLE_Implementation_Template.c
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version V0.3.0
  * @date    18-Jan-2021
  * @brief   BLE Implementation template file.
  *          This file should be copied to the application folder and renamed
  *          to BLE_Implementation.c.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0094, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0094
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "BLE_Manager.h"

/* Exported Variables --------------------------------------------------------*/
int32_t  NeedToClearSecureDB=0;

/* Private functions ---------------------------------------------------------*/
static uint32_t DebugConsoleCommandParsing(uint8_t * att_data, uint8_t data_length);
static void DisconnectionCompletedFunction(void);
static void ConnectionCompletedFunction(uint16_t ConnectionHandle);

/* For Aci Gatt Tx Pool Available Event */
static void AciGattTxPoolAvailableEventFunction(void);

/**********************************************************************************************
 * Callback functions prototypes to manage the extended configuration characteristic commands *
 **********************************************************************************************/
static void ExtConfigCustomCommandCallback(BLE_CustomCommadResult_t *CustomCommand);
static void ExtConfigRebootOnDFUModeCommandCallback(void);
static void ExtConfigPowerOffCommandCallback(void);
static void ExtConfigSetNameCommandCallback(uint8_t *NewName);
static void ExtConfigReadCustomCommandsCallback(JSON_Array *JSON_SensorArray);
static void ExtConfigSetTimeCommandCallback(uint8_t *NewTime);
static void ExtConfigSetDateCommandCallback(uint8_t *NewDate);
static void ExtConfigSetWiFiCommandCallback(BLE_WiFi_CredAcc_t NewWiFiCred);
static void ExtConfigChangePinCommandCallback(uint32_t NewPin);
static void ExtConfigReadCertCommandCallback(uint8_t *Certificate);
static void ExtConfigClearDBCommandCallback();
static void ExtConfigSetCertCommandCallback(uint8_t *Certificate);
static void ExtExtConfigUidCommandCallback(uint8_t **UID);
static void ExtConfigInfoCommandCallback(uint8_t *Answer);
static void ExtConfigHelpCommandCallback(uint8_t *Answer);
static void ExtConfigPowerStatusCommandCallback(uint8_t *Answer);
static void ExtConfigVersionFwCommandCallback(uint8_t *Answer);

/** @brief Initialize the BlueNRG stack and services
  * @param  None
  * @retval None
  */
void BluetoothInit(void)
{
  /* BlueNRG stack setting */
  BlueNRG_StackValue.ConfigValueOffsets                   = CONFIG_DATA_PUBADDR_OFFSET;
  BlueNRG_StackValue.ConfigValuelength                    = CONFIG_DATA_PUBADDR_LEN;
  BlueNRG_StackValue.GAP_Roles                            = GAP_PERIPHERAL_ROLE;
  BlueNRG_StackValue.IO_capabilities                      = IO_CAP_DISPLAY_ONLY;
  BlueNRG_StackValue.AuthenticationRequirements           = BONDING;
  BlueNRG_StackValue.MITM_ProtectionRequirements          = MITM_PROTECTION_REQUIRED;
  BlueNRG_StackValue.SecureConnectionSupportOptionCode    = SC_IS_SUPPORTED;
  BlueNRG_StackValue.SecureConnectionKeypressNotification = KEYPRESS_IS_NOT_SUPPORTED;
  
  /* To set the TX power level of the bluetooth device ( -2,1 dBm )*/
  BlueNRG_StackValue.EnableHighPowerMode= 1; /*  High Power */
  
  /* Values: 0x00 ... 0x31 - The value depends on the device */
  BlueNRG_StackValue.PowerAmplifierOutputLevel =4;
  
  /* BlueNRG services setting */
  BlueNRG_StackValue.EnableConfig    = 1;
  BlueNRG_StackValue.EnableConsole   = 1;
  BlueNRG_StackValue.EnableExtConfig = 0;
  
  /* For Enabling the Secure Connection */
  BlueNRG_StackValue.EnableSecureConnection=0;
  /* Default Secure PIN */
  BlueNRG_StackValue.SecurePIN=123456;

#ifdef BLE_MANAGER_PRINTF
  BlueNRG_StackValue.EnableRandomSecurePIN = 1;
#else /* BLE_MANAGER_PRINTF */
  BlueNRG_StackValue.EnableRandomSecurePIN = 0;
#endif /* BLE_MANAGER_PRINTF */
  
  BlueNRG_StackValue.AdvertisingFilter    = NO_WHITE_LIST_USE;
  
  if(BlueNRG_StackValue.EnableSecureConnection) {
    /* Using the Secure Connection, the Rescan should be done by BLE chip */    
    BlueNRG_StackValue.ForceRescan =0;
  } else {
    BlueNRG_StackValue.ForceRescan =1;
  }
  
  InitBleManager();
}

/**
 * @brief  Custom Service Initialization.
 * @param  None
 * @retval None
 */
void BLE_InitCustomService(void) {
  /* Define Custom Function for Debug Console Command parsing */
  CustomDebugConsoleParsingCallback = &DebugConsoleCommandParsing;
  
  /* Define Custom Function for Connection Completed */
  CustomConnectionCompleted = &ConnectionCompletedFunction;
  
  /* Define Custom Function for Disconnection Completed */
  CustomDisconnectionCompleted = &DisconnectionCompletedFunction;
  
  /***********************************************************************************
   * Callback functions to manage the extended configuration characteristic commands *
   ***********************************************************************************/
  CustomExtConfigCustomCommandCallback = &ExtConfigCustomCommandCallback;
  CustomExtConfigUidCommandCallback = ExtExtConfigUidCommandCallback;
  CustomExtConfigRebootOnDFUModeCommandCallback = &ExtConfigRebootOnDFUModeCommandCallback;
  CustomExtConfigPowerOffCommandCallback = &ExtConfigPowerOffCommandCallback;
  CustomExtConfigSetNameCommandCallback = &ExtConfigSetNameCommandCallback;
  CustomExtConfigReadCustomCommandsCallback = &ExtConfigReadCustomCommandsCallback;
  CustomExtConfigSetTimeCommandCallback = &ExtConfigSetTimeCommandCallback;
  CustomExtConfigSetDateCommandCallback = &ExtConfigSetDateCommandCallback;
  CustomExtConfigSetWiFiCommandCallback = &ExtConfigSetWiFiCommandCallback;
  CustomExtConfigChangePinCommandCallback = &ExtConfigChangePinCommandCallback;
  CustomExtConfigReadCertCommandCallback = &ExtConfigReadCertCommandCallback;
  CustomExtConfigClearDBCommandCallback = &ExtConfigClearDBCommandCallback;
  CustomExtConfigSetCertCommandCallback = &ExtConfigSetCertCommandCallback;
  CustomExtConfigInfoCommandCallback = &ExtConfigInfoCommandCallback;
  CustomExtConfigHelpCommandCallback = &ExtConfigHelpCommandCallback;
  CustomExtConfigPowerStatusCommandCallback = &ExtConfigPowerStatusCommandCallback;
  CustomExtConfigVersionFwCommandCallback = &ExtConfigVersionFwCommandCallback;
  
  /**
  * For each features, user can assign here the pointer at the function for the read request data.
  * For example for the environmental features:
  * 
  * CustomReadRequestEnv = &ReadRequestEnvFunction;
  * 
  * User can define and insert in the BLE_Implementation.c source code the functions for the read request data
  * ReadRequestEnvFunction function is already defined.
  *
  */
  
  /**
  * User can added here the custom service initialization for the selected BLE features.
  * For example for the environmental features:
  * 
  * //BLE_InitEnvService(PressEnable,HumEnable,NumTempEnabled)
  * BleManagerAddChar(BleCharPointer= BLE_InitEnvService(1, 1, 1));
  */
  
}

/**
 * @brief  Set Custom Advertize Data.
 * @param  uint8_t *manuf_data: Advertize Data
 * @retval None
 */
void BLE_SetCustomAdvertizeData(uint8_t *manuf_data)
{
  /**
  * User can add here the custom advertize data setting  for the selected BLE features.
  * For example for the environmental features:
  * 
  *		//Custom advertize data setting for the environmental features
  *		BLE_SetEnvAdvertizeData(manuf_data);
  */
}


/**
 * @brief  Aci Gatt Tx Pool Available Event Function.
 * @param  None
 * @retval None
 */
static void AciGattTxPoolAvailableEventFunction(void)
{
  BLE_MANAGER_PRINTF("Call to AciGattTxPoolAvailableEventFunction\r\n");
}

/**
 * @brief  This function makes the parsing of the Debug Console Commands
 * @param  uint8_t *att_data attribute data
 * @param  uint8_t data_length length of the data
 * @retval uint32_t SendBackData true/false
 */
static uint32_t DebugConsoleCommandParsing(uint8_t * att_data, uint8_t data_length)
{
  uint32_t SendBackData = 1;

    /* Help Command */
    if(!strncmp("help",(char *)(att_data),4)) {
      /* Print Legend */
      SendBackData=0;

      BytesToWrite =sprintf((char *)BufferToWrite,
         "help\n");
      Term_Update(BufferToWrite,BytesToWrite);
    }
	
	/* Add here the parsing for others commands in the debug console  */
	
  return SendBackData;
}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None 
 * @retval None
 */
static void DisconnectionCompletedFunction(void)
{
  BLE_MANAGER_PRINTF("Call to DisconnectionCompletedFunction\r\n");
  BLE_MANAGER_DELAY(100);
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  None 
 * @retval None
 */
static void ConnectionCompletedFunction(uint16_t ConnectionHandle)
{
  BLE_MANAGER_PRINTF("Call to ConnectionCompletedFunction\r\n");
  BLE_MANAGER_DELAY(100);
}

/***********************************************************************************
 * Callback functions to manage the extended configuration characteristic commands *
 ***********************************************************************************/

/**
 * @brief  Callback Function for managing the custom command
 * @param  BLE_CustomCommadResult_t *CustomCommand:
 * @param                            uint8_t *CommandName: Nome of the command
 * @param                            CustomCommand->CommandType: Type of the command
 * @param                            int32_t IntValue:    Integer or boolean parameter
 * @param                            uint8_t *StringValue: String parameter
 * @retval None
 */
static void  ExtConfigCustomCommandCallback(BLE_CustomCommadResult_t *CustomCommand)
{
  STBOX1_PRINTF("Received Custom Command:\r\n");
  STBOX1_PRINTF("\tCommand Name: <%s>\r\n", CustomCommand->CommandName);
  STBOX1_PRINTF("\tCommand Type: <%d>\r\n", CustomCommand->CommandType);
    
  switch(CustomCommand->CommandType) { 
    case BLE_CUSTOM_COMMAND_VOID:
      if(!strncmp((char *)CustomCommand->CommandName,"BleManagerReset",15)) {
          aci_gap_terminate(CurrentConnectionHandle,0x13 /* */);
          HAL_Delay(5000);
          needToResetBLE=1;
      }
    break;
    case BLE_CUSTOM_COMMAND_INTEGER:
      STBOX1_PRINTF("\tInt    Value: <%d>\r\n", CustomCommand->IntValue);
    break;
    case BLE_CUSTOM_COMMAND_ENUM_INTEGER:
      STBOX1_PRINTF("\tInt     Enum: <%d>\r\n", CustomCommand->IntValue);
    break;
    case BLE_CUSTOM_COMMAND_BOOLEAN:
      STBOX1_PRINTF("\tInt    Value: <%d>\r\n", CustomCommand->IntValue);
    break;
    case  BLE_CUSTOM_COMMAND_STRING:
      STBOX1_PRINTF("\tString Value: <%s>\r\n", CustomCommand->StringValue);
    break;
    case  BLE_CUSTOM_COMMAND_ENUM_STRING:
      STBOX1_PRINTF("\tString  Enum: <%s>\r\n", CustomCommand->StringValue);
    break;
  }
}

/**
 * @brief  Custom commands definition
 * @param  JSON_Array *JSON_SensorArray
 * @retval None
 */
static void ExtConfigReadCustomCommandsCallback(JSON_Array *JSON_SensorArray)
{
  /* Clear the previous Costom Command List */
  ClearCustomCommandsList();
  
  /* Add all the custom Commands */
  if(AddCustomCommand("IntValue1", //Name
                      BLE_CUSTOM_COMMAND_INTEGER, //Type
                      -100, //MIN
                      200,  //MAX
                      NULL, //Enum Int
                      NULL, //Enum String
                      NULL, //Description
                      JSON_SensorArray)) {
    STBOX1_PRINTF("Added Command <%s>\r\n","IntValue1");
  } else {
     STBOX1_PRINTF("Error Adding Command <%s>\r\n","IntValue1");
     return;
  }
  
  if(AddCustomCommand("IntValue2", //Name
                      BLE_CUSTOM_COMMAND_INTEGER, //Type
                      10, //MIN
                      3000,  //MAX
                      NULL, //Enum Int
                      NULL, //Enum String
                      NULL, //Description
                      JSON_SensorArray)) {
    STBOX1_PRINTF("Added Command <%s>\r\n","IntValue2");
  } else {
     STBOX1_PRINTF("Error Adding Command <%s>\r\n","IntValue2");
     return;
  }

  if(AddCustomCommand("BleManagerReset", //Name
                      BLE_CUSTOM_COMMAND_VOID, //Type
                      BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN, //MIN
                      BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN,  //MAX
                      NULL, //Enum Int
                      NULL, //Enum String
                      "Reset the Bluetooth", //Description
                      JSON_SensorArray)) {
    STBOX1_PRINTF("Added Command <%s>\r\n","Command1");
  } else {
     STBOX1_PRINTF("Error Adding Command <%s>\r\n","Command1");
     return;
  }
  
  if(AddCustomCommand("StringValue1", //Name
                      BLE_CUSTOM_COMMAND_STRING, //Type
                      BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN, //MIN
                      20,  //MAX
                      NULL, //Enum Int
                      NULL, //Enum String
                      NULL, //Description
                      JSON_SensorArray)) {
    STBOX1_PRINTF("Added Command <%s>\r\n","StringValue1");
  } else {
     STBOX1_PRINTF("Error Adding Command <%s>\r\n","StringValue1");
     return;
  }
  
  if(AddCustomCommand("BooleanValue", //Name
                      BLE_CUSTOM_COMMAND_BOOLEAN, //Type
                      BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN, //MIN
                      BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN,  //MAX
                      NULL, //Enum Int
                      NULL, //Enum String
                      "Example for Boolean", //Description
                      JSON_SensorArray)) {
    STBOX1_PRINTF("Added Command <%s>\r\n","BooleanValue");
  } else {
     STBOX1_PRINTF("Error Adding Command <%s>\r\n","BooleanValue");
     return;
  }
     
  if(AddCustomCommand("StringValue2", //Name
                      BLE_CUSTOM_COMMAND_STRING, //Type
                      4, //MIN
                      10,  //MAX
                      NULL, //Enum Int
                      NULL, //Enum String
                      "It's possible to add a  very very very very very very long description", //Description
                      JSON_SensorArray)) {
    STBOX1_PRINTF("Added Command <%s>\r\n","StringValue2");
  } else {
    STBOX1_PRINTF("Error Adding Command <%s>\r\n","StringValue2");
    return;
  }
  
  //Example of Enum String Custom Command
  {
    //The Last value should be NULL
    char *ValidStringValues[]={"Ciao", "Buona","Giornata",NULL};
    if(AddCustomCommand("StringEnum", //Name
                        BLE_CUSTOM_COMMAND_ENUM_STRING, //Type
                        BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN, //MIN
                        BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN,  //MAX
                        NULL, //Enum Int
                        (void *)ValidStringValues, //Enum String
                        "Example of Enum String", //Description
                        JSON_SensorArray)) {
      STBOX1_PRINTF("Added Command <%s>\r\n","StringEnum");
    } else {
      STBOX1_PRINTF("Error Adding Command <%s>\r\n","StringEnum");
      return;
    }
  } 
  
  //Example of Enum Int Custom Command
  {
    //The Last value should be BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN
    int32_t ValidIntValues[]={-1,12,123,321,BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN};
    if(AddCustomCommand("IntEnum", //Name
                        BLE_CUSTOM_COMMAND_ENUM_INTEGER, //Type
                        BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN, //MIN
                        BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN,  //MAX
                        (void *) ValidIntValues, //Enum Int
                        NULL, //Enum String
                        "Example of Enum Integer", //Description
                        JSON_SensorArray)) {
      STBOX1_PRINTF("Added Command <%s>\r\n","IntEnum");
    } else {
      STBOX1_PRINTF("Error Adding Command <%s>\r\n","IntEnum");
      return;
    }
  } 
  
  //Just one Example of one Invalid Command
  if(AddCustomCommand("ReadCert", //Name
                      BLE_CUSTOM_COMMAND_STRING, //Type
                      4, //MIN
                      10,  //MAX
                      NULL, //Enum Int
                      NULL, //Enum String
                      "Invalid Command...", //Description
                      JSON_SensorArray)) {
    STBOX1_PRINTF("Added Command <%s>\r\n","ReadCert");
  } else {
    STBOX1_PRINTF("Error Adding Command <%s>\r\n","ReadCert");
    return;//not mandatory... it's the last one
  }
}

/**
 * @brief  Callback Function for managing the DFU command
 * @param  None
 * @retval None
 */
static void ExtConfigRebootOnDFUModeCommandCallback(void)
{
  BLE_MANAGER_PRINTF("RebootOnDFUModeCommandCallback\r\n");
  
  /* Insert here the code for managing the received command */
  /* Reboot the board on DFU mode */
  //HAL_NVIC_SystemReset();
}

/**
 * @brief  Callback Function for answering to the UID command
 * @param  uint8_t **UID STM32 UID Return value
 * @retval None
 */
static void ExtExtConfigUidCommandCallback(uint8_t **UID)
{
  *UID = (uint8_t *)STM32_UUID;
}


/**
 * @brief  Callback Function for answering to Info command
 * @param  uint8_t *Answer Return String
 * @retval None
 */
static void ExtConfigInfoCommandCallback(uint8_t *Answer)
{
  sprintf((char *)Answer,"STMicroelectronics %s:\n"
    "Version %c.%c.%c\n"
    "STM32L4R9ZI-SensorTile.box board\n"
    "(HAL %ld.%ld.%ld_%ld)\n"
    "Compiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
    " (IAR)",
#elif defined (__CC_ARM)
    " (KEIL)",
#elif defined (__GNUC__)
    " (STM32CubeIDE)",
#endif
    STBOX1_PACKAGENAME,
    STBOX1_VERSION_MAJOR,
    STBOX1_VERSION_MINOR,
    STBOX1_VERSION_PATCH,
    HAL_GetHalVersion() >>24,
    (HAL_GetHalVersion() >>16)&0xFF,
    (HAL_GetHalVersion() >> 8)&0xFF,
     HAL_GetHalVersion()      &0xFF,
     __DATE__,__TIME__);
}

/**
 * @brief  Callback Function for answering to Help command
 * @param  uint8_t *Answer Return String
 * @retval None
 */
static void ExtConfigHelpCommandCallback(uint8_t *Answer)
{
  sprintf((char *)Answer,"Print out some help\nCiao\nLuca");
}

/**
 * @brief  Callback Function for answering to PowerStatus command
 * @param  uint8_t *Answer Return String
 * @retval None
 */
static void ExtConfigPowerStatusCommandCallback(uint8_t *Answer)
{
  sprintf((char *)Answer,"Plug the Battery");
}
  
/**
 * @brief  Callback Function for answering to VersionFw command
 * @param  uint8_t *Answer Return String
 * @retval None
 */
static void ExtConfigVersionFwCommandCallback(uint8_t *Answer)
{
  sprintf((char *)Answer,"%s_%s_%c.%c.%c",
      BLE_STM32_MICRO,
      BLE_FW_PACKAGENAME,
      BLE_VERSION_FW_MAJOR,
      BLE_VERSION_FW_MINOR,
      BLE_VERSION_FW_PATCH);
}

/**
 * @brief  Callback Function for managing the PowerOff command
 * @param  None
 * @retval None
 */
static void ExtConfigPowerOffCommandCallback(void)
{
  BLE_MANAGER_PRINTF("ExtConfigPowerOffCommandCallback\r\n");
  
  /* Insert here the code for managing the received command */
}

/**
 * @brief  Callback Function for managing the SetName command
 * @param  uint8_t *NewName
 * @retval None
 */
static void ExtConfigSetNameCommandCallback(uint8_t *NewName)
{ 
  BLE_MANAGER_PRINTF("New Board Name = <%s>\r\n", NewName);
  /* Change the Board Name */
  sprintf(BlueNRG_StackValue.BoardName,"%s",NewName);
}

/**
 * @brief  Callback Function for managing the SetTime command
 * @param  uint8_t *NewTime
 * @retval None
 */
static void ExtConfigSetTimeCommandCallback(uint8_t *NewTime)
{
  BLE_MANAGER_PRINTF("New Board Time= <%s>\r\n", NewTime);
  
  /* Insert here the code for changing the RTC time */
}

/**
 * @brief  Callback Function for managing the SetDate command
 * @param  uint8_t *NewDate
 * @retval None
 */
static void ExtConfigSetDateCommandCallback(uint8_t *NewDate)
{
  BLE_MANAGER_PRINTF("New Board Date= <%s>\r\n", NewDate);
  
  /* Insert here the code for changing the RTC Date */
}

/**
 * @brief  Callback Function for managing the SetWiFi command
 * @param  BLE_WiFi_CredAcc_t NewWiFiCred
 * @retval None
 */
static void ExtConfigSetWiFiCommandCallback(BLE_WiFi_CredAcc_t NewWiFiCred)
{
  BLE_MANAGER_PRINTF("NewWiFiCred=\r\n");
  BLE_MANAGER_PRINTF("\tSSID    = <%s>\r\n", NewWiFiCred.SSID);
  BLE_MANAGER_PRINTF("\tPassWd = <%s>\r\n", NewWiFiCred.PassWd);
  BLE_MANAGER_PRINTF("\tSecurity= <%s>\r\n", NewWiFiCred.Security);
  
  /* Insert here the code for changing the Wi-Fi Credential */
}

/**
 * @brief  Callback Function for managing the ReadCert command
 * @param  uint8_t *Certificate to register 
 * @retval None
 */
static void ExtConfigReadCertCommandCallback(uint8_t *Certificate)
{
  const char CertFromSTsafe[] = {
"-----BEGIN CERTIFICATE-----\r\n"
"MIIBjjCCATSgAwIBAgILAgnwIEAhzCJbATkwCgYIKoZIzj0EAwIwTzELMAkGA1UE\r\n"
"BhMCTkwxHjAcBgNVBAoMFVNUTWljcm9lbGVjdHJvbmljcyBudjEgMB4GA1UEAwwX\r\n"
"U1RNIFNUU0FGRS1BIFBST0QgQ0EgMDEwIBcNMjAwMjI2MDAwMDAwWhgPMjA1MDAy\r\n"
"MjYwMDAwMDBaMEYxCzAJBgNVBAYTAkZSMRswGQYDVQQKDBJTVE1pY3JvZWxlY3Ry\r\n"
"b25pY3MxGjAYBgNVBAMMEVNUU0FGRS1BMTEwIEVWQUwyMFkwEwYHKoZIzj0CAQYI\r\n"
"KoZIzj0DAQcDQgAEQCibQYjdHzn8yUyHPbZq1QUYEzSh0SrB2rkj/jDroUNqFkjF\r\n"
"d5mZ5ZxVjFz1mbZUvAIBrwvrT7XpOmVuMRzJRDAKBggqhkjOPQQDAgNIADBFAiB5\r\n"
"yNIKxSMcazW0IvclwZyeo83pVC1Q3tIKSIZJZbP2EgIhAOx7kYZnLUlyuckX0HU4\r\n"
"Tel4Ayt9RewWGxHPZIo4K+JR\r\n"
"-----END CERTIFICATE-----\r\n"
};
                      
  sprintf((char *)Certificate,"%s",CertFromSTsafe);
}

/**
 * @brief  Callback Function for managing the ChangePin command
 * @param  uint32_t NewPin
 * @retval None
 */
static void ExtConfigChangePinCommandCallback(uint32_t NewPin)
{
   BLE_MANAGER_PRINTF("New Board Pin= <%d>\r\n", NewPin);

   BlueNRG_StackValue.SecurePIN=NewPin;
}

/**
 * @brief  Callback Function for managing the ClearDB command
 * @param  None
 * @retval None
 */
static void ExtConfigClearDBCommandCallback()
{
  BLE_MANAGER_PRINTF("ExtConfigClearDBCommandCallback\r\n");
  NeedToClearSecureDB=1;
}

/**
 * @brief  Callback Function for managing the SetCert command
 * @param  uint8_t *Certificate registerd certificate
 * @retval None
 */
static void ExtConfigSetCertCommandCallback(uint8_t *Certificate)
{ 
  BLE_MANAGER_PRINTF("Certificate From Dashboard= <%s>\r\n", Certificate);
}

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
