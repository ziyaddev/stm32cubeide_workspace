/**
  ******************************************************************************
  * @file    readme.txt
  * @author  System Research & Applications Team - Catania Lab.
  * @version V2.4.0
  * @date    07-June-2021
  * @brief   Description of the Application FW.
  ******************************************************************************
  *
  * Copyright (c) 2021 STMicroelectronics. All rights reserved.
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0055, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                               www.st.com/SLA0055
  *
  ******************************************************************************
  */

Application Description 

 This firmware package includes Components Device Drivers, Board Support Package
 and example application for the following STMicroelectronics elements:
 - X-NUCLEO-BNRG2A1 Bluetooth Low energy expansion boards
 - X-NUCLEO-IKS01A3 Expansion board for four MEMS sensor devices:
       HTS221, LPS22HH, LSM6DSO, LIS2MDL
	   DIL24 with ISM330DLC
 - X-NUCLEO-CCA02M2 Digital MEMS microphones expansion board
 - NUCLEO-F446RE NUCLEO boards
 - MotionSP software provides real-time vibration analysis in time domain and frequency domain.

 The Example application initializes all the Components and Library creating 4 Custom Bluetooth services:
 - The first service exposes all the HW characteristics related to MEMS sensor devices: Temperature, Humidity, Pressure, Magnetometer, 
   Gyroscope, Accelleromenter and Microphones Signal Noise dB level.
 - The second service exposes the SW characteristic: the Frequency Domain Processing using the MotionSP algorithm.
 - The third Service exposes the console services where we have stdin/stdout and stderr capabilities
 - The last Service is used for configuration purpose
 
 For NUCLEO boards the example application allows the user to control the initialization phase via UART.
 Launch a terminal application and set the UART port to 115200 bps, 8 bit, No Parity, 1 stop bit.
 
 This example must be used with the related ST BLE Sensor Android/iOS application available on Play/itune store (Version 4.10.0 or higher),
 in order to read the sent information by Bluetooth Low Energy protocol
 
                   ---------------------------------------------
                   | Important Hardware Additional Information |
			       ---------------------------------------------
1) There is an hardware conflict between the X-NUCLEO-IKS01A3 expansion board and the X-NUCLEO-CCAM02M2
   expansion board through the Arduino UNO R3 extension connector.
   The hardware conflict is onto 5,6 and 7 pin of the CN9 arduido connector.
   For X-NUCLEO-IKS01A3:
   - In the pin 5 (SB43) and 6 (SB45) there are the interrupts INT1 and INT2 for LSM6DSO component (used for the feature hardware)
   - In the pin 7 (SB47) there is the interrupt for LPS22HH component
   For X-NUCLEO-CCAM02M2:
   - In the pin 5 (SB7)  there is the clock for the Microphones
   - In the pin 6 (SB15) there is the clock x2 for the Microphones
   - In the pin 7 (SB17) there is the connection for the microphone PDM34 (solder bridge open as default)
   For this reason the related gpio pins is not configured.

2) For only X-NUCLEO-IKS01A3 you can setect I2C or SPI connection mode for DIL24 socket by modification in the file iks01a3_conf.h USE_SPI_FOR_DIL24 define as:
   - 0 --> for enable I2C connection 
   - 1 --> for enable SPI connection
  I2C connection is enabled as default.
  These hardware patch to be performed on the X-NUCLEO-IKS01A3 to enable SPI support on DIL24 socket:
   - Open solder bridges  SB5, SB12, SB19 and SB23
   - Close solder bridges SB6, SB10, SB18 and SB22
  
3) BlueNRG-2 library does not work with the stock firmware that is loaded in the BLE module of X-NUCLEO-BNRG2A1 expansion board.
   For this reason:
   - first of all, it is needed to solder on X-NUCLEO-BNRG2A1, if it is not soldered, a 0 Ohm resistor at R117
   - then you can use a standard ST-Link V2-1 with 5 jumper wires female-female together with STSW-BNRGFLASHER software tool
    (currently available only for Windows PC) in order to update the firmware of the BLE module of X-NUCLEO-BNRG2A1.
   Read user manual for more details.
   
                   ---------------------------------------------
                   | Important Software Additional Information |
			       ---------------------------------------------
- The IKS01A3 BSP and ISM330DLC have been treated as an exception in this package since in their version are not available API used to execute the predctive maintenance analysis.
The patchs (ism330dlc_Patch.c, iks01a3_motion_sensors_ex_Patch.c, iks01a3_motion_sensors_Patch.c), located in user-space (\STM32F446RE-Nucleo\Demonstrations\Predictive_Maintenance\Patch),
replaces the original BSP file which have been disabled, allowing the whole software structure to work flawlessly.

- BSP driver for X-NUCLEO-BNRG2A1 aren't used. Please refer to BlueNRG-2 middleware in order to handle it.

                              --------------------
                              | VERY IMPORTANT : |
                              --------------------
 1) This example support the Firmware-Over-The-Air (FOTA) update using the ST BLE Sensor Android/iOS application (Version 3.0.0 and above)
 2) This example must run starting at address 0x08004000 in memory and works ONLY if the BootLoader 
    is saved at the beginning of the FLASH (address 0x08000000)
 3) For each IDE (IAR/µVision/System Workbench) and for NUCLEO-F446RE platform,
    there are some scripts *.bat and *.sh that makes the following operations:
     - Full Flash Erase
     - Load the BootLoader on the rigth flash region
     - Load the Program (after the compilation) on the rigth flash region (This could be used for a FOTA)
     - Dump back one single binary that contain BootLoader+Program that could be
       flashed at the flash beginning (address 0x08000000) (This COULD BE NOT used for FOTA)
     - Reset the board

                              ----------------------
                              | Known Limitations: |
                              ----------------------
 - ODR frequency limitation for Nucleo-F446RE with IKS01A3 using I2C connection (maximum value is 1660 Hz without optimize code)
 - MotionSP API do not available with lsm6dso (on board IKS01A3) components.


 Inside the Binary Directory there are the following binaries:
Binary/
¦   +-- NUCLEO-F446RE_PredictiveMaintenance_I2C_v2.4.0.bin		(Program without BootLoader. COULD BE USED     for FOTA)
¦   +-- NUCLEO-F446RE_PredictiveMaintenance_I2C_BL_v2.4.0.bin	(Program with BootLoader.    COULD NOT BE USED for FOTA)
¦   +-- NUCLEO-F446RE_PredictiveMaintenance_SPI_v2.4.0.bin		(Program without BootLoader. COULD BE USED     for FOTA)
¦   +-- NUCLEO-F446RE_PredictiveMaintenance_SPI_BL_v2.4.0.bin	(Program with BootLoader.    COULD NOT BE USED for FOTA)

@par Hardware and Software environment

  - This example runs on Sensor expansion board attached to STM32F446RE devices
    can be easily tailored to any other supported device and development board.
    
  - This example must be used with the related ST BLE Sensor Android/iOS application (Version 4.10.0 or higher) available on Play/itune store,
    in order to read the sent information by Bluetooth Low Energy protocol
    
@par STM32Cube packages:
  - STM32F4xx drivers from STM32CubeF4 V1.26.0
@par X-CUBE packages:
  - X-CUBE-BLE2 V3.2.0
  - X-CUBE-MEMS1 V8.3.0
  - X-CUBE-MEMSMIC1 V5.5.0

@par How to use it ? 

This package contains projects for 3 IDEs viz. IAR, µVision and System Workbench. 
In order to make the  program work, you must do the following:
 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.

For IAR:
 - Open IAR toolchain (this firmware has been successfully tested with Embedded Workbench V8.50.9).
 - Open the IAR project file STM32F446RE-Nucleo\Demonstrations\Predictive_Maintenance\EWARM\PredictiveMaintenance.eww.
 - Rebuild all files and run these script that you find on the same directory:
	- CleanPREDMNT1_IAR_F446.bat

For µVision:
 - Open µVision toolchain (this firmware has been successfully tested with MDK-ARM Professional Version: 5.32.0)
 - Open the µVision project file STM32F446RE-Nucleo\Demonstrations\Predictive_Maintenance\Project.uvprojx.
 - Rebuild all files and run these script that you find on the same directory:
	- CleanPREDMNT1_MDK-ARM_F446.bat
		
For System Workbench:
 - Open STM32CubeIDE (this firmware has been successfully tested with Version 1.6.1)
 - Set the default workspace proposed by the IDE (please be sure that there are not spaces in the workspace path).
 - Press "File" -> "Import" -> "Existing Projects into Workspace"; press "Browse" in the "Select root directory" and choose the path where the System
   Workbench project is located (it should be STM32F446RE-Nucleo\Demonstrations\Predictive_Maintenance\STM32CubeIDE\STM32F446RE-Nucleo\. 
 - Rebuild all files and and run these script that you find on the same directory:
   - if you are on windows and you had installed the STM32 ST-Link utility:
		- CleanPREDMNT1_STM32CubeIDE_F446.bat
   - Otherwise (Linux/iOS or Windows without the STM32 ST-Link Utility):
		- CleanPREDMNT1_STM32CubeIDE_F446.sh
		
 /******************* (C) COPYRIGHT 2021 STMicroelectronics *****END OF FILE****/
