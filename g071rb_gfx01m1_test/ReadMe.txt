/**
  @page X-NUCLEO-GFX01M1 HelloWorld application

  @verbatim
  ******************** (C) COPYRIGHT 2020 STMicroelectronics *******************
  * @file    GFX01M1_HelloWorld/ReadMe.txt
  * @author  MCD Application Team
  * @brief   Description of the X-NUCLEO-GFX01M1 HelloWorld application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0048
  *
  ******************************************************************************
  @endverbatim

@par Application Description

How to use the X-CUBE-DISPLAY API to display image when using the X-NUCLEO-GFX01M1 expansion board
with a NUCLEO-G071RB STM32 board.

@par Hardware and Software environment

  - This application runs on Display expansion board attached to STM32G071RB devices.
  - If you power the Nucleo board via USB 3.0 port, please check that you have flashed the last version of
    the firmware of ST-Link v2 inside the Nucleo board. In order to flash the last available firmware of the
	  ST-Link v2, you can use the STM32CubeProgrammer Utility.
  - This application has been tested with STMicroelectronics NUCLEO-G071RB RevC and can be easily tailored to
    any other supported device and development board.

@par How to use it ?

This package contains projects for 3 IDEs viz. IAR, µVision and STM32CubeIDE. In order to make the
program work, you must do the following:
 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.

For IAR:
 - Open IAR toolchain.
 - Open the IAR project file EWARM\Project.eww.
 - Rebuild all files and load your image into target memory.
 - Run the application.

For µVision:
 - Open µVision 5 toolchain.
 - Open the µVision project file MDK-ARM\Project.uvprojx.
 - Rebuild all files and load your image into target memory.
 - Run the application.

For STM32CubeIDE:
 - Open STM32CubeIDE.
 - Set the default workspace proposed by the IDE (please be sure that there are not spaces in the workspace path).
 - Press "File" -> "Import" -> "Existing Projects into Workspace"; press "Browse" in the "Select root directory" and choose the path where the project is located (it should be <ProjectName>\STM32CubeIDE).
 - Rebuild all files and load your image into target memory.
 - Run the application.

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */

