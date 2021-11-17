/**
  @page Demo   STM32H7B3I-DK TouchGFX Demonstration Firmware
 
  @verbatim
  ******************************************************************************
  * @file    TouchGFX/readme.txt 
  * @author  MCD Application Team 
  * @brief   Description of STM32H7B3I-DK Touch GFX Demonstration
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  @endverbatim

The STM32 H7 TouchGFX demonstration is running on STM32H7B3I-DK boards

For more details please refer to the TouchGFX subsection in the full demonstration readme.txt file ../readme.txt

@par Hardware and Software environment

  - This demonstration runs on STM32H7B3xx devices.
    
  - This demonstration has been tested with STMicroelectronics STM32H7B3I-DK revB boards


@par How to use it?

In order to make the program work, you must do the following :

1- Open STM32CubeProgrammer, go to "External loaders" menu then check "MX25LM51245G_STM32H7B3I_DISCO" box 
2- Connect the STM32H7B3I-DK board to PC with USB cable through CN14 and click on STM32CubeProgrammer "Connect" button
3- Use the generated hex file  with STM32CubeProgrammer to program both internal Flash and external OSPI memory:
   - From Erasing and programming Menu browse to select the hex file
     then click on start programming
   
   You can also use the script "program_hexfile.bat" (available within the binaries package) to directly program this hex file into 
   the STM32H7B3I-DK board.
   This script will invoke the STM32CubeProgrammer with the correct OSPI flasher (MX25LM51245G_STM32H7B3I_DISCO)  
   
-> The internal Flash and the external OSPI are now programmed and the demonstration is shown on the board. 


* <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
*/
