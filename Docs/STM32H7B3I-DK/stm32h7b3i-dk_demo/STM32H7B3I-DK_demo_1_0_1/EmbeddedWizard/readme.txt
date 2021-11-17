/**
  @page Demo STM32H7B3I-DK Embedded Wizard GUI Demonstration Firmware
 
  @verbatim
  *********************** (C) COPYRIGHT 2019 TARA Systems **********************
  * @file    readme.txt 
  * @author  TARA Systems
  * @brief   Description of STM32H7B3I-DK GUI Demonstration
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 TARA Systems.
  * All rights reserved.</center></h2>
  *
  * This software is delivered "as is" and shows the usage of other software 
  * components. It is provided as an example software which is intended to be 
  * modified and extended according to particular requirements.
  * 
  * TARA Systems hereby disclaims all warranties and conditions with regard to
  * the software, including all implied warranties and conditions of
  * merchantability and non-infringement of any third party IPR or other rights
  * which may result from the use or the inability to use the software.
  *
  ******************************************************************************
  @endverbatim

The STM32 H7 Embedded Wizard demonstration is running on STM32H7B3I-DK boards

For more details please refer to the Embedded Wizard subsection in the full demonstration readme.txt file ../readme.txt

@par Hardware and Software environment

  - This demonstration runs on STM32H7B3xx devices.
    
  - This demonstration has been tested with STMicroelectronics STM32H7B3I-DK revB boards

@par How to use it ?

To load the demonstration, use STM32CubeProgrammer to program both internal Flash and external OSPI memory.

In order to program the demonstration you must do the following:
1- Open STM32CubeProgrammer, go to "External loaders" menu then check "MX25LM51245G_STM32H7B3I_DISCO" box 
2- Connect the STM32H7B3I-DK board to PC with USB cable through CN14 and click on STM32CubeProgrammer "Connect" button
3- Use the generated hex file  with STM32CubeProgrammer to program both internal Flash and external OSPI memory:
   - From Erasing and programming Menu browse to select the hex file
     then click on start programming
   
   You can also use the script "program_hexfile.bat" (available within the binaries package) to directly program this hex file into 
   the STM32H7B3I-DK board.
   This script will invoke the STM32CubeProgrammer with the correct OSPI flasher (MX25LM51245G_STM32H7B3I_DISCO)

@par Getting started with Embedded Wizard

In order to get familiar with Embedded Wizard and the GUI development work-flow,
we highly recommend to study our online documentation:
  https://doc.embedded-wizard.de/welcome-to-embedded-wizard
  https://doc.embedded-wizard.de/quick-tour
  https://doc.embedded-wizard.de/getting-started-stm32

Furthermore, we have collected many 'Questions and Answers' covering typical
Embedded Wizard programming aspects. Please visit our community:
  http://ask.embedded-wizard.de

Please use this forum to drop your questions, answers and ideas.

* <h3><center>&copy; COPYRIGHT TARA Systems</center></h3>
*/
