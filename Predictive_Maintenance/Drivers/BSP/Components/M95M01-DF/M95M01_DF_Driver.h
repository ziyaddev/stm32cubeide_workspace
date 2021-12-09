/**
  ******************************************************************************
  * @file    M95M01_DF_driver.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version V1.0.1
  * @date    08-Feb-2019
  * @brief   This file contains definitions for the M95M01_DF_driver.c firmware driver
  ******************************************************************************
 * @attention
 *
 * COPYRIGHT(c) 2019 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __M95M01_DF_DRIVER__H
#define __M95M01_DF_DRIVER__H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Uncomment the line below to expanse the "assert_param" macro in the  drivers code */
//#define USE_FULL_ASSERT_M95M01_DF

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT_M95M01_DF

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function which reports
  *         the name of the source file and the source line number of the call
  *         that failed. If expr is true, it returns no value.
  * @retval None
  */
#define M95M01_DF_assert_param(expr) ((expr) ? (void)0 : M95M01_DF_assert_failed((uint8_t *)__FILE__, __LINE__))

/* Exported functions ------------------------------------------------------- */
void M95M01_DF_assert_failed(uint8_t* file, uint32_t line);
#else
#define M95M01_DF_assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT_M95M01_DF */

/** @addtogroup BSP BSP
  * @{
  */

/** @addtogroup Component Component
  * @{
  */

/** @addtogroup M95M01_DF M95M01-DF
  * @{
  */

/** @addtogroup M95M01_DF_DRIVER M95M01_DF_DRIVER
  * @{
  */

/** @defgroup M95M01_DF_DRIVER_Exported_Types M95M01_DF_DRIVER Exported Types
  * @{
  */

/**
  * @brief  Error code type.
  */
typedef enum {M95M01_DF_OK = (uint8_t)0, M95M01_DF_ERROR = !M95M01_DF_OK} M95M01_DF_Error_et;

/**
* @brief  State type.
*/
typedef enum {M95M01_DF_DISABLE = (uint8_t)0, M95M01_DF_ENABLE = !M95M01_DF_DISABLE} M95M01_DF_State_et;
#define IS_M95M01_DF_State(MODE) ((MODE == M95M01_DF_ENABLE) || (MODE == M95M01_DF_DISABLE) )

/**
  * @brief  Bit status type.
  */
typedef enum {M95M01_DF_RESET = (uint8_t)0, M95M01_DF_SET = !M95M01_DF_RESET} M95M01_DF_BitStatus_et;
#define IS_M95M01_DF_BitStatus(MODE) ((MODE == M95M01_DF_RESET) || (MODE == M95M01_DF_SET))

/**
  * @brief  M95M01_DF input pins.
  */
typedef enum
{
  S = 0,
  HOLD,
  W
} M95M01_DF_PinName_et;

/**
  * @brief  M95M01_DF Instruction set
  */
typedef enum
{
  WREN    = 0x06, //!< Write Enable
  WRDI    = 0x04, //!< Write Disable
  RDSR    = 0x05, //!< Read Status Register
  WRSR    = 0x01, //!< Write Status Register
  READ    = 0x03, //!< Read from Eeprom Array
  WRITE   = 0x02, //!< Write to Eeprom Array
  RDID    = 0x83, //!< Read Identification Page
  WRID    = 0x82, //!< Write Identification Page
  RDLS    = 0x83, //!< Reads the Identification Page lock status
  LID     = 0x82  //!< Locks the Identification page in read-only mode
} M95M01_DF_Instructions_et;

/**
  * @brief    M95M01_DF Write-protected block size
  * @details  To be used inside the status register
  */
typedef enum
{
  BP_NONE         = 0,  //!< None
  BP_UPPERQUARTER = 1,  //!< Upper quarter, 0x018000 - 0x01FFFF
  BP_UPPERHALF    = 2,  //!< Upper half, 0x010000 - 0x01FFFF
  BP_WHOLEMEMORY  = 3   //!< Whole memory, 0x000000 - 0x01FFFF
} M95M01_DF_ProtectedBlock_et;

/**
  * @brief  M95M01_DF Driver Version Info structure definition.
  */
typedef struct
{
  uint8_t   Major;
  uint8_t   Minor;
  uint8_t   Point;
} M95M01_DF_DriverVersion_st;

/**
* @}
*/

/** @defgroup M95M01_DF_DRIVER_Exported_Constants M95M01_DF_DRIVER Exported Constants
  * @{
  */

/**
  * @brief  Bitfield positioning.
  */
#define M95M01_DF_BIT(x)            ((uint8_t)x)

/**
  * @brief  Driver version.
  */
#define M95M01_DF_DRIVER_VERSION_MAJOR (uint8_t)0
#define M95M01_DF_DRIVER_VERSION_MINOR (uint8_t)1
#define M95M01_DF_DRIVER_VERSION_POINT (uint8_t)0

/**
  * @brief  EEPROM sizes.
  */
#define M95M01_DF_MEMORY_SIZE       0x20000u    /* in byte */
#define M95M01_DF_PAGE_SIZE         0x100u      /* in byte */
#define M95M01_DF_PAGE_MAX          (uint16_t)((M95M01_DF_MEMORY_SIZE / M95M01_DF_PAGE_SIZE) - 1)
/**
  * @brief  First address of EEPROM N-page.
  */
#define M95M01_DF_ADDRESS_PAGE(_N_) (uint32_t)((_N_) * M95M01_DF_PAGE_SIZE)

/**
  * @brief   Status Register format.
  */
#define M95M01_DF_SRWD_BIT          M95M01_DF_BIT(7)
#define M95M01_DF_BP_BIT            M95M01_DF_BIT(2)
#define M95M01_DF_WEL_BIT           M95M01_DF_BIT(1)
#define M95M01_DF_WIP_BIT           M95M01_DF_BIT(0)

#define M95M01_DF_SRWD_MASK         (uint8_t)0x80
#define M95M01_DF_BP_MASK           (uint8_t)0x0C
#define M95M01_DF_WEL_MASK          (uint8_t)0x02
#define M95M01_DF_WIP_MASK          (uint8_t)0x01

/**
  * @}
  */

/** @defgroup M95M01_DF_DRIVER_Exported_Functions M95M01_DF_DRIVER Exported Functions
  * @{
  */

M95M01_DF_Error_et M95M01_DF_Get_DriverVersion(M95M01_DF_DriverVersion_st *Version);
M95M01_DF_Error_et M95M01_DF_Activate(void *handle);
M95M01_DF_Error_et M95M01_DF_DeActivate(void *handle);
M95M01_DF_Error_et M95M01_DF_Hold(void *handle);
M95M01_DF_Error_et M95M01_DF_NotHold(void *handle);
M95M01_DF_Error_et M95M01_DF_WriteProtect(void *handle);
M95M01_DF_Error_et M95M01_DF_NotWriteProtect(void *handle);
M95M01_DF_Error_et M95M01_DF_WREN(void *handle);
M95M01_DF_Error_et M95M01_DF_WRDI(void *handle);
M95M01_DF_Error_et M95M01_DF_RDSR(void *handle, uint8_t *data);
M95M01_DF_Error_et M95M01_DF_WRSR(void *handle, uint8_t *data);
M95M01_DF_Error_et M95M01_DF_READ(void *handle, uint32_t address, uint8_t *data, uint16_t datasize);
M95M01_DF_Error_et M95M01_DF_WRITE(void *handle, uint32_t address, uint8_t *data, uint16_t datasize);
M95M01_DF_Error_et M95M01_DF_WIP_CHECK(void *handle, uint8_t *wip );
M95M01_DF_Error_et M95M01_DF_GET_BP(void *handle, uint8_t *bp );
M95M01_DF_Error_et M95M01_DF_SET_BP(void *handle, uint8_t bp );

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

#ifdef __cplusplus
}
#endif

#endif /* __M95M01_DF_DRIVER__H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
