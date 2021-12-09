/**
  ******************************************************************************
  * @file    bfa001Vx_bus.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 1.0.0
  * @date    12 November 2020
  * @brief   Header file for the BSP BUS IO driver.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BFA001Vx_BUS_H
#define __BFA001Vx_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>  
  
  
/* Global UART handle */
extern UART_HandleTypeDef hSrvUart;    /* Global UART handle */
  
#define USE_HAL_SPI_REGISTER_CALLBACKS 0
#define USE_HAL_I2C_REGISTER_CALLBACKS 0

/** @addtogroup STEVAL-BFA001Vx
  * @{
  */

/** @defgroup STEVAL-BFA001Vx_BUS STEVAL-BFA001Vx BUS
  * @{
  */
  
  
/** @defgroup STEVAL-BFA001Vx_BUS_EXPORTED_TYPEDEF STEVAL-BFA001Vx BUS Exported Typedef
  * @{
  */
  
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
typedef struct
{
  pI2C_CallbackTypeDef  pMspI2cInitCb;
  pI2C_CallbackTypeDef  pMspI2cDeInitCb;
} BSP_I2C_Cb_t;
#endif /* (USE_HAL_I2C_REGISTER_CALLBACKS == 1) */

#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
typedef struct
{
  pSPI_CallbackTypeDef  pMspSpiInitCb;
  pSPI_CallbackTypeDef  pMspSpiDeInitCb;
} BSP_SPI_Cb_t;
#endif /* (USE_HAL_SPI_REGISTER_CALLBACKS == 1) */
/**
  * @}
  */

/** @defgroup STEVAL-BFA001Vx_BUS_EXPORTED_FUNCTIONS STEVAL-BFA001Vx BUS Exported Functions
  * @{
  */

/* BUS IO driver over I2C Peripheral */
int32_t BSP_I2C1_Init(void);
int32_t BSP_I2C1_DeInit(void);
int32_t BSP_I2C1_IsReady(void);
int32_t BSP_I2C1_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t len);
int32_t BSP_I2C1_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t len);
int32_t BSP_I2C1_WriteReg16(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t len);
int32_t BSP_I2C1_ReadReg16(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t len);
int32_t BSP_I2C1_Send(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_Recv(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_SendRecv(uint16_t DevAddr, uint8_t *pTxdata, uint8_t *pRxdata, uint16_t Length);

/* BUS IO driver over SPI Peripheral */
int32_t BSP_SPI1_Init(void);
int32_t BSP_SPI1_DeInit(void);
int32_t BSP_SPI1_Send(uint8_t *pData, uint16_t len);
int32_t BSP_SPI1_Recv(uint8_t *pData, uint16_t len);
int32_t BSP_SPI1_SendRecv(uint8_t *pTxData, uint8_t *pRxData, uint16_t len);

int32_t BSP_GetTick(void);

HAL_StatusTypeDef UART_Global_Init(void); 

#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
int32_t BSP_BUS_RegisterDefaultMspCallbacks(void);
int32_t BSP_BUS_RegisterMspCallbacks(BSP_BUS_Cb_t *Callbacks);
#endif /* ((USE_HAL_I2C_REGISTER_CALLBACKS == 1) || (USE_HAL_SPI_REGISTER_CALLBACKS == 1)) */

/**
  * @}
  */

/** @defgroup STEVAL-BFA001Vx_PERIPH_CONF_EXPORTED_DEFINES STEVAL-BFA001Vx PERIPH_CONF Exported Defines
  * @{
  */

#define SRV_UART                                UART5
#define SRV_UART_CLK_ENABLE()                   __HAL_RCC_UART5_CLK_ENABLE()
#define SRV_UART_CLK_DISABLE()                  __HAL_RCC_UART5_CLK_DISABLE()
#define SRV_UART_FORCE_RESET()                  __USART5_FORCE_RESET()
#define SRV_UART_RELEASE_RESET()                __USART5_RELEASE_RESET()
#define SRV_UART_RX_Pin                         GPIO_PIN_2
#define SRV_UART_RX_GPIO_Port                   GPIOD
#define SRV_UART_RX_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOD_CLK_ENABLE()
#define SRV_UART_RX_GPIO_AF                     GPIO_AF8_UART5
#define SRV_UART_TX_Pin                         GPIO_PIN_12
#define SRV_UART_TX_GPIO_Port                   GPIOC
#define SRV_UART_TX_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()
#define SRV_UART_TX_GPIO_AF                     GPIO_AF8_UART5
#define SRV_UART_IRQn                           UART5_IRQn

#define SRV_UART_IRQHandler                     UART5_IRQHandler

#define SRV_UART_RX_DMA                         DMA1_Stream0
#define SRV_UART_RX_DMA_CH                      DMA_CHANNEL_4
#define SRV_UART_DMA_CLK_ENABLE()               __HAL_RCC_DMA1_CLK_ENABLE();
#define SRV_UART_RX_DMA_Stream_IRQn             DMA1_Stream0_IRQn

#define SRV_UART_RX_DMA_Stream_IRQHandler       DMA1_Stream0_IRQHandler
#define SRV_UART_TX_DMA                         DMA1_Stream7
#define SRV_UART_TX_DMA_CH                      DMA_CHANNEL_4
#define SRV_UART_TX_DMA_Stream_IRQn             DMA1_Stream7_IRQn

#define SRV_UART_TX_DMA_Stream_IRQHandler       DMA1_Stream7_IRQHandler

/**
  * @}
  */   

 
/** @addtogroup STEVAL-BFA001Vx_IO_I2C_BUS STEVAL-BFA001Vx IO I2C BUS
  * @{
  */

#define ENV_I2C                         I2C1
#define ENV_I2C_CLK_ENABLE()            __I2C1_CLK_ENABLE()
#define ENV_I2C_CLK_DISABLE()           __I2C1_CLK_DISABLE()
#define ENV_I2C_SDA_GPIO_CLK_ENABLE()   __GPIOB_CLK_ENABLE()
#define ENV_I2C_SCL_GPIO_CLK_ENABLE()   __GPIOB_CLK_DISABLE()
#define ENV_I2C_SDA_GPIO_PORT           GPIOB
#define ENV_I2C_SCL_GPIO_PORT           GPIOB
#define ENV_I2C_SDA_PIN                 GPIO_PIN_7
#define ENV_I2C_SCL_PIN                 GPIO_PIN_8
#define ENV_I2C_SDA_AF                  GPIO_AF4_I2C1
#define ENV_I2C_SCL_AF                  GPIO_AF4_I2C1
#define ENV_I2C_FORCE_RESET()           __I2C1_FORCE_RESET()
#define ENV_I2C_RELEASE_RESET()         __I2C1_RELEASE_RESET()
#define ENV_I2C_IRQn                    I2C1_IRQn

#define ENV_I2C_IRQHandler              I2C1_IRQHandler
#define ENV_I2C_EV_IRQn                 I2C1_EV_IRQn
#define ENV_I2C_ER_IRQn                 I2C1_ER_IRQn

/**
  * @}
  */   

/** @addtogroup STEVAL-BFA001Vx_SPI_BUS STEVAL-BFA001Vx SPI BUS
  * @{
  */   

#define MOTION_SPI                      SPI1
#define MOTION_SPI_MODE                 SPI_MODE_MASTER
#define MOTION_SPI_DIRECTON             SPI_DIRECTION_2LINES
#define MOTION_SPI_DATASIZE             SPI_DATASIZE_8BIT
#define MOTION_SPI_CLKPOLARITY          SPI_POLARITY_LOW
#define MOTION_SPI_CLKPHASE             SPI_PHASE_1EDGE
#define MOTION_SPI_NSS                  SPI_NSS_SOFT
#define MOTION_SPI_FIRSTBIT             SPI_FIRSTBIT_MSB
#define MOTION_SPI_TIMODE               SPI_TIMODE_DISABLED
#define MOTION_SPI_CRCPOLYNOMIAL        7
#define MOTION_SPI_BAUNDRATEPRESCALER   SPI_BAUDRATEPRESCALER_16
#define MOTION_SPI_CRCCALCULATION       SPI_CRCCALCULATION_DISABLED
#define MOTION_SPI_MAX_CLOCK            10000000 /* in MHz */
#define MOTION_SPI_BUS_CLOCK            HAL_RCC_GetPCLK2Freq()
#define SPIx_TIMEOUT_MAX                1000 /*<! Maximum timeout value for BUS waiting loops */
#define SPIx_BUFFERSIZE                 264
   
#define MOTION_SPI_CLK_ENABLE()         __HAL_RCC_SPI1_CLK_ENABLE()
#define MOTION_SPI_CLK_DISABLE()        __HAL_RCC_SPI1_CLK_DISABLE()
#define MOTION_SPI_FORCE_RESET()        __HAL_RCC_SPI1_FORCE_RESET()
#define MOTION_SPI_RELEASE_RESET()      __HAL_RCC_SPI1_RELEASE_RESET()   

   
// SPI Reset Pin: PA.8
#define MOTION_SPI_RESET_PIN            GPIO_PIN_8
#define MOTION_SPI_RESET_MODE           GPIO_MODE_OUTPUT_PP
#define MOTION_SPI_RESET_PULL           GPIO_PULLUP
#define MOTION_SPI_RESET_SPEED          GPIO_SPEED_LOW
#define MOTION_SPI_RESET_ALTERNATE      0
#define MOTION_SPI_RESET_PORT           GPIOA
#define MOTION_SPI_RESET_CLK_ENABLE()   __GPIOA_CLK_ENABLE()

// SCLK: PB.3
#define MOTION_SPI_SCLK_PIN             GPIO_PIN_3
#define MOTION_SPI_SCLK_MODE            GPIO_MODE_AF_PP
#define MOTION_SPI_SCLK_PULL            GPIO_PULLUP
#define MOTION_SPI_SCLK_SPEED           GPIO_SPEED_HIGH
#define MOTION_SPI_SCLK_ALTERNATE       GPIO_AF5_SPI1
#define MOTION_SPI_SCLK_PORT            GPIOB
#define MOTION_SPI_SCLK_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define MOTION_SPI_SCLK_CLK_DISABLE()   __HAL_RCC_GPIOB_CLK_DISABLE()

// MISO (Master Input Slave Output): PB.4
#define MOTION_SPI_MISO_PIN             GPIO_PIN_4
#define MOTION_SPI_MISO_MODE            GPIO_MODE_AF_PP
#define MOTION_SPI_MISO_PULL            GPIO_NOPULL
#define MOTION_SPI_MISO_SPEED           GPIO_SPEED_HIGH
#define MOTION_SPI_MISO_ALTERNATE       GPIO_AF5_SPI1
#define MOTION_SPI_MISO_PORT            GPIOB
#define MOTION_SPI_MISO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define MOTION_SPI_MISO_CLK_DISABLE()   __HAL_RCC_GPIOB_CLK_DISABLE()

// MOSI (Master Output Slave Input): PB.5
#define MOTION_SPI_MOSI_PIN             GPIO_PIN_5
#define MOTION_SPI_MOSI_MODE            GPIO_MODE_AF_PP
#define MOTION_SPI_MOSI_PULL            GPIO_NOPULL
#define MOTION_SPI_MOSI_SPEED           GPIO_SPEED_HIGH
#define MOTION_SPI_MOSI_ALTERNATE       GPIO_AF5_SPI1
#define MOTION_SPI_MOSI_PORT            GPIOB
#define MOTION_SPI_MOSI_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define MOTION_SPI_MOSI_CLK_DISABLE()   __HAL_RCC_GPIOB_CLK_DISABLE()

// NSS/CSN/CS: PG.2
#define MOTION_SPI_CS_PIN               GPIO_PIN_2
#define MOTION_SPI_CS_MODE              GPIO_MODE_OUTPUT_PP
#define MOTION_SPI_CS_PULL              GPIO_PULLUP
#define MOTION_SPI_CS_SPEED             GPIO_SPEED_HIGH
#define MOTION_SPI_CS_ALTERNATE         GPIO_AF5_SPI1
#define MOTION_SPI_CS_PORT              GPIOG
#define MOTION_SPI_CS_CLK_ENABLE()      __HAL_RCC_GPIOG_CLK_ENABLE()
#define MOTION_SPI_CS_CLK_DISABLE()     __HAL_RCC_GPIOG_CLK_DISABLE()
 
// IRQ: PA.0
#define MOTION_SPI_IRQ_PIN            GPIO_PIN_0
#define MOTION_SPI_IRQ_MODE           GPIO_MODE_IT_RISING
#define MOTION_SPI_IRQ_PULL           GPIO_NOPULL
#define MOTION_SPI_IRQ_SPEED          GPIO_SPEED_HIGH
#define MOTION_SPI_IRQ_ALTERNATE      0
#define MOTION_SPI_IRQ_PORT           GPIOA
#define MOTION_SPI_IRQ_CLK_ENABLE()   __GPIOA_CLK_ENABLE()
   
#define MOTION_SPI_EXTI_IRQn          EXTI0_IRQn
#define MOTION_SPI_EXTI_IRQHandler    EXTI0_IRQHandler
#define MOTION_SPI_EXTI_PIN           MOTION_SPI_IRQ_PIN
#define MOTION_SPI_EXTI_PORT          MOTION_SPI_IRQ_PORT

#define MOTION_SPI_RX_DMA                       DMA2_Stream2
#define MOTION_SPI_RX_DMA_CH                    DMA_CHANNEL_3
#define MOTION_SPI_DMA_CLK_ENABLE()             __HAL_RCC_DMA2_CLK_ENABLE();
#define MOTION_SPI_RX_DMA_Stream_IRQn           DMA2_Stream2_IRQn
#define MOTION_SPI_RX_DMA_Stream_IRQHandler     DMA2_Stream2_IRQHandler
#define MOTION_SPI_TX_DMA                       DMA2_Stream3
#define MOTION_SPI_TX_DMA_CH                    DMA_CHANNEL_3
#define MOTION_SPI_TX_DMA_Stream_IRQn           DMA2_Stream3_IRQn
#define MOTION_SPI_TX_DMA_Stream_IRQHandler     DMA2_Stream3_IRQHandler
 
/**
  * @}
  */ 
  
   
/** @addtogroup STEVAL-BFA001Vx_IO_MEMORY STEVAL-BFA001Vx IO Memory
  * @{
  */

#define M95M01_DF_HOLD_GPIO_PORT                GPIOI
#define M95M01_DF_HOLD_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOI_CLK_ENABLE()
#define M95M01_DF_HOLD_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOI_CLK_DISABLE()
#define M95M01_DF_HOLD_PIN                      GPIO_PIN_5
#define M95M01_DF_W_GPIO_PORT                   GPIOI
#define M95M01_DF_W_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOI_CLK_ENABLE()
#define M95M01_DF_W_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOI_CLK_DISABLE()
#define M95M01_DF_W_PIN                         GPIO_PIN_6


/**
  * @}
  */

/** @addtogroup STEVAL-BFA001Vx_IO_IOLINK STEVAL-BFA001Vx IO-Link
  * @{
  */

#define L6362A_EN_DIAG_GPIO_PORT                GPIOE
#define L6362A_EN_DIAG_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOE_CLK_ENABLE()
#define L6362A_EN_DIAG_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOE_CLK_DISABLE()
#define L6362A_EN_DIAG_PIN                      GPIO_PIN_7
#define L6362A_OL_GPIO_PORT                     GPIOE
#define L6362A_OL_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOE_CLK_ENABLE()
#define L6362A_OL_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOE_CLK_DISABLE()
#define L6362A_OL_PIN                           GPIO_PIN_8
#define L6362A_EXTI_IRQn                        EXTI9_5_IRQn


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

#endif /* __BFA001Vx_BUS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
