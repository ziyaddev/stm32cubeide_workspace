/**
  ******************************************************************************
  * File Name          : Target/mem_conf.h
  * Description        : This file provides code for the configuration
  *                      of the MEM instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MEM_CONF_H__
#define __MEM_CONF_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx.h"
#include "stm32l4xx_nucleo_bus.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* BUS IO Instance handlers */
extern  SPI_HandleTypeDef                   hspi2;
extern  DMA_HandleTypeDef                   hdma_spi2_rx;
extern  DMA_HandleTypeDef                   hdma_spi2_tx;
#define hMEMSPI                             hspi2
#define hMEMDMA_rx                          hdma_spi2_rx
#define hMEMDMA_tx                          hdma_spi2_tx

/* BUS IO functions */
#define MEM_SPI_Init                        BSP_SPI2_Init
#define MEM_SPI_DeInit                      BSP_SPI2_DeInit
#define MEM_SPI_Recv                        BSP_SPI2_Recv
#define MEM_SPI_Send                        BSP_SPI2_Send
#define MEM_SPI_SendRecv                    BSP_SPI2_SendRecv
#define MEM_SPI_Recv_DMA                    BSP_SPI2_Recv_DMA
#define MEM_SPI_Send_DMA                    BSP_SPI2_Send_DMA
#define MEM_SPI_SendRecv_DMA                BSP_SPI2_SendRecv_DMA

/* CS Pin mapping */
#define MEM_CS_GPIO_PORT                    GPIOB
#define MEM_CS_GPIO_PIN                     GPIO_PIN_9

#define MEM_INTERFACE_MODE                  MEM_INTERFACE_SPI_MODE

/* Exported macro ------------------------------------------------------------*/
/* Chip Select macro definition */
#define MEM_CS_LOW()                        WRITE_REG(GPIOB->BRR, GPIO_PIN_9)
#define MEM_CS_HIGH()                       WRITE_REG(GPIOB->BSRR, GPIO_PIN_9)

/* Exported functions prototypes ---------------------------------------------*/

#ifdef __cplusplus
}
#endif
#endif /* __MEM_CONF_H__ */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
