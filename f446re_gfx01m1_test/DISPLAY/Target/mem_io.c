/**
  ******************************************************************************
  * File Name          : Target/mem_io.c
  * Description        : This file provides code for the configuration
  *                      of the MEM IO instances.
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

/* Includes ------------------------------------------------------------------*/
#include "mem_io.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32F4XX_NUCLEO
  * @{
  */

/** @defgroup STM32F4XX_NUCLEO_MEM STM32F4XX_NUCLEO MEM
  * @brief      This file includes the MEM driver of
  *             STM32F4XX_NUCLEO boards.
  * @{
  */

/** @defgroup STM32F4XX_NUCLEO_MEM_Private_Types Private Types
  * @{
  */

/** @defgroup STM32F4XX_NUCLEO_MEM_Private_Constants Private Constants
  * @{
  */
/* Number of MEM instances */
#define MEM_INSTANCES_NBR                       1U
/* Wrap Around value */
#define MEM_WRAP_AROUND_64BYTE                  MX25L6433F_WRAP_AROUND_64BYTE
/* Used Memory Commands */
#define MEM_READ_CMD                            MX25L6433F_READ_CMD
#define MEM_SET_BURST_LENGTH_CMD                MX25L6433F_SET_BURST_LENGTH_CMD

/**
  * @}
  */

/** @defgroup STM32F4XX_NUCLEO_MEM_Private_Macros Private Macros
  * @{
  */
#define BSP_MEM_CHECK_PARAMS(Instance)

/**
  * @}
  */

/** @defgroup STM32F4XX_NUCLEO_MEM_Exported_Variables Exported Variables
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F4XX_NUCLEO_MEM_Private_Variables Private Variables
  * @{
  */
static void                 *MemCompObj;
static MEM_Drv_t            *MemDrv;
static MX25L6433F_IO_t      IOCtx;
static MX25L6433F_Object_t  ObjCtx;
static uint8_t __IO         isBusy;

/**
  * @}
  */

/** @defgroup STM32F4XX_NUCLEO_MEM_Private_FunctionPrototypes Private Functions
  * @{
  */
static int32_t MEM_Probe(uint32_t Instance);

/**
  * @}
  */

/** @addtogroup STM32F4XX_NUCLEO_MEM_Exported_Functions
  * @{
  */
/**
  * @brief  Initializes the MEM.
  * @param  Instance    MEM Instance
  * @retval BSP status
  */
int32_t BSP_MEM_Init(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  BSP_MEM_CHECK_PARAMS(Instance);

  /* Reset Read state */
  isBusy = 0;

  /* Reset Handlers */
  MemCompObj = NULL;
  MemDrv = NULL;

  ret = MEM_Probe(Instance);

  return ret;
}

/**
  * @brief  De-Initializes the MEM resources.
  * @param  Instance MEM Instance
  * @retval BSP status
  */
int32_t BSP_MEM_DeInit(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  BSP_MEM_CHECK_PARAMS(Instance);

  if(MemDrv->DeInit != NULL)
  {
    if(MemDrv->DeInit(MemCompObj) < 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
  }
  else
  {
    ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }

  return ret;
}

/**
  * @brief  Write Data to QuadSPI Memory Interface.
  * @param  Instance MEM Instance
  * @param  pData pointer to Data Memory to be written
  * @param  Address Memory address to write to
  * @param  Size pointer to Data Memory to be written
  * @retval BSP status
  */
int32_t BSP_MEM_BlockErase(uint32_t Instance, uint32_t BlockAddress, uint32_t BlockSize)
{
  int32_t ret = BSP_ERROR_NONE;

  BSP_MEM_CHECK_PARAMS(Instance);

  if(isBusy)
  {
    ret = BSP_ERROR_BUSY;
  }
  else
  {
    isBusy = (1U << 2);  /* Erase Chip OP is 0x4 */

    if(MemDrv->BlockErase != NULL)
    {
      if(MemDrv->BlockErase(MemCompObj, BlockAddress, BlockSize) < 0)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
    }
    else
    {
      ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
    }

    isBusy = 0;
  }

  return ret;
}

/**
  * @brief  Write Data to QuadSPI Memory Interface.
  * @param  Instance MEM Instance
  * @param  pData pointer to Data Memory to be written
  * @param  Address Memory address to write to
  * @param  Size pointer to Data Memory to be written
  * @retval BSP status
  */
int32_t BSP_MEM_ChipErase(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  BSP_MEM_CHECK_PARAMS(Instance);

  if(isBusy)
  {
    ret = BSP_ERROR_BUSY;
  }
  else
  {
    isBusy = (1U << 3);  /* Erase Chip OP is 0x8 */

    if(MemDrv->ChipErase != NULL)
    {
      if(MemDrv->ChipErase(MemCompObj) < 0)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
    }
    else
    {
      ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
    }

    isBusy = 0;
  }

  return ret;
}

/**
  * @brief  Read Data from Serial Periphiral Memory Interface.
  * @param  Instance MEM Instance
  * @param  pData pointer to Data Memory to be filled
  * @param  Address Memory address to read from
  * @param  Size pointer to Data Memory to be filled
  * @retval BSP status
  */
int32_t BSP_MEM_ReadData(uint32_t Instance, uint8_t *pData, uint32_t Address, uint32_t Size)
{
  int32_t ret = BSP_ERROR_NONE;

  BSP_MEM_CHECK_PARAMS(Instance);

  if(isBusy)
  {
    ret = BSP_ERROR_BUSY;
  }
  else
  {
    isBusy = (1U << 0);  /* Read OP is 0x1 */

    if(MemDrv->Read != NULL)
    {
      if(MemDrv->Read(MemCompObj, MEM_INTERFACE_MODE, pData, Address, Size) < 0)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
    }
    else
    {
      ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
    }

    isBusy = 0;
  }

  return ret;
}

/**
  * @brief  Read Data from Serial Periphiral Memory Interface using DMA.
  * @param  Instance MEM Instance
  * @param  pData pointer to Data Memory to be filled
  * @param  Address Memory address to read from
  * @param  Size pointer to Data Memory to be filled
  * @retval BSP status
  */
int32_t BSP_MEM_ReadDataDMA(uint32_t Instance, uint8_t *pData, uint32_t Address, uint32_t Size)
{
  int32_t ret = BSP_ERROR_NONE;

  BSP_MEM_CHECK_PARAMS(Instance);

  if(isBusy)
  {
    ret = BSP_ERROR_BUSY;
  }
  else
  {
    isBusy = (1U << 0);  /* Read OP is 0x1 */

    if(MemDrv->ReadDMA != NULL)
    {
      if(MemDrv->ReadDMA(MemCompObj, MEM_INTERFACE_MODE, pData, Address, Size) < 0)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
    }
    else
    {
      ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
    }
  }

  return ret;
}

/**
  * @brief  Write Data to QuadSPI Memory Interface.
  * @param  Instance MEM Instance
  * @param  pData pointer to Data Memory to be written
  * @param  Address Memory address to write to
  * @param  Size pointer to Data Memory to be written
  * @retval BSP status
  */
int32_t BSP_MEM_WriteData(uint32_t Instance, uint8_t *pData, uint32_t Address, uint32_t Size)
{
  int32_t ret = BSP_ERROR_NONE;

  BSP_MEM_CHECK_PARAMS(Instance);

  if(isBusy)
  {
    ret = BSP_ERROR_BUSY;
  }
  else
  {
    isBusy = (1U << 1);  /* Write OP is 0x2 */

    if(MemDrv->Program != NULL)
    {
      if(MemDrv->Program(MemCompObj, MEM_INTERFACE_MODE, pData, Address, Size) < 0)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
    }
    else
    {
      ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
    }

    isBusy = 0;
  }

  return ret;
}

/**
  * @brief  Write Data to QuadSPI Memory Interface using DMA.
  * @param  Instance MEM Instance
  * @param  pData pointer to Data Memory to be written
  * @param  Address Memory address to write to
  * @param  Size pointer to Data Memory to be written
  * @retval BSP status
  */
int32_t BSP_MEM_WriteDataDMA(uint32_t Instance, uint8_t *pData, uint32_t Address, uint32_t Size)
{
  int32_t ret = BSP_ERROR_NONE;

  BSP_MEM_CHECK_PARAMS(Instance);

  if(isBusy)
  {
    ret = BSP_ERROR_BUSY;
  }
  else
  {
    isBusy = (1U << 1);  /* Write OP is 0x2 */

    if(MemDrv->ProgramDMA != NULL)
    {
      if(MemDrv->ProgramDMA(MemCompObj, MEM_INTERFACE_MODE, pData, Address, Size) < 0)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
    }
    else
    {
      ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
    }
  }

  return ret;
}

/**
  * @brief  Enable Memory Mapped Mode.
  * @param  Instance MEM Instance
  * @retval Zero if no transfert, transfert Operation code otherwise
  */
int32_t BSP_MEM_EnableMemoryMappedMode(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  BSP_MEM_CHECK_PARAMS(Instance);

  if(isBusy)
  {
    ret = BSP_ERROR_BUSY;
  }
  else
  {
    if(MemDrv->EnableMemoryMappedMode != NULL)
    {
      if(MemDrv->EnableMemoryMappedMode(MemCompObj, MEM_INTERFACE_MODE) < 0)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
    }
    else
    {
      ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
    }
  }

  return ret;
}

/**
  * @brief  Get the status of the SPI Transfer.
  * @param  Instance MEM Instance
  * @retval Zero if no transfert, transfert Operation code otherwise
  */
uint8_t BSP_MEM_GetTransfertStatus(uint32_t Instance)
{
  if (Instance < MEM_INSTANCES_NBR)
  {
    return isBusy;
  }

  return 0;
}

/**
  * @brief  Wait for until complete SPI Transfer.
  * @param  Instance MEM Instance
  * @retval None
  */
void BSP_MEM_WaitForTransferToBeDone(uint32_t Instance)
{
  if (Instance < MEM_INSTANCES_NBR)
  {
    while (isBusy);
  }
}

/**
  * @brief  Signal transfert event.
  * @param  Instance MEM Instance
  * @param  Event Transfert ID : '1' read done, '2' 'write done'
  * @retval None
  */
__weak void BSP_MEM_SignalTransfertDone(uint32_t Instance, uint8_t Event)
{
  /* This is the user's Callback to be implemented at the application level */
}

/**
  * @}
  */

/** @defgroup STM32F4XX_NUCLEO_MEM_Private_Functions Private Functions
  * @{
  */
/** @defgroup SPI Driver for external memory interface
  * @{
  */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
  /**
  * @brief  Transfert Read Complete CallBack
  * @param  hspi SPI Handler
  */
static void SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if(hspi == ObjCtx.handle)
  {
    /* Set the nCS */
    MEM_CS_HIGH();

    /* Signal Transfert Done Event */
    BSP_MEM_SignalTransfertDone(0, isBusy);

    /* Reset Transfert state */
    isBusy = 0;
  }
}
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */

/**
  * @brief  Set Burst Read length
  * @param  burstconfig  Burst length
  */
static int32_t MEM_SetBurstReadLength( uint8_t burstconfig )
{
  int32_t ret = BSP_ERROR_NONE;

  uint8_t cmd[2] = { MEM_SET_BURST_LENGTH_CMD
                   , burstconfig
                   };

  // Chip select go low to start a flash command
  MEM_CS_LOW();

  // Send SBL command and config data
  ret = MEM_SPI_Send( cmd, 2 );

  // Chip select go high to end a flash command
  MEM_CS_HIGH();

  return ret;
}

/**
  * @brief  Software Reset
  * @param  None
  */
static int32_t MEM_SoftwareReset( void )
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t cmd;

  MEM_CS_LOW();
  cmd = MX25L6433F_RESET_ENABLE_CMD;
  ret = MEM_SPI_Send( &cmd, 1 );
  MEM_CS_HIGH();

  if(ret == BSP_ERROR_NONE)
  {
    MEM_CS_LOW();
    cmd = MX25L6433F_RESET_MEMORY_CMD;
    ret = MEM_SPI_Send( &cmd, 1 );
    MEM_CS_HIGH();
    HAL_Delay(20);
  }

  return ret;
}

/**
  * @brief  Initializes MEM low level.
  * @retval BSP status
  */
static int32_t MEM_IO_Init(void)
{
  int32_t ret = BSP_ERROR_NONE;

  ret = MEM_SPI_Init();

  if(ret == BSP_ERROR_NONE)
  {
    ret = MEM_SoftwareReset();
  }

  if(ret == BSP_ERROR_NONE)
  {
    /* MEM Initialization */
    MemCompObj = &ObjCtx;
    MemDrv = (MEM_Drv_t *)&MX25L6433F_MEM_Driver;
    ObjCtx.IsInitialized = 0;

    if(MemDrv->Init(MemCompObj) < 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
  }

  if(ret == BSP_ERROR_NONE)
  {
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
    HAL_SPI_RegisterCallback((SPI_HandleTypeDef *)ObjCtx.handle, HAL_SPI_RX_COMPLETE_CB_ID, SPI_RxCpltCallback);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */

    /* Enable Burst Read - 64-bytes */
    ret = MEM_SetBurstReadLength(MEM_WRAP_AROUND_64BYTE);
  }

  return ret;
}

/**
  * @brief  DeInitializes MEM low level
  * @retval BSP status
  */
static int32_t MEM_IO_DeInit(void)
{
  int32_t ret = BSP_ERROR_NONE;

  BSP_MEM_CHECK_PARAMS(Instance);

  ret = MEM_SPI_DeInit();

  return ret;
}

/**
  * @brief  Send data to the selected Memory at specified Address.
  * @param  pData  pointer to data to write to the the specified address memory.
  * @param  Address Memory address to be filled
  * @param  Size length of data to write to the the specified address memory
  * @retval BSP status
  */
static int32_t MEM_IO_SendData(uint8_t *pData, uint32_t Address, uint32_t Size)
{
  return BSP_ERROR_FEATURE_NOT_SUPPORTED;
}

/**
  * @brief  Send data using DMA to the selected Memory at specified Address.
  * @param  pData  pointer to data to write to the the specified address memory.
  * @param  Address Memory address to be filled
  * @param  Size length of data to write to the the specified address memory
  * @retval BSP status
  */
static int32_t MEM_IO_SendDataDMA(uint8_t *pData, uint32_t Address, uint32_t Size)
{
  return BSP_ERROR_FEATURE_NOT_SUPPORTED;
}

/**
  * @brief  Read Data from Serial Periphiral Memory Interface.
  * @param  pData pointer to Data Memory to be filled
  * @param  Address Memory address to read from
  * @param  Size pointer to Data Memory to be filled
  * @retval BSP status
  */
static int32_t MEM_IO_RecvData(uint8_t *pData, uint32_t Address, uint32_t Size)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t cmd[4] = { MEM_READ_CMD
                   , ((Address >> 16) & 0xFF)
                   , ((Address >>  8) & 0xFF)
                   , (Address & 0xFF)
                   };

  /* Reset the nCS pin */
  MEM_CS_LOW();

  ret = MEM_SPI_Send(cmd, 4);
  if(ret == BSP_ERROR_NONE)
  {
    ret = MEM_SPI_Recv(pData, Size);
  }

  /* Set the nCS */
  MEM_CS_HIGH();

  return ret;
}

/**
  * @brief  Read Data from Serial Periphiral Memory Interface using DMA.
  * @param  pData pointer to Data Memory to be filled
  * @param  Address Memory address to read from
  * @param  Size pointer to Data Memory to be filled
  * @retval BSP status
  */
static int32_t MEM_IO_RecvDataDMA(uint8_t *pData, uint32_t Address, uint32_t Size)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t cmd[4] = { MEM_READ_CMD
                   , ((Address >> 16) & 0xFF)
                   , ((Address >>  8) & 0xFF)
                   , (Address & 0xFF)
                   };

  /* Reset the nCS pin */
  MEM_CS_LOW();

  ret = MEM_SPI_Send(cmd, 4);
  if(ret == BSP_ERROR_NONE)
  {
    ret = MEM_SPI_Recv_DMA(pData, Size);
  }

  return ret;
}

/**
  * @brief  Register Bus IOs for instance 0 if MX25L6433F ID is OK
  * @param  Instance    MEM Instance
  * @retval BSP status
  */
static int32_t MEM_Probe(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Configure the lcd driver : map to MEM_IO function*/
  IOCtx.Init              = MEM_IO_Init;
  IOCtx.DeInit            = MEM_IO_DeInit;
  IOCtx.SendData          = MEM_IO_SendData;
  IOCtx.SendDataDMA       = MEM_IO_SendDataDMA;
  IOCtx.RecvData          = MEM_IO_RecvData;
  IOCtx.RecvDataDMA       = MEM_IO_RecvDataDMA;

  /* Set Base Address */
  IOCtx.Address           = MEM_BASE_ADDRESS;

  /* Register OSPI handle */
  ObjCtx.handle           = &hMEMSPI;

  if(MX25L6433F_RegisterBusIO(&ObjCtx, &IOCtx) != MX25L6433F_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }

  return ret;
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

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
