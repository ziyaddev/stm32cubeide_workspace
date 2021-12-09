/**
  ******************************************************************************
  * @file    bfa001Vx_motion_sensors.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 1.0.0
  * @date    12 November 2020
  * @brief   This file provides a set of functions needed to manage
  *          the motion sensors
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

/* Includes ------------------------------------------------------------------*/
#include "bfa001Vx.h"
#include "bfa001Vx_motion_sensors.h"

/** @addtogroup BSP
 * @{
 */

/** @addtogroup STEVAL-BFA001Vx
 * @{
 */

/** @defgroup STEVAL-BFA001Vx_MOTION_SENSOR STEVAL-BFA001Vx MOTION SENSOR
 * @{
 */

/** @defgroup STEVAL-BFA001Vx_MOTION_SENSOR_Exported_Variables STEVAL-BFA001Vx MOTION SENSOR Exported Variables
 * @{
 */

extern SPI_HandleTypeDef hspi1;
extern void *MotionCompObj[MOTION_INSTANCES_NBR]; /* This "redundant" line is here to fulfil MISRA C-2012 rule 8.4 */
void *MotionCompObj[MOTION_INSTANCES_NBR];

/**
 * @}
 */

/** @defgroup STEVAL-BFA001Vx_MOTION_SENSOR_Private_Variables STEVAL-BFA001Vx MOTION SENSOR Private Variables
 * @{
 */

/* We define a jump table in order to get the correct index from the desired function. */
/* This table should have a size equal to the maximum value of a function plus 1.      */
static uint32_t FunctionIndex[5] = {0, 0, 1, 1, 2};
static MOTION_SENSOR_FuncDrv_t *MotionFuncDrv[MOTION_INSTANCES_NBR][MOTION_FUNCTIONS_NBR];
static MOTION_SENSOR_CommonDrv_t *MotionDrv[MOTION_INSTANCES_NBR];
static MOTION_SENSOR_Ctx_t MotionCtx[MOTION_INSTANCES_NBR];

/**
 * @}
 */

/** @defgroup STEVAL-BFA001Vx_MOTION_SENSOR_Private_Function_Prototypes STEVAL-BFA001Vx MOTION SENSOR Private Function Prototypes
 * @{
 */
#if (USE_MOTION_SENSOR_IIS3DWB_0 == 1)
static int32_t IIS3DWB_0_Probe(uint32_t Functions);
static int32_t BSP_IIS3DWB_Init(void);
static int32_t BSP_IIS3DWB_DeInit(void);
static int32_t BSP_IIS3DWB_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len);
static int32_t BSP_IIS3DWB_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len);
#endif

#if (USE_MOTION_SENSOR_ISM330DLC_0 == 1)
static int32_t ISM330DLC_0_Probe(uint32_t Functions);
static int32_t BSP_ISM330DLC_Init(void);
static int32_t BSP_ISM330DLC_DeInit(void);
static int32_t BSP_ISM330DLC_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len);
static int32_t BSP_ISM330DLC_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len);
#endif

/**
 * @}
 */

/** @defgroup STEVAL-BFA001Vx_MOTION_SENSOR_Exported_Functions STEVAL-BFA001Vx MOTION SENSOR Exported Functions
 * @{
 */

/**
  * @brief  Initializes the motion sensors
  * @param  Instance Motion sensor instance
  * @param  Functions MEMS function
  * @retval BSP status
  */
int32_t BSP_MOTION_SENSOR_Init(uint32_t Instance, uint32_t Functions)
{
  int32_t ret = BSP_ERROR_NONE;
  uint32_t function = MOTION_GYRO;
  uint32_t i;
  uint32_t component_functions = 0;
  MOTION_SENSOR_Capabilities_t cap;

  switch (Instance)
  {
#if (USE_MOTION_SENSOR_IIS3DWB_0 == 1)
    case BFA001Vx_IIS3DWB_0:
      if (IIS3DWB_0_Probe(Functions) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_NO_INIT;
      }
      if (MotionDrv[Instance]->GetCapabilities(MotionCompObj[Instance], (void *)&cap) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_UNKNOWN_COMPONENT;
      }
      if (cap.Acc == 1U)
      {
        component_functions |= MOTION_ACCELERO;
      }
      break;
#endif

#if (USE_MOTION_SENSOR_ISM330DLC_0 == 1)
    case BFA001Vx_ISM330DLC_0:
      if (ISM330DLC_0_Probe(Functions) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_NO_INIT;
      }
      if (MotionDrv[Instance]->GetCapabilities(MotionCompObj[Instance], (void *)&cap) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_UNKNOWN_COMPONENT;
      }
      if (cap.Acc == 1U)
      {
        component_functions |= MOTION_ACCELERO;
      }
      if (cap.Gyro == 1U)
      {
        component_functions |= MOTION_GYRO;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }

  for (i = 0; i < MOTION_FUNCTIONS_NBR; i++)
  {
    if (((Functions & function) == function) && ((component_functions & function) == function))
    {
      if (MotionFuncDrv[Instance][FunctionIndex[function]]->Enable(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_COMPONENT_FAILURE;
      }
    }
    function = function << 1;
  }

  return ret;
}

/**
 * @brief  Deinitialize Motion sensor
 * @param  Instance Motion sensor instance
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_DeInit(uint32_t Instance)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (MotionDrv[Instance]->DeInit(MotionCompObj[Instance]) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
 * @brief  Get motion sensor instance capabilities
 * @param  Instance Motion sensor instance
 * @param  Capabilities pointer to motion sensor capabilities
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_GetCapabilities(uint32_t Instance, MOTION_SENSOR_Capabilities_t *Capabilities)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (MotionDrv[Instance]->GetCapabilities(MotionCompObj[Instance], Capabilities) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
 * @brief  Get WHOAMI value
 * @param  Instance Motion sensor instance
 * @param  Id WHOAMI value
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_ReadID(uint32_t Instance, uint8_t *Id)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (MotionDrv[Instance]->ReadID(MotionCompObj[Instance], Id) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
 * @brief  Enable Motion sensor
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_Enable(uint32_t Instance, uint32_t Function)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->Enable(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Disable Motion sensor
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_Disable(uint32_t Instance, uint32_t Function)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->Disable(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Get accelero axes data
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 * @param  Axes pointer to axes data structure
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_GetAxes(uint32_t Instance, uint32_t Function, BSP_MOTION_SENSOR_Axes_t *Axes)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->GetAxes(MotionCompObj[Instance], Axes) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Get accelero axes raw data
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 * @param  Axes pointer to axes raw data structure
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_GetAxesRaw(uint32_t Instance, uint32_t Function, BSP_MOTION_SENSOR_AxesRaw_t *Axes)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->GetAxesRaw(MotionCompObj[Instance], Axes) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Get accelero sensitivity
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 * @param  Sensitivity pointer to sensitivity read value
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_GetSensitivity(uint32_t Instance, uint32_t Function, float *Sensitivity)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->GetSensitivity(MotionCompObj[Instance],
          Sensitivity) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Get accelero Output Data Rate
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 * @param  Odr pointer to Output Data Rate read value
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_GetOutputDataRate(uint32_t Instance, uint32_t Function, float *Odr)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->GetOutputDataRate(MotionCompObj[Instance], Odr) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Get accelero Full Scale
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 * @param  Fullscale pointer to Fullscale read value
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_GetFullScale(uint32_t Instance, uint32_t Function, int32_t *Fullscale)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->GetFullScale(MotionCompObj[Instance], Fullscale) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Set accelero Output Data Rate
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 * @param  Odr Output Data Rate value to be set
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_SetOutputDataRate(uint32_t Instance, uint32_t Function, float Odr)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->SetOutputDataRate(MotionCompObj[Instance], Odr) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Set accelero Full Scale
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 * @param  Fullscale Fullscale value to be set
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_SetFullScale(uint32_t Instance, uint32_t Function, int32_t Fullscale)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->SetFullScale(MotionCompObj[Instance], Fullscale) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

#if (USE_MOTION_SENSOR_IIS3DWB_0  == 1)
/**
  * @brief  Register Bus IOs for instance 0 if component ID is OK
  * @param  Functions MEMS function
  * @retval BSP status
  */
static int32_t IIS3DWB_0_Probe(uint32_t Functions)
{
  IIS3DWB_IO_t             io_ctx;
  uint8_t                  id;
  static IIS3DWB_Object_t  iis3dwb_obj_0;
  IIS3DWB_Capabilities_t   cap;
  int32_t ret = BSP_ERROR_NONE;
  
  /* Configure the accelero driver */
  io_ctx.BusType     = IIS3DWB_SPI_4WIRES_BUS; /* SPI 4-Wires */
  io_ctx.Address     = 0x0;
  io_ctx.Init        = BSP_IIS3DWB_Init;
  io_ctx.DeInit      = BSP_IIS3DWB_DeInit;
  io_ctx.ReadReg     = BSP_IIS3DWB_ReadReg;
  io_ctx.WriteReg    = BSP_IIS3DWB_WriteReg;
  io_ctx.GetTick     = BSP_GetTick;
  
  if (IIS3DWB_RegisterBusIO(&iis3dwb_obj_0, &io_ctx) != IIS3DWB_OK)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else if (IIS3DWB_ReadID(&iis3dwb_obj_0, &id) != IIS3DWB_OK)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else if (id != IIS3DWB_ID)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else
  {
    (void)IIS3DWB_GetCapabilities(&iis3dwb_obj_0, &cap);
    MotionCtx[BFA001Vx_IIS3DWB_0].Functions = ((uint32_t)cap.Gyro) | ((uint32_t)cap.Acc << 1) | ((uint32_t)cap.Magneto << 2);
    
    MotionCompObj[BFA001Vx_IIS3DWB_0] = &iis3dwb_obj_0;
    /* The second cast (void *) is added to bypass Misra R11.3 rule */
    MotionDrv[BFA001Vx_IIS3DWB_0] = (MOTION_SENSOR_CommonDrv_t *)(void *)&IIS3DWB_COMMON_Driver;
    
    if (((Functions & MOTION_ACCELERO) == MOTION_ACCELERO) && (cap.Acc == 1U))
    {
      /* The second cast (void *) is added to bypass Misra R11.3 rule */
      MotionFuncDrv[BFA001Vx_IIS3DWB_0][FunctionIndex[MOTION_ACCELERO]] = (MOTION_SENSOR_FuncDrv_t *)(void *)&IIS3DWB_ACC_Driver;
      
      if (MotionDrv[BFA001Vx_IIS3DWB_0]->Init(MotionCompObj[BFA001Vx_IIS3DWB_0]) != IIS3DWB_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
  }
  return ret;
}

static int32_t BSP_IIS3DWB_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;
  
  /* Configure IIS3DWB INT1 pin */
  IIS3DWB_INT1_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = IIS3DWB_INT1_PORT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(IIS3DWB_INT1_PORT, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(IIS3DWB_INT1_EXTI_IRQn, IIS3DWB_INT1_EXTI_IRQ_PP, IIS3DWB_INT1_EXTI_IRQ_SP);
  HAL_NVIC_EnableIRQ(IIS3DWB_INT1_EXTI_IRQn);

  /* Configure IIS3DWB INT2 pin */
  IIS3DWB_INT2_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = IIS3DWB_INT2_PORT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(IIS3DWB_INT2_PORT, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(IIS3DWB_INT2_EXTI_IRQn, IIS3DWB_INT2_EXTI_IRQ_PP, IIS3DWB_INT2_EXTI_IRQ_SP);
  HAL_NVIC_EnableIRQ(IIS3DWB_INT2_EXTI_IRQn);

  /* Configure IIS3DWB CS pin */
  HAL_GPIO_WritePin(IIS3DWB_CS_PORT, IIS3DWB_CS_PIN, GPIO_PIN_SET);
  
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  
  IIS3DWB_CS_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = IIS3DWB_CS_PIN;
  HAL_GPIO_Init(IIS3DWB_CS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(IIS3DWB_CS_PORT, IIS3DWB_CS_PIN, GPIO_PIN_SET);

  /* Configure SPI */
  if(BSP_SPI1_Init() == BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

static int32_t BSP_IIS3DWB_DeInit(void)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;

  if(BSP_SPI1_DeInit() == BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

static int32_t BSP_IIS3DWB_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;

  /* CS Enable */
  HAL_GPIO_WritePin(IIS3DWB_CS_PORT, IIS3DWB_CS_PIN, GPIO_PIN_RESET);

  if (BSP_SPI1_Send(&dataReg, 1) != 1)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  if (BSP_SPI1_Send(pdata, len) != len)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  /* CS Disable */
  HAL_GPIO_WritePin(IIS3DWB_CS_PORT, IIS3DWB_CS_PIN, GPIO_PIN_SET);

  return ret;
}

static int32_t BSP_IIS3DWB_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;

  dataReg |= 0x80;

  /* CS Enable */
  HAL_GPIO_WritePin(IIS3DWB_CS_PORT, IIS3DWB_CS_PIN, GPIO_PIN_RESET);

  if (BSP_SPI1_Send(&dataReg, 1) != 1)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  if (BSP_SPI1_Recv(pdata, len) != len)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  /* CS Disable */
  HAL_GPIO_WritePin(IIS3DWB_CS_PORT, IIS3DWB_CS_PIN, GPIO_PIN_SET);

  return ret;
}
#endif

#if (USE_MOTION_SENSOR_ISM330DLC_0  == 1)
/**
  * @brief  Register Bus IOs for instance 0 if component ID is OK
  * @param  Functions MEMS function
  * @retval BSP status
  */
static int32_t ISM330DLC_0_Probe(uint32_t Functions)
{

  ISM330DLC_IO_t                io_ctx;
  uint8_t                       id;
  static ISM330DLC_Object_t     ism330dlc_obj_0;
  ISM330DLC_Capabilities_t      cap;
  int32_t ret = BSP_ERROR_NONE;

  /* Configure the accelero driver */
  io_ctx.BusType     = ISM330DLC_SPI_4WIRES_BUS;  /* SPI 4-Wires */
  io_ctx.Address     = 0x0;
  io_ctx.Init        = BSP_ISM330DLC_Init;
  io_ctx.DeInit      = BSP_ISM330DLC_DeInit;
  io_ctx.ReadReg     = BSP_ISM330DLC_ReadReg;
  io_ctx.WriteReg    = BSP_ISM330DLC_WriteReg;
  io_ctx.GetTick     = BSP_GetTick;

  if (ISM330DLC_RegisterBusIO(&ism330dlc_obj_0, &io_ctx) != ISM330DLC_OK)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else if (ISM330DLC_ReadID(&ism330dlc_obj_0, &id) != ISM330DLC_OK)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else if (id != ISM330DLC_ID)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else
  {
    (void)ISM330DLC_GetCapabilities(&ism330dlc_obj_0, &cap);
    MotionCtx[BFA001Vx_ISM330DLC_0].Functions = ((uint32_t)cap.Gyro) | ((uint32_t)cap.Acc << 1);

    MotionCompObj[BFA001Vx_ISM330DLC_0] = &ism330dlc_obj_0;
    /* The second cast (void *) is added to bypass Misra R11.3 rule */
    MotionDrv[BFA001Vx_ISM330DLC_0] = (MOTION_SENSOR_CommonDrv_t *)(void *)&ISM330DLC_COMMON_Driver;

    if (((Functions & MOTION_GYRO) == MOTION_GYRO) && (cap.Gyro == 1U))
    {
      /* The second cast (void *) is added to bypass Misra R11.3 rule */
      MotionFuncDrv[BFA001Vx_ISM330DLC_0][FunctionIndex[MOTION_GYRO]] = (MOTION_SENSOR_FuncDrv_t *)(void *)&ISM330DLC_GYRO_Driver;

      if (MotionDrv[BFA001Vx_ISM330DLC_0]->Init(MotionCompObj[BFA001Vx_ISM330DLC_0]) != ISM330DLC_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    if (((Functions & MOTION_ACCELERO) == MOTION_ACCELERO) && (cap.Acc == 1U))
    {
      /* The second cast (void *) is added to bypass Misra R11.3 rule */
      MotionFuncDrv[BFA001Vx_ISM330DLC_0][FunctionIndex[MOTION_ACCELERO]] = (MOTION_SENSOR_FuncDrv_t *)(void *)&ISM330DLC_ACC_Driver;

      if (MotionDrv[BFA001Vx_ISM330DLC_0]->Init(MotionCompObj[BFA001Vx_ISM330DLC_0]) != ISM330DLC_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
  }
  return ret;
}

/**
 * @brief  Initialize SPI bus for ISM330DLC
 * @retval BSP status
 */
static int32_t BSP_ISM330DLC_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;
  
  /* Configure ISM330DLC INT1 pin */
  ISM330DLC_INT1_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = ISM330DLC_INT1_PORT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ISM330DLC_INT1_PORT, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(ISM330DLC_INT1_EXTI_IRQn, ISM330DLC_INT1_EXTI_IRQ_PP, ISM330DLC_INT1_EXTI_IRQ_SP);
  HAL_NVIC_EnableIRQ(ISM330DLC_INT1_EXTI_IRQn);

  /* Configure IIS3DWB INT2 pin */
  ISM330DLC_INT2_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = ISM330DLC_INT2_PORT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ISM330DLC_INT2_PORT, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(ISM330DLC_INT2_EXTI_IRQn, ISM330DLC_INT2_EXTI_IRQ_PP, ISM330DLC_INT2_EXTI_IRQ_SP);
  HAL_NVIC_EnableIRQ(ISM330DLC_INT2_EXTI_IRQn);

  /* Configure IIS3DWB CS pin */
  HAL_GPIO_WritePin(ISM330DLC_CS_PORT, ISM330DLC_CS_PIN, GPIO_PIN_SET);
  
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  
  ISM330DLC_CS_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = ISM330DLC_CS_PIN;
  HAL_GPIO_Init(ISM330DLC_CS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(ISM330DLC_CS_PORT, ISM330DLC_CS_PIN, GPIO_PIN_SET);

  /* Configure SPI */
  if(BSP_SPI1_Init() == BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
 * @brief  DeInitialize SPI bus for ISM330DLC
 * @retval BSP status
 */
static int32_t BSP_ISM330DLC_DeInit(void)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;

  if(BSP_SPI1_DeInit() == BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
 * @brief  Write register by SPI bus for ISM330DLC
 * @param  Addr not used, it is only for BSP compatibility
 * @param  Reg the starting register address to be written
 * @param  pdata the pointer to the data to be written
 * @param  len the length of the data to be written
 * @retval BSP status
 */
int32_t BSP_ISM330DLC_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;

  /* CS Enable */
  HAL_GPIO_WritePin(ISM330DLC_CS_PORT, ISM330DLC_CS_PIN, GPIO_PIN_RESET);

  if (BSP_SPI1_Send(&dataReg, 1) != 1)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  if (BSP_SPI1_Send(pdata, len) != len)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  /* CS Disable */
  HAL_GPIO_WritePin(ISM330DLC_CS_PORT, ISM330DLC_CS_PIN, GPIO_PIN_SET);

  return ret;
}

/**
 * @brief  Read register by SPI bus for ISM330DLC
 * @param  Addr not used, it is only for BSP compatibility
 * @param  Reg the starting register address to be read
 * @param  pdata the pointer to the data to be read
 * @param  len the length of the data to be read
 * @retval BSP status
 */
int32_t BSP_ISM330DLC_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;
  
  dataReg |= 0x80;
  
  /* CS Enable */
  HAL_GPIO_WritePin(ISM330DLC_CS_PORT, ISM330DLC_CS_PIN, GPIO_PIN_RESET);
  
  if (BSP_SPI1_Send(&dataReg, 1) != 1)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  if (BSP_SPI1_Recv(pdata, len) != len)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }
  
  /* CS Disable */
  HAL_GPIO_WritePin(ISM330DLC_CS_PORT, ISM330DLC_CS_PIN, GPIO_PIN_SET);
  
  return ret;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
