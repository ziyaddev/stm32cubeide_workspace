################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/BlueNRG-2/hci/controller/bluenrg1_gap_aci.c \
C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/BlueNRG-2/hci/controller/bluenrg1_gatt_aci.c \
C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/BlueNRG-2/hci/controller/bluenrg1_hal_aci.c \
C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/BlueNRG-2/hci/controller/bluenrg1_l2cap_aci.c 

OBJS += \
./Middlewares/BlueNRG-2/HCI/controller/bluenrg1_gap_aci.o \
./Middlewares/BlueNRG-2/HCI/controller/bluenrg1_gatt_aci.o \
./Middlewares/BlueNRG-2/HCI/controller/bluenrg1_hal_aci.o \
./Middlewares/BlueNRG-2/HCI/controller/bluenrg1_l2cap_aci.o 

C_DEPS += \
./Middlewares/BlueNRG-2/HCI/controller/bluenrg1_gap_aci.d \
./Middlewares/BlueNRG-2/HCI/controller/bluenrg1_gatt_aci.d \
./Middlewares/BlueNRG-2/HCI/controller/bluenrg1_hal_aci.d \
./Middlewares/BlueNRG-2/HCI/controller/bluenrg1_l2cap_aci.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/BlueNRG-2/HCI/controller/bluenrg1_gap_aci.o: C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/BlueNRG-2/hci/controller/bluenrg1_gap_aci.c Middlewares/BlueNRG-2/HCI/controller/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DUSE_STM32F4XX_NUCLEO -D_DEBUG_ -D__FPU_PRESENT=1U -DARM_MATH_CM4 -DUSE_IKS01A3 -c -I../../../Inc -I../../../Patch -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/DSP/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/IKS01A3 -I../../../Drivers/BSP/Components/Common -I../../../Drivers/BSP/Components/lps22hh -I../../../Drivers/BSP/Components/lis2mdl -I../../../Drivers/BSP/Components/lsm6dso -I../../../Middlewares/ST/STM32_MetaDataManager -I../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../Middlewares/Third_Party/parson -I../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../Middlewares/ST/STM32_MotionSP_Library/Inc -I../../../Middlewares/ST/BlueNRG-2/includes -I../../../Drivers/BSP/Components/ism330dlc -I../../../Drivers/BSP/CCA02M2 -I../../../Drivers/BSP/Components/hts221 -I../../../Middlewares/ST/BlueNRG-2/utils -I../../../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/BlueNRG-2/HCI/controller/bluenrg1_gatt_aci.o: C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/BlueNRG-2/hci/controller/bluenrg1_gatt_aci.c Middlewares/BlueNRG-2/HCI/controller/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DUSE_STM32F4XX_NUCLEO -D_DEBUG_ -D__FPU_PRESENT=1U -DARM_MATH_CM4 -DUSE_IKS01A3 -c -I../../../Inc -I../../../Patch -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/DSP/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/IKS01A3 -I../../../Drivers/BSP/Components/Common -I../../../Drivers/BSP/Components/lps22hh -I../../../Drivers/BSP/Components/lis2mdl -I../../../Drivers/BSP/Components/lsm6dso -I../../../Middlewares/ST/STM32_MetaDataManager -I../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../Middlewares/Third_Party/parson -I../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../Middlewares/ST/STM32_MotionSP_Library/Inc -I../../../Middlewares/ST/BlueNRG-2/includes -I../../../Drivers/BSP/Components/ism330dlc -I../../../Drivers/BSP/CCA02M2 -I../../../Drivers/BSP/Components/hts221 -I../../../Middlewares/ST/BlueNRG-2/utils -I../../../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/BlueNRG-2/HCI/controller/bluenrg1_hal_aci.o: C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/BlueNRG-2/hci/controller/bluenrg1_hal_aci.c Middlewares/BlueNRG-2/HCI/controller/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DUSE_STM32F4XX_NUCLEO -D_DEBUG_ -D__FPU_PRESENT=1U -DARM_MATH_CM4 -DUSE_IKS01A3 -c -I../../../Inc -I../../../Patch -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/DSP/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/IKS01A3 -I../../../Drivers/BSP/Components/Common -I../../../Drivers/BSP/Components/lps22hh -I../../../Drivers/BSP/Components/lis2mdl -I../../../Drivers/BSP/Components/lsm6dso -I../../../Middlewares/ST/STM32_MetaDataManager -I../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../Middlewares/Third_Party/parson -I../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../Middlewares/ST/STM32_MotionSP_Library/Inc -I../../../Middlewares/ST/BlueNRG-2/includes -I../../../Drivers/BSP/Components/ism330dlc -I../../../Drivers/BSP/CCA02M2 -I../../../Drivers/BSP/Components/hts221 -I../../../Middlewares/ST/BlueNRG-2/utils -I../../../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/BlueNRG-2/HCI/controller/bluenrg1_l2cap_aci.o: C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/BlueNRG-2/hci/controller/bluenrg1_l2cap_aci.c Middlewares/BlueNRG-2/HCI/controller/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DUSE_STM32F4XX_NUCLEO -D_DEBUG_ -D__FPU_PRESENT=1U -DARM_MATH_CM4 -DUSE_IKS01A3 -c -I../../../Inc -I../../../Patch -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/DSP/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/IKS01A3 -I../../../Drivers/BSP/Components/Common -I../../../Drivers/BSP/Components/lps22hh -I../../../Drivers/BSP/Components/lis2mdl -I../../../Drivers/BSP/Components/lsm6dso -I../../../Middlewares/ST/STM32_MetaDataManager -I../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../Middlewares/Third_Party/parson -I../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../Middlewares/ST/STM32_MotionSP_Library/Inc -I../../../Middlewares/ST/BlueNRG-2/includes -I../../../Drivers/BSP/Components/ism330dlc -I../../../Drivers/BSP/CCA02M2 -I../../../Drivers/BSP/Components/hts221 -I../../../Middlewares/ST/BlueNRG-2/utils -I../../../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-BlueNRG-2d-2-2f-HCI-2f-controller

clean-Middlewares-2f-BlueNRG-2d-2-2f-HCI-2f-controller:
	-$(RM) ./Middlewares/BlueNRG-2/HCI/controller/bluenrg1_gap_aci.d ./Middlewares/BlueNRG-2/HCI/controller/bluenrg1_gap_aci.o ./Middlewares/BlueNRG-2/HCI/controller/bluenrg1_gatt_aci.d ./Middlewares/BlueNRG-2/HCI/controller/bluenrg1_gatt_aci.o ./Middlewares/BlueNRG-2/HCI/controller/bluenrg1_hal_aci.d ./Middlewares/BlueNRG-2/HCI/controller/bluenrg1_hal_aci.o ./Middlewares/BlueNRG-2/HCI/controller/bluenrg1_l2cap_aci.d ./Middlewares/BlueNRG-2/HCI/controller/bluenrg1_l2cap_aci.o

.PHONY: clean-Middlewares-2f-BlueNRG-2d-2-2f-HCI-2f-controller

