################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/ziyad/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/BlueNRG-2/hci/bluenrg1_events.c \
/Users/ziyad/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/BlueNRG-2/hci/bluenrg1_events_cb.c \
/Users/ziyad/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/BlueNRG-2/hci/bluenrg1_hci_le.c 

OBJS += \
./Middlewares/BlueNRG-2/HCI/bluenrg1_events.o \
./Middlewares/BlueNRG-2/HCI/bluenrg1_events_cb.o \
./Middlewares/BlueNRG-2/HCI/bluenrg1_hci_le.o 

C_DEPS += \
./Middlewares/BlueNRG-2/HCI/bluenrg1_events.d \
./Middlewares/BlueNRG-2/HCI/bluenrg1_events_cb.d \
./Middlewares/BlueNRG-2/HCI/bluenrg1_hci_le.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/BlueNRG-2/HCI/bluenrg1_events.o: /Users/ziyad/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/BlueNRG-2/hci/bluenrg1_events.c Middlewares/BlueNRG-2/HCI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DUSE_STM32F4XX_NUCLEO -D_DEBUG_ -D__FPU_PRESENT=1U -DARM_MATH_CM4 -DUSE_IKS01A3 -c -I../../../Inc -I../../../Patch -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/DSP/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/IKS01A3 -I../../../Drivers/BSP/Components/Common -I../../../Drivers/BSP/Components/lps22hh -I../../../Drivers/BSP/Components/lis2mdl -I../../../Drivers/BSP/Components/lsm6dso -I../../../Middlewares/ST/STM32_MetaDataManager -I../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../Middlewares/Third_Party/parson -I../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../Middlewares/ST/STM32_MotionSP_Library/Inc -I../../../Middlewares/ST/BlueNRG-2/includes -I../../../Drivers/BSP/Components/ism330dlc -I../../../Drivers/BSP/CCA02M2 -I../../../Drivers/BSP/Components/hts221 -I../../../Middlewares/ST/BlueNRG-2/utils -I../../../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/BlueNRG-2/HCI/bluenrg1_events_cb.o: /Users/ziyad/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/BlueNRG-2/hci/bluenrg1_events_cb.c Middlewares/BlueNRG-2/HCI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DUSE_STM32F4XX_NUCLEO -D_DEBUG_ -D__FPU_PRESENT=1U -DARM_MATH_CM4 -DUSE_IKS01A3 -c -I../../../Inc -I../../../Patch -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/DSP/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/IKS01A3 -I../../../Drivers/BSP/Components/Common -I../../../Drivers/BSP/Components/lps22hh -I../../../Drivers/BSP/Components/lis2mdl -I../../../Drivers/BSP/Components/lsm6dso -I../../../Middlewares/ST/STM32_MetaDataManager -I../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../Middlewares/Third_Party/parson -I../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../Middlewares/ST/STM32_MotionSP_Library/Inc -I../../../Middlewares/ST/BlueNRG-2/includes -I../../../Drivers/BSP/Components/ism330dlc -I../../../Drivers/BSP/CCA02M2 -I../../../Drivers/BSP/Components/hts221 -I../../../Middlewares/ST/BlueNRG-2/utils -I../../../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/BlueNRG-2/HCI/bluenrg1_hci_le.o: /Users/ziyad/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/BlueNRG-2/hci/bluenrg1_hci_le.c Middlewares/BlueNRG-2/HCI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DUSE_STM32F4XX_NUCLEO -D_DEBUG_ -D__FPU_PRESENT=1U -DARM_MATH_CM4 -DUSE_IKS01A3 -c -I../../../Inc -I../../../Patch -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/DSP/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/IKS01A3 -I../../../Drivers/BSP/Components/Common -I../../../Drivers/BSP/Components/lps22hh -I../../../Drivers/BSP/Components/lis2mdl -I../../../Drivers/BSP/Components/lsm6dso -I../../../Middlewares/ST/STM32_MetaDataManager -I../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../Middlewares/Third_Party/parson -I../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../Middlewares/ST/STM32_MotionSP_Library/Inc -I../../../Middlewares/ST/BlueNRG-2/includes -I../../../Drivers/BSP/Components/ism330dlc -I../../../Drivers/BSP/CCA02M2 -I../../../Drivers/BSP/Components/hts221 -I../../../Middlewares/ST/BlueNRG-2/utils -I../../../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-BlueNRG-2d-2-2f-HCI

clean-Middlewares-2f-BlueNRG-2d-2-2f-HCI:
	-$(RM) ./Middlewares/BlueNRG-2/HCI/bluenrg1_events.d ./Middlewares/BlueNRG-2/HCI/bluenrg1_events.o ./Middlewares/BlueNRG-2/HCI/bluenrg1_events_cb.d ./Middlewares/BlueNRG-2/HCI/bluenrg1_events_cb.o ./Middlewares/BlueNRG-2/HCI/bluenrg1_hci_le.d ./Middlewares/BlueNRG-2/HCI/bluenrg1_hci_le.o

.PHONY: clean-Middlewares-2f-BlueNRG-2d-2-2f-HCI

