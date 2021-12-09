################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/STM32_BLE_Manager/Src/BLE_AudioLevel.c \
C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/STM32_BLE_Manager/Src/BLE_Environmental.c \
C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/STM32_BLE_Manager/Src/BLE_FFT_AlarmAccPeakStatus.c \
C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/STM32_BLE_Manager/Src/BLE_FFT_AlarmSpeedStatus.c \
C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/STM32_BLE_Manager/Src/BLE_FFT_AlarmSubrangeStatus.c \
C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/STM32_BLE_Manager/Src/BLE_FFT_Amplitude.c \
C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/STM32_BLE_Manager/Src/BLE_Inertial.c \
C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/STM32_BLE_Manager/Src/BLE_Manager.c \
C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/STM32_BLE_Manager/Src/BLE_TimeDomain.c 

OBJS += \
./Middlewares/STM32_BLE_Manager/BLE_AudioLevel.o \
./Middlewares/STM32_BLE_Manager/BLE_Environmental.o \
./Middlewares/STM32_BLE_Manager/BLE_FFT_AlarmAccPeakStatus.o \
./Middlewares/STM32_BLE_Manager/BLE_FFT_AlarmSpeedStatus.o \
./Middlewares/STM32_BLE_Manager/BLE_FFT_AlarmSubrangeStatus.o \
./Middlewares/STM32_BLE_Manager/BLE_FFT_Amplitude.o \
./Middlewares/STM32_BLE_Manager/BLE_Inertial.o \
./Middlewares/STM32_BLE_Manager/BLE_Manager.o \
./Middlewares/STM32_BLE_Manager/BLE_TimeDomain.o 

C_DEPS += \
./Middlewares/STM32_BLE_Manager/BLE_AudioLevel.d \
./Middlewares/STM32_BLE_Manager/BLE_Environmental.d \
./Middlewares/STM32_BLE_Manager/BLE_FFT_AlarmAccPeakStatus.d \
./Middlewares/STM32_BLE_Manager/BLE_FFT_AlarmSpeedStatus.d \
./Middlewares/STM32_BLE_Manager/BLE_FFT_AlarmSubrangeStatus.d \
./Middlewares/STM32_BLE_Manager/BLE_FFT_Amplitude.d \
./Middlewares/STM32_BLE_Manager/BLE_Inertial.d \
./Middlewares/STM32_BLE_Manager/BLE_Manager.d \
./Middlewares/STM32_BLE_Manager/BLE_TimeDomain.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/STM32_BLE_Manager/BLE_AudioLevel.o: C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/STM32_BLE_Manager/Src/BLE_AudioLevel.c Middlewares/STM32_BLE_Manager/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DUSE_STM32F4XX_NUCLEO -D_DEBUG_ -D__FPU_PRESENT=1U -DARM_MATH_CM4 -DUSE_IKS01A3 -c -I../../../Inc -I../../../Patch -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/DSP/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/IKS01A3 -I../../../Drivers/BSP/Components/Common -I../../../Drivers/BSP/Components/lps22hh -I../../../Drivers/BSP/Components/lis2mdl -I../../../Drivers/BSP/Components/lsm6dso -I../../../Middlewares/ST/STM32_MetaDataManager -I../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../Middlewares/Third_Party/parson -I../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../Middlewares/ST/STM32_MotionSP_Library/Inc -I../../../Middlewares/ST/BlueNRG-2/includes -I../../../Drivers/BSP/Components/ism330dlc -I../../../Drivers/BSP/CCA02M2 -I../../../Drivers/BSP/Components/hts221 -I../../../Middlewares/ST/BlueNRG-2/utils -I../../../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/STM32_BLE_Manager/BLE_Environmental.o: C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/STM32_BLE_Manager/Src/BLE_Environmental.c Middlewares/STM32_BLE_Manager/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DUSE_STM32F4XX_NUCLEO -D_DEBUG_ -D__FPU_PRESENT=1U -DARM_MATH_CM4 -DUSE_IKS01A3 -c -I../../../Inc -I../../../Patch -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/DSP/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/IKS01A3 -I../../../Drivers/BSP/Components/Common -I../../../Drivers/BSP/Components/lps22hh -I../../../Drivers/BSP/Components/lis2mdl -I../../../Drivers/BSP/Components/lsm6dso -I../../../Middlewares/ST/STM32_MetaDataManager -I../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../Middlewares/Third_Party/parson -I../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../Middlewares/ST/STM32_MotionSP_Library/Inc -I../../../Middlewares/ST/BlueNRG-2/includes -I../../../Drivers/BSP/Components/ism330dlc -I../../../Drivers/BSP/CCA02M2 -I../../../Drivers/BSP/Components/hts221 -I../../../Middlewares/ST/BlueNRG-2/utils -I../../../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/STM32_BLE_Manager/BLE_FFT_AlarmAccPeakStatus.o: C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/STM32_BLE_Manager/Src/BLE_FFT_AlarmAccPeakStatus.c Middlewares/STM32_BLE_Manager/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DUSE_STM32F4XX_NUCLEO -D_DEBUG_ -D__FPU_PRESENT=1U -DARM_MATH_CM4 -DUSE_IKS01A3 -c -I../../../Inc -I../../../Patch -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/DSP/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/IKS01A3 -I../../../Drivers/BSP/Components/Common -I../../../Drivers/BSP/Components/lps22hh -I../../../Drivers/BSP/Components/lis2mdl -I../../../Drivers/BSP/Components/lsm6dso -I../../../Middlewares/ST/STM32_MetaDataManager -I../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../Middlewares/Third_Party/parson -I../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../Middlewares/ST/STM32_MotionSP_Library/Inc -I../../../Middlewares/ST/BlueNRG-2/includes -I../../../Drivers/BSP/Components/ism330dlc -I../../../Drivers/BSP/CCA02M2 -I../../../Drivers/BSP/Components/hts221 -I../../../Middlewares/ST/BlueNRG-2/utils -I../../../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/STM32_BLE_Manager/BLE_FFT_AlarmSpeedStatus.o: C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/STM32_BLE_Manager/Src/BLE_FFT_AlarmSpeedStatus.c Middlewares/STM32_BLE_Manager/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DUSE_STM32F4XX_NUCLEO -D_DEBUG_ -D__FPU_PRESENT=1U -DARM_MATH_CM4 -DUSE_IKS01A3 -c -I../../../Inc -I../../../Patch -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/DSP/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/IKS01A3 -I../../../Drivers/BSP/Components/Common -I../../../Drivers/BSP/Components/lps22hh -I../../../Drivers/BSP/Components/lis2mdl -I../../../Drivers/BSP/Components/lsm6dso -I../../../Middlewares/ST/STM32_MetaDataManager -I../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../Middlewares/Third_Party/parson -I../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../Middlewares/ST/STM32_MotionSP_Library/Inc -I../../../Middlewares/ST/BlueNRG-2/includes -I../../../Drivers/BSP/Components/ism330dlc -I../../../Drivers/BSP/CCA02M2 -I../../../Drivers/BSP/Components/hts221 -I../../../Middlewares/ST/BlueNRG-2/utils -I../../../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/STM32_BLE_Manager/BLE_FFT_AlarmSubrangeStatus.o: C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/STM32_BLE_Manager/Src/BLE_FFT_AlarmSubrangeStatus.c Middlewares/STM32_BLE_Manager/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DUSE_STM32F4XX_NUCLEO -D_DEBUG_ -D__FPU_PRESENT=1U -DARM_MATH_CM4 -DUSE_IKS01A3 -c -I../../../Inc -I../../../Patch -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/DSP/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/IKS01A3 -I../../../Drivers/BSP/Components/Common -I../../../Drivers/BSP/Components/lps22hh -I../../../Drivers/BSP/Components/lis2mdl -I../../../Drivers/BSP/Components/lsm6dso -I../../../Middlewares/ST/STM32_MetaDataManager -I../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../Middlewares/Third_Party/parson -I../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../Middlewares/ST/STM32_MotionSP_Library/Inc -I../../../Middlewares/ST/BlueNRG-2/includes -I../../../Drivers/BSP/Components/ism330dlc -I../../../Drivers/BSP/CCA02M2 -I../../../Drivers/BSP/Components/hts221 -I../../../Middlewares/ST/BlueNRG-2/utils -I../../../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/STM32_BLE_Manager/BLE_FFT_Amplitude.o: C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/STM32_BLE_Manager/Src/BLE_FFT_Amplitude.c Middlewares/STM32_BLE_Manager/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DUSE_STM32F4XX_NUCLEO -D_DEBUG_ -D__FPU_PRESENT=1U -DARM_MATH_CM4 -DUSE_IKS01A3 -c -I../../../Inc -I../../../Patch -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/DSP/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/IKS01A3 -I../../../Drivers/BSP/Components/Common -I../../../Drivers/BSP/Components/lps22hh -I../../../Drivers/BSP/Components/lis2mdl -I../../../Drivers/BSP/Components/lsm6dso -I../../../Middlewares/ST/STM32_MetaDataManager -I../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../Middlewares/Third_Party/parson -I../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../Middlewares/ST/STM32_MotionSP_Library/Inc -I../../../Middlewares/ST/BlueNRG-2/includes -I../../../Drivers/BSP/Components/ism330dlc -I../../../Drivers/BSP/CCA02M2 -I../../../Drivers/BSP/Components/hts221 -I../../../Middlewares/ST/BlueNRG-2/utils -I../../../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/STM32_BLE_Manager/BLE_Inertial.o: C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/STM32_BLE_Manager/Src/BLE_Inertial.c Middlewares/STM32_BLE_Manager/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DUSE_STM32F4XX_NUCLEO -D_DEBUG_ -D__FPU_PRESENT=1U -DARM_MATH_CM4 -DUSE_IKS01A3 -c -I../../../Inc -I../../../Patch -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/DSP/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/IKS01A3 -I../../../Drivers/BSP/Components/Common -I../../../Drivers/BSP/Components/lps22hh -I../../../Drivers/BSP/Components/lis2mdl -I../../../Drivers/BSP/Components/lsm6dso -I../../../Middlewares/ST/STM32_MetaDataManager -I../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../Middlewares/Third_Party/parson -I../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../Middlewares/ST/STM32_MotionSP_Library/Inc -I../../../Middlewares/ST/BlueNRG-2/includes -I../../../Drivers/BSP/Components/ism330dlc -I../../../Drivers/BSP/CCA02M2 -I../../../Drivers/BSP/Components/hts221 -I../../../Middlewares/ST/BlueNRG-2/utils -I../../../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/STM32_BLE_Manager/BLE_Manager.o: C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/STM32_BLE_Manager/Src/BLE_Manager.c Middlewares/STM32_BLE_Manager/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DUSE_STM32F4XX_NUCLEO -D_DEBUG_ -D__FPU_PRESENT=1U -DARM_MATH_CM4 -DUSE_IKS01A3 -c -I../../../Inc -I../../../Patch -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/DSP/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/IKS01A3 -I../../../Drivers/BSP/Components/Common -I../../../Drivers/BSP/Components/lps22hh -I../../../Drivers/BSP/Components/lis2mdl -I../../../Drivers/BSP/Components/lsm6dso -I../../../Middlewares/ST/STM32_MetaDataManager -I../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../Middlewares/Third_Party/parson -I../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../Middlewares/ST/STM32_MotionSP_Library/Inc -I../../../Middlewares/ST/BlueNRG-2/includes -I../../../Drivers/BSP/Components/ism330dlc -I../../../Drivers/BSP/CCA02M2 -I../../../Drivers/BSP/Components/hts221 -I../../../Middlewares/ST/BlueNRG-2/utils -I../../../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/STM32_BLE_Manager/BLE_TimeDomain.o: C:/Users/akchoucz/STM32CubeIDE/workspace_1.7.0/Predictive_Maintenance/Middlewares/ST/STM32_BLE_Manager/Src/BLE_TimeDomain.c Middlewares/STM32_BLE_Manager/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DUSE_STM32F4XX_NUCLEO -D_DEBUG_ -D__FPU_PRESENT=1U -DARM_MATH_CM4 -DUSE_IKS01A3 -c -I../../../Inc -I../../../Patch -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/DSP/Include -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/BSP/IKS01A3 -I../../../Drivers/BSP/Components/Common -I../../../Drivers/BSP/Components/lps22hh -I../../../Drivers/BSP/Components/lis2mdl -I../../../Drivers/BSP/Components/lsm6dso -I../../../Middlewares/ST/STM32_MetaDataManager -I../../../Middlewares/ST/STM32_BLE_Manager/Inc -I../../../Middlewares/Third_Party/parson -I../../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../../Middlewares/ST/STM32_MotionSP_Library/Inc -I../../../Middlewares/ST/BlueNRG-2/includes -I../../../Drivers/BSP/Components/ism330dlc -I../../../Drivers/BSP/CCA02M2 -I../../../Drivers/BSP/Components/hts221 -I../../../Middlewares/ST/BlueNRG-2/utils -I../../../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-STM32_BLE_Manager

clean-Middlewares-2f-STM32_BLE_Manager:
	-$(RM) ./Middlewares/STM32_BLE_Manager/BLE_AudioLevel.d ./Middlewares/STM32_BLE_Manager/BLE_AudioLevel.o ./Middlewares/STM32_BLE_Manager/BLE_Environmental.d ./Middlewares/STM32_BLE_Manager/BLE_Environmental.o ./Middlewares/STM32_BLE_Manager/BLE_FFT_AlarmAccPeakStatus.d ./Middlewares/STM32_BLE_Manager/BLE_FFT_AlarmAccPeakStatus.o ./Middlewares/STM32_BLE_Manager/BLE_FFT_AlarmSpeedStatus.d ./Middlewares/STM32_BLE_Manager/BLE_FFT_AlarmSpeedStatus.o ./Middlewares/STM32_BLE_Manager/BLE_FFT_AlarmSubrangeStatus.d ./Middlewares/STM32_BLE_Manager/BLE_FFT_AlarmSubrangeStatus.o ./Middlewares/STM32_BLE_Manager/BLE_FFT_Amplitude.d ./Middlewares/STM32_BLE_Manager/BLE_FFT_Amplitude.o ./Middlewares/STM32_BLE_Manager/BLE_Inertial.d ./Middlewares/STM32_BLE_Manager/BLE_Inertial.o ./Middlewares/STM32_BLE_Manager/BLE_Manager.d ./Middlewares/STM32_BLE_Manager/BLE_Manager.o ./Middlewares/STM32_BLE_Manager/BLE_TimeDomain.d ./Middlewares/STM32_BLE_Manager/BLE_TimeDomain.o

.PHONY: clean-Middlewares-2f-STM32_BLE_Manager
