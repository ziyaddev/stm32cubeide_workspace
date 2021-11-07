################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
/Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/TouchGFX/target/generated/OSWrappers.cpp \
/Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/TouchGFX/target/generated/STM32DMA.cpp \
/Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/TouchGFX/target/generated/TouchGFXConfiguration.cpp \
/Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/TouchGFX/target/generated/TouchGFXGeneratedDataReader.cpp \
/Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/TouchGFX/target/generated/TouchGFXGeneratedHAL.cpp 

OBJS += \
./Application/User/TouchGFX/target/generated/OSWrappers.o \
./Application/User/TouchGFX/target/generated/STM32DMA.o \
./Application/User/TouchGFX/target/generated/TouchGFXConfiguration.o \
./Application/User/TouchGFX/target/generated/TouchGFXGeneratedDataReader.o \
./Application/User/TouchGFX/target/generated/TouchGFXGeneratedHAL.o 

CPP_DEPS += \
./Application/User/TouchGFX/target/generated/OSWrappers.d \
./Application/User/TouchGFX/target/generated/STM32DMA.d \
./Application/User/TouchGFX/target/generated/TouchGFXConfiguration.d \
./Application/User/TouchGFX/target/generated/TouchGFXGeneratedDataReader.d \
./Application/User/TouchGFX/target/generated/TouchGFXGeneratedHAL.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/TouchGFX/target/generated/OSWrappers.o: /Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/TouchGFX/target/generated/OSWrappers.cpp Application/User/TouchGFX/target/generated/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0plus -std=gnu++14 -g3 -DSTM32G071xx -DUSE_HAL_DRIVER -DDEBUG -c -I../../TouchGFX/App -I../../TouchGFX/target/generated -I../../TouchGFX/target -I../../Core/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/touchgfx/framework/include -I../../TouchGFX/generated/fonts/include -I../../TouchGFX/generated/gui_generated/include -I../../TouchGFX/generated/images/include -I../../TouchGFX/generated/texts/include -I../../TouchGFX/gui/include -I../../TouchGFX/generated/videos/include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -femit-class-debug-always -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/TouchGFX/target/generated/STM32DMA.o: /Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/TouchGFX/target/generated/STM32DMA.cpp Application/User/TouchGFX/target/generated/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0plus -std=gnu++14 -g3 -DSTM32G071xx -DUSE_HAL_DRIVER -DDEBUG -c -I../../TouchGFX/App -I../../TouchGFX/target/generated -I../../TouchGFX/target -I../../Core/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/touchgfx/framework/include -I../../TouchGFX/generated/fonts/include -I../../TouchGFX/generated/gui_generated/include -I../../TouchGFX/generated/images/include -I../../TouchGFX/generated/texts/include -I../../TouchGFX/gui/include -I../../TouchGFX/generated/videos/include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -femit-class-debug-always -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/TouchGFX/target/generated/TouchGFXConfiguration.o: /Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/TouchGFX/target/generated/TouchGFXConfiguration.cpp Application/User/TouchGFX/target/generated/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0plus -std=gnu++14 -g3 -DSTM32G071xx -DUSE_HAL_DRIVER -DDEBUG -c -I../../TouchGFX/App -I../../TouchGFX/target/generated -I../../TouchGFX/target -I../../Core/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/touchgfx/framework/include -I../../TouchGFX/generated/fonts/include -I../../TouchGFX/generated/gui_generated/include -I../../TouchGFX/generated/images/include -I../../TouchGFX/generated/texts/include -I../../TouchGFX/gui/include -I../../TouchGFX/generated/videos/include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -femit-class-debug-always -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/TouchGFX/target/generated/TouchGFXGeneratedDataReader.o: /Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/TouchGFX/target/generated/TouchGFXGeneratedDataReader.cpp Application/User/TouchGFX/target/generated/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0plus -std=gnu++14 -g3 -DSTM32G071xx -DUSE_HAL_DRIVER -DDEBUG -c -I../../TouchGFX/App -I../../TouchGFX/target/generated -I../../TouchGFX/target -I../../Core/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/touchgfx/framework/include -I../../TouchGFX/generated/fonts/include -I../../TouchGFX/generated/gui_generated/include -I../../TouchGFX/generated/images/include -I../../TouchGFX/generated/texts/include -I../../TouchGFX/gui/include -I../../TouchGFX/generated/videos/include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -femit-class-debug-always -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/TouchGFX/target/generated/TouchGFXGeneratedHAL.o: /Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/TouchGFX/target/generated/TouchGFXGeneratedHAL.cpp Application/User/TouchGFX/target/generated/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0plus -std=gnu++14 -g3 -DSTM32G071xx -DUSE_HAL_DRIVER -DDEBUG -c -I../../TouchGFX/App -I../../TouchGFX/target/generated -I../../TouchGFX/target -I../../Core/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/touchgfx/framework/include -I../../TouchGFX/generated/fonts/include -I../../TouchGFX/generated/gui_generated/include -I../../TouchGFX/generated/images/include -I../../TouchGFX/generated/texts/include -I../../TouchGFX/gui/include -I../../TouchGFX/generated/videos/include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -femit-class-debug-always -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

