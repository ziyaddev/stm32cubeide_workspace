################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/Core/Src/MB1642BDataReader.c \
/Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/Core/Src/MB1642BDisplayDriver.c 

CPP_SRCS += \
/Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/Core/Src/MB1642BButtonController.cpp 

OBJS += \
./Drivers/MB1642B/MB1642BButtonController.o \
./Drivers/MB1642B/MB1642BDataReader.o \
./Drivers/MB1642B/MB1642BDisplayDriver.o 

C_DEPS += \
./Drivers/MB1642B/MB1642BDataReader.d \
./Drivers/MB1642B/MB1642BDisplayDriver.d 

CPP_DEPS += \
./Drivers/MB1642B/MB1642BButtonController.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MB1642B/MB1642BButtonController.o: /Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/Core/Src/MB1642BButtonController.cpp Drivers/MB1642B/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0plus -std=gnu++14 -g3 -DSTM32G071xx -DUSE_HAL_DRIVER -DDEBUG -c -I../../TouchGFX/App -I../../TouchGFX/target/generated -I../../TouchGFX/target -I../../Core/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/touchgfx/framework/include -I../../TouchGFX/generated/fonts/include -I../../TouchGFX/generated/gui_generated/include -I../../TouchGFX/generated/images/include -I../../TouchGFX/generated/texts/include -I../../TouchGFX/gui/include -I../../TouchGFX/generated/videos/include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -femit-class-debug-always -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/MB1642B/MB1642BDataReader.o: /Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/Core/Src/MB1642BDataReader.c Drivers/MB1642B/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DSTM32G071xx -DUSE_HAL_DRIVER -DDEBUG -c -I../../TouchGFX/App -I../../TouchGFX/target/generated -I../../TouchGFX/target -I../../Core/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/touchgfx/framework/include -I../../TouchGFX/generated/fonts/include -I../../TouchGFX/generated/gui_generated/include -I../../TouchGFX/generated/images/include -I../../TouchGFX/generated/texts/include -I../../TouchGFX/gui/include -I../../TouchGFX/generated/videos/include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/MB1642B/MB1642BDisplayDriver.o: /Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/Core/Src/MB1642BDisplayDriver.c Drivers/MB1642B/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DSTM32G071xx -DUSE_HAL_DRIVER -DDEBUG -c -I../../TouchGFX/App -I../../TouchGFX/target/generated -I../../TouchGFX/target -I../../Core/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/touchgfx/framework/include -I../../TouchGFX/generated/fonts/include -I../../TouchGFX/generated/gui_generated/include -I../../TouchGFX/generated/images/include -I../../TouchGFX/generated/texts/include -I../../TouchGFX/gui/include -I../../TouchGFX/generated/videos/include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

