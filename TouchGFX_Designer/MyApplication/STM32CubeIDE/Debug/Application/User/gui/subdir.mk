################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
/Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/TouchGFX/gui/src/common/FrontendApplication.cpp \
/Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/TouchGFX/gui/src/model/Model.cpp \
/Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/TouchGFX/gui/src/screen1_screen/Screen1Presenter.cpp \
/Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/TouchGFX/gui/src/screen1_screen/Screen1View.cpp 

OBJS += \
./Application/User/gui/FrontendApplication.o \
./Application/User/gui/Model.o \
./Application/User/gui/Screen1Presenter.o \
./Application/User/gui/Screen1View.o 

CPP_DEPS += \
./Application/User/gui/FrontendApplication.d \
./Application/User/gui/Model.d \
./Application/User/gui/Screen1Presenter.d \
./Application/User/gui/Screen1View.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/gui/FrontendApplication.o: /Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/TouchGFX/gui/src/common/FrontendApplication.cpp Application/User/gui/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0plus -std=gnu++14 -g3 -DSTM32G071xx -DUSE_HAL_DRIVER -DDEBUG -c -I../../TouchGFX/App -I../../TouchGFX/target/generated -I../../TouchGFX/target -I../../Core/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/touchgfx/framework/include -I../../TouchGFX/generated/fonts/include -I../../TouchGFX/generated/gui_generated/include -I../../TouchGFX/generated/images/include -I../../TouchGFX/generated/texts/include -I../../TouchGFX/gui/include -I../../TouchGFX/generated/videos/include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -femit-class-debug-always -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/gui/Model.o: /Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/TouchGFX/gui/src/model/Model.cpp Application/User/gui/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0plus -std=gnu++14 -g3 -DSTM32G071xx -DUSE_HAL_DRIVER -DDEBUG -c -I../../TouchGFX/App -I../../TouchGFX/target/generated -I../../TouchGFX/target -I../../Core/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/touchgfx/framework/include -I../../TouchGFX/generated/fonts/include -I../../TouchGFX/generated/gui_generated/include -I../../TouchGFX/generated/images/include -I../../TouchGFX/generated/texts/include -I../../TouchGFX/gui/include -I../../TouchGFX/generated/videos/include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -femit-class-debug-always -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/gui/Screen1Presenter.o: /Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/TouchGFX/gui/src/screen1_screen/Screen1Presenter.cpp Application/User/gui/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0plus -std=gnu++14 -g3 -DSTM32G071xx -DUSE_HAL_DRIVER -DDEBUG -c -I../../TouchGFX/App -I../../TouchGFX/target/generated -I../../TouchGFX/target -I../../Core/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/touchgfx/framework/include -I../../TouchGFX/generated/fonts/include -I../../TouchGFX/generated/gui_generated/include -I../../TouchGFX/generated/images/include -I../../TouchGFX/generated/texts/include -I../../TouchGFX/gui/include -I../../TouchGFX/generated/videos/include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -femit-class-debug-always -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/gui/Screen1View.o: /Users/ziyad/STM32CubeIDE/workspace_1.7.0/TouchGFX_Designer/MyApplication/TouchGFX/gui/src/screen1_screen/Screen1View.cpp Application/User/gui/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0plus -std=gnu++14 -g3 -DSTM32G071xx -DUSE_HAL_DRIVER -DDEBUG -c -I../../TouchGFX/App -I../../TouchGFX/target/generated -I../../TouchGFX/target -I../../Core/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc -I../../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/touchgfx/framework/include -I../../TouchGFX/generated/fonts/include -I../../TouchGFX/generated/gui_generated/include -I../../TouchGFX/generated/images/include -I../../TouchGFX/generated/texts/include -I../../TouchGFX/gui/include -I../../TouchGFX/generated/videos/include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -femit-class-debug-always -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

