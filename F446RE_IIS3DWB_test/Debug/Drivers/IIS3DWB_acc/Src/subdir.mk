################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/IIS3DWB_acc/Src/iis3dwb_reg.c 

OBJS += \
./Drivers/IIS3DWB_acc/Src/iis3dwb_reg.o 

C_DEPS += \
./Drivers/IIS3DWB_acc/Src/iis3dwb_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/IIS3DWB_acc/Src/%.o: ../Drivers/IIS3DWB_acc/Src/%.c Drivers/IIS3DWB_acc/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/IIS3DWB_acc/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-IIS3DWB_acc-2f-Src

clean-Drivers-2f-IIS3DWB_acc-2f-Src:
	-$(RM) ./Drivers/IIS3DWB_acc/Src/iis3dwb_reg.d ./Drivers/IIS3DWB_acc/Src/iis3dwb_reg.o

.PHONY: clean-Drivers-2f-IIS3DWB_acc-2f-Src

