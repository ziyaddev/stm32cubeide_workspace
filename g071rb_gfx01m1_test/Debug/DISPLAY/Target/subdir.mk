################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DISPLAY/Target/lcd_io.c \
../DISPLAY/Target/mem_io.c 

OBJS += \
./DISPLAY/Target/lcd_io.o \
./DISPLAY/Target/mem_io.o 

C_DEPS += \
./DISPLAY/Target/lcd_io.d \
./DISPLAY/Target/mem_io.d 


# Each subdirectory must supply rules for building sources it contributes
DISPLAY/Target/%.o: ../DISPLAY/Target/%.c DISPLAY/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G071xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I../DISPLAY/App -I../Drivers/BSP/Components/Common/ -I../Drivers/BSP/Components/Common -I../Drivers/BSP/Components/ili9341 -I../DISPLAY/Target -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

