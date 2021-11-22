################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DISPLAY/Target/key_io.c \
../DISPLAY/Target/lcd_io.c \
../DISPLAY/Target/mem_io.c 

OBJS += \
./DISPLAY/Target/key_io.o \
./DISPLAY/Target/lcd_io.o \
./DISPLAY/Target/mem_io.o 

C_DEPS += \
./DISPLAY/Target/key_io.d \
./DISPLAY/Target/lcd_io.d \
./DISPLAY/Target/mem_io.d 


# Each subdirectory must supply rules for building sources it contributes
DISPLAY/Target/%.o: ../DISPLAY/Target/%.c DISPLAY/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../DISPLAY/App -I../DISPLAY/Target -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/Components/mx25l6433f -I../Drivers/BSP/Components/Common -I../Drivers/BSP/Components/ili9341 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

