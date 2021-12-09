################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include PredictiveMaintenance/User/Patch/subdir.mk
-include PredictiveMaintenance/User/subdir.mk
-include PredictiveMaintenance/STM32CubeIDE/subdir.mk
-include Middlewares/json/subdir.mk
-include Middlewares/STM32_MotionSP_Library/subdir.mk
-include Middlewares/STM32_BLE_Manager/subdir.mk
-include Middlewares/MetaDataManager/subdir.mk
-include Middlewares/BlueNRG-2/Utils/subdir.mk
-include Middlewares/BlueNRG-2/HCI/hci_tl_patterns/Basic/subdir.mk
-include Middlewares/BlueNRG-2/HCI/controller/subdir.mk
-include Middlewares/BlueNRG-2/HCI/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/subdir.mk
-include Drivers/CMSIS/subdir.mk
-include Drivers/BSP/X-NUCLEO-IKS01A3/subdir.mk
-include Drivers/BSP/X-NUCLEO-CCA02M2/subdir.mk
-include Drivers/BSP/Components/ism330dlc/subdir.mk
-include Drivers/BSP/Components/IKS01A3/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := NUCLEO-F446RE_PredictiveMaintenance
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
NUCLEO-F446RE_PredictiveMaintenance.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
NUCLEO-F446RE_PredictiveMaintenance.list \

OBJCOPY_BIN += \
NUCLEO-F446RE_PredictiveMaintenance.bin \


# All Target
all: main-build

# Main-build Target
main-build: NUCLEO-F446RE_PredictiveMaintenance.elf secondary-outputs

# Tool invocations
NUCLEO-F446RE_PredictiveMaintenance.elf: $(OBJS) $(USER_OBJS) ../LinkerScript.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "NUCLEO-F446RE_PredictiveMaintenance.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"../LinkerScript.ld" --specs=nosys.specs -Wl,-Map="NUCLEO-F446RE_PredictiveMaintenance.map" -Wl,--gc-sections -static -L../../../Drivers/CMSIS/Lib/GCC -L../../../Middlewares/ST/STM32_Audio/Addons/PDM/Lib --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

NUCLEO-F446RE_PredictiveMaintenance.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "NUCLEO-F446RE_PredictiveMaintenance.list"
	@echo 'Finished building: $@'
	@echo ' '

NUCLEO-F446RE_PredictiveMaintenance.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "NUCLEO-F446RE_PredictiveMaintenance.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) NUCLEO-F446RE_PredictiveMaintenance.bin NUCLEO-F446RE_PredictiveMaintenance.elf NUCLEO-F446RE_PredictiveMaintenance.list default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
