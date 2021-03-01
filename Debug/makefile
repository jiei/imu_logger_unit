################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F3xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
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

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
imu_logger_unit.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
imu_logger_unit.list \

OBJCOPY_BIN += \
imu_logger_unit.bin \


# All Target
all: imu_logger_unit.elf secondary-outputs

# Tool invocations
imu_logger_unit.elf: $(OBJS) $(USER_OBJS) /home/jack/STM32CubeIDE/workspace_1.4.0/imu_logger_unit/STM32F303K8TX_FLASH.ld
	arm-none-eabi-gcc -o "imu_logger_unit.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/jack/STM32CubeIDE/workspace_1.4.0/imu_logger_unit/STM32F303K8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="imu_logger_unit.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

imu_logger_unit.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "imu_logger_unit.list"
	@echo 'Finished building: $@'
	@echo ' '

imu_logger_unit.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "imu_logger_unit.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
