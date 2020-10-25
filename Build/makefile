-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/Src/subdir.mk
-include App/Startup/subdir.mk
-include App/Src/subdir.mk
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
Firmware.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Firmware.list \

OBJCOPY_BIN += \
Firmware.bin \


# All Target
all: Firmware.elf secondary-outputs

# Tool invocations
Firmware.elf: $(OBJS) $(USER_OBJS) ../cortexM0.ld
	arm-none-eabi-gcc -o "Firmware.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"../cortexM0.ld" --specs=nosys.specs -Wl,-Map="Firmware.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Firmware.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Firmware.list"
	@echo 'Finished building: $@'
	@echo ' '

Firmware.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Firmware.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *.elf *.o *.map *.bin
	-$(RM) App/Src/*.o
	-$(RM) App/Src/*.d
	-$(RM) App/Startup/*.o
	-$(RM) App/Startup/*.d
	-$(RM) App/Src/*.su
	-$(RM) Drivers/Src/*.o
	-$(RM) Drivers/Src/*.d
	-$(RM) Drivers/Src/*.su
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)
	-@echo 'There you go buddy!'
.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
