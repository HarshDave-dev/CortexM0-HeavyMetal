S_SRCS += \
../App/Startup/cortexM0Startup.s 

OBJS += \
./App/Startup/cortexM0Startup.o 

S_DEPS += \
./App/Startup/cortexM0Startup.d 


# Each subdirectory must supply rules for building sources it contributes
App/Startup/cortexM0Startup.o: ../App/Startup/cortexM0Startup.s
	arm-none-eabi-gcc -mcpu=cortex-m0 -g3 -c -x assembler-with-cpp -MMD -MP -MF"App/Startup/cortexM0Startup.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

