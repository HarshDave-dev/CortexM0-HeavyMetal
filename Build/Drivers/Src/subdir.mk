C_SRCS += \
../Drivers/Src/cortexM0Systick.c \
../Drivers/Src/cortexM0Gpio.c \
../Drivers/Src/cortexM0ResetClockConf.c 

OBJS += \
./Drivers/Src/cortexM0Systick.o \
./Drivers/Src/cortexM0Gpio.o \
./Drivers/Src/cortexM0ResetClockConf.o 

C_DEPS += \
./Drivers/Src/cortexM0Systick.d \
./Drivers/Src/cortexM0Gpio.d \
./Drivers/Src/cortexM0ResetClockConf.d 


Drivers/Src/cortexM0Systick.o: ../Drivers/Src/cortexM0Systick.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F072xB -DDEBUG -c -I../Core/Inc -I../Drivers/Inc  -O0 -ffunction-sections -fdata-sections  -fstack-usage -MMD -MP -MF"Drivers/Src/cortexM0Systick.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/Src/cortexM0Gpio.o: ../Drivers/Src/cortexM0Gpio.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F072xB -DDEBUG -c -I../Core/Inc -I../Drivers/Inc  -O0 -ffunction-sections -fdata-sections  -fstack-usage -MMD -MP -MF"Drivers/Src/cortexM0Gpio.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/Src/cortexM0ResetClockConf.o: ../Drivers/Src/cortexM0ResetClockConf.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F072xB -DDEBUG -c -I../Core/Inc -I../Drivers/Inc  -O0 -ffunction-sections -fdata-sections  -fstack-usage -MMD -MP -MF"Drivers/Src/cortexM0ResetClockConf.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

