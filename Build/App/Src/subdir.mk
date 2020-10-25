C_SRCS += \
../App/Src/main.c \
../App/Src/interrupt_handler.c

OBJS += \
./App/Src/main.o \
./App/Src/interrupt_handler.o

C_DEPS += \
./App/Src/main.d \
./App/Src/interrupt_handler.d


App/Src/main.o: ../App/Src/main.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F072xB -DDEBUG -c -I../App/Inc -I../Drivers/Inc -I../Drivers/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"App/Src/main.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
App/Src/interrupt_handler.o: ../App/Src/interrupt_handler.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F072xB -DDEBUG -c -I../App/Inc -I../Drivers/Inc -I../Drivers/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"App/Src/stm32f0xx_it.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

