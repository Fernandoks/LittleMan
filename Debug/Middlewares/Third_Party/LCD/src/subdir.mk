################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/LCD/src/tm_stm32_delay.c \
../Middlewares/Third_Party/LCD/src/tm_stm32_gpio.c \
../Middlewares/Third_Party/LCD/src/tm_stm32_hd44780.c 

OBJS += \
./Middlewares/Third_Party/LCD/src/tm_stm32_delay.o \
./Middlewares/Third_Party/LCD/src/tm_stm32_gpio.o \
./Middlewares/Third_Party/LCD/src/tm_stm32_hd44780.o 

C_DEPS += \
./Middlewares/Third_Party/LCD/src/tm_stm32_delay.d \
./Middlewares/Third_Party/LCD/src/tm_stm32_gpio.d \
./Middlewares/Third_Party/LCD/src/tm_stm32_hd44780.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/LCD/src/tm_stm32_delay.o: ../Middlewares/Third_Party/LCD/src/tm_stm32_delay.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I"C:/Users/Fernando/Dropbox/Estudos/Embedded/CubeWorkbench/LittleMan2_IT/Middlewares/Third_Party/LCD/inc" -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/LCD/src/tm_stm32_delay.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/LCD/src/tm_stm32_gpio.o: ../Middlewares/Third_Party/LCD/src/tm_stm32_gpio.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I"C:/Users/Fernando/Dropbox/Estudos/Embedded/CubeWorkbench/LittleMan2_IT/Middlewares/Third_Party/LCD/inc" -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/LCD/src/tm_stm32_gpio.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/LCD/src/tm_stm32_hd44780.o: ../Middlewares/Third_Party/LCD/src/tm_stm32_hd44780.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I"C:/Users/Fernando/Dropbox/Estudos/Embedded/CubeWorkbench/LittleMan2_IT/Middlewares/Third_Party/LCD/inc" -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/LCD/src/tm_stm32_hd44780.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

