################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Components/hx8347g/hx8347g.c 

OBJS += \
./Core/Components/hx8347g/hx8347g.o 

C_DEPS += \
./Core/Components/hx8347g/hx8347g.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Components/hx8347g/%.o Core/Components/hx8347g/%.su: ../Core/Components/hx8347g/%.c Core/Components/hx8347g/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Components-2f-hx8347g

clean-Core-2f-Components-2f-hx8347g:
	-$(RM) ./Core/Components/hx8347g/hx8347g.d ./Core/Components/hx8347g/hx8347g.o ./Core/Components/hx8347g/hx8347g.su

.PHONY: clean-Core-2f-Components-2f-hx8347g

