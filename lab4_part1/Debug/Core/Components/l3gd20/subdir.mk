################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Components/l3gd20/l3gd20.c 

OBJS += \
./Core/Components/l3gd20/l3gd20.o 

C_DEPS += \
./Core/Components/l3gd20/l3gd20.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Components/l3gd20/%.o Core/Components/l3gd20/%.su: ../Core/Components/l3gd20/%.c Core/Components/l3gd20/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Core/Components/lis3mdl -I../Core/Components/Common -I../Core/Components/hts221 -I../Core/Components/lps22hb -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Components-2f-l3gd20

clean-Core-2f-Components-2f-l3gd20:
	-$(RM) ./Core/Components/l3gd20/l3gd20.d ./Core/Components/l3gd20/l3gd20.o ./Core/Components/l3gd20/l3gd20.su

.PHONY: clean-Core-2f-Components-2f-l3gd20

