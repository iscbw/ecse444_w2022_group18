################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Components/wm8994/wm8994.c 

OBJS += \
./Core/Components/wm8994/wm8994.o 

C_DEPS += \
./Core/Components/wm8994/wm8994.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Components/wm8994/%.o Core/Components/wm8994/%.su: ../Core/Components/wm8994/%.c Core/Components/wm8994/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Components-2f-wm8994

clean-Core-2f-Components-2f-wm8994:
	-$(RM) ./Core/Components/wm8994/wm8994.d ./Core/Components/wm8994/wm8994.o ./Core/Components/wm8994/wm8994.su

.PHONY: clean-Core-2f-Components-2f-wm8994

