################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/icode/tmotor/tmotor.c 

OBJS += \
./Core/icode/tmotor/tmotor.o 

C_DEPS += \
./Core/icode/tmotor/tmotor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/icode/tmotor/%.o Core/icode/tmotor/%.su: ../Core/icode/tmotor/%.c Core/icode/tmotor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-icode-2f-tmotor

clean-Core-2f-icode-2f-tmotor:
	-$(RM) ./Core/icode/tmotor/tmotor.d ./Core/icode/tmotor/tmotor.o ./Core/icode/tmotor/tmotor.su

.PHONY: clean-Core-2f-icode-2f-tmotor

