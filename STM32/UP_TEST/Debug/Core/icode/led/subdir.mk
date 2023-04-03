################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/icode/led/led.c 

OBJS += \
./Core/icode/led/led.o 

C_DEPS += \
./Core/icode/led/led.d 


# Each subdirectory must supply rules for building sources it contributes
Core/icode/led/%.o Core/icode/led/%.su: ../Core/icode/led/%.c Core/icode/led/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F405xx -D__TARGET_FPU_VFP -D__FPU_PRESENT=1 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"F:/DZ/Legged robot/Hyleg/STM32/UP_TEST/Math" -I"F:/DZ/Legged robot/Hyleg/STM32/UP_TEST/Drivers/CMSIS/DSP/Inc" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-icode-2f-led

clean-Core-2f-icode-2f-led:
	-$(RM) ./Core/icode/led/led.d ./Core/icode/led/led.o ./Core/icode/led/led.su

.PHONY: clean-Core-2f-icode-2f-led

