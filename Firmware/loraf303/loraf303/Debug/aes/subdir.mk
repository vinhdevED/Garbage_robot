################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../aes/lmic.c \
../aes/other.c 

OBJS += \
./aes/lmic.o \
./aes/other.o 

C_DEPS += \
./aes/lmic.d \
./aes/other.d 


# Each subdirectory must supply rules for building sources it contributes
aes/%.o aes/%.su aes/%.cyclo: ../aes/%.c aes/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-aes

clean-aes:
	-$(RM) ./aes/lmic.cyclo ./aes/lmic.d ./aes/lmic.o ./aes/lmic.su ./aes/other.cyclo ./aes/other.d ./aes/other.o ./aes/other.su

.PHONY: clean-aes

