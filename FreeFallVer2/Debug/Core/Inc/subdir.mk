################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/FLASH_SECTOR_F4.c \
../Core/Inc/Function.c 

OBJS += \
./Core/Inc/FLASH_SECTOR_F4.o \
./Core/Inc/Function.o 

C_DEPS += \
./Core/Inc/FLASH_SECTOR_F4.d \
./Core/Inc/Function.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/%.o Core/Inc/%.su: ../Core/Inc/%.c Core/Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I"C:/Users/Thanh_Engineer/Documents/Liintech/DocAndPrograms/STM32_Programs/Main_Program_UART6_Interrupt_Method/FreeFallVer2/Drivers/STM32F4xx_HAL_Driver/Inc" -I../Core/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc

clean-Core-2f-Inc:
	-$(RM) ./Core/Inc/FLASH_SECTOR_F4.d ./Core/Inc/FLASH_SECTOR_F4.o ./Core/Inc/FLASH_SECTOR_F4.su ./Core/Inc/Function.d ./Core/Inc/Function.o ./Core/Inc/Function.su

.PHONY: clean-Core-2f-Inc

