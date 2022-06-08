################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/004_ButtonInterrupt.c \
../Src/sysmem.c 

OBJS += \
./Src/004_ButtonInterrupt.o \
./Src/sysmem.o 

C_DEPS += \
./Src/004_ButtonInterrupt.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DSTM32F429I_DISC1 -DSTM32 -DSTM32F429ZITx -DSTM32F4 -c -I../Inc -I"C:/Users/Micha/OneDrive/Documents/Embedded_C/MCU1/my_workspace/target/stm32f4xx_drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/004_ButtonInterrupt.d ./Src/004_ButtonInterrupt.o ./Src/sysmem.d ./Src/sysmem.o

.PHONY: clean-Src

