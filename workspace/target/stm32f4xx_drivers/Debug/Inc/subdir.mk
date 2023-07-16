################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Inc/001_LedToggle.c \
../Inc/002_LedButton.c \
../Inc/003_ExternalLedButton.c \
../Inc/004_ButtonInterrupt.c \
../Inc/004_ButtonInterrupt_Test.c \
../Inc/005_SPITxTesting.c \
../Inc/006_SPITxOnlyArduino.c \
../Inc/main.c \
../Inc/syscalls.c \
../Inc/sysmem.c 

OBJS += \
./Inc/001_LedToggle.o \
./Inc/002_LedButton.o \
./Inc/003_ExternalLedButton.o \
./Inc/004_ButtonInterrupt.o \
./Inc/004_ButtonInterrupt_Test.o \
./Inc/005_SPITxTesting.o \
./Inc/006_SPITxOnlyArduino.o \
./Inc/main.o \
./Inc/syscalls.o \
./Inc/sysmem.o 

C_DEPS += \
./Inc/001_LedToggle.d \
./Inc/002_LedButton.d \
./Inc/003_ExternalLedButton.d \
./Inc/004_ButtonInterrupt.d \
./Inc/004_ButtonInterrupt_Test.d \
./Inc/005_SPITxTesting.d \
./Inc/006_SPITxOnlyArduino.d \
./Inc/main.d \
./Inc/syscalls.d \
./Inc/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Inc/%.o Inc/%.su Inc/%.cyclo: ../Inc/%.c Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F429I_DISC1 -DSTM32 -DSTM32F429ZITx -DSTM32F4 -c -I../Inc -I../Drivers/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Inc

clean-Inc:
	-$(RM) ./Inc/001_LedToggle.cyclo ./Inc/001_LedToggle.d ./Inc/001_LedToggle.o ./Inc/001_LedToggle.su ./Inc/002_LedButton.cyclo ./Inc/002_LedButton.d ./Inc/002_LedButton.o ./Inc/002_LedButton.su ./Inc/003_ExternalLedButton.cyclo ./Inc/003_ExternalLedButton.d ./Inc/003_ExternalLedButton.o ./Inc/003_ExternalLedButton.su ./Inc/004_ButtonInterrupt.cyclo ./Inc/004_ButtonInterrupt.d ./Inc/004_ButtonInterrupt.o ./Inc/004_ButtonInterrupt.su ./Inc/004_ButtonInterrupt_Test.cyclo ./Inc/004_ButtonInterrupt_Test.d ./Inc/004_ButtonInterrupt_Test.o ./Inc/004_ButtonInterrupt_Test.su ./Inc/005_SPITxTesting.cyclo ./Inc/005_SPITxTesting.d ./Inc/005_SPITxTesting.o ./Inc/005_SPITxTesting.su ./Inc/006_SPITxOnlyArduino.cyclo ./Inc/006_SPITxOnlyArduino.d ./Inc/006_SPITxOnlyArduino.o ./Inc/006_SPITxOnlyArduino.su ./Inc/main.cyclo ./Inc/main.d ./Inc/main.o ./Inc/main.su ./Inc/syscalls.cyclo ./Inc/syscalls.d ./Inc/syscalls.o ./Inc/syscalls.su ./Inc/sysmem.cyclo ./Inc/sysmem.d ./Inc/sysmem.o ./Inc/sysmem.su

.PHONY: clean-Inc

