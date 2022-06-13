################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/001_LedToggle.c \
../Src/002_LedButton.c \
../Src/003_ExternalLedButton.c \
../Src/004_ButtonInterrupt.c \
../Src/004_ButtonInterrupt_Test.c \
../Src/005_SPITxTesting.c \
../Src/main.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/001_LedToggle.o \
./Src/002_LedButton.o \
./Src/003_ExternalLedButton.o \
./Src/004_ButtonInterrupt.o \
./Src/004_ButtonInterrupt_Test.o \
./Src/005_SPITxTesting.o \
./Src/main.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/001_LedToggle.d \
./Src/002_LedButton.d \
./Src/003_ExternalLedButton.d \
./Src/004_ButtonInterrupt.d \
./Src/004_ButtonInterrupt_Test.d \
./Src/005_SPITxTesting.d \
./Src/main.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DSTM32F429I_DISC1 -DSTM32 -DSTM32F429ZITx -DSTM32F4 -c -I../Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/001_LedToggle.d ./Src/001_LedToggle.o ./Src/002_LedButton.d ./Src/002_LedButton.o ./Src/003_ExternalLedButton.d ./Src/003_ExternalLedButton.o ./Src/004_ButtonInterrupt.d ./Src/004_ButtonInterrupt.o ./Src/004_ButtonInterrupt_Test.d ./Src/004_ButtonInterrupt_Test.o ./Src/005_SPITxTesting.d ./Src/005_SPITxTesting.o ./Src/main.d ./Src/main.o ./Src/syscalls.d ./Src/syscalls.o ./Src/sysmem.d ./Src/sysmem.o

.PHONY: clean-Src

