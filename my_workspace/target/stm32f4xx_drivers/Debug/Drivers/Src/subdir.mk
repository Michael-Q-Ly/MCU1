################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/stm32f429xx_gpio_driver.c 

OBJS += \
./Drivers/Src/stm32f429xx_gpio_driver.o 

C_DEPS += \
./Drivers/Src/stm32f429xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DSTM32F429I_DISC1 -DSTM32 -DSTM32F429ZITx -DSTM32F4 -c -I../Inc -I"C:/Users/Micha/OneDrive/Documents/Embedded_C/MCU1/my_workspace/target/stm32f4xx_drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -specs=rdimon.specs -lc -lrdimon -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/stm32f429xx_gpio_driver.d ./Drivers/Src/stm32f429xx_gpio_driver.o

.PHONY: clean-Drivers-2f-Src

