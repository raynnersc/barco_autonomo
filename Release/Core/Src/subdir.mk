################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/API_STM32F103_SERVOMOTOR_MG90D.c \
../Core/Src/Bluetooth_BLE_V4.2_JDY-18.c \
../Core/Src/HMC5883L.c \
../Core/Src/I2Cdev.c \
../Core/Src/L293D_STM32F103.c \
../Core/Src/QMC5883.c \
../Core/Src/Sail_Algorithms.c \
../Core/Src/main.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c 

OBJS += \
./Core/Src/API_STM32F103_SERVOMOTOR_MG90D.o \
./Core/Src/Bluetooth_BLE_V4.2_JDY-18.o \
./Core/Src/HMC5883L.o \
./Core/Src/I2Cdev.o \
./Core/Src/L293D_STM32F103.o \
./Core/Src/QMC5883.o \
./Core/Src/Sail_Algorithms.o \
./Core/Src/main.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o 

C_DEPS += \
./Core/Src/API_STM32F103_SERVOMOTOR_MG90D.d \
./Core/Src/Bluetooth_BLE_V4.2_JDY-18.d \
./Core/Src/HMC5883L.d \
./Core/Src/I2Cdev.d \
./Core/Src/L293D_STM32F103.d \
./Core/Src/QMC5883.d \
./Core/Src/Sail_Algorithms.d \
./Core/Src/main.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/API_STM32F103_SERVOMOTOR_MG90D.d ./Core/Src/API_STM32F103_SERVOMOTOR_MG90D.o ./Core/Src/API_STM32F103_SERVOMOTOR_MG90D.su ./Core/Src/Bluetooth_BLE_V4.2_JDY-18.d ./Core/Src/Bluetooth_BLE_V4.2_JDY-18.o ./Core/Src/Bluetooth_BLE_V4.2_JDY-18.su ./Core/Src/HMC5883L.d ./Core/Src/HMC5883L.o ./Core/Src/HMC5883L.su ./Core/Src/I2Cdev.d ./Core/Src/I2Cdev.o ./Core/Src/I2Cdev.su ./Core/Src/L293D_STM32F103.d ./Core/Src/L293D_STM32F103.o ./Core/Src/L293D_STM32F103.su ./Core/Src/QMC5883.d ./Core/Src/QMC5883.o ./Core/Src/QMC5883.su ./Core/Src/Sail_Algorithms.d ./Core/Src/Sail_Algorithms.o ./Core/Src/Sail_Algorithms.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su

.PHONY: clean-Core-2f-Src

