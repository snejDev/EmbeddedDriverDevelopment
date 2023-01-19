################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/SPI_CMD_TxRx/005_SPI_CmdTxRx_v1.c 

OBJS += \
./Src/SPI_CMD_TxRx/005_SPI_CmdTxRx_v1.o 

C_DEPS += \
./Src/SPI_CMD_TxRx/005_SPI_CmdTxRx_v1.d 


# Each subdirectory must supply rules for building sources it contributes
Src/SPI_CMD_TxRx/%.o Src/SPI_CMD_TxRx/%.su: ../Src/SPI_CMD_TxRx/%.c Src/SPI_CMD_TxRx/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F407VETx -c -I../Inc -I"E:/Electronics/MicrocontrollerProgramming/STM32/DriverDevelopment/DeviceHeaderFile/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-SPI_CMD_TxRx

clean-Src-2f-SPI_CMD_TxRx:
	-$(RM) ./Src/SPI_CMD_TxRx/005_SPI_CmdTxRx_v1.d ./Src/SPI_CMD_TxRx/005_SPI_CmdTxRx_v1.o ./Src/SPI_CMD_TxRx/005_SPI_CmdTxRx_v1.su

.PHONY: clean-Src-2f-SPI_CMD_TxRx

