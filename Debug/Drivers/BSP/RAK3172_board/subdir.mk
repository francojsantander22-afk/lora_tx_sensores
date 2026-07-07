################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/RAK3172_board/RAK3172_radio.c 

OBJS += \
./Drivers/BSP/RAK3172_board/RAK3172_radio.o 

C_DEPS += \
./Drivers/BSP/RAK3172_board/RAK3172_radio.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/RAK3172_board/%.o Drivers/BSP/RAK3172_board/%.su Drivers/BSP/RAK3172_board/%.cyclo: ../Drivers/BSP/RAK3172_board/%.c Drivers/BSP/RAK3172_board/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../SubGHz_Phy/App -I../SubGHz_Phy/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/lpm/tiny_lpm -I../Utilities/timer -I../Drivers/BSP/RAK3172_board -I../Middlewares/Third_Party/SubGHz_Phy/radio_driver -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-RAK3172_board

clean-Drivers-2f-BSP-2f-RAK3172_board:
	-$(RM) ./Drivers/BSP/RAK3172_board/RAK3172_radio.cyclo ./Drivers/BSP/RAK3172_board/RAK3172_radio.d ./Drivers/BSP/RAK3172_board/RAK3172_radio.o ./Drivers/BSP/RAK3172_board/RAK3172_radio.su

.PHONY: clean-Drivers-2f-BSP-2f-RAK3172_board

