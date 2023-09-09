################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Astrocast/astronode_application.c \
../Drivers/Astrocast/astronode_transport.c 

OBJS += \
./Drivers/Astrocast/astronode_application.o \
./Drivers/Astrocast/astronode_transport.o 

C_DEPS += \
./Drivers/Astrocast/astronode_application.d \
./Drivers/Astrocast/astronode_transport.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Astrocast/%.o Drivers/Astrocast/%.su Drivers/Astrocast/%.cyclo: ../Drivers/Astrocast/%.c Drivers/Astrocast/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/mzeml/embedded/G431_Test_Uart/Drivers/Astrocast" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Astrocast

clean-Drivers-2f-Astrocast:
	-$(RM) ./Drivers/Astrocast/astronode_application.cyclo ./Drivers/Astrocast/astronode_application.d ./Drivers/Astrocast/astronode_application.o ./Drivers/Astrocast/astronode_application.su ./Drivers/Astrocast/astronode_transport.cyclo ./Drivers/Astrocast/astronode_transport.d ./Drivers/Astrocast/astronode_transport.o ./Drivers/Astrocast/astronode_transport.su

.PHONY: clean-Drivers-2f-Astrocast

