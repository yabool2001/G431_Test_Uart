################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/My_RTC_G431/my_rtc_g431.c 

OBJS += \
./Drivers/My_RTC_G431/my_rtc_g431.o 

C_DEPS += \
./Drivers/My_RTC_G431/my_rtc_g431.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/My_RTC_G431/%.o Drivers/My_RTC_G431/%.su Drivers/My_RTC_G431/%.cyclo: ../Drivers/My_RTC_G431/%.c Drivers/My_RTC_G431/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/mzeml/embedded/G431_Test_Uart/Drivers/Astrocast" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-My_RTC_G431

clean-Drivers-2f-My_RTC_G431:
	-$(RM) ./Drivers/My_RTC_G431/my_rtc_g431.cyclo ./Drivers/My_RTC_G431/my_rtc_g431.d ./Drivers/My_RTC_G431/my_rtc_g431.o ./Drivers/My_RTC_G431/my_rtc_g431.su

.PHONY: clean-Drivers-2f-My_RTC_G431

