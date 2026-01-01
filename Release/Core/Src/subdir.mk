################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc_sampling.c \
../Core/Src/lcd_display.c \
../Core/Src/main.c \
../Core/Src/mppt.c \
../Core/Src/pwm.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/work_protect.c 

OBJS += \
./Core/Src/adc_sampling.o \
./Core/Src/lcd_display.o \
./Core/Src/main.o \
./Core/Src/mppt.o \
./Core/Src/pwm.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/work_protect.o 

C_DEPS += \
./Core/Src/adc_sampling.d \
./Core/Src/lcd_display.d \
./Core/Src/main.d \
./Core/Src/mppt.d \
./Core/Src/pwm.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/work_protect.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I"G:/Order/Pak Aji/MPPT Interleaved Boost/Software/MPPT_Interleaved_Boost/MPPT_Interleaved_Boost/Drivers/csrc" -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc_sampling.cyclo ./Core/Src/adc_sampling.d ./Core/Src/adc_sampling.o ./Core/Src/adc_sampling.su ./Core/Src/lcd_display.cyclo ./Core/Src/lcd_display.d ./Core/Src/lcd_display.o ./Core/Src/lcd_display.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mppt.cyclo ./Core/Src/mppt.d ./Core/Src/mppt.o ./Core/Src/mppt.su ./Core/Src/pwm.cyclo ./Core/Src/pwm.d ./Core/Src/pwm.o ./Core/Src/pwm.su ./Core/Src/stm32f1xx_hal_msp.cyclo ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_it.cyclo ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.cyclo ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su ./Core/Src/work_protect.cyclo ./Core/Src/work_protect.d ./Core/Src/work_protect.o ./Core/Src/work_protect.su

.PHONY: clean-Core-2f-Src

