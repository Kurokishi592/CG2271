################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/2271UART.c \
../source/mtb.c \
../source/semihost_hardfault.c 

C_DEPS += \
./source/2271UART.d \
./source/mtb.d \
./source/semihost_hardfault.d 

OBJS += \
./source/2271UART.o \
./source/mtb.o \
./source/semihost_hardfault.o 


# Each subdirectory must supply rules for building sources it contributes
source/%.o: ../source/%.c source/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DCPU_MCXC444VLH -DCPU_MCXC444VLH_cm0plus -DSDK_DEBUGCONSOLE=1 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -DSDK_DEBUGCONSOLE_UART -DSERIAL_PORT_TYPE_UART=1 -DSDK_OS_FREE_RTOS -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -I"C:\Users\John\Documents\MCUXpressoIDE_25.6.136\workspace\2271UART\board" -I"C:\Users\John\Documents\MCUXpressoIDE_25.6.136\workspace\2271UART\source" -I"C:\Users\John\Documents\MCUXpressoIDE_25.6.136\workspace\2271UART\drivers" -I"C:\Users\John\Documents\MCUXpressoIDE_25.6.136\workspace\2271UART\CMSIS" -I"C:\Users\John\Documents\MCUXpressoIDE_25.6.136\workspace\2271UART\CMSIS\m-profile" -I"C:\Users\John\Documents\MCUXpressoIDE_25.6.136\workspace\2271UART\utilities" -I"C:\Users\John\Documents\MCUXpressoIDE_25.6.136\workspace\2271UART\utilities\debug_console\config" -I"C:\Users\John\Documents\MCUXpressoIDE_25.6.136\workspace\2271UART\device" -I"C:\Users\John\Documents\MCUXpressoIDE_25.6.136\workspace\2271UART\device\periph2" -I"C:\Users\John\Documents\MCUXpressoIDE_25.6.136\workspace\2271UART\utilities\debug_console" -I"C:\Users\John\Documents\MCUXpressoIDE_25.6.136\workspace\2271UART\component\serial_manager" -I"C:\Users\John\Documents\MCUXpressoIDE_25.6.136\workspace\2271UART\component\lists" -I"C:\Users\John\Documents\MCUXpressoIDE_25.6.136\workspace\2271UART\utilities\str" -I"C:\Users\John\Documents\MCUXpressoIDE_25.6.136\workspace\2271UART\component\uart" -I"C:\Users\John\Documents\MCUXpressoIDE_25.6.136\workspace\2271UART\freertos\freertos-kernel\include" -I"C:\Users\John\Documents\MCUXpressoIDE_25.6.136\workspace\2271UART\freertos\freertos-kernel\portable\GCC\ARM_CM0" -I"C:\Users\John\Documents\MCUXpressoIDE_25.6.136\workspace\2271UART\freertos\freertos-kernel\template" -I"C:\Users\John\Documents\MCUXpressoIDE_25.6.136\workspace\2271UART\freertos\freertos-kernel\template\ARM_CM0" -O0 -fno-common -g3 -gdwarf-4 -Wall -c -ffunction-sections -fdata-sections -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m0plus -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-source

clean-source:
	-$(RM) ./source/2271UART.d ./source/2271UART.o ./source/mtb.d ./source/mtb.o ./source/semihost_hardfault.d ./source/semihost_hardfault.o

.PHONY: clean-source

