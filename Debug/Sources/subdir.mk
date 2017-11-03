################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Sources/Calculate.c \
../Sources/Command.c \
../Sources/MyPIT.c \
../Sources/main.c \
../Sources/median.c 

OBJS += \
./Sources/Calculate.o \
./Sources/Command.o \
./Sources/MyPIT.o \
./Sources/main.o \
./Sources/median.o 

C_DEPS += \
./Sources/Calculate.d \
./Sources/Command.d \
./Sources/MyPIT.d \
./Sources/main.d \
./Sources/median.d 


# Each subdirectory must supply rules for building sources it contributes
Sources/%.o: ../Sources/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -I"C:\Users\Declan\Documents\GitKraken\Power Meter\Library" -I"C:/Users/Declan/Documents/GitKraken/Power Meter/Static_Code/IO_Map" -I"C:/Users/Declan/Documents/GitKraken/Power Meter/Sources" -I"C:/Users/Declan/Documents/GitKraken/Power Meter/Generated_Code" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


