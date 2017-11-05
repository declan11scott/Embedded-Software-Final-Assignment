################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Sources/Calculate.c \
../Sources/Command.c \
../Sources/MyFIFO.c \
../Sources/MyFlash.c \
../Sources/MyPIT.c \
../Sources/MyPacket.c \
../Sources/MyUART.c \
../Sources/main.c 

OBJS += \
./Sources/Calculate.o \
./Sources/Command.o \
./Sources/MyFIFO.o \
./Sources/MyFlash.o \
./Sources/MyPIT.o \
./Sources/MyPacket.o \
./Sources/MyUART.o \
./Sources/main.o 

C_DEPS += \
./Sources/Calculate.d \
./Sources/Command.d \
./Sources/MyFIFO.d \
./Sources/MyFlash.d \
./Sources/MyPIT.d \
./Sources/MyPacket.d \
./Sources/MyUART.d \
./Sources/main.d 


# Each subdirectory must supply rules for building sources it contributes
Sources/%.o: ../Sources/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -I"C:\Users\Declan\Documents\GitKraken\Power Meter\Library" -I"C:/Users/Declan/Documents/GitKraken/Power Meter/Static_Code/IO_Map" -I"C:/Users/Declan/Documents/GitKraken/Power Meter/Sources" -I"C:/Users/Declan/Documents/GitKraken/Power Meter/Generated_Code" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


