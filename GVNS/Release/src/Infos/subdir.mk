################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Infos/InputSettings.cpp 

OBJS += \
./src/Infos/InputSettings.o 

CPP_DEPS += \
./src/Infos/InputSettings.d 


# Each subdirectory must supply rules for building sources it contributes
src/Infos/%.o: ../src/Infos/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


