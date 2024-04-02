################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Heuristics/VND/VND.cpp 

OBJS += \
./src/Heuristics/VND/VND.o 

CPP_DEPS += \
./src/Heuristics/VND/VND.d 


# Each subdirectory must supply rules for building sources it contributes
src/Heuristics/VND/%.o: ../src/Heuristics/VND/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


