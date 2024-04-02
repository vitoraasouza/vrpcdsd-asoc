################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Heuristics/VNS/VNS.cpp 

OBJS += \
./src/Heuristics/VNS/VNS.o 

CPP_DEPS += \
./src/Heuristics/VNS/VNS.d 


# Each subdirectory must supply rules for building sources it contributes
src/Heuristics/VNS/%.o: ../src/Heuristics/VNS/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


