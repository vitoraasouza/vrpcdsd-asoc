################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Heuristics/Constructive/ConstructiveHeuristics.cpp 

OBJS += \
./src/Heuristics/Constructive/ConstructiveHeuristics.o 

CPP_DEPS += \
./src/Heuristics/Constructive/ConstructiveHeuristics.d 


# Each subdirectory must supply rules for building sources it contributes
src/Heuristics/Constructive/%.o: ../src/Heuristics/Constructive/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


