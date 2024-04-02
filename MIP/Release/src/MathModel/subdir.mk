################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/MathModel/CompactModel.cpp 

OBJS += \
./src/MathModel/CompactModel.o 

CPP_DEPS += \
./src/MathModel/CompactModel.d 


# Each subdirectory must supply rules for building sources it contributes
src/MathModel/%.o: ../src/MathModel/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/YOUR_CPLEX_INSTALLATION_FOLDER/cplex/include -I/YOUR_CPLEX_INSTALLATION_FOLDER/concert/include -O3 -Wall -c -fmessage-length=0 -m64 -O -fPIC -fno-strict-aliasing -fexceptions -DIL_STD -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


