################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Utils/Utils.cpp 

OBJS += \
./src/Utils/Utils.o 

CPP_DEPS += \
./src/Utils/Utils.d 


# Each subdirectory must supply rules for building sources it contributes
src/Utils/%.o: ../src/Utils/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/YOUR_CPLEX_INSTALLATION_FOLDER/cplex/include -I/YOUR_CPLEX_INSTALLATION_FOLDER/concert/include -O3 -Wall -c -fmessage-length=0 -m64 -O -fPIC -fno-strict-aliasing -fexceptions -DIL_STD -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


