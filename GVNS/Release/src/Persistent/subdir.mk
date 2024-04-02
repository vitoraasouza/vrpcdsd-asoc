################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Persistent/Edge.cpp \
../src/Persistent/Instance.cpp \
../src/Persistent/Node.cpp \
../src/Persistent/Vehicle.cpp 

OBJS += \
./src/Persistent/Edge.o \
./src/Persistent/Instance.o \
./src/Persistent/Node.o \
./src/Persistent/Vehicle.o 

CPP_DEPS += \
./src/Persistent/Edge.d \
./src/Persistent/Instance.d \
./src/Persistent/Node.d \
./src/Persistent/Vehicle.d 


# Each subdirectory must supply rules for building sources it contributes
src/Persistent/%.o: ../src/Persistent/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


