################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Solution/Route.cpp \
../src/Solution/Schedule.cpp \
../src/Solution/ScheduleManager.cpp \
../src/Solution/Solution.cpp 

OBJS += \
./src/Solution/Route.o \
./src/Solution/Schedule.o \
./src/Solution/ScheduleManager.o \
./src/Solution/Solution.o 

CPP_DEPS += \
./src/Solution/Route.d \
./src/Solution/Schedule.d \
./src/Solution/ScheduleManager.d \
./src/Solution/Solution.d 


# Each subdirectory must supply rules for building sources it contributes
src/Solution/%.o: ../src/Solution/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


